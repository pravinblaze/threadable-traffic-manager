
/// ALSM: Agent Lifecycle and State Managerment
/// This file has functionality to update the local cache of kinematic states
/// and manage memory and cleanup for varying number of vehicles in the simulation.

#pragma once

#include <memory>
#include <mutex>
#include <unordered_map>

#include "carla/client/Actor.h"
#include "carla/client/ActorList.h"
#include "carla/client/Client.h"
#include "carla/client/Vehicle.h"
#include "carla/client/Walker.h"
#include "carla/client/World.h"
#include "carla/Memory.h"
#include "carla/rpc/ActorId.h"
#include "boost/pointer_cast.hpp"

#include "AtomicActorSet.h"
#include "DataStructures.h"
#include "InMemoryMap.h"
#include "LocalizationUtils.h"
#include "SimpleWaypoint.h"

#include "Parameters.h"

namespace carla
{
namespace traffic_manager
{

namespace ALSMConstants
{
static const float HYBRID_MODE_DT = 0.05f;
static const float PHYSICS_RADIUS = 50.0f;
} // namespace ALSMConstants
using namespace ALSMConstants;

namespace cg = carla::geom;
namespace cc = carla::client;

using ActorId = carla::ActorId;
using ActorPtr = carla::SharedPtr<cc::Actor>;
using ActorMap = std::unordered_map<ActorId, ActorPtr>;
using ActorList = carla::SharedPtr<cc::ActorList>;
using Buffer = std::deque<std::shared_ptr<SimpleWaypoint>>;
using BufferMap = std::unordered_map<carla::ActorId, Buffer>;
using BufferMapPtr = std::shared_ptr<BufferMap>;
using LaneChangeLocationMap = std::unordered_map<ActorId, cg::Location>;
using KinematicStateMap = std::unordered_map<ActorId, KinematicState>;
using StaticAttributeMap = std::unordered_map<ActorId, StaticAttributes>;
using IdToIndexMap = std::unordered_map<ActorId, unsigned long>;
using LocalMapPtr = std::shared_ptr<InMemoryMap>;
using TimePoint = chr::time_point<chr::system_clock, chr::nanoseconds>;

void AgentLifecycleAndStateManagement(AtomicActorSet &registered_vehicles,
                                      std::vector<ActorPtr> &vehicle_list,
                                      ActorMap &unregistered_actors,
                                      int &registered_vehicles_state,
                                      IdToIndexMap &vehicle_id_to_index_map,
                                      BufferMapPtr &buffer_map_ptr,
                                      TrackTraffic &track_traffic,
                                      std::unordered_map<ActorId, double> &idle_time,
                                      ActorPtr &hero_vehicle,
                                      LaneChangeLocationMap &last_lane_change_location,
                                      KinematicStateMap &kinematic_state_map,
                                      StaticAttributeMap &static_attribute_map,
                                      TimePoint previous_update_instance,
                                      const Parameters &parameters,
                                      const cc::World &world,
                                      const LocalMapPtr &local_map)
{

  bool hybrid_physics_mode = parameters.GetHybridPhysicsMode();

  bool is_deleted_actors_present = false;
  std::set<unsigned int> world_vehicle_ids;
  std::set<unsigned int> world_pedestrian_ids;
  std::vector<ActorId> registered_list_to_be_deleted;
  std::vector<ActorId> unregistered_list_to_be_deleted;

  ActorList world_actors = world.GetActors();
  ActorList world_vehicles = world_actors->Filter("vehicle.*");
  ActorList world_pedestrians = world_actors->Filter("walker.*");

  // Scanning for new unregistered vehicles.
  for (auto iter = world_vehicles->begin(); iter != world_vehicles->end(); ++iter)
  {
    // Building set containing current world vehicle ids.
    world_vehicle_ids.insert((*iter)->GetId);
    const auto unregistered_id = (*iter)->GetId();
    if (!registered_vehicles.Contains(unregistered_id)
        && unregistered_actors.find(unregistered_id) == unregistered_actors.end())
    {
      unregistered_actors.insert({unregistered_id, *iter});
    }
  }

  // Scanning for new pedestrians.
  for (auto iter = world_pedestrians->begin(); iter != world_pedestrians->end(); ++iter)
  {
    // Building set containing current world pedestrian ids.
    world_pedestrian_ids.insert((*iter)->GetId);
    const auto unregistered_id = (*iter)->GetId();
    if (unregistered_actors.find(unregistered_id) == unregistered_actors.end())
    {
      unregistered_actors.insert({unregistered_id, *iter});
    }
  }

  // Identify hero vehicle if currently not present
  // and system is in hybrid physics mode.
  if (hybrid_physics_mode && hero_vehicle == nullptr)
  {
    for (auto iter = unregistered_actors.begin(); iter != unregistered_actors.end(); ++iter)
    {
      ActorPtr actor_ptr = iter->second;
      for (auto &&attribute : actor_ptr->GetAttributes())
      {
        if (attribute.GetId() == "role_name" && attribute.GetValue() == "hero")
        {
          hero_vehicle = actor_ptr;
          break;
        }
      }
    }
  }
  // Invalidate hero actor pointer if it is not alive anymore.
  else if (hybrid_physics_mode && hero_vehicle != nullptr
           && world_vehicle_ids.find(hero_vehicle->GetId()) == world_vehicle_ids.end())
  {
    hero_vehicle = nullptr;
  }

  // Search for invalid/destroyed registered vehicles.
  for (auto &actor : vehicle_list)
  {
    ActorId deletion_id = actor->GetId();
    if (world_vehicle_ids.find(deletion_id) == world_vehicle_ids.end())
    {
      registered_list_to_be_deleted.push_back(deletion_id);
      track_traffic.DeleteActor(deletion_id);
      last_lane_change_location.erase(deletion_id);
      kinematic_state_map.erase(deletion_id);
      static_attribute_map.erase(deletion_id);
      idle_time.erase(deletion_id);
    }
  }

  // Clearing the registered actor list.
  if (!registered_list_to_be_deleted.empty())
  {
    registered_vehicles.Remove(registered_list_to_be_deleted);
    vehicle_list.clear();
    vehicle_list = registered_vehicles.GetList();
    is_deleted_actors_present = true;
  }

  // Building a list of registered actors and connecting
  // the vehicle ids to their position indices on data arrays.
  if (is_deleted_actors_present || (registered_vehicles_state != registered_vehicles.GetState()))
  {
    vehicle_list.clear();
    registered_list_to_be_deleted.clear();
    vehicle_list = registered_vehicles.GetList();
    uint64_t index = 0u;
    vehicle_id_to_index_map.clear();
    for (auto &actor : vehicle_list)
    {
      vehicle_id_to_index_map.insert({actor->GetId(), index});
      ++index;
    }
    registered_vehicles_state = registered_vehicles.GetState();
  }

  // Regularly update unregistered actor states and clean up any invalid actors.
  for (auto iter = unregistered_actors.begin(); iter != unregistered_actors.cend(); ++iter)
  {
    if (registered_vehicles.Contains(iter->first) || world_vehicle_ids.find(iter->first) == world_vehicle_ids.end())
    {
      unregistered_list_to_be_deleted.push_back(iter->first);
    }
    else
    {
      // Updating data structures.
      cg::Location location = iter->second->GetLocation();
      const auto type = iter->second->GetTypeId();

      SimpleWaypointPtr nearest_waypoint = nullptr;
      if (type.front() == 'v')
      {
        nearest_waypoint = local_map->GetWaypointInVicinity(location);
      }
      else if (type.front() == 'w')
      {
        nearest_waypoint = local_map->GetPedWaypoint(location);
      }

      if (nearest_waypoint == nullptr)
      {
        nearest_waypoint = local_map->GetWaypoint(location);
      }

      track_traffic.UpdateUnregisteredGridPosition(iter->first, nearest_waypoint);
    }
  }

  // Removing invalid/destroyed unregistered actors.
  for (auto deletion_id : unregistered_list_to_be_deleted)
  {
    unregistered_actors.erase(deletion_id);
    track_traffic.DeleteActor(deletion_id);
    kinematic_state_map.erase(deletion_id);
    static_attribute_map.erase(deletion_id);
  }

  // Location of hero vehicle if present.
  cg::Location hero_location;
  if (hybrid_physics_mode && hero_vehicle != nullptr)
  {
    hero_location = hero_vehicle->GetLocation();
  }

  // Using (1/20)s time delta for computing velocity.
  float dt = HYBRID_MODE_DT;
  // Skipping velocity update if elapsed time is less than 0.05s in asynchronous, hybrid mode.
  if (!parameters.GetSynchronousMode())
  {
    TimePoint current_instance = chr::system_clock::now();
    chr::duration<float> elapsed_time = current_instance - previous_update_instance;
    if (elapsed_time.count() > dt)
    {
      previous_update_instance = current_instance;
    }
    else if (hybrid_physics_mode)
    {
      return;
    }
  }

  // Update kinematic state and static attributes for all registered vehicles.
  for (const Actor &vehicle : vehicle_list)
  {

    ActorId actor_id = vehicle->GetId();
    cg::Transform vehicle_transform = vehicle->GetTransform();
    cg::Location vehicle_location = vehicle_transform.location;
    cg::Rotation vehicle_rotation = vehicle_transform.rotation;

    // Update static attribute map if entry not present.
    if (static_attribute_map.find(actor_id) == static_attribute_map.end())
    {
      auto vehicle_ptr = boost::static_pointer_cast<cc::Vehicle>(vehicle);
      cg::Vector3D dimensions = vehicle_ptr->GetBoundingBox().extent;
      static_attribute_map.insert({actor_id, {ActorType::Vehicle,
                                              dimensions.x, dimensions.y, dimensions.z,
                                              vehicle_ptr->GetSpeedLimit()}});
    }

    // Adding entry if not present.
    if (kinematic_state_map.find(actor_id) == kinematic_state_map.end())
    {
      kinematic_state_map.insert({actor_id, KinematicState{true, vehicle_location, vehicle_rotation, cg::Vector3D()}});
    }

    // Check if current actor is in range of hero actor and enable physics in hybrid mode.
    bool in_range_of_hero_actor = false;
    if (hybrid_physics_mode && hero_vehicle != nullptr
        && (cg::Math::DistanceSquared(vehicle_location, hero_location) < std::pow(PHYSICS_RADIUS, 2)))
    {
      in_range_of_hero_actor = true;
    }
    bool enable_physics = hybrid_physics_mode ? in_range_of_hero_actor : true;
    kinematic_state_map.at(actor_id).physics_enabled = enable_physics;
    vehicle->SetSimulatePhysics(enable_physics);

    // When we say velocity, we usually mean velocity for a vehicle along it's heading.
    // Velocity component due to rotation can be removed by taking dot product with heading vector.
    cg::Vector3D heading = vehicle->GetTransform().GetForwardVector();
    if (enable_physics)
    {

      kinematic_state_map.at(actor_id).velocity = cg::Math::Dot(vehicle->GetVelocity(), heading) * heading;
    }
    else
    {

      cg::Vector3D displacement = (vehicle_location - kinematic_state_map.at(actor_id).location);
      cg::Vector3D displacement_along_heading = cg::Math::Dot(displacement, heading) * heading;
      cg::Vector3D velocity = displacement_along_heading / dt;
      kinematic_state_map.at(actor_id).velocity = velocity;
    }

    // Updating location after velocity is computed.
    kinematic_state_map.at(actor_id).location = vehicle_location;
  }

  // Update kinematic state and static attributes for unregistered actors.
  for (auto &unregistered_actor: unregistered_actors)
  {
    const ActorId actor_id = unregistered_actor.first;
    const ActorPtr actor_ptr = unregistered_actor.second;
    // Update static attribute map if entry not present.
    if (static_attribute_map.find(actor_id) == static_attribute_map.end())
    {
      const std::string type_id = actor_ptr->GetTypeId();
      ActorType actor_type;
      cg::Vector3D dimensions;
      float speed_limit = -1.0f;
      if (type_id.front() == 'v')
      {
        auto vehicle_ptr = boost::static_pointer_cast<cc::Vehicle>(actor_ptr);
        dimensions = vehicle_ptr->GetBoundingBox().extent;
        actor_type = ActorType::Vehicle;
        speed_limit = vehicle_ptr->GetSpeedLimit();
      }
      else if (type_id.front() == 'w')
      {
        auto walker_ptr = boost::static_pointer_cast<cc::Walker>(actor_ptr);
        dimensions = walker_ptr->GetBoundingBox().extent;
        actor_type = ActorType::Pedestrian;
      }
      static_attribute_map.insert({actor_id, {actor_type, dimensions.x, dimensions.y, dimensions.z, speed_limit}});
    }

    // Update kinematic state for the actor.
    const cg::Transform actor_transform = actor_ptr->GetTransform();
    const cg::Location actor_location = actor_transform.location;
    const cg::Rotation actor_rotation = actor_transform.rotation;
    const cg::Vector3D actor_velocity = actor_ptr->GetVelocity();
    const KinematicState kinematic_state = KinematicState{true, actor_location, actor_rotation, actor_velocity};
    // Adding entry if not present.
    if (kinematic_state_map.find(actor_id) == kinematic_state_map.end())
    {
      kinematic_state_map.insert({actor_id, kinematic_state});
    }
    else
    {
      kinematic_state_map.at(actor_id) = kinematic_state;
    }
  }
}

} // namespace traffic_manager
} // namespace carla
