
/// This file has functionality to maintain a horizon of waypoints ahead
/// of the vehicle for it to follow.
/// The class is also responsible for managing lane change decisions and
/// modify the waypoint trajectory appropriately.

#include <deque>
#include <memory>

#include "carla/client/Timestamp.h"
#include "carla/rpc/ActorId.h"

#include "DataStructures.h"
#include "InMemoryMap.h"
#include "LocalizationUtils.h"
#include "Parameters.h"
#include "SimpleWaypoint.h"
#include "VehicleStateAndAttributeQuery.h"

namespace carla
{
namespace traffic_manager
{

namespace LocalizationConstants
{
static const float MINIMUM_HORIZON_LENGTH = 30.0f;
static const float MAXIMUM_HORIZON_LENGTH = 60.0f;
static const float TARGET_WAYPOINT_TIME_HORIZON = 1.0f;
static const float TARGET_WAYPOINT_HORIZON_LENGTH = 5.0f;
static const float MINIMUM_JUNCTION_LOOK_AHEAD = 10.0f;
static const float HIGHWAY_SPEED = 50.0f / 3.6f;
static const float ARBITRARY_MAX_SPEED = 100.0f / 3.6f;
static const float HORIZON_RATE = (MAXIMUM_HORIZON_LENGTH - MINIMUM_HORIZON_LENGTH) / ARBITRARY_MAX_SPEED;
static const float MINIMUM_LANE_CHANGE_DISTANCE = 10.0f;
static const float MAXIMUM_LANE_OBSTACLE_DISTANCE = 50.0f;
static const float MAXIMUM_LANE_OBSTACLE_CURVATURE = 0.6f;
static const unsigned int UNREGISTERED_ACTORS_SCAN_INTERVAL = 10;
static const float BLOCKED_TIME_THRESHOLD = 90.0f;
static const float DELTA_TIME_BETWEEN_DESTRUCTIONS = 10.0f;
static const float STOPPED_VELOCITY_THRESHOLD = 0.8f; // meters per second.
static const float INTER_LANE_CHANGE_DISTANCE = 10.0f;
static const float MAX_COLLISION_RADIUS = 100.0f;
static const float POSITION_WINDOW_SIZE = 2.1f;
} // namespace LocalizationConstants
using namespace LocalizationConstants;

namespace cc = carla::client;

using ActorId = carla::rpc::ActorId;
using KinematicStateMap = std::unordered_map<ActorId, KinematicState>;
using Buffer = std::deque<std::shared_ptr<SimpleWaypoint>>;
using BufferMap = std::unordered_map<carla::ActorId, Buffer>;
using BufferMapPtr = std::shared_ptr<BufferMap>;
using LocalMapPtr = std::shared_ptr<InMemoryMap>;

// TODO: Return structure of structures to feed C, TL, MP.
void Loclization(const ActorId actor_id,
                 cc::Timestamp current_timestamp,
                 BufferMapPtr& buffer_map,
                 const KinematicStateMap& state_map,
                 std::unordered_map<ActorId, double>& idle_time,
                 TrackTraffic& track_traffic,
                 LocalMapPtr& local_map,
                 Parameters& parameters) {

  const cg::Location vehicle_location = GetLocation(state_map, actor_id);
  const cg::Vector3D heading_vector = GetHeading(state_map, actor_id);
  const cg::Vector3D vehicle_velocity_vector = GetVelocity(state_map, actor_id);
  const float vehicle_speed = vehicle_velocity_vector.Length();

  // Initializing idle times.
  if (idle_time.find(actor_id) == idle_time.end() && current_timestamp.elapsed_seconds != 0) {
    idle_time[actor_id] = current_timestamp.elapsed_seconds;
  }

  // Speed dependent waypoint horizon length.
  const float horizon_square = std::min(std::pow(vehicle_speed * HORIZON_RATE + MINIMUM_HORIZON_LENGTH, 2.0f),
                                        std::pow(MAXIMUM_HORIZON_LENGTH, 2.0f));

  if (buffer_map->find(actor_id) == buffer_map->end()) {
    buffer_map->insert({actor_id, Buffer()});
  }

  Buffer &waypoint_buffer = buffer_map->at(actor_id);

  // Clear buffer if vehicle is too far from the first waypoint in the buffer.
  if (!waypoint_buffer.empty() &&
      cg::Math::DistanceSquared(waypoint_buffer.front()->GetLocation(), vehicle_location) > std::pow(30.0f, 2)) {

    auto number_of_pops = waypoint_buffer.size();
    for (uint64_t j = 0u; j < number_of_pops; ++j) {
      PopWaypoint(actor_id, track_traffic, waypoint_buffer);
    }
  }

  if (!waypoint_buffer.empty()) {
    // Purge passed waypoints.
    float dot_product = DeviationDotProduct(vehicle_location, heading_vector, waypoint_buffer.front()->GetLocation());
    while (dot_product <= 0.0f && !waypoint_buffer.empty()) {

      PopWaypoint(actor_id, track_traffic, waypoint_buffer);
      if (!waypoint_buffer.empty()) {
        dot_product = DeviationDotProduct(vehicle_location, heading_vector, waypoint_buffer.front()->GetLocation());
      }
    }

    // Purge waypoints too far from the front of the buffer.
    while (!waypoint_buffer.empty()
            && waypoint_buffer.back()->DistanceSquared(waypoint_buffer.front()) > horizon_square) {
      PopWaypoint(actor_id, track_traffic, waypoint_buffer, false);
    }
  }

  // Initializing buffer if it is empty.
  if (waypoint_buffer.empty()) {
    SimpleWaypointPtr closest_waypoint = local_map->GetWaypointInVicinity(vehicle_location);
    if (closest_waypoint == nullptr) {
      closest_waypoint = local_map->GetWaypoint(vehicle_location);
    }
    PushWaypoint(actor_id, track_traffic, waypoint_buffer, closest_waypoint);
  }

  // Assign a lane change.

  const ChangeLaneInfo lane_change_info = parameters.GetForceLaneChange(vehicle);
  bool force_lane_change = lane_change_info.change_lane;
  bool lane_change_direction = lane_change_info.direction;

  if (!force_lane_change) {
    float perc_keep_right = parameters.GetKeepRightPercentage(vehicle);
    if (perc_keep_right >= 0.0f && perc_keep_right >= (rand() % 101)) {
        force_lane_change = true;
        lane_change_direction = true;
    }
  }

  const SimpleWaypointPtr front_waypoint = waypoint_buffer.front();
  const double lane_change_distance = std::pow(std::max(10.0f * vehicle_speed, INTER_LANE_CHANGE_DISTANCE), 2);

  if (((parameters.GetAutoLaneChange(vehicle) || force_lane_change) && !front_waypoint->CheckJunction())
      && (last_lane_change_location.find(actor_id) == last_lane_change_location.end()
          || cg::Math::DistanceSquared(last_lane_change_location.at(actor_id), vehicle_location)
              > lane_change_distance )) {

    SimpleWaypointPtr change_over_point = AssignLaneChange(
        vehicle, vehicle_location, force_lane_change, lane_change_direction);

    if (change_over_point != nullptr) {
      if (last_lane_change_location.find(actor_id) != last_lane_change_location.end()) {
        last_lane_change_location.at(actor_id) = vehicle_location;
      } else {
        last_lane_change_location.insert({actor_id, vehicle_location});
      }
      auto number_of_pops = waypoint_buffer.size();
      for (uint64_t j = 0u; j < number_of_pops; ++j) {
        PopWaypoint(waypoint_buffer, actor_id);
      }
      PushWaypoint(waypoint_buffer, actor_id, change_over_point);
    }
  }

  // Populating the buffer.
  while (waypoint_buffer.back()->DistanceSquared(waypoint_buffer.front()) <= horizon_square) {

    std::vector<SimpleWaypointPtr> next_waypoints = waypoint_buffer.back()->GetNextWaypoint();
    uint64_t selection_index = 0u;
    // Pseudo-randomized path selection if found more than one choice.
    if (next_waypoints.size() > 1) {
      selection_index = static_cast<uint64_t>(rand()) % next_waypoints.size();
    }
    SimpleWaypointPtr next_wp = next_waypoints.at(selection_index);
    if (next_wp == nullptr) {
      for (auto& wp: next_waypoints) {
        if (wp != nullptr) {
          next_wp = wp;
          break;
        }
      }
    }
    PushWaypoint(waypoint_buffer, actor_id, next_wp);
  }

  // Begining point of the waypoint buffer;
  const SimpleWaypointPtr& updated_front_waypoint = waypoint_buffer.front();

  // Updating geodesic grid position for actor.
  track_traffic.UpdateGridPosition(actor_id, waypoint_buffer);
  // WayPoint Binning Changes
  // Generating output.
  const float target_point_distance = std::max(std::ceil(vehicle_speed * TARGET_WAYPOINT_TIME_HORIZON),
      TARGET_WAYPOINT_HORIZON_LENGTH);

  std::pair<SimpleWaypointPtr,uint64_t> target_waypoint_index_pair = track_traffic.GetTargetWaypoint(waypoint_buffer, target_point_distance);
  SimpleWaypointPtr &target_waypoint = target_waypoint_index_pair.first;
  const cg::Location target_location = target_waypoint->GetLocation();
  float dot_product = DeviationDotProduct(vehicle, vehicle_location, target_location);
  float cross_product = DeviationCrossProduct(vehicle, vehicle_location, target_location);
  dot_product = 1.0f - dot_product;
  if (cross_product < 0.0f) {
    dot_product *= -1.0f;
  }

  float distance = 0.0f; // TODO: use in PID

  // Filtering out false junctions on highways:
  // on highways, if there is only one possible path and the section is
  // marked as intersection, ignore it.
  const auto vehicle_reference = boost::static_pointer_cast<cc::Vehicle>(vehicle);
  const float speed_limit = vehicle_reference->GetSpeedLimit();
  const float look_ahead_distance = std::max(2.0f * vehicle_speed, MINIMUM_JUNCTION_LOOK_AHEAD);

  std::pair<SimpleWaypointPtr,uint64_t> look_ahead_point_index_pair = track_traffic.GetTargetWaypoint(waypoint_buffer, look_ahead_distance);
  SimpleWaypointPtr &look_ahead_point = look_ahead_point_index_pair.first;
  uint64_t &look_ahead_index = look_ahead_point_index_pair.second;
  bool approaching_junction = false;
  if (look_ahead_point->CheckJunction() && !(updated_front_waypoint->CheckJunction())) {
    if (speed_limit*3.6f > HIGHWAY_SPEED) {
      for (uint64_t j = 0u; (j < look_ahead_index) && !approaching_junction; ++j) {
        SimpleWaypointPtr swp = waypoint_buffer.at(j);
        if (swp->GetNextWaypoint().size() > 1) {
          approaching_junction = true;
        }
      }
    } else {
      approaching_junction = true;
    }
  }

  // Reset the variables when no longer approaching a junction.
  if (!approaching_junction && approached[actor_id]){
    final_safe_points[actor_id] = nullptr;
    approached[actor_id] = false;
  }

  // Only do once, when the junction has just been seen.
  else if (approaching_junction && !approached[actor_id]){

    SimpleWaypointPtr final_point = nullptr;
    final_point = GetSafeLocationAfterJunction(vehicle_reference, waypoint_buffer);
    if(final_point != nullptr){
      final_safe_points[actor_id] = final_point;
      approaching_junction = false;
      approached[actor_id] = true;
    }
  }

  // Determining possible collision candidates around the ego vehicle.
  ActorIdSet overlapping_actor_set = track_traffic.GetOverlappingVehicles(actor_id);
  using ActorInfoMap = std::unordered_map<ActorId, std::pair<Actor, cg::Location>>;
  ActorInfoMap overlapping_actor_info;
  std::vector<ActorId> collision_candidate_ids;

  // Run through vehicles with overlapping paths, obtain actor reference,
  // and filter them based on distance to ego vehicle.
  Actor overlapping_actor_ptr = nullptr;
  for (ActorId overlapping_actor_id: overlapping_actor_set) {

    // If actor is part of the registered actors.
    if (vehicle_id_to_index.find(overlapping_actor_id) != vehicle_id_to_index.end()) {
      overlapping_actor_ptr = actor_list.at(vehicle_id_to_index.at(overlapping_actor_id));
    }
    // If actor is part of the unregistered actors.
    else if (unregistered_actors.find(overlapping_actor_id) != unregistered_actors.end()) {
      overlapping_actor_ptr = unregistered_actors.at(overlapping_actor_id);
    }
    // If actor is within maximum collision avoidance range.
    if (overlapping_actor_ptr!=nullptr && overlapping_actor_ptr->IsAlive()
        && cg::Math::DistanceSquared(overlapping_actor_ptr->GetLocation(),
                                      vehicle_location) < std::pow(MAX_COLLISION_RADIUS, 2))
    {
      overlapping_actor_info.insert({overlapping_actor_id,
                                      {overlapping_actor_ptr, overlapping_actor_ptr->GetLocation()}});
      collision_candidate_ids.push_back(overlapping_actor_id);
    }
  }

  // Sorting collision candidates in accending order of distance to current vehicle.
  std::sort(collision_candidate_ids.begin(), collision_candidate_ids.end(),
            [&overlapping_actor_info, &vehicle_location] (const ActorId& a_id_1, const ActorId& a_id_2) {
              const cg::Location& e_loc = vehicle_location;
              const cg::Location& loc_1 = overlapping_actor_info.at(a_id_1).second;
              const cg::Location& loc_2 = overlapping_actor_info.at(a_id_2).second;
              return (cg::Math::DistanceSquared(e_loc, loc_1) < cg::Math::DistanceSquared(e_loc, loc_2));
            });

  // Constructing output vector.
  std::vector<std::tuple<ActorId, Actor, cg::Vector3D>> collision_candidates;
  std::for_each(collision_candidate_ids.begin(), collision_candidate_ids.end(),
                [&overlapping_actor_info, &collision_candidates, this] (const ActorId& a_id) {
                  collision_candidates.push_back({a_id,
                                                  overlapping_actor_info.at(a_id).first,
                                                  this->GetVelocity(a_id)});
                });

  // Sampling waypoint window for teleportation in hybrid physics mode.
  std::vector<SimpleWaypointPtr> position_window;
  if (hybrid_physics_mode) {

    cg::Vector3D heading = vehicle->GetTransform().GetForwardVector();
    bool window_begin = false;
    SimpleWaypointPtr begining_wp;
    bool window_complete = false;

    for (uint32_t j = 0u; j < waypoint_buffer.size() && !window_complete; ++j) {

      SimpleWaypointPtr &swp = waypoint_buffer.at(j);
      cg::Vector3D relative_position = swp->GetLocation() - vehicle_location;

      // Sample waypoints in front of the vehicle;
      if (!window_begin && cg::Math::Dot(relative_position, heading) > 0.0f) {
        window_begin = true;
        begining_wp = swp;
      }

      if (window_begin && !window_complete) {
        if (swp->DistanceSquared(begining_wp) > std::pow(POSITION_WINDOW_SIZE, 2)) {
          // Stop when maximum size is reached.
          window_complete = true;
        } else {
          position_window.push_back(swp);
        }
      }
    }
  }

  // Editing output frames.
  LocalizationToPlannerData &planner_message = current_planner_frame->at(i);
  planner_message.actor = vehicle;
  planner_message.deviation = dot_product;
  planner_message.distance = distance;
  planner_message.approaching_true_junction = approaching_junction;
  planner_message.velocity = vehicle_velocity_vector;
  planner_message.position_window = std::move(position_window);
  planner_message.physics_enabled = IsPhysicsEnabled(actor_id);

  LocalizationToCollisionData &collision_message = current_collision_frame->at(i);
  collision_message.actor = vehicle;
  collision_message.buffer = waypoint_buffer;
  collision_message.overlapping_actors = std::move(collision_candidates);
  collision_message.closest_waypoint = updated_front_waypoint;
  collision_message.junction_look_ahead_waypoint = waypoint_buffer.at(look_ahead_index);
  collision_message.safe_point_after_junction = final_safe_points[actor_id];
  collision_message.velocity = vehicle_velocity_vector;

  LocalizationToTrafficLightData &traffic_light_message = current_traffic_light_frame->at(i);
  traffic_light_message.actor = vehicle;
  traffic_light_message.closest_waypoint = updated_front_waypoint;
  traffic_light_message.junction_look_ahead_waypoint = waypoint_buffer.at(look_ahead_index);

  // Updating idle time when necessary.
  UpdateIdleTime(vehicle);
////////////--------------------------- outside ------------------------//////////////////
    if (IsVehicleStuck(maximum_idle_time.first)) {
      TryDestroyVehicle(maximum_idle_time.first);
    }

    // Updating maximum idle time to null for the next iteration.
    maximum_idle_time = std::make_pair(nullptr, current_timestamp.elapsed_seconds);
}

} // namespace traffic_manager
} // namespace carla
