
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
using IdleTimeMap = std::unordered_map<ActorId, double>;
using Buffer = std::deque<std::shared_ptr<SimpleWaypoint>>;
using BufferMap = std::unordered_map<carla::ActorId, Buffer>;
using BufferMapPtr = std::shared_ptr<BufferMap>;
using LocalMapPtr = std::shared_ptr<InMemoryMap>;
using LaneChangeLocationMap = std::unordered_map<ActorId, cg::Location>;
using StaticAttributeMap = std::unordered_map<ActorId, StaticAttributes>;

// TODO: Return structure of structures to feed C, TL, MP.
void Loclization(const ActorId actor_id,
                 cc::Timestamp current_timestamp,
                 BufferMapPtr& buffer_map,
                 const KinematicStateMap& state_map,
                 IdleTimeMap& idle_time,
                 std::pair<ActorId, double> max_idle_time,
                 TrackTraffic& track_traffic,
                 LocalMapPtr& local_map,
                 Parameters& parameters,
                 LaneChangeLocationMap& last_lane_change_location) {

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
  const ChangeLaneInfo lane_change_info = parameters.GetForceLaneChange(actor_id);
  bool force_lane_change = lane_change_info.change_lane;
  bool lane_change_direction = lane_change_info.direction;

  if (!force_lane_change) {
    float perc_keep_right = parameters.GetKeepRightPercentage(actor_id);
    if (perc_keep_right >= 0.0f && perc_keep_right >= (rand() % 101)) {
        force_lane_change = true;
        lane_change_direction = true;
    }
  }

  const SimpleWaypointPtr front_waypoint = waypoint_buffer.front();
  const double lane_change_distance = std::pow(std::max(10.0f * vehicle_speed, INTER_LANE_CHANGE_DISTANCE), 2);

  if (((parameters.GetAutoLaneChange(actor_id) || force_lane_change) && !front_waypoint->CheckJunction())
      && (last_lane_change_location.find(actor_id) == last_lane_change_location.end()
          || cg::Math::DistanceSquared(last_lane_change_location.at(actor_id), vehicle_location)
              > lane_change_distance )) {

    SimpleWaypointPtr change_over_point = AssignLaneChange(actor_id,
                                                           vehicle_location, vehicle_speed,
                                                           buffer_map, track_traffic,
                                                           force_lane_change, lane_change_direction);

    if (change_over_point != nullptr) {
      if (last_lane_change_location.find(actor_id) != last_lane_change_location.end()) {
        last_lane_change_location.at(actor_id) = vehicle_location;
      } else {
        last_lane_change_location.insert({actor_id, vehicle_location});
      }
      auto number_of_pops = waypoint_buffer.size();
      for (uint64_t j = 0u; j < number_of_pops; ++j) {
        PopWaypoint(actor_id, track_traffic, waypoint_buffer);
      }
      PushWaypoint(actor_id, track_traffic, waypoint_buffer, change_over_point);
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
    PushWaypoint(actor_id, track_traffic, waypoint_buffer, next_wp);
  }

  // Begining point of the waypoint buffer;
  const SimpleWaypointPtr& updated_front_waypoint = waypoint_buffer.front();

  // Updating geodesic grid position for actor.
  track_traffic.UpdateGridPosition(actor_id, waypoint_buffer);
  // WayPoint Binning Changes
  // Generating output.
  const float target_point_distance = std::max(std::ceil(vehicle_speed * TARGET_WAYPOINT_TIME_HORIZON),
      TARGET_WAYPOINT_HORIZON_LENGTH);

  using TargetWPInfo = std::pair<SimpleWaypointPtr,uint64_t>;
  TargetWPInfo target_waypoint_index_pair = track_traffic.GetTargetWaypoint(waypoint_buffer, target_point_distance);
  SimpleWaypointPtr &target_waypoint = target_waypoint_index_pair.first;
  const cg::Location target_location = target_waypoint->GetLocation();
  float dot_product = DeviationDotProduct(vehicle_location, heading_vector, target_location);
  float cross_product = DeviationCrossProduct(vehicle_location, heading_vector, target_location);
  dot_product = 1.0f - dot_product;
  if (cross_product < 0.0f) {
    dot_product *= -1.0f;
  }

  // Determining possible collision candidates around the ego vehicle.
  ActorIdSet overlapping_vehicle_set = track_traffic.GetOverlappingVehicles(actor_id);
  std::vector<ActorId> collision_candidate_ids;

  // Run through vehicles with overlapping paths,
  // and filter them based on distance to ego vehicle.
  float collision_radius_square = std::pow(MAX_COLLISION_RADIUS, 2);
  for (ActorId overlapping_actor_id: overlapping_vehicle_set) {

    // If actor is within maximum collision avoidance range.
    cg::Location overlapping_actor_location = GetLocation(state_map, actor_id);
    if (cg::Math::DistanceSquared(overlapping_actor_location, vehicle_location) < collision_radius_square)
    {
      collision_candidate_ids.push_back(overlapping_actor_id);
    }
  }

  // Sorting collision candidates in accending order of distance to current vehicle.
  std::sort(collision_candidate_ids.begin(), collision_candidate_ids.end(),
            [&state_map, &vehicle_location] (const ActorId& a_id_1, const ActorId& a_id_2) {
              const cg::Location& e_loc = vehicle_location;
              const cg::Location& loc_1 = GetLocation(state_map, a_id_1);
              const cg::Location& loc_2 = GetLocation(state_map, a_id_2);
              return (cg::Math::DistanceSquared(e_loc, loc_1) < cg::Math::DistanceSquared(e_loc, loc_2));
            });

  // Data to share with motion planning.
  LocalizationToPlannerData &planner_message = current_planner_frame->at(i);
  planner_message.actor_id = actor_id;
  planner_message.deviation = dot_product;

  // Data to share with for collision avoidance.
  LocalizationToCollisionData &collision_message = current_collision_frame->at(i);
  collision_message.actor_id = actor_id;
  collision_message.overlapping_actors = std::move(collision_candidate_ids);

  // Updating idle time when necessary.
  UpdateIdleTime(vehicle);
////////////--------------------------- outside ------------------------//////////////////
    if (IsVehicleStuck(maximum_idle_time.first)) {
      TryDestroyVehicle(maximum_idle_time.first);
    }

    // Updating maximum idle time to null for the next iteration.
    maximum_idle_time = std::make_pair(nullptr, current_timestamp.elapsed_seconds);
}

SimpleWaypointPtr AssignLaneChange(const ActorId &actor_id,
                                   const cg::Location &vehicle_location,
                                   const float &vehicle_speed,
                                   BufferMapPtr &buffer_map,
                                   TrackTraffic &track_traffic,
                                   bool force, bool direction)
{

  // Waypoint representing the new starting point for the waypoint buffer
  // due to lane change. Remains nullptr if lane change not viable.
  SimpleWaypointPtr change_over_point = nullptr;

  // Retrieve waypoint buffer for current vehicle.
  const Buffer& waypoint_buffer = buffer_map->at(actor_id);

  // Check buffer is not empty.
  if (!waypoint_buffer.empty())
  {
    // Get the left and right waypoints for the current closest waypoint.
    const SimpleWaypointPtr& current_waypoint = waypoint_buffer.front();
    const SimpleWaypointPtr left_waypoint = current_waypoint->GetLeftWaypoint();
    const SimpleWaypointPtr right_waypoint = current_waypoint->GetRightWaypoint();

    // Retrieve vehicles with overlapping waypoint buffers with current vehicle.
    const auto blocking_vehicles = track_traffic.GetOverlappingVehicles(actor_id);

    // Find immediate in-lane obstacle and check if any are too close to initiate lane change.
    bool obstacle_too_close = false;
    float minimum_squared_distance = std::numeric_limits<float>::infinity();
    ActorId obstacle_actor_id = 0u;
    for (auto i = blocking_vehicles.begin();
          i != blocking_vehicles.end() && !obstacle_too_close && !force;
          ++i)
    {
      const ActorId &other_actor_id = *i;
      // Find vehicle in buffer map and check if it's buffer is not empty.
      if (buffer_map->find(other_actor_id) != buffer_map->end()
          && !buffer_map->at(other_actor_id).empty())
      {
        const Buffer& other_buffer = buffer_map->at(other_actor_id);
        const SimpleWaypointPtr& other_current_waypoint = other_buffer.front();
        const cg::Location other_location = other_current_waypoint->GetLocation();

        const cg::Vector3D reference_heading = current_waypoint->GetForwardVector();
        cg::Vector3D reference_to_other = other_current_waypoint->GetLocation()
                                          - current_waypoint->GetLocation();
        const cg::Vector3D other_heading = other_current_waypoint->GetForwardVector();

        // Check both vehicles are not in junction,
        // Check if the other vehicle is in front of the current vehicle,
        // Check if the two vehicles have acceptable angular deviation between their headings.
        if (!current_waypoint->CheckJunction()
            && !other_current_waypoint->CheckJunction()
            && other_current_waypoint->GetWaypoint()->GetRoadId() == current_waypoint->GetWaypoint()->GetRoadId()
            && other_current_waypoint->GetWaypoint()->GetLaneId() == current_waypoint->GetWaypoint()->GetLaneId()
            && cg::Math::Dot(reference_heading, reference_to_other) > 0.0f
            && cg::Math::Dot(reference_heading, other_heading) > MAXIMUM_LANE_OBSTACLE_CURVATURE)
        {
          float squared_distance = cg::Math::DistanceSquared(vehicle_location, other_location);
          // Abort if the obstacle is too close.
          if (squared_distance > std::pow(MINIMUM_LANE_CHANGE_DISTANCE, 2))
          {
            // Remember if the new vehicle is closer.
            if (squared_distance < minimum_squared_distance
                && squared_distance < std::pow(MAXIMUM_LANE_OBSTACLE_DISTANCE, 2.0f))
            {
              minimum_squared_distance = squared_distance;
              obstacle_actor_id = other_actor_id;
            }
          } else {
            obstacle_too_close = true;
          }
        }
      }
    }

    // If a valid immediate obstacle found.
    if (!obstacle_too_close && obstacle_actor_id != 0u && !force)
    {
      const Buffer& other_buffer = buffer_map->at(obstacle_actor_id);
      const SimpleWaypointPtr& other_current_waypoint = other_buffer.front();
      const auto other_neighbouring_lanes = {other_current_waypoint->GetLeftWaypoint(),
                                              other_current_waypoint->GetRightWaypoint()};

      // Flags reflecting whether adjacent lanes are free near the obstacle.
      bool distant_left_lane_free = false;
      bool distant_right_lane_free = false;

      // Check if the neighbouring lanes near the obstructing vehicle are free of other vehicles.
      bool left_right = true;
      for (auto& candidate_lane_wp: other_neighbouring_lanes) {
        if (candidate_lane_wp != nullptr &&
            track_traffic.GetPassingVehicles(candidate_lane_wp->GetId()).size() == 0) {

          if (left_right) distant_left_lane_free = true;
          else distant_right_lane_free = true;

        }
        left_right = !left_right;
      }

      // Based on what lanes are free near the obstacle,
      // find the change over point with no vehicles passing through them.
      if (distant_right_lane_free && right_waypoint != nullptr
          && track_traffic.GetPassingVehicles(right_waypoint->GetId()).size() == 0)
      {
        change_over_point = right_waypoint;
      } else if (distant_left_lane_free && left_waypoint != nullptr
                  && track_traffic.GetPassingVehicles(left_waypoint->GetId()).size() == 0)
      {
        change_over_point = left_waypoint;
      }
    } else if (force) {
      if (direction && right_waypoint != nullptr) {
        change_over_point = right_waypoint;
      } else if (!direction && left_waypoint != nullptr) {
        change_over_point = left_waypoint;
      }
    }

    if (change_over_point != nullptr)
    {
      const float change_over_distance =  cg::Math::Clamp(1.5f*vehicle_speed, 3.0f, 20.0f);
      const auto starting_point = change_over_point;
      while (change_over_point->DistanceSquared(starting_point) < std::pow(change_over_distance, 2) &&
            !change_over_point->CheckJunction()) {
        change_over_point = change_over_point->GetNextWaypoint()[0];
      }
    }
  }

  return change_over_point;
}

void UpdateIdleTime(IdleTimeMap& idle_time, std::pair<ActorId, double>& max_idle_time, const ActorId& actor_id) {
  if (idle_time.find(actor_id) == idle_time.end()) {
    return;
  }

  const auto vehicle = boost::static_pointer_cast<cc::Vehicle>(actor);
  if (GetVelocity(actor_id).Length() > STOPPED_VELOCITY_THRESHOLD
      || (vehicle->IsAtTrafficLight() && vehicle->GetTrafficLightState() != TLS::Green)) {
    idle_time[actor->GetId()] = current_timestamp.elapsed_seconds;
  }

  // Checking maximum idle time.
  if (max_idle_time.first == nullptr || max_idle_time.second > idle_time[actor->GetId()]) {
    max_idle_time = std::make_pair(actor, idle_time[actor->GetId()]);
  }
}

bool IsVehicleStuck(const Actor& actor) {
  if (actor == nullptr) {
    return false;
  }

  if (idle_time.find(actor->GetId()) != idle_time.end()) {
    auto delta_idle_time = current_timestamp.elapsed_seconds - idle_time.at(actor->GetId());
    if (delta_idle_time >= BLOCKED_TIME_THRESHOLD) {
      return true;
    }
  }
  return false;
}

} // namespace traffic_manager
} // namespace carla
