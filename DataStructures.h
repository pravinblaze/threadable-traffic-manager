
/// This file contains definitions of data structures used in traffic manager.

#include <string>
#include <vector>

#include "carla/geom/Location.h"
#include "carla/geom/Rotation.h"
#include "carla/geom/Vector3D.h"
#include "carla/rpc/ActorId.h"
#include "carla/rpc/TrafficLightState.h"

namespace carla
{
namespace traffic_manager
{

namespace cg = carla::geom;

using ActorId = carla::rpc::ActorId;
using TLS = carla::rpc::TrafficLightState;

struct KinematicState
{
  bool physics_enabled;
  cg::Location location;
  cg::Rotation rotation;
  cg::Vector3D velocity;
};

enum ActorType {Vehicle, Pedestrian};

struct StaticAttributes
{
  ActorType actor_type;
  float half_length;
  float half_width;
  float half_height;
  float speed_limit;
};

struct TrafficLightState
{
  TLS tl_state;
  bool at_traffic_light;
};

struct CollisionHazardData
{
  bool hazard;
  float available_distance_margin;
  ActorId hazard_actor_id;
};

}
} // namespace carla
