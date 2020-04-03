
/// This file contains definitions of data structures used in traffic manager.

#include <string>
#include <vector>

#include "carla/geom/Location.h"
#include "carla/geom/Rotation.h"
#include "carla/geom/Vector3D.h"

namespace carla
{
namespace traffic_manager
{

namespace cg = carla::geom;

struct KinematicState
{
  bool physics_enabled;
  cg::Location location;
  cg::Rotation rotation;
  cg::Vector3D velocity;
};

struct StaticVehicleAttributes
{
  std::string type_id;
  float half_length;
  float half_width;
  float half_height;
};

}
} // namespace carla
