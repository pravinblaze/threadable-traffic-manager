
/// This file contains various constants used in traffic manager
/// arranged into sensible namespaces for re-usability across
/// different files.

#define RATE(MaxY, MinY, DiffX) (MaxY - MinY) / DiffX

namespace carla
{
namespace traffic_manager
{
namespace constants
{

namespace VehicleRemoval
{
static const float STOPPED_VELOCITY_THRESHOLD = 0.8f;
static const float BLOCKED_TIME_THRESHOLD = 90.0f;
static const float DELTA_TIME_BETWEEN_DESTRUCTIONS = 10.0f;
} // namespace VehicleRemoval

namespace HybridMode
{
static const float HYBRID_MODE_DT = 0.05f;
static const float PHYSICS_RADIUS = 50.0f;
} // namespace HybridMode

namespace PathBufferUpdate
{
static const float MAX_START_DISTANCE = 30.0f;
static const float MINIMUM_HORIZON_LENGTH = 30.0f;
static const float MAXIMUM_HORIZON_LENGTH = 60.0f;
static const float HORIZON_RATE = RATE(MAXIMUM_HORIZON_LENGTH,
                                       MINIMUM_HORIZON_LENGTH,
                                       SpeedThreshold::ARBITRARY_MAX_SPEED);
} // namespace PathBufferUpdate

namespace SpeedThreshold
{
static const float HIGHWAY_SPEED = 50.0f / 3.6f;
static const float ARBITRARY_MAX_SPEED = 100.0f / 3.6f;
} // namespace SpeedThreshold

namespace WaypointSelection
{
static const float TARGET_WAYPOINT_TIME_HORIZON = 1.0f;
static const float TARGET_WAYPOINT_HORIZON_LENGTH = 5.0f;
static const float MINIMUM_JUNCTION_LOOK_AHEAD = 10.0f;
} // namespace WaypointSelection

namespace LaneChange
{
static const float MINIMUM_LANE_CHANGE_DISTANCE = 10.0f;
static const float MAXIMUM_LANE_OBSTACLE_DISTANCE = 50.0f;
static const float MAXIMUM_LANE_OBSTACLE_CURVATURE = 0.6f;
static const float INTER_LANE_CHANGE_DISTANCE = 10.0f;
} // namespace LaneChange

namespace Collision
{
static const float MAX_COLLISION_RADIUS = 100.0f;
} // namespace Collision

namespace FrameMemory
{
static const unsigned int INITIAL_SIZE = 50u;
static const unsigned int GROWTH_STEP_SIZE = 50u;
} // namespace FrameMemory

} // namespace constant
} // namespace traffic_manager
} // namespace carla
