
/// File containing functionality to retrieve kinematic state details for vehicles.

#include "carla/rpc/ActorId.h"

#include "DataStructures.h"

namespace carla
{
namespace traffic_manager
{

using ActorId = carla::rpc::ActorId;
using KinematicStateMap = std::unordered_map<ActorId, KinematicState>;

cg::Location GetLocation(const KinematicStateMap& state_map, const ActorId actor_id)
{
    cg::Location location;
    if (state_map.find(actor_id) != state_map.end())
    {
        location = state_map.at(actor_id).location;
    }

    return location;
}

cg::Rotation GetRotation(const KinematicStateMap& state_map, const ActorId actor_id)
{
    cg::Rotation rotation;
    if (state_map.find(actor_id) != state_map.end())
    {
        rotation = state_map.at(actor_id).rotation;
    }

    return rotation;
}

cg::Vector3D GetHeading(const KinematicStateMap& state_map, const ActorId actor_id)
{
    cg::Vector3D heading;
    if (state_map.find(actor_id) != state_map.end())
    {
        heading = state_map.at(actor_id).rotation.GetForwardVector();
    }

    return heading;
}

cg::Vector3D GetVelocity(const KinematicStateMap& state_map, const ActorId actor_id)
{
    cg::Vector3D velocity;
    if (state_map.find(actor_id) != state_map.end())
    {
        velocity = state_map.at(actor_id).velocity;
    }

    return velocity;
}

bool IsPhysicsEnabled(const KinematicStateMap& state_map, const ActorId actor_id)
{
    bool physics_enabled = true;
    if (state_map.find(actor_id) != state_map.end())
    {
        physics_enabled = state_map.at(actor_id).physics_enabled;
    }

    return physics_enabled;
}

} // namespace traffic_manager
} // namespace carla
