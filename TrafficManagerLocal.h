// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

// TODO: remove unused imports.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <random>
#include <unordered_set>
#include <vector>

#include "carla/StringUtil.h"
#include "carla/geom/Transform.h"
#include "carla/Logging.h"
#include "carla/Memory.h"

#include "carla/client/Actor.h"
#include "carla/client/ActorList.h"
#include "carla/client/BlueprintLibrary.h"
#include "carla/client/Client.h"
#include "carla/client/DebugHelper.h"
#include "carla/client/detail/EpisodeProxy.h"
#include "carla/client/detail/Simulator.h"
#include "carla/client/Map.h"
#include "carla/client/TrafficLight.h"
#include "carla/client/World.h"

#include "ALSM.h"
#include "AtomicActorSet.h"
#include "AtomicMap.h"
#include "InMemoryMap.h"
#include "Parameters.h"
#include "LocalizationUtils.h"

// TODO: Import these files from local folder.
#include "carla/trafficmanager/TrafficManagerBase.h"
#include "carla/trafficmanager/TrafficManagerServer.h"

namespace carla {
namespace traffic_manager {

  using TimePoint = chr::time_point<chr::system_clock, chr::nanoseconds>;
  using TLS = carla::rpc::TrafficLightState;
  using TLGroup = std::vector<carla::SharedPtr<carla::client::TrafficLight>>;

  /// The function of this class is to integrate all the various stages of
  /// the traffic manager appropriately using messengers.
  class TrafficManagerLocal : public TrafficManagerBase {

  private:

    /// Carla's client connection object.
    carla::client::detail::EpisodeProxy episodeProxyTM;
    /// Carla client and object.
    const cc::World world;
    /// PID controller parameters.
    std::vector<float> longitudinal_PID_parameters;
    std::vector<float> longitudinal_highway_PID_parameters;
    std::vector<float> lateral_PID_parameters;
    std::vector<float> lateral_highway_PID_parameters;
    /// Set of all actors registered with traffic manager.
    AtomicActorSet registered_vehicles;
    /// State counter to track changes in registered actors.
    int registered_vehicles_state;
    /// Map connecting actor ids to indices of data arrays.
    std::unordered_map<ActorId, unsigned long> vehicle_id_to_index;
    /// List of vehicles registered with the traffic manager in
    /// current update cycle.
    std::vector<ActorPtr> vehicle_list;
    /// A structure used to keep track of actors spawned outside of traffic
    /// manager.
    std::unordered_map<ActorId, ActorPtr> unregistered_actors;
    /// Pointer to local map cache.
    LocalMapPtr local_map;
    /// Structures to hold waypoint buffers for all vehicles.
    std::shared_ptr<BufferMap> buffer_map;
    /// Object for tracking paths of the traffic vehicles.
    TrackTraffic track_traffic;
    /// Map of all vehicles' idle time.
    std::unordered_map<ActorId, double> idle_time;
    /// Simulated seconds since the beginning of the current episode when the last actor was destroyed.
    double elapsed_last_actor_destruction = 0.0;
    /// Map to keep track of last lane change location.
    std::unordered_map<ActorId, cg::Location> last_lane_change_location;
    /// Reference of hero vehicle.
    ActorPtr hero_vehicle {nullptr};
    /// Switch indicating hybrid physics mode.
    bool hybrid_physics_mode {false};
    /// Structure to hold kinematic state for all vehicles.
    KinematicStateMap kinematic_state_map;
    /// Structure to hold static attributes of vehicles.
    StaticAttributeMap static_attribute_map;
    /// Time instance used to calculate dt in asynchronous mode.
    TimePoint previous_update_instance;
    /// Carla's debug helper object.
    carla::client::DebugHelper debug_helper;
    /// Parameterization object.
    Parameters parameters;
    /// Traffic manager server instance.
    TrafficManagerServer server;
    /// Switch to turn on / turn off traffic manager.
    std::atomic<bool> run_traffic_manger {true};
    /// Flags to signal step begin and end.
    std::atomic<bool> step_begin {false};
    std::atomic<bool> step_end {false};
    /// Mutex for progressing synchronous execution.
    std::mutex step_execution_mutex;
    /// Condition variables for progressing synchronous execution.
    std::condition_variable step_begin_trigger;
    std::condition_variable step_end_trigger;
    std::condition_variable step_complete_trigger;
    /// Single worker thread for sequential execution of sub-components.
    std::unique_ptr<std::thread> worker_thread;

    /// Method to check if all traffic lights are frozen in a group.
    bool CheckAllFrozen(TLGroup tl_to_freeze);

  public:

    /// Private constructor for singleton lifecycle management.
    TrafficManagerLocal(std::vector<float> longitudinal_PID_parameters,
                        std::vector<float> longitudinal_highway_PID_parameters,
                        std::vector<float> lateral_PID_parameters,
                        std::vector<float> lateral_highway_PID_parameters,
                        float perc_decrease_from_limit,
                        cc::detail::EpisodeProxy& episodeProxy,
                        uint16_t &RPCportTM);

    /// Destructor.
    virtual ~TrafficManagerLocal();

    /// Method to setup InMemoryMap.
    void SetupLocalMap();

    /// To start the TrafficManager.
    void Start();

    /// Initiates thread to run the TrafficManager sequentially.
    void Run();

    /// To stop the TrafficManager.
    void Stop();

    /// To reset the traffic manager.
    void Reset();

    /// This method registers a vehicle with the traffic manager.
    void RegisterVehicles(const std::vector<ActorPtr> &actor_list);

    /// This method unregisters a vehicle from traffic manager.
    void UnregisterVehicles(const std::vector<ActorId>& actor_id_list);

    /// Method to set a vehicle's % decrease in velocity with respect to the speed limit.
    /// If less than 0, it's a % increase.
    void SetPercentageSpeedDifference(const ActorPtr &actor, const float percentage);

    /// Methos to set a global % decrease in velocity with respect to the speed limit.
    /// If less than 0, it's a % increase.
    void SetGlobalPercentageSpeedDifference(float const percentage);

    /// Method to set collision detection rules between vehicles.
    void SetCollisionDetection(const ActorPtr &reference_actor, const ActorPtr &other_actor, const bool detect_collision);

    /// Method to force lane change on a vehicle.
    /// Direction flag can be set to true for left and false for right.
    void SetForceLaneChange(const ActorPtr &actor, const bool direction);

    /// Enable/disable automatic lane change on a vehicle.
    void SetAutoLaneChange(const ActorPtr &actor, const bool enable);

    /// Method to specify how much distance a vehicle should maintain to
    /// the leading vehicle.
    void SetDistanceToLeadingVehicle(const ActorPtr &actor, const float distance);

    /// Method to specify the % chance of ignoring collisions with any walker.
    void SetPercentageIgnoreWalkers(const ActorPtr &actor, const float perc);

    /// Method to specify the % chance of ignoring collisions with any vehicle.
    void SetPercentageIgnoreVehicles(const ActorPtr &actor, const float perc);

    /// Method to specify the % chance of running any traffic light.
    void SetPercentageRunningLight(const ActorPtr &actor, const float perc);

    /// Method to specify the % chance of running any traffic sign.
    void SetPercentageRunningSign(const ActorPtr &actor, const float perc);

    /// Method to switch traffic manager into synchronous execution.
    void SetSynchronousMode(bool mode);

    /// Method to set Tick timeout for synchronous execution.
    void SetSynchronousModeTimeOutInMiliSecond(double time);

    /// Method to provide synchronous tick.
    bool SynchronousTick();

    /// Method to reset all traffic light groups to the initial stage.
    void ResetAllTrafficLights();

    /// Get CARLA episode information.
    carla::client::detail::EpisodeProxy& GetEpisodeProxy();

    /// Get list of all registered vehicles.
    std::vector<ActorId> GetRegisteredVehiclesIDs();

    /// Method to specify how much distance a vehicle should maintain to
    /// the Global leading vehicle.
    void SetGlobalDistanceToLeadingVehicle(const float distance);

    /// Method to set probabilistic preference to keep on the right lane.
    void SetKeepRightPercentage(const ActorPtr &actor, const float percentage);

    /// Method to set hybrid physics mode.
    void SetHybridPhysicsMode(const bool mode_switch);
  };

} // namespace traffic_manager
} // namespace carla
