// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "TrafficManagerLocal.h"

namespace carla {
namespace traffic_manager {

TrafficManagerLocal::TrafficManagerLocal(
    std::vector<float> longitudinal_PID_parameters,
    std::vector<float> longitudinal_highway_PID_parameters,
    std::vector<float> lateral_PID_parameters,
    std::vector<float> lateral_highway_PID_parameters,
    float perc_difference_from_limit,
    cc::detail::EpisodeProxy& episodeProxy,
    uint16_t& RPCportTM)
  : episodeProxyTM(episodeProxy),
    world(cc::World(episodeProxyTM)),
    longitudinal_PID_parameters(longitudinal_PID_parameters),
    longitudinal_highway_PID_parameters(longitudinal_highway_PID_parameters),
    lateral_PID_parameters(lateral_PID_parameters),
    lateral_highway_PID_parameters(lateral_highway_PID_parameters),
    debug_helper(carla::client::DebugHelper{episodeProxyTM}),
    server(TrafficManagerServer(RPCportTM, static_cast<carla::traffic_manager::TrafficManagerBase *>(this))) {

  parameters.SetGlobalPercentageSpeedDifference(perc_difference_from_limit);

  registered_vehicles_state = -1;
  buffer_map = std::make_shared<BufferMap>();

  SetupLocalMap();

  Start();
}

TrafficManagerLocal::~TrafficManagerLocal() {
  episodeProxyTM.Lock()->DestroyTrafficManager(server.port());
  Release();
}

void TrafficManagerLocal::SetupLocalMap() {
  const carla::SharedPtr<cc::Map> world_map = world.GetMap();
  local_map = std::make_shared<InMemoryMap>(world_map);
  local_map->SetUp();
}

void TrafficManagerLocal::Start() {
  run_traffic_manger.store(true);
  worker_thread = std::make_unique<std::thread>(&TrafficManagerLocal::Run, this);
}

void TrafficManagerLocal::Run() {
  bool sync_mode = false;
  while (run_traffic_manger.load()) {
    sync_mode = parameters.GetSynchronousMode();
    // Wait for external trigger to initiate cycle in synchronous mode.
    if (sync_mode) {
      std::unique_lock<std::mutex> lock(step_execution_mutex);
      while (!step_begin.load()) {
        step_begin_trigger.wait_for(lock, 1ms, [this]() {return step_begin.load();});
      }
      step_begin.store(false);
    }

    AgentLifecycleAndStateManagement(registered_vehicles, vehicle_list, unregistered_actors,
                                     registered_vehicles_state, vehicle_id_to_index, buffer_map,
                                     track_traffic, idle_time, hero_vehicle, last_lane_change_location,
                                     kinematic_state_map, static_attribute_map, previous_update_instance,
                                     parameters, world, local_map);

    // Wait for external trigger to complete cycle in synchronous mode.
    if (sync_mode) {
      std::unique_lock<std::mutex> lock(step_execution_mutex);
      while (!step_end.load()) {
        step_end_trigger.wait_for(lock, 1ms, [this]() {return step_end.load();});
      }
      // Set flag to false, unblock RunStep() call and release mutex lock.
      step_end.store(false);
      step_complete_trigger.notify_one();
    }
  }
}

void TrafficManagerLocal::Stop() {
  run_traffic_manger.store(false);
  if(worker_thread) {
    if(worker_thread->joinable()){
      worker_thread->join();
    }
    worker_thread.release();
  }
}

void TrafficManagerLocal::Reset() {

  Stop();
  episodeProxyTM = episodeProxyTM.Lock()->GetCurrentEpisode();
  SetupLocalMap();
  Start();
}

bool TrafficManagerLocal::SynchronousTick() {
  if (parameters.GetSynchronousMode()) {
    std::unique_lock<std::mutex> lock(step_execution_mutex);

    step_begin.store(true);
    step_begin_trigger.notify_one();

    step_end.store(true);
    step_end_trigger.notify_one();

    while (step_end.load()) {
      step_complete_trigger.wait_for(lock, 1ms, [this]() {return !step_end.load();});
    }
  }

  return true;
}

void TrafficManagerLocal::RegisterVehicles(const std::vector<ActorPtr>& vehicle_list) {
  registered_vehicles.Insert(vehicle_list);
}

void TrafficManagerLocal::UnregisterVehicles(const std::vector<ActorId>& actor_id_list) {
  registered_vehicles.Remove(actor_id_list);
}

void TrafficManagerLocal::SetPercentageSpeedDifference(const ActorPtr &actor, const float percentage) {
  parameters.SetPercentageSpeedDifference(actor, percentage);
}

void TrafficManagerLocal::SetGlobalPercentageSpeedDifference(const float percentage) {
  parameters.SetGlobalPercentageSpeedDifference(percentage);
}

void TrafficManagerLocal::SetCollisionDetection(const ActorPtr &reference_actor, const ActorPtr &other_actor, const bool detect_collision) {
  parameters.SetCollisionDetection(reference_actor, other_actor, detect_collision);
}

void TrafficManagerLocal::SetForceLaneChange(const ActorPtr &actor, const bool direction) {
  parameters.SetForceLaneChange(actor, direction);
}

void TrafficManagerLocal::SetAutoLaneChange(const ActorPtr &actor, const bool enable) {
  parameters.SetAutoLaneChange(actor, enable);
}

void TrafficManagerLocal::SetDistanceToLeadingVehicle(const ActorPtr &actor, const float distance) {
  parameters.SetDistanceToLeadingVehicle(actor, distance);
}

void TrafficManagerLocal::SetGlobalDistanceToLeadingVehicle(const float distance) {
  parameters.SetGlobalDistanceToLeadingVehicle(distance);
}

void TrafficManagerLocal::SetPercentageIgnoreWalkers(const ActorPtr &actor, const float perc) {
  parameters.SetPercentageIgnoreWalkers(actor, perc);
}

void TrafficManagerLocal::SetPercentageIgnoreVehicles(const ActorPtr &actor, const float perc) {
  parameters.SetPercentageIgnoreVehicles(actor, perc);
}

void TrafficManagerLocal::SetPercentageRunningLight(const ActorPtr &actor, const float perc) {
  parameters.SetPercentageRunningLight(actor, perc);
}

void TrafficManagerLocal::SetPercentageRunningSign(const ActorPtr &actor, const float perc) {
  parameters.SetPercentageRunningSign(actor, perc);
}

void TrafficManagerLocal::SetKeepRightPercentage(const ActorPtr &actor, const float percentage) {
  parameters.SetKeepRightPercentage(actor, percentage);
}

void TrafficManagerLocal::SetHybridPhysicsMode(const bool mode_switch) {
  parameters.SetHybridPhysicsMode(mode_switch);
}

bool TrafficManagerLocal::CheckAllFrozen(TLGroup tl_to_freeze) {
  for (auto& elem : tl_to_freeze) {
    if (!elem->IsFrozen() || elem->GetState() != TLS::Red) {
      return false;
    }
  }
  return true;
}

void TrafficManagerLocal::ResetAllTrafficLights() {

  auto Filter = [&](auto &actors, auto &wildcard_pattern) {
    std::vector<carla::client::detail::ActorVariant> filtered;
    for (auto &&actor : actors) {
      if (carla::StringUtil::Match(carla::client::detail::ActorVariant(actor).GetTypeId(), wildcard_pattern)) {
        filtered.push_back(actor);
      }
    }
    return filtered;
  };

  // Get all actors of the world.
  auto world_actorsList = episodeProxyTM.Lock()->GetAllTheActorsInTheEpisode();

  // Filter based on wildcard pattern.
  const auto world_traffic_lights = Filter(world_actorsList, "*traffic_light*");

  std::vector<TLGroup> list_of_all_groups;
  TLGroup tl_to_freeze;
  std::vector<carla::ActorId> list_of_ids;
  for (auto tl : world_traffic_lights) {
    if (!(std::find(list_of_ids.begin(), list_of_ids.end(), tl.GetId()) != list_of_ids.end())) {
      const TLGroup tl_group = boost::static_pointer_cast<cc::TrafficLight>(tl.Get(episodeProxyTM))->GetGroupTrafficLights();
      list_of_all_groups.push_back(tl_group);
      for (uint64_t i=0u; i<tl_group.size(); i++) {
        list_of_ids.push_back(tl_group.at(i).get()->GetId());
        if(i!=0u) {
          tl_to_freeze.push_back(tl_group.at(i));
        }
      }
    }
  }

  for (TLGroup& tl_group : list_of_all_groups) {
    tl_group.front()->SetState(TLS::Green);
    std::for_each(
        tl_group.begin()+1, tl_group.end(),
        [] (auto& tl) {tl->SetState(TLS::Red);});
  }

  while (!CheckAllFrozen(tl_to_freeze)) {
    for (auto& tln : tl_to_freeze) {
      tln->SetState(TLS::Red);
      tln->Freeze(true);
    }
  }

}

void TrafficManagerLocal::SetSynchronousMode(bool mode) {
  parameters.SetSynchronousMode(mode);
}

void TrafficManagerLocal::SetSynchronousModeTimeOutInMiliSecond(double time) {
  parameters.SetSynchronousModeTimeOutInMiliSecond(time);
}

carla::client::detail::EpisodeProxy& TrafficManagerLocal::GetEpisodeProxy() {
  return episodeProxyTM;
}

std::vector<ActorId> TrafficManagerLocal::GetRegisteredVehiclesIDs() {
  return registered_vehicles.GetIDList();
}

} // namespace traffic_manager
} // namespace carla
