#include <algorithm>
#include <chrono>
#include <random>
#include <string>
#include <thread>

#include "boost/pointer_cast.hpp"
#include "carla/client/Actor.h"
#include "carla/client/BlueprintLibrary.h"
#include "carla/client/Client.h"
#include "carla/client/Map.h"
#include "carla/client/Vehicle.h"
#include "carla/client/World.h"
#include "carla/geom/Transform.h"
#include "carla/Memory.h"
#include "carla/rpc/ActorId.h"
#include "carla/rpc/Command.h"

//////// Threadable TM Build Test files //////////
#include "ALSM.h"
//////////////////////////////////////////////////

namespace cc = carla::client;
namespace cg = carla::geom;

using namespace std::literals::chrono_literals;

using ActorId = carla::rpc::ActorId;
using Actor = carla::SharedPtr<cc::Actor>;
using Vehicle = carla::SharedPtr<cc::Vehicle>;

std::string HOST = "localhost";
uint PORT = 2000;
uint NUMBER_OF_VEHICLES = 300;

int main() {

    auto client = cc::Client(HOST, PORT);
    cc::World world = client.GetWorld();
    using Map = carla::SharedPtr<cc::Map>;
    Map world_map = world.GetMap();

    carla::traffic_manager::TrafficManager tm = client.GetInstanceTM();
    tm.SetHybridPhysicsMode(true);

    carla::SharedPtr<cc::BlueprintLibrary> bp_lib = world.GetBlueprintLibrary()->Filter("vehicle.*");
    std::vector<cc::ActorBlueprint> filtered_bp_lib;
    for (auto& bp: *bp_lib.get()) {
        std::string type_id = bp.GetId();
        if (type_id != "vehicle.tesla.cybertruck"
            && type_id != "vehicle.bmw.isseta"
            && type_id != "vehicle.volkswagen.t2"
            && type_id != "vehicle.carla.carlacola"
            && bp.GetAttribute("number_of_wheels") == 4)
        {
            filtered_bp_lib.push_back(bp);
        }
    }

    std::vector<cg::Transform> spawn_points = world_map->GetRecommendedSpawnPoints();
    auto rng = std::default_random_engine{};
    std::shuffle(std::begin(spawn_points), std::end(spawn_points), rng);

    std::vector<Actor> actor_list;
    std::vector<carla::rpc::Command> command_array;
    for (uint i=0u; i< NUMBER_OF_VEHICLES && i< spawn_points.size(); ++i)
    {
        Actor actor = world.SpawnActor(filtered_bp_lib.at(i%filtered_bp_lib.size()), spawn_points.at(i));
        actor_list.push_back(actor);
    }

    for (Actor& actor: actor_list)
    {
        Vehicle vehicle = boost::static_pointer_cast<cc::Vehicle>(actor);
        vehicle->SetAutopilot(true);
    }

    uint i =0u;
    while (true) {
        std::cout << "Running TM " << ++i << std::endl;
        sleep(1);
    }

    return 0;
}
