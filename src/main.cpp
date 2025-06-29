// main.cpp
#include "waypoint_navigator.hpp"
#include <mavsdk/mavsdk.h>
#include <iostream>
#include <thread>

using namespace mavsdk;

void wait_until_discover(Mavsdk &mavsdk)
{
    while (mavsdk.systems().empty())
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "Waiting for drone...\n";
    }
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <waypoints.yaml>\n";
        return 1;
    }

    const std::string yaml_path = argv[1];

    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
    auto result = mavsdk.add_any_connection("udpin://0.0.0.0:14540");
    if (result != ConnectionResult::Success)
    {
        std::cerr << "Connection failed: " << result << "\n";
        return 1;
    }

    wait_until_discover(mavsdk);
    auto system = mavsdk.systems().at(0);

    waypoint_navigator::WaypointNavigator navigator(system);

    if (!navigator.loadWaypointsFromYaml(yaml_path))
    {
        std::cerr << "Failed to load waypoints from YAML." << std::endl;
        return 1;
    }

    // Instantiate plugins.
    auto telemetry = Telemetry{system};
    telemetry.set_rate_position_velocity_ned(1.0); // 1 Hz update rate
    telemetry.set_rate_attitude_euler(1.0);        // also throttle euler attitude updates
    telemetry.subscribe_position_velocity_ned([&telemetry, &navigator](Telemetry::PositionVelocityNed local_position)
                                              {
        auto telem_euler = telemetry.attitude_euler();
        navigator.updateLocalPose(local_position, telem_euler);
        std::cout << "Position: NED (" << local_position.position.north_m << ", "
                  << local_position.position.east_m << ", "
                  << local_position.position.down_m << ")\n";
        std::cout << "Yaw: " << telem_euler.yaw_deg << " deg\n"; });
    telemetry.subscribe_flight_mode([&navigator](Telemetry::FlightMode flight_mode)
                                    {
        navigator.updateFlightMode(flight_mode);
        std::cout << "Flight mode: " << flight_mode << "\n"; });
    while (true)
    {
        std::cout << "\nA: Arm\nD: Disarm\nT: Takeoff\nM: Mission\nL: Land\nQ: Quit\nEnter command: ";
        char cmd;
        std::cin >> cmd;
        cmd = std::toupper(cmd);

        if (cmd == 'A')
        {
            navigator.doArm();
        }
        else if (cmd == 'D')
        {
            navigator.doDisarm();
        }
        else if (cmd == 'T')
        {
            navigator.setTaskState(waypoint_navigator::TaskState::TAKEOFF);
        }
        else if (cmd == 'M')
        {
            navigator.setTaskState(waypoint_navigator::TaskState::MISSION);
        }
        else if (cmd == 'L')
        {
            navigator.setTaskState(waypoint_navigator::TaskState::LAND);
        }
        else if (cmd == 'Q')
        {
            break;
        }
    }

    return 0;
}
