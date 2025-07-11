// main.cpp
#include "waypoint_navigator/waypoint_navigator.hpp"
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
// 延时函数
void delay(int seconds)
{
    std::cout << "Waiting for " << seconds << " seconds..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <waypoints.yaml>\n";
        return 1;
    }

    const std::string yaml_path = argv[1];

    // Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
    mavsdk::Mavsdk mavsdk{mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::GroundStation}};

    // auto result = mavsdk.add_any_connection("udpin://0.0.0.0:14540");
    auto result = mavsdk.add_any_connection("udp://0.0.0.0:14540");

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


    //Starting autonomous mission sequence
    std::cout << "\n=== Starting autonomous mission sequence ===\n";

    // 1. 解锁
    std::cout << "Step 1: Arming...\n";
    navigator.doArm();
    delay(3);  // 等待解锁完成

    // 2. 起飞
    std::cout << "Step 2: Taking off...\n";
    navigator.setTaskState(waypoint_navigator::TaskState::TAKEOFF);
    delay(10);  // 等待起飞到目标高度

    // 3. 执行任务
    std::cout << "Step 3: Starting mission...\n";        
    while (true) {
        navigator.setTaskState(waypoint_navigator::TaskState::MISSION);
        auto current_state = navigator.getTaskState();
        
        if (current_state == waypoint_navigator::TaskState::LAND || 
            current_state == waypoint_navigator::TaskState::IDLE) {
            std::cout << "Mission completed. Exiting mission loop.\n";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

    }

    // //4.降落
    // std::cout << "Step 4: Landing...\n";
    // navigator.setTaskState(waypoint_navigator::TaskState::LAND);
    // 等待降落完成
    // while (navigator.getTaskState() != waypoint_navigator::TaskState::IDLE) {
    //     std::cout <<111111<<std::endl;
    //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //     break;
    //     std::cout <<2222222<<std::endl;
    // }
    // std::cout <<3333333<<std::endl;
    return 0;
}
