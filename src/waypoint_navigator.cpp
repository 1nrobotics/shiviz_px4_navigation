#include "waypoint_navigator.hpp"

#include <iostream>
#include <thread>
#include <cmath>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <fstream>
using namespace mavsdk;
using namespace std::chrono;

namespace waypoint_navigator
{
    WaypointNavigator::WaypointNavigator(std::shared_ptr<System> system)
        : action_(std::make_shared<Action>(*system)),
          telemetry_(std::make_shared<Telemetry>(*system)),
          offboard_(std::make_shared<Offboard>(*system)),
          task_state_(TaskState::IDLE),
          running_(true),
          has_taken_off_(false),
          has_heading_target_(false),
          has_aligned_to_path_yaw_(false),
          has_reached_waypoint_pos_(false),
          waypoint_index_(0)
    {
        // Define some example waypoints (north, east, down, yaw_deg)
        waypoints_ = {
            {5.0f, 0.0f, -1.5f, 0.0f},
            {5.0f, 5.0f, -1.5f, 90.0f},
            {0.0f, 5.0f, -1.5f, 180.0f}};

        // Start the runner thread
        runnerThread_ = std::thread(&WaypointNavigator::runner, this);
    }

    WaypointNavigator::~WaypointNavigator()
    {
        running_ = false;
        if (runnerThread_.joinable())
        {
            runnerThread_.join();
            std::cout << "WaypointNavigator instance destroyed\n";
        }
    }

    void WaypointNavigator::setTaskState(TaskState new_state)
    {
        task_state_.store(new_state, std::memory_order_relaxed);
    }

    void WaypointNavigator::updateLocalPose(const Telemetry::PositionVelocityNed &position,
                                            const Telemetry::EulerAngle &attitude)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        local_position_ = position;
        attitude_ = attitude;
    }

    void WaypointNavigator::updateFlightMode(const Telemetry::FlightMode &flight_mode)
    {
        std::lock_guard<std::mutex> lock(flight_mode_mutex_);
        current_flight_mode_ = flight_mode;
    }

    bool WaypointNavigator::loadWaypointsFromYaml(const std::string &yaml_path)
    {
        try
        {
            YAML::Node config = YAML::LoadFile(yaml_path);
            if (!config["waypoints"])
            {
                std::cerr << "No 'waypoints' key found in YAML file: " << yaml_path << std::endl;
                return false;
            }

            std::vector<std::tuple<float, float, float, float>> loaded_waypoints;

            for (const auto &node : config["waypoints"])
            {
                if (!node.IsSequence() || node.size() != 4)
                {
                    std::cerr << "Invalid waypoint format. Each waypoint must be a list of 4 floats [x, y, z, yaw]." << std::endl;
                    return false;
                }

                float x = node[0].as<float>();
                float y = node[1].as<float>();
                float z = node[2].as<float>();
                float yaw = node[3].as<float>();

                loaded_waypoints.emplace_back(x, y, z, yaw);
            }

            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                waypoints_ = std::move(loaded_waypoints);
            }

            std::cout << "Successfully loaded " << waypoints_.size() << " waypoints from " << yaml_path << std::endl;
            return true;
        }
        catch (const YAML::Exception &e)
        {
            std::cerr << "YAML parse error: " << e.what() << std::endl;
            return false;
        }
    }

    void WaypointNavigator::runner()
    {
        while (running_)
        {
            TaskState current_state = task_state_.load(std::memory_order_relaxed);
            switch (current_state)
            {
            case TaskState::IDLE:
                // idle, do nothing
                break;
            case TaskState::TAKEOFF:
                if (!has_taken_off_)
                {
                    {
                        std::lock_guard<std::mutex> lock(pose_mutex_);
                        take_off_position_ = local_position_;
                        take_off_attitude_ = attitude_;
                        if (std::isnan(take_off_position_.position.north_m) || std::isnan(take_off_position_.position.east_m) ||
                            std::isnan(take_off_position_.position.down_m) || std::isnan(take_off_attitude_.yaw_deg))
                        {
                            std::cerr << "Error: One or more values of local pose feedback are NaN!" << std::endl;
                            offboard_->stop();
                            task_state_ = TaskState::LAND;
                        }
                    }
                    doTakeoff();
                    has_taken_off_ = true;
                }
                break;
            case TaskState::MISSION:
                if (!has_taken_off_)
                {
                    std::cerr << "Cannot run mission without taking off first.\n";
                    task_state_ = TaskState::IDLE;
                    break;
                }
                if (!setOffboardMode())
                {
                    std::cerr << "Failed to set Offboard mode.\n";
                    task_state_ = TaskState::IDLE;
                    break;
                }
                doRunMission();
                break;
            case TaskState::LAND:
                if (!has_taken_off_)
                {
                    std::cerr << "Cannot land without taking off first.\n";
                    task_state_ = TaskState::IDLE;
                    break;
                }
                doLand();
                has_taken_off_ = false; // Reset after landing
                break;
            }
            std::this_thread::sleep_for(100ms);
        }
    }

    void WaypointNavigator::doArm()
    {
        std::cout << "Arming...\n";
        auto result = action_->arm();
        if (result != Action::Result::Success)
        {
            std::cerr << "Arming failed: " << result << "\n";
        }
        else
        {
            std::cout << "Armed successfully.\n";
        }
    }

    void WaypointNavigator::doDisarm()
    {
        std::cout << "Disarming...\n";
        auto result = action_->disarm();
        if (result != Action::Result::Success)
        {
            std::cerr << "Disarming failed: " << result << "\n";
        }
        else
        {
            std::cout << "Disarmed successfully.\n";
        }
    }

    void WaypointNavigator::doTakeoff()
    {
        std::cout << "Taking off...\n";
        auto result = action_->takeoff();
        if (result != Action::Result::Success)
        {
            std::cerr << "Takeoff failed: " << result << "\n";
            task_state_ = TaskState::IDLE;
            return;
        }
        std::this_thread::sleep_for(10s);
        std::cout << "Takeoff complete.\n";
        task_state_ = TaskState::IDLE;
    }

    void WaypointNavigator::doRunMission()
    {
        if (waypoints_.empty())
        {
            std::cout << "Waypoints not loaded, aborting mission.\n";
            offboard_->stop();
            task_state_ = TaskState::LAND;
            return;
        }

        if (waypoint_index_ >= waypoints_.size())
        {
            std::cout << "Mission complete. Initiating landing sequence.\n";
            offboard_->stop();
            task_state_ = TaskState::LAND;
            return;
        }

        auto [north_wp, east_wp, down_wp, yaw_wp] = waypoints_[waypoint_index_];

        float current_position_north;
        float current_position_east;
        float current_position_down;
        float current_yaw;

        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            current_position_north = local_position_.position.north_m;
            current_position_east = local_position_.position.east_m;
            current_position_down = local_position_.position.down_m;
            current_yaw = attitude_.yaw_deg;

            if (std::isnan(current_position_north) || std::isnan(current_position_east) ||
                std::isnan(current_position_down) || std::isnan(current_yaw))
            {
                std::cerr << "Error: One or more values of local pose feedback are NaN!" << std::endl;
                offboard_->stop();
                task_state_ = TaskState::LAND;
            }
        }

        // Step 1: Align to path yaw
        if (!has_aligned_to_path_yaw_)
        {
            if (!has_heading_target_)
            {
                if (waypoint_index_ == 0)
                {
                    current_setpoint_.north_m = take_off_position_.position.north_m;
                    current_setpoint_.east_m = take_off_position_.position.east_m;
                }
                else
                {
                    current_setpoint_.north_m = std::get<0>(waypoints_[waypoint_index_ - 1]);
                    current_setpoint_.east_m = std::get<1>(waypoints_[waypoint_index_ - 1]);
                }

                current_setpoint_.yaw_deg = atan2(east_wp - current_setpoint_.east_m, north_wp - current_setpoint_.north_m) * 180.0f / M_PI; // Convert to degrees
                print_once_("Setting heading target to: " + std::to_string(current_setpoint_.yaw_deg) + " degrees");

                has_heading_target_ = true;
            }

            float heading_error = normalize_angle(current_setpoint_.yaw_deg - current_yaw);

            if (std::abs(heading_error) > 5.0f) // Allow some tolerance
            {
                print_once_("Aligning to path yaw: " + std::to_string(current_setpoint_.yaw_deg) + ", remaining angle to adjust: " + std::to_string(heading_error) + " degrees");
                Offboard::PositionNedYaw setpoint{};
                setpoint.north_m = current_setpoint_.north_m;
                setpoint.east_m = current_setpoint_.east_m;
                setpoint.down_m = down_wp;
                setpoint.yaw_deg = current_setpoint_.yaw_deg;
                offboard_->set_position_ned(setpoint);
                return;
            }
            else
            {
                print_once_("Aligned to path yaw: " + std::to_string(current_setpoint_.yaw_deg) + " degrees");
                has_aligned_to_path_yaw_ = true; // Mark as aligned to path yaw
            }
        }

        // Step 2: Fly to waypoint position
        if (!has_reached_waypoint_pos_)
        {
            print_once_("Going to waypoint " + std::to_string(waypoint_index_ + 1) +
                        ": (" + std::to_string(north_wp) + ", " + std::to_string(east_wp) +
                        ", " + std::to_string(down_wp) + ") with yaw " + std::to_string(yaw_wp));
            Offboard::PositionNedYaw setpoint{};
            setpoint.north_m = north_wp;
            setpoint.east_m = east_wp;
            setpoint.down_m = down_wp;
            setpoint.yaw_deg = current_setpoint_.yaw_deg;
            offboard_->set_position_ned(setpoint);

            float dx = north_wp - current_position_north;
            float dy = east_wp - current_position_east;
            float dz = down_wp - current_position_down;
            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (dist < 0.2f)
            {
                has_reached_waypoint_pos_ = true; // Mark as reached waypoint position
                print_once_("Reached waypoint number: " + std::to_string(waypoint_index_ + 1) + "'s position" +
                            ": (" + std::to_string(north_wp) + ", " + std::to_string(east_wp) +
                            ", " + std::to_string(down_wp) + ")");
            }

            return;
        }
        // Step 3: Position reached, align to waypoint yaw
        float final_yaw_error = normalize_angle(yaw_wp - current_yaw);

        if (std::abs(final_yaw_error) > 5.0f) // Allow some tolerance
        {
            print_once_("Aligning to waypoint yaw: " + std::to_string(yaw_wp) +
                        ", remaining angle to adjust: " + std::to_string(final_yaw_error) + " degrees");
            Offboard::PositionNedYaw align_setpoint{};
            align_setpoint.north_m = north_wp;
            align_setpoint.east_m = east_wp;
            align_setpoint.down_m = down_wp;
            align_setpoint.yaw_deg = yaw_wp;
            offboard_->set_position_ned(align_setpoint);
            return;
        }
        std::cout << "Reached waypoint " << (waypoint_index_ + 1) << "\n";
        waypoint_index_++;
        has_heading_target_ = false;       // Reset heading target for next waypoint
        has_aligned_to_path_yaw_ = false;  // Reset alignment for next waypoint
        has_reached_waypoint_pos_ = false; // Reset reached position flag for next waypoint
        return;
    }

    void WaypointNavigator::doLand()
    {
        std::cout << "Landing...\n";
        auto result = action_->land();
        if (result != Action::Result::Success)
        {
            std::cerr << "Landing failed: " << result << "\n";
        }
        else
        {
            std::cout << "Landing started.\n";
        }
        task_state_ = TaskState::IDLE;
    }

    bool WaypointNavigator::setOffboardMode()
    {
        if (!offboard_->is_active())
        {
            Offboard::PositionNedYaw initial_setpoint{};
            initial_setpoint.down_m = -2.0f;
            initial_setpoint.north_m = take_off_position_.position.north_m;
            initial_setpoint.east_m = take_off_position_.position.east_m;
            initial_setpoint.yaw_deg = take_off_attitude_.yaw_deg;
            offboard_->set_position_ned(initial_setpoint);

            auto result = offboard_->start();
            if (result != Offboard::Result::Success)
            {
                std::cerr << "Failed to set offboard mode: " << result << "\n";
                return false;
            }
            std::cout << "Offboard mode set successfully.\n";
        }
        return true;
    }

    float WaypointNavigator::normalize_angle(float angle_deg)
    {
        // Normalize the angle to the range [-180, 180]
        while (angle_deg > 180)
        {
            angle_deg -= 360; // Subtract 360 degrees if the angle is greater than 180
        }
        while (angle_deg <= -180)
        {
            angle_deg += 360; // Add 360 degrees if the angle is less than or equal to -180
        }
        return angle_deg;
    }
} // namespace waypoint_navigator