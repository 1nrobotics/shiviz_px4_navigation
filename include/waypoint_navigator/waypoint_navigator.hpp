#pragma once

#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>

#include <memory>
#include <tuple>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>
#include <iostream>

class PrintOnce
{
private:
    std::string last_msg;

public:
    // Constructor
    PrintOnce() : last_msg("") {}

    // Function call operator to print the message once
    void operator()(const std::string &msg)
    {
        if (msg != last_msg)
        {
            std::cout << msg << std::endl;
            last_msg = msg;
        }
    }

    // Reset the last message
    void reset()
    {
        last_msg = "";
    }
};

namespace waypoint_navigator
{
    enum class TaskState
    {
        IDLE,
        TAKEOFF,
        MISSION,
        LAND
    };

    class WaypointNavigator
    {
    public:
        explicit WaypointNavigator(std::shared_ptr<mavsdk::System> system);
        ~WaypointNavigator();

        void setTaskState(TaskState new_state);
        void updateLocalPose(const mavsdk::Telemetry::PositionVelocityNed &position,
                             const mavsdk::Telemetry::EulerAngle &attitude);
        void updateFlightMode(const mavsdk::Telemetry::FlightMode &flight_mode);
        bool loadWaypointsFromYaml(const std::string &yaml_path);
        // ToDo: we should not need to expose following for safety reasons, but for now we do
        void doArm();
        void doDisarm();

    private:
        void runner();
        void doTakeoff();
        void doRunMission();
        void doLand();
        bool setOffboardMode();
        float normalize_angle(float angle_deg);

        std::shared_ptr<mavsdk::Action> action_;
        std::shared_ptr<mavsdk::Telemetry> telemetry_;
        std::shared_ptr<mavsdk::Offboard> offboard_;

        std::thread runnerThread_;
        std::atomic<TaskState> task_state_;

        std::mutex pose_mutex_;
        mavsdk::Telemetry::PositionVelocityNed local_position_;
        mavsdk::Telemetry::EulerAngle attitude_;

        mavsdk::Telemetry::PositionVelocityNed take_off_position_;
        mavsdk::Telemetry::EulerAngle take_off_attitude_;

        std::mutex flight_mode_mutex_;
        mavsdk::Telemetry::FlightMode current_flight_mode_;

        mavsdk::Offboard::PositionNedYaw current_setpoint_;

        std::vector<std::tuple<float, float, float, float>> waypoints_;
        std::atomic<bool> running_;

        bool has_taken_off_;            // Flag to check if takeoff has been initiated
        bool has_heading_target_;       // Flag to check if heading target has been set
        bool has_aligned_to_path_yaw_;  // Flag to check if aligned to path yaw
        bool has_reached_waypoint_pos_; // Flag to check if reached the current waypoint

        unsigned int waypoint_index_; // Current waypoint index

        PrintOnce print_once_; // Print once utility to avoid duplicate messages
    };
} // namespace waypoint_navigator