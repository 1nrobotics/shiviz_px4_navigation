#pragma once

#include <memory>
#include <tuple>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>
#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/RCIn.h>
#include <std_msgs/Byte.h>

#include <tf/transform_listener.h>

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
        WaypointNavigator(ros::NodeHandle &node_handle);
        ~WaypointNavigator() {};

        void setTaskState(TaskState new_state);
        bool loadWaypointsFromYaml(const std::string &yaml_path);

        // ROS callbacks
        void cmdCallback(const std_msgs::Byte::ConstPtr &msg);
        void uavStateCallback(const mavros_msgs::State::ConstPtr &msg);
        void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void missionTimerCallback(const ros::TimerEvent &event);
        void rcCallback(const mavros_msgs::RCIn::ConstPtr &msg);

        void commandLand();

    private:
        void doTakeoff();
        void doRunMission();
        void doLand();
        bool setOffboardMode();
        // // --- Mission steps
        // bool alignToPathYaw(float north_wp, float east_wp, float up_wp, float &path_yaw);
        // bool flyToWaypoint(float north, float east, float up, float path_yaw);
        // bool alignToWaypointYaw(float north, float east, float up, float target_yaw_deg);

        // --- Utilities
        void publishPositionTarget(float east, float north, float up, float yaw_rad);
        double getYawFromPose(const geometry_msgs::PoseStamped &pose_msg);
        void setYawToPose(geometry_msgs::PoseStamped &pose_msg, double yaw);
        float normalize_angle(float angle_deg);

        inline double deg2rad(double degrees)
        {
            return degrees * M_PI / 180.0;
        }

        inline double rad2deg(double radians)
        {
            return radians * 180.0 / M_PI;
        }

        std::atomic<TaskState> task_state_;
        std::atomic<TaskState> last_task_state_;
        std::mutex pose_mutex_;
        std::mutex flight_mode_mutex_;
        std::vector<std::tuple<float, float, float, float>> waypoints_;
        std::atomic<bool> running_;
        std::string waypoint_file_;
        mavros_msgs::State uav_current_state_;

        bool has_taken_off_;            // Flag to check if takeoff has been initiated
        bool has_heading_target_;       // Flag to check if heading target has been set
        bool has_aligned_to_path_yaw_;  // Flag to check if aligned to path yaw
        bool has_reached_waypoint_pos_; // Flag to check if reached the current waypoint

        unsigned int waypoint_index_; // Current waypoint index

        PrintOnce print_once_; // Print once utility to avoid duplicate messages

        ros::NodeHandle nh_;
        ros::Subscriber cmd_sub_;
        ros::Subscriber uav_state_sub_;
        ros::Subscriber uav_pose_enu_sub_;
        ros::Subscriber rc_sub_;

        ros::Publisher local_pos_sp_pub_;
        ros::Publisher local_raw_sp_pub_;

        ros::ServiceClient arming_client_;
        ros::ServiceClient takeoff_client_;
        ros::ServiceClient land_client_;
        ros::ServiceClient set_mode_client_;

        ros::Timer mission_timer_;

        geometry_msgs::PoseStamped uav_local_pose_enu_;
        geometry_msgs::PoseStamped take_off_pose_enu_;
        geometry_msgs::PoseStamped current_setpoint_;
        double take_off_yaw_enu_;
        float take_off_height_;
        bool use_rc_;
        uint16_t last_ch8_;
    };
} // namespace waypoint_navigator