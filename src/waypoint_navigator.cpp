#include "waypoint_navigator/waypoint_navigator.hpp"

#include <iostream>
#include <thread>
#include <cmath>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <fstream>
using namespace std::chrono;

namespace waypoint_navigator
{
    WaypointNavigator::WaypointNavigator(ros::NodeHandle &node_handle)
        : nh_(node_handle),
          task_state_(TaskState::IDLE),
          last_task_state_(TaskState::IDLE),
          has_taken_off_(false),
          has_heading_target_(false),
          has_aligned_to_path_yaw_(false),
          has_reached_waypoint_pos_(false),
          waypoint_index_(0),
          last_ch8_(0)
    {
        // Define some example waypoints (north, east, up, yaw_deg)
        waypoints_ = {
            {5.0f, 0.0f, 1.5f, 0.0f},
            {5.0f, 5.0f, 1.5f, -90.0f},
            {0.0f, 5.0f, 1.5f, 180.0f}};
        nh_.param<std::string>("waypoint_file", waypoint_file_, "/path/to/default/waypoints.yaml");
        nh_.param<float>("takeoff_height", take_off_height_, 3.5f);
        nh_.param<bool>("use_rc", use_rc_, false);

        if (!use_rc_)
        {
            ROS_WARN("Using offboard control without RC override. Ensure safety!");
            cmd_sub_ = nh_.subscribe<std_msgs::Byte>("/user_cmd", 1, &WaypointNavigator::cmdCallback, this);
        }
        else
        {
            ROS_INFO("RC override enabled for safety.");
            rc_sub_ = nh_.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 1, &WaypointNavigator::rcCallback, this);
        }
        uav_state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 1, &WaypointNavigator::uavStateCallback, this);
        uav_pose_enu_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &WaypointNavigator::uavPoseCallback, this);

        local_pos_sp_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        local_raw_sp_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        takeoff_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
        land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        mission_timer_ = nh_.createTimer(ros::Duration(0.1), &WaypointNavigator::missionTimerCallback, this);

        if (this->loadWaypointsFromYaml(waypoint_file_))
        {
            ROS_INFO("WaypointNavigator node is ready! Loaded %zu waypoints from: %s; takeoff height: %.2f m",
                     waypoints_.size(), waypoint_file_.c_str(), take_off_height_);
        }
        else
        {
            ROS_WARN("Failed to load waypoints from %s. Using default waypoints; takeoff height: %.2f m",
                     waypoint_file_.c_str(), take_off_height_);
        }
    }

    void WaypointNavigator::cmdCallback(const std_msgs::Byte::ConstPtr &msg)
    {
        int cmd = msg->data;

        ROS_INFO("User command received: %d", cmd);

        switch (cmd)
        {
        case 1:
        {
            if (setOffboardMode())
            {
                ROS_INFO("offboard mode activated going to run takeoff");
                setTaskState(TaskState::TAKEOFF);
            }
            break;
        }
        case 2:
            setTaskState(TaskState::MISSION);
            break;
        case 3:
            setTaskState(TaskState::LAND);
            break;
        default:
            ROS_WARN("Unknown command: %d", cmd);
            break;
        }
    }

    void WaypointNavigator::rcCallback(const mavros_msgs::RCIn::ConstPtr &msg)
    {
        uint16_t ch7 = msg->channels[6]; // RCIn is 0-based index (ch7 is index 6)
        uint16_t ch8 = msg->channels[7]; // ch8 is index 7

        // LAND: Always takes priority
        if (ch8 >= 1800 && ch8 <= 2000 && last_task_state_ != TaskState::LAND)
        {
            setTaskState(TaskState::LAND);
            last_task_state_ = TaskState::LAND;
            ROS_INFO("RC Command: LAND (CH8=%d)", ch8);
            last_ch8_ = ch8;
            return;
        }

        // IDLE: Edge-triggered only when entering 1000-1200 zone
        if (ch8 >= 1000 && ch8 <= 1200 &&
            !(last_ch8_ >= 1000 && last_ch8_ <= 1200) &&
            last_task_state_ != TaskState::IDLE)
        {
            setTaskState(TaskState::IDLE);
            ROS_INFO("RC Command: IDLE (CH8 %d -> CH8 %d)", last_ch8_, ch8);
            last_task_state_ = TaskState::IDLE;
            last_ch8_ = ch8;
            return;
        }

        // Update CH8 for next call
        last_ch8_ = ch8;

        // TAKEOFF: only from IDLE
        if (last_task_state_ == TaskState::IDLE && ch7 >= 1400 && ch7 <= 1600)
        {
            if (setOffboardMode())
            {
                ROS_INFO("offboard mode activated going to run takeoff");
                setTaskState(TaskState::TAKEOFF);
                last_task_state_ = TaskState::TAKEOFF;
                ROS_INFO("RC Command: TAKEOFF (CH7=%d)", ch7);
            }
            return;
        }

        // MISSION: only from TAKEOFF
        if (last_task_state_ == TaskState::TAKEOFF && ch7 >= 1800 && ch7 <= 2000)
        {
            setTaskState(TaskState::MISSION);
            last_task_state_ = TaskState::MISSION;
            ROS_INFO("RC Command: MISSION (CH7=%d)", ch7);
            return;
        }
    }

    void WaypointNavigator::uavStateCallback(const mavros_msgs::State::ConstPtr &msg)
    {
        uav_current_state_ = *msg;
    }

    void WaypointNavigator::uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        uav_local_pose_enu_ = *msg;
    }

    void WaypointNavigator::setTaskState(TaskState new_state)
    {
        task_state_.store(new_state, std::memory_order_relaxed);
    }

    bool WaypointNavigator::loadWaypointsFromYaml(const std::string &yaml_path)
    {
        try
        {
            YAML::Node config = YAML::LoadFile(yaml_path);
            if (!config["waypoints"])
            {
                ROS_WARN("No 'waypoints' key found in YAML file: %s", yaml_path.c_str());
                return false;
            }

            std::vector<std::tuple<float, float, float, float>> loaded_waypoints;

            for (const auto &node : config["waypoints"])
            {
                if (!node.IsSequence() || node.size() != 4)
                {
                    ROS_ERROR("Invalid waypoint format in YAML. Each waypoint must be a list of 4 floats [x, y, z, yaw]. File: %s", yaml_path.c_str());
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

            ROS_INFO("Successfully loaded %zu waypoints from %s", waypoints_.size(), yaml_path.c_str());
            return true;
        }
        catch (const YAML::Exception &e)
        {
            ROS_ERROR("YAML parse error in file %s: %s", yaml_path.c_str(), e.what());
            return false;
        }
    }

    void WaypointNavigator::missionTimerCallback(const ros::TimerEvent &)
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
                    take_off_pose_enu_ = uav_local_pose_enu_;
                    take_off_pose_enu_.pose.position.z = take_off_height_;
                    take_off_yaw_enu_ = getYawFromPose(uav_local_pose_enu_);
                    if (std::isnan(take_off_pose_enu_.pose.position.x) || std::isnan(take_off_pose_enu_.pose.position.y) ||
                        std::isnan(take_off_pose_enu_.pose.position.z) || std::isnan(take_off_yaw_enu_))
                    {
                        std::cerr << "Error: One or more values of local pose feedback are NaN!" << std::endl;
                        // offboard_->stop();
                        task_state_ = TaskState::LAND;
                    }
                }
                doTakeoff();
                has_taken_off_ = true;
            }
            doTakeoff();
            break;
        case TaskState::MISSION:
            if (!has_taken_off_)
            {
                std::cerr << "Cannot run mission without taking off first.\n";
                task_state_ = TaskState::IDLE;
                break;
            }
            if (uav_current_state_.mode != "OFFBOARD")
            {
                if (!setOffboardMode())
                {
                    std::cerr << "Failed to set Offboard mode.\n";
                    task_state_ = TaskState::IDLE;
                    break;
                }
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
            waypoint_index_ = 0;    // Reset waypoint index after landing
            break;
        }
    }

    void WaypointNavigator::doTakeoff()
    {
        geometry_msgs::PoseStamped local_pose_setpoint;
        local_pose_setpoint.pose.position.x = take_off_pose_enu_.pose.position.x;
        local_pose_setpoint.pose.position.y = take_off_pose_enu_.pose.position.y;
        local_pose_setpoint.pose.position.z = take_off_height_;
        local_pose_setpoint.header.stamp = ros::Time::now();
        setYawToPose(local_pose_setpoint, deg2rad(take_off_yaw_enu_));
        local_pos_sp_pub_.publish(local_pose_setpoint);
        ROS_INFO_THROTTLE(1.0, "Vehicle taking off...");
        return;
    }

    void WaypointNavigator::publishPositionTarget(float east, float north, float up, float yaw_rad)
    {
        mavros_msgs::PositionTarget sp;
        sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        sp.type_mask =
            mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        sp.position.x = east;
        sp.position.y = north;
        sp.position.z = up;
        sp.yaw = yaw_rad;

        local_raw_sp_pub_.publish(sp);
    }

    void WaypointNavigator::doRunMission()
    {
        if (waypoints_.empty())
        {
            ROS_INFO("Waypoints not loaded, aborting mission.");
            task_state_ = TaskState::LAND;
            return;
        }

        if (waypoint_index_ >= waypoints_.size())
        {
            ROS_INFO("Mission complete. Initiating landing sequence.");
            task_state_ = TaskState::LAND;
            return;
        }

        auto [north_wp, east_wp, up_wp, yaw_wp_from_north] = waypoints_[waypoint_index_];

        float current_position_east;
        float current_position_north;
        float current_position_up;
        float current_yaw_from_north_rad;

        // following lock is actually unnecessary since we deal with single-threaded application
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            current_position_east = uav_local_pose_enu_.pose.position.x;
            current_position_north = uav_local_pose_enu_.pose.position.y;
            current_position_up = uav_local_pose_enu_.pose.position.z;
            current_yaw_from_north_rad = getYawFromPose(uav_local_pose_enu_) - M_PI / 2; // Convert ENU yaw to NED yaw

            if (std::isnan(current_position_north) || std::isnan(current_position_east) ||
                std::isnan(current_position_up) || std::isnan(current_yaw_from_north_rad))
            {
                ROS_ERROR("Error: One or more values of local pose feedback are NaN!");
                // offboard_->stop();
                task_state_ = TaskState::LAND;
            }
        }

        // Step 1: Align to path yaw
        if (!has_aligned_to_path_yaw_)
        {
            if (!has_heading_target_)
            {
                // If we are still at the first waypoint, set the current setpoint to the takeoff position
                if (waypoint_index_ == 0)
                {
                    current_setpoint_.pose.position = take_off_pose_enu_.pose.position;
                }
                else
                {
                    current_setpoint_.pose.position.x = std::get<0>(waypoints_[waypoint_index_ - 1]);
                    current_setpoint_.pose.position.y = std::get<1>(waypoints_[waypoint_index_ - 1]);
                    current_setpoint_.pose.position.z = std::get<2>(waypoints_[waypoint_index_ - 1]);
                }
                double current_north = current_setpoint_.pose.position.x; // ENU y -> NEU x
                double current_east = current_setpoint_.pose.position.y;  // ENU x -> NEU y
                double delta_north = north_wp - current_north;
                double delta_east = east_wp - current_east;
                double path_yaw_from_east = atan2(delta_north, delta_east);
                setYawToPose(current_setpoint_, path_yaw_from_east);
                ROS_INFO_THROTTLE(1.0, "Setting heading target to: %.2f degrees",
                                  rad2deg(path_yaw_from_east));
                has_heading_target_ = true;
            }

            float heading_error = normalize_angle(rad2deg(getYawFromPose(current_setpoint_) - M_PI_2 - current_yaw_from_north_rad));

            if (std::abs(heading_error) > 5.0f) // Allow some tolerance
            {
                // std::cout << "waypoint index: " << waypoint_index_ << std::endl;
                ROS_INFO_THROTTLE(1.0, "Aligning to path yaw (from north): %.2f degrees, remaining angle to adjust: %.2f degrees",
                                  rad2deg(getYawFromPose(current_setpoint_) - M_PI_2),
                                  heading_error);
                mavros_msgs::PositionTarget pose_sp;
                pose_sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                // We want only position + yaw; ignore velocity, acceleration, and yaw rate
                pose_sp.type_mask =
                    mavros_msgs::PositionTarget::IGNORE_VX |
                    mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

                // current waypoint is in NEU frame whereas pose_sp is in ENU frame
                pose_sp.position.x = current_setpoint_.pose.position.y;
                pose_sp.position.y = current_setpoint_.pose.position.x; // meters
                pose_sp.position.z = current_setpoint_.pose.position.z;

                // --- Set yaw in radians ---
                pose_sp.yaw = getYawFromPose(current_setpoint_);
                local_raw_sp_pub_.publish(pose_sp);
                return;
            }
            else
            {
                ROS_INFO_THROTTLE(1.0, "Aligned to path yaw from north: %.2f degrees",
                                  rad2deg(getYawFromPose(current_setpoint_) - M_PI_2));
                has_aligned_to_path_yaw_ = true; // Mark as aligned to path yaw
            }
        }

        // Step 2: Fly to waypoint position
        if (!has_reached_waypoint_pos_)
        {
            mavros_msgs::PositionTarget pose_sp;
            pose_sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            // We want only position + yaw; ignore velocity, acceleration, and yaw rate
            pose_sp.type_mask =
                mavros_msgs::PositionTarget::IGNORE_VX |
                mavros_msgs::PositionTarget::IGNORE_VY |
                mavros_msgs::PositionTarget::IGNORE_VZ |
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            pose_sp.position.x = east_wp; // meters
            pose_sp.position.y = north_wp;
            pose_sp.position.z = up_wp;
            pose_sp.yaw = getYawFromPose(current_setpoint_);
            local_raw_sp_pub_.publish(pose_sp);

            float dx = east_wp - current_position_east;
            float dy = north_wp - current_position_north;
            float dz = up_wp - current_position_up;
            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            ROS_INFO_THROTTLE(1.0, "Going to waypoint %u: (%.2f, %.2f, %.2f) with yaw %.1f degrees, remaining distance: %.2f meters",
                              waypoint_index_ + 1,
                              north_wp,
                              east_wp,
                              up_wp,
                              yaw_wp_from_north,
                              dist);
            if (dist < 0.2f)
            {
                has_reached_waypoint_pos_ = true; // Mark as reached waypoint position
                ROS_INFO_THROTTLE(1.0, "Reached waypoint %u position: (%.2f, %.2f, %.2f)",
                                  waypoint_index_ + 1,
                                  north_wp,
                                  east_wp,
                                  up_wp);
            }

            return;
        }

        // Step 3: Position reached, align to waypoint yaw
        float final_yaw_error = normalize_angle(yaw_wp_from_north - rad2deg(current_yaw_from_north_rad));

        if (std::abs(final_yaw_error) > 5.0f) // Allow some tolerance
        {
            ROS_INFO_THROTTLE(1.0, "Aligning to waypoint yaw: %.1f deg, remaining angle to adjust: %.1f deg",
                              yaw_wp_from_north,
                              final_yaw_error);

            mavros_msgs::PositionTarget pose_sp;
            pose_sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            // We want only position + yaw; ignore velocity, acceleration, and yaw rate
            pose_sp.type_mask =
                mavros_msgs::PositionTarget::IGNORE_VX |
                mavros_msgs::PositionTarget::IGNORE_VY |
                mavros_msgs::PositionTarget::IGNORE_VZ |
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            pose_sp.position.y = north_wp;
            pose_sp.position.x = east_wp;
            pose_sp.position.z = up_wp;
            pose_sp.yaw = deg2rad(yaw_wp_from_north) + M_PI / 2; // NED yaw to ENU yaw
            local_raw_sp_pub_.publish(pose_sp);
            return;
        }
        ROS_INFO("Reached waypoint %u position and orientation.", waypoint_index_ + 1);
        waypoint_index_++;
        has_heading_target_ = false;       // Reset heading target for next waypoint
        has_aligned_to_path_yaw_ = false;  // Reset alignment for next waypoint
        has_reached_waypoint_pos_ = false; // Reset reached position flag for next waypoint
        return;
    }

    void WaypointNavigator::doLand()
    {
        ros::Rate rate(20.0);

        mavros_msgs::CommandTOL land_cmd;
        land_cmd.request.altitude = 0;  // ignored by PX4
        land_cmd.request.latitude = 0;  // optional (NAN/0 if unused)
        land_cmd.request.longitude = 0; // optional (NAN/0 if unused)
        land_cmd.request.min_pitch = 0.0;
        land_cmd.request.yaw = 0.0;

        while (!(land_client_.call(land_cmd) && land_cmd.response.success))
        {
            ROS_WARN("Landing command failed, retrying...");
            ros::spinOnce();
            rate.sleep();
        }

        ROS_INFO("Vehicle landing...");
        task_state_ = TaskState::IDLE;
    }

    bool WaypointNavigator::setOffboardMode()
    {
        ros::Rate rate(20.0);
        ros::Time last_request = ros::Time::now();
        mavros_msgs::PositionTarget init_sp;
        init_sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // To be converted to NED in setpoint_raw plugin
        // FIXME check
        init_sp.position.x = uav_local_pose_enu_.pose.position.x;
        init_sp.position.y = uav_local_pose_enu_.pose.position.y;
        init_sp.position.z = uav_local_pose_enu_.pose.position.z;
        init_sp.velocity.x = 0;
        init_sp.velocity.y = 0;
        init_sp.velocity.z = 0;
        init_sp.acceleration_or_force.x = 0;
        init_sp.acceleration_or_force.y = 0;
        init_sp.acceleration_or_force.z = 0;
        init_sp.yaw = 0;

        // send a few setpoints before starting
        for (int i = 10; ros::ok() && i > 0; --i)
        {
            local_raw_sp_pub_.publish(init_sp);
            ros::spinOnce();
            rate.sleep();
        }

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        bool is_mode_ready = false;
        last_request = ros::Time::now();
        mavros_msgs::CommandBool arm_cmd;

        arm_cmd.request.value = true;

        while (!is_mode_ready)
        {
            if (uav_current_state_.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                ROS_INFO("Try set offboard");
                if (set_mode_client_.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            }
            else
            {
                if (!uav_current_state_.armed && (ros::Time::now() - last_request > ros::Duration(1.0)))
                {
                    ROS_INFO("Try Arming");
                    if (arming_client_.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
            local_raw_sp_pub_.publish(init_sp);
            is_mode_ready = (uav_current_state_.mode == "OFFBOARD" && uav_current_state_.armed);
            ros::spinOnce();
            rate.sleep();
        }

        if (is_mode_ready)
        {
            ROS_INFO("Offboard mode activated!");
        }

        return is_mode_ready;
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

    double WaypointNavigator::getYawFromPose(const geometry_msgs::PoseStamped &pose_msg)
    {
        tf::Quaternion q(
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w);

        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        return yaw; // in radians, range [-pi, pi]
    }

    void WaypointNavigator::setYawToPose(geometry_msgs::PoseStamped &pose_msg, double yaw)
    {
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, yaw); // roll = 0, pitch = 0, yaw = given value

        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
    }
} // namespace waypoint_navigator