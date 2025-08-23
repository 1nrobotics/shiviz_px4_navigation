#include "waypoint_navigator/waypoint_navigator.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_offboard");
    ros::NodeHandle nh("~");
    waypoint_navigator::WaypointNavigator waypoint_navigator(nh);
    ros::spin();
    return 0;
}