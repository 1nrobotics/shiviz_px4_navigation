# shiviz_px4_navigation

## Clone the Repository

```bash
git clone git@github.com:mamariomiamo/shiviz_px4_navigation.git
cd shiviz_px4_navigation
git submodule update --init --recursive
```

## Build Instructions

```bash
mkdir build
cd build
cmake ..
make
```

## Running the UAV Control

After launching the PX4 Gazebo simulation (see below), run:

```bash
./uav_control ../config/waypoints.yaml
```

## PX4 Gazebo Simulation Setup

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --branch v1.14.4 --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
exec $SHELL
make px4_sitl gazebo-classic_iris
```

## MAVROS Setup

In a new terminal:

```bash
roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557
```

## Example ROS Commands

Set a local position setpoint:

```bash
rostopic pub -r 20 /mavros/setpoint_position/local geometry_msgs/PoseStamped \
  "{ header: {stamp: now, frame_id: 'map'}, pose: {position: {x: 0.0, y: 2.0, z: 1.0}, orientation: {w: 1.0}}}"
```

Echo the setpoint topic:

```bash
rostopic echo /mavros/setpoint_position/local
```

Set OFFBOARD mode:

```bash
rosservice call /mavros/set_mode "custom_mode: 'OFFBOARD'"
```

Arm the vehicle:

```bash
rosservice call /mavros/cmd/arming "value: true"
```
