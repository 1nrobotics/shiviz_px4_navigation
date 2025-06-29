# Setup
```
git clone git@github.com:mamariomiamo/shiviz_px4_navigation.git && git submodule update --init --recursive
```
# Building
```
mkdir build && cd build && cmake .. && make
```
# After launching PX4 gazebo sim
```
./uav_control ../config/waypoints.yaml
```
