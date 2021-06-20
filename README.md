Navigation
==========

Implementation and testing of path tracking algorithms. Uses Gazebo for simulated testing.
See individual README for information about the different parts. It is developed for ROS noetic, however most of the system should be compatible with older versions as well. The exception is the Audibot repository. We use the noetic branch in [Audibot](https://github.com/robustify/audibot/tree/noetic-devel). This could be changed to the melodic branch.

Setup
-----

**Downloading:**

```terminal
cd ~catkin_ws/src
git clone https://github.com/TorBorve/Navigation.git
cd ..
catkin build (or catkin_make)
```

**Launching:**

```terminal
source devel/setup.bash
roslaunch path_tracking demo.launch
```
