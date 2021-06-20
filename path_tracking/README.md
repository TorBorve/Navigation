Path tracking
=============

Implementation of path tracking algorithms.

To launch path_tracking, path_server and Gazebo in one launch file you can use demo.launch:

```terminal
roslaunch path_tracking demo.launch
```

Overview
--------

**PathTracker class**: A abstract class that the different path tracking algotithms inherits from. Contains publishers for throttle, steering and brake. Also a path message wich is recived by the path subscriber. Incluedes a odometry subscriber.

Currently the Stanley path tracking algorithm is implemened in the Stanley class. It is launched as follows:

```terminal
roslaunch path_tracking stanley.launch
```