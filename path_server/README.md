Path server
===========

Path server makes it possible to save, record and load path. To choose between the different commands you can specify it when launching:

```terminal
roslaunch path_server path_server.launch command:=record
```

Or by editing the default parameters in the path_server.launch file and the launching:

```terminal
roslaunch path_server path_server.launch
```

Overview
---------

### Commands

**load**: Use the load command when you want to publish an existing path that is saved. The path to this file should be the *filePath* parameter. The path is published to a topic named after the *pathTopic* parameter.

**save**: Use the save command when you want to save an published path to file. The path is save at the *filePath* parameter. And the existing path should be published to the *pathtopic* topic.

**record**: Use the record command when you want to recorde where the vehicle is driving and saving it to file. This requires that the vehicle publishes odometry to the topic *odomTopic*. The *resolution* parameter specifies how close the points on the path should be, measured in meters. *resolution*=1.0 would imply that a point is added the the path when the vehicle is atleast 1.0 m from the previus point on the path.