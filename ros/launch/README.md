This directory summarizes simple launch scripts for the autonomous navigation. Note that before you use these launch scripts, you need to launch the ROS driver for Parrot BEBOP 2 [bebop_autonomy](https://github.com/AutonomyLab/bebop_autonomy) as

```
$ roslaunch bebop_driver bebop_node.launch
```



# Mapping

To start mapping, you can launch **mapping.launch**. This script executes ORB SLAM2 as the SLAM mode and loads **bebop_slam.yaml** located at ros/src/orbslam-map-saving-extension/orb_slam2_ros/settings/. You can change parameters such as name of the map by editing the yaml file. The map will be automatically saved after killing the mapping process.



# Localization

Once you built the map, you can use **localization.launch**. This script also executes ORB SLAM2 as the localization mode, that does not update the map. In addition, this script has a **use_pose_fuser** argument. If this is true, the pose fuser that fuses the ORB SLAM2 and Odometry estimates using Kalman filter is used as the localization module. To change localization parameters, please revise **bebop_localization.yaml** located at ros/src/orbslam-map-saving-extension/orb_slam2_ros/settings/.



# Path recording and saving

You can create a path for the autonomous navigation with **path_recording.launch**. This script executes path recorder that stores the estimated pose by ORB SLAM2 and creates the path. Then, you can use **path_saving.launch** to save the path. The path will be saved as a text file, that contains x, y, z, qx, qy, qz, qw. You can change the name of the text file by editing path_saving.launch. Note that you do not need to launch localization.launch if path_recording.launch is launched since it includes localization.launch.



# Path following

After saving the path, you can test the autonomous navigation with **path_following.launch**. This script executes modules used for the autonomous navigation and the joy stick controller. Note that the velocity command is managed by the velocity selector node and the priority of the joy stick controller is positive in the initial state if the joy stick is available (this is for safety). You need to decrease the priority to start the autonomous navigation. In addition, you need to change ROS parameter named **/path_following/stop** as

```
$ rosparam set /path_follower/stop false
```

