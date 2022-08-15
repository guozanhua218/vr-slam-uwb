# How to run hector slam

```
roslaunch mapping hector_slam.launch
```

## Topic needed

```
1. TF_Odom : /odom                  (Type : tf)
    
2. lidar pc : /velodyne_points      (Type : sensor_msgs/PointCloud2)  
```

## Odom info

```
rosrun tf tf_echo /slam_map /scanmatcher_frame
```
