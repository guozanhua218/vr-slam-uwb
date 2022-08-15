#include <stdio.h>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

using namespace ros;
using namespace std;
using namespace pcl;
using namespace message_filters;

sensor_msgs::PointCloud2 lidar_filter_points;
Publisher pub_lidar_crop;
double x_max,x_min,y_max,y_min,z_max,z_min;
double x_ur5_max,x_ur5_min,y_ur5_max,y_ur5_min,z_ur5_max,z_ur5_min;
Publisher pub_lidar_filter_points;





void callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
  PointCloud<PointXYZ>::Ptr pc (new PointCloud<PointXYZ>);
  PointCloud<PointXYZ>::Ptr lidar_filter (new PointCloud<PointXYZ>);
  fromROSMsg (*pc_msg, *pc);

  for (size_t i = 0; i < pc->points.size(); i++){
    if ((pc->points[i].x>x_min) && (pc->points[i].x<x_max) && (pc->points[i].y>y_min) && (pc->points[i].y<y_max) && (pc->points[i].z>z_min) && (pc->points[i].z<z_max));
    // else if((pc->points[i].x>x_ur5_min) && (pc->points[i].x<x_ur5_max) && (pc->points[i].y>y_ur5_min) && (pc->points[i].y<y_ur5_max) && (pc->points[i].z>z_ur5_min) && (pc->points[i].z<z_ur5_max));
    else lidar_filter->points.push_back(pc->points[i]);
  }

  // tf::TransformListener listener;
  // tf::StampedTransform tf_pose;
  // Eigen::Matrix4f pose_matrix;

  // try
  //   {
  //       ros::Duration _seconds(0.2);
  //       listener.waitForTransform("velodyne1", "velodyne_tmp", ros::Time(0), _seconds);
  //       listener.lookupTransform("velodyne1", "velodyne_tmp", ros::Time(0), tf_pose);
  //   }
  //   catch (tf::TransformException ex)
  //   {
  //       ROS_ERROR("%s", ex.what());
  //       return;
  //   }
  // pcl_ros::transformAsMatrix(tf_pose, pose_matrix);
  // pcl::transformPointCloud(*lidar_filter, *lidar_filter, pose_matrix);

  toROSMsg(*lidar_filter, lidar_filter_points);
  // lidar_filter_points.header = pc_msg->header;
  lidar_filter_points.header.frame_id = "velodyne_tmp";
  pub_lidar_filter_points.publish(lidar_filter_points);
}

int main(int argc, char **argv)
{
    init(argc, argv, "lidar_crop");
    NodeHandle nh("");
    param::get("~x_max", x_max);
    param::get("~y_max", y_max);
    param::get("~z_max", z_max);
    param::get("~x_min", x_min);
    param::get("~y_min", y_min);
    param::get("~z_min", z_min);

    // param::get("~x_ur5_max", x_ur5_max);
    // param::get("~y_ur5_max", y_ur5_max);
    // param::get("~z_ur5_max", z_ur5_max);
    // param::get("~x_ur5_min", x_ur5_min);
    // param::get("~y_ur5_min", y_ur5_min);
    // param::get("~z_ur5_min", z_ur5_min);

    ros::Subscriber sub = nh.subscribe("/robot/velodyne1/velodyne_points", 10, callback);
    pub_lidar_filter_points = nh.advertise<sensor_msgs::PointCloud2>("/robot/lidar_crop", 10);

    ROS_INFO("lidar points crop");
    spin();
    return 0;
}
