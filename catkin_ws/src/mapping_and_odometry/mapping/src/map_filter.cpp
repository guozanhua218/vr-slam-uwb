
#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>

using namespace ros;
using namespace std;
using namespace pcl;

class MapFilter
{
private:
    PassThrough<PointXYZ> passX;
    PassThrough<PointXYZ> passY;
    PassThrough<PointXYZ> passZ;
    VoxelGrid<PointXYZ> voxel;
    tf::TransformListener listener;
    tf::StampedTransform tf_pose;
    Eigen::Matrix4f pose_matrix;
    Eigen::Matrix4f camera_init_to_map;

    float x_range = 10;
    float y_range = 10;
    float z_range = 20;
    float px = 0, py = 0, pz = 0;
    float leaf_size = 0.25;
    sensor_msgs::PointCloud2 ros_map;
    PointCloud<PointXYZ>::Ptr pc;

    Publisher pub_map;
    Subscriber sub_map;

public:
    void pc_cb(const sensor_msgs::PointCloud2 msg);
    MapFilter(NodeHandle &nh);
    ~MapFilter();
};

MapFilter::MapFilter(NodeHandle &nh)
{
    param::get("~x_range", x_range);
    param::get("~y_range", y_range);
    param::get("~z_range", z_range);
    param::get("~voxel_size", leaf_size);

    ROS_INFO("filter x range %f", x_range);
    ROS_INFO("filter y range %f", y_range);
    ROS_INFO("filter z range %f", z_range);
    ROS_INFO("filter voxel size %f", leaf_size);

    ros_map.header.frame_id = "map";
    pc.reset(new PointCloud<PointXYZ>);
    passX.setFilterFieldName("x");
    passY.setFilterFieldName("y");
    passZ.setFilterFieldName("z");
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    try
    {
        ros::Duration five_seconds(5.0);
        listener.waitForTransform("/map", "/camera_init", ros::Time(0), five_seconds);
        listener.lookupTransform("/map", "/camera_init", ros::Time(0), tf_pose);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    pcl_ros::transformAsMatrix(tf_pose, camera_init_to_map);

    pub_map = nh.advertise<sensor_msgs::PointCloud2>("/map_part", 1);
    sub_map = nh.subscribe("laser_cloud_surround", 1, &MapFilter::pc_cb, this);
    ROS_INFO("map filter initialized");
}

MapFilter::~MapFilter()
{
}

void MapFilter::pc_cb(const sensor_msgs::PointCloud2 msg)
{
    fromROSMsg(msg, *pc);
    try
    {
        ros::Duration five_seconds(5.0);
        listener.waitForTransform("/map", "/base_link", ros::Time(0), five_seconds);
        listener.lookupTransform("/map", "/base_link", ros::Time(0), tf_pose);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    pcl_ros::transformAsMatrix(tf_pose, pose_matrix);

    pcl::transformPointCloud(*pc, *pc, camera_init_to_map);

    px = pose_matrix(0, 3);
    passX.setFilterLimits(px - x_range, px + x_range);
    passX.setInputCloud(pc);
    passX.filter(*pc);

    py = pose_matrix(1, 3);
    passY.setFilterLimits(py - y_range, py + y_range);
    passY.setInputCloud(pc);
    passY.filter(*pc);

    pz = pose_matrix(2, 3);
    passZ.setFilterLimits(pz - z_range, pz + z_range);
    passZ.setInputCloud(pc);
    passZ.filter(*pc);

    voxel.setInputCloud(pc);
    voxel.filter(*pc);

    ROS_INFO("filtered cloud numbers %d", pc->size());

    toROSMsg(*pc, ros_map);
    ros_map.header.frame_id = "map";
    ros_map.header.stamp = msg.header.stamp;
    pub_map.publish(ros_map);
}

int main(int argc, char **argv)
{
    init(argc, argv, "mapfilter");
    NodeHandle nh;

    MapFilter mapfilter(nh);

    spin();

    return 0;
}
