
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
    IterativeClosestPoint<PointXYZ, PointXYZ> icp;

    tf::TransformListener listener;
    tf::StampedTransform tf_pose;
    Eigen::Matrix4f pose_matrix;
    Eigen::Matrix4f camera_init_to_map;

    float x_range = 10;
    float y_range = 10;
    float z_range = 20;
    float leaf_size = 0.25;
    bool use_icp = 1;
    sensor_msgs::PointCloud2 ros_map;
    PointCloud<PointXYZ>::Ptr pc;
    PointCloud<PointXYZ>::Ptr map;

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
    param::get("~use_icp", use_icp);

    ROS_INFO("filter x range %f", x_range);
    ROS_INFO("filter y range %f", y_range);
    ROS_INFO("filter z range %f", z_range);
    ROS_INFO("filter voxel size %f", leaf_size);

    ros_map.header.frame_id = "map";
    pc.reset(new PointCloud<PointXYZ>);
    map.reset(new PointCloud<PointXYZ>);
    passX.setFilterFieldName("x");
    passY.setFilterFieldName("y");
    passZ.setFilterFieldName("z");
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    icp.setMaxCorrespondenceDistance(1);
    icp.setTransformationEpsilon(1e-12);
    icp.setEuclideanFitnessEpsilon(1e-3);
    icp.setMaximumIterations(100);


    pub_map = nh.advertise<sensor_msgs::PointCloud2>("icp_map", 1);
    sub_map = nh.subscribe("lidar_crop", 1, &MapFilter::pc_cb, this);
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
        listener.waitForTransform("/map", "/velodyne_tmp", ros::Time(0), five_seconds);
        listener.lookupTransform("/map", "/velodyne_tmp", ros::Time(0), tf_pose);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    pcl_ros::transformAsMatrix(tf_pose, pose_matrix);
    pcl::transformPointCloud(*pc, *pc, pose_matrix);

    if (use_icp){
        if(map->size()>0){
            icp.setInputSource(pc);
            icp.setInputTarget(map);
            icp.align(*pc);

            cout << "ICP:------------" << endl;
            cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;
        }
    }
    
    *map += *pc;

    voxel.setInputCloud(pc);
    voxel.filter(*pc);

    ROS_INFO("filtered cloud numbers %d", pc->size());

    toROSMsg(*map, ros_map);
    ros_map.header.frame_id = "map";
    ros_map.header.stamp = msg.header.stamp;
    pub_map.publish(ros_map);

    sleep(3);
}

int main(int argc, char **argv)
{
    init(argc, argv, "mapfilter");
    NodeHandle nh;

    MapFilter mapfilter(nh);

    spin();

    return 0;
}
