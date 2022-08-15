
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

class MapBuilder
{
private:
    VoxelGrid<PointXYZ> voxel;
    IterativeClosestPoint<PointXYZ, PointXYZ> icp;

    tf::TransformListener listener;
    tf::StampedTransform tf_pose;

    float leaf_size = 0.25;
    bool use_icp = false;
    sensor_msgs::PointCloud2 ros_map;
    PointCloud<PointXYZ>::Ptr map;
    PointCloud<PointXYZ>::Ptr pc;
    Eigen::Matrix4f tf_icp;

    Publisher pub_map;
    Subscriber sub_map;

public:
    void pc_cb(const sensor_msgs::PointCloud2 msg);
    MapBuilder(NodeHandle &nh);
    ~MapBuilder();
};

MapBuilder::MapBuilder(NodeHandle &nh)
{
    param::get("~voxel_size", leaf_size);
    param::get("~use_icp", use_icp);

    ROS_INFO("filter voxel size %f", leaf_size);
    ROS_INFO("use_icp %s", use_icp ? "true" : "false");

    ros_map.header.frame_id = "map";
    pc.reset(new PointCloud<PointXYZ>);
    map.reset(new PointCloud<PointXYZ>);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    icp.setMaxCorrespondenceDistance(1);
    icp.setTransformationEpsilon(1e-12);
    icp.setEuclideanFitnessEpsilon(1e-3);
    icp.setMaximumIterations(100);

    pub_map = nh.advertise<sensor_msgs::PointCloud2>("loam_map", 1);
    sub_map = nh.subscribe("robot_points", 1, &MapBuilder::pc_cb, this);
    ROS_INFO("map builder initialized");
}

MapBuilder::~MapBuilder()
{
}

void MapBuilder::pc_cb(const sensor_msgs::PointCloud2 msg)
{
    fromROSMsg(msg, *pc);

    if (use_icp && map->size() != 0)
    {
        icp.setInputSource(pc);
        icp.setInputTarget(map);
        icp.align(*pc);
        tf_icp = icp.getFinalTransformation();

        // transformPointCloud(*pc, *pc, tf_icp);
        cout << "ICP:------------" << endl;
        cout << tf_icp << endl;
        cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;
    }

    *map += *pc;
    pc->points.clear();
    voxel.setInputCloud(map);
    voxel.filter(*map);

    ROS_INFO("map size %d", map->size());

    toROSMsg(*map, ros_map);
    ros_map.header.frame_id = "map";
    ros_map.header.stamp = msg.header.stamp;
    pub_map.publish(ros_map);
}

int main(int argc, char **argv)
{
    init(argc, argv, "map_builder");
    NodeHandle nh;

    MapBuilder mapbuilder(nh);

    spin();

    return 0;
}
