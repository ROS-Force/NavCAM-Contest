#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <thread>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl_conversions/pcl_conversions.h>

//using namespace std::chrono_literals;

class RGBD_Segmentation
{
private:
    // The current nodehandle
    ros::NodeHandle n;
    // A publisher to publish messages
    ros::Publisher pc_pub;
    // A subscriber
    ros::Subscriber pc_sub;

    void pcCallback(const sensor_msgs::PointCloud2 input)
    {

        pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(input, *cloud);

        //pcl::IndicesPtr indices(new std::vector<int>);
        //pcl::PassThrough<pcl::PointXYZRGB> pass;
        //pass.setInputCloud(cloud);
        //pass.setFilterFieldName("z");
        //pass.setFilterLimits(0.0, 1.0);
        //pass.filter(*indices);

        pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
        reg.setInputCloud(cloud);
        //reg.setIndices(indices);
        reg.setSearchMethod(tree);
        reg.setDistanceThreshold(10);
        reg.setPointColorThreshold(6);
        reg.setRegionColorThreshold(5);
        reg.setMinClusterSize(600);

        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

        sensor_msgs::PointCloud2 pc_output;
        pcl::toROSMsg(*colored_cloud, pc_output);
        pc_output.header.frame_id = "camera_link";

        this->pc_pub.publish(pc_output);
    }

public:
    RGBD_Segmentation()
    {
        // Initialize ROS
        this->n = ros::NodeHandle();
        ros::NodeHandle nh("~");

        // Read private params
        //nh.param<float>("linear_speed", this->lin_speed, 1.0);
        //nh.param<float>("angular_speed", this->ang_speed, 1.0);
        //nh.param<std::string>("spawn_turtle_name", this->turtle_name, "");

        // Create a publisher object, able to push messages
        this->pc_pub = n.advertise<sensor_msgs::PointCloud2>("cmd_vel", 1);
        // Create a subscriber object, able to push messages
        this->pc_sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 10, &RGBD_Segmentation::pcCallback, this);
    }
};

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "talker");

    // Create our controller object and run it
    auto controller = RGBD_Segmentation();
    ros::spin();

    // And make good on our promise
    return 0;
}
