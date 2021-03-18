#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <thread>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl_conversions/pcl_conversions.h>

#include <object_detection/BoundingBox.h>
#include <object_detection/BoundingBoxes.h>

//using namespace std::chrono_literals;

#define DEBUG

typedef pcl::PointCloud<pcl::PointXYZRGB> PCLPointCloudRGB;

class RGBD_Segmentation
{
private:
    // The current nodehandle
    ros::NodeHandle n;
    // A publisher to publish messages
    ros::Publisher pc_pub;
    // A subscriber
    ros::Subscriber pc_sub;

    // A publisher to publish messages
    ros::Publisher yolorgb_pub;
    // A subscriber
    ros::Subscriber yolo_sub;

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> segmentationRGRGB;
    pcl::VoxelGrid<pcl::PointXYZRGB> filterVoxelGrid;
    pcl::search::Search<pcl::PointXYZRGB>::Ptr searchAlgorithm;

    PCLPointCloudRGB::Ptr latestPointCloud = nullptr;

    static PCLPointCloudRGB::Ptr getColoredCloud(const PCLPointCloudRGB::Ptr cloud_filtered, std::vector<pcl::PointIndices> clusters)
    {

        PCLPointCloudRGB::Ptr colored_cloud(new PCLPointCloudRGB()), // total colored cloud
            currentClusterCloud(new PCLPointCloudRGB());             // cloud for each cluster

        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud_filtered);

        for (auto indices : clusters)
        {
            int r = rand() % 256, b = rand() % 256, g = rand() % 256; // rng color
            int32_t rgb = (static_cast<uint32_t>(r) << 16 |
                           static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

            extract.setIndices(pcl::PointIndices::Ptr(&indices));
            extract.filter(*currentClusterCloud);

            for (auto &p : currentClusterCloud->points)
                p.rgb = rgb;

            *colored_cloud += *currentClusterCloud;
        }
        return colored_cloud;
    }

    static void publishPointCloud(ros::Publisher pub, PCLPointCloudRGB::Ptr input, std::string frameId = "camera_depth_optical_frame")
    {
        sensor_msgs::PointCloud2 pc_output;
        pcl::toROSMsg(*input, pc_output);
        pc_output.header.frame_id = frameId;
        pub.publish(pc_output);
    }

    void pcCallback(const sensor_msgs::PointCloud2 input)
    {

        PCLPointCloudRGB::Ptr cloud(new PCLPointCloudRGB());
        PCLPointCloudRGB::Ptr cloud_filtered(new PCLPointCloudRGB());

        pcl::fromROSMsg(input, *cloud);

#ifdef DEBUG
        std::cerr << "PointCloud before voxel filtering: " << cloud->width * cloud->height
                  << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;
#endif
        this->filterVoxelGrid.setInputCloud(cloud);
        this->filterVoxelGrid.filter(*cloud_filtered);

#ifdef DEBUG
        std::cerr << "PointCloud after voxel filtering: " << cloud_filtered->width * cloud_filtered->height
                  << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;
#endif
        this->latestPointCloud = cloud_filtered;

        publishPointCloud(this->pc_pub, cloud_filtered);
    }

    void yoloCallback(const object_detection::BoundingBoxes boxes)
    {

        // Build the filter
        pcl::ConditionalRemoval<pcl::PointXYZRGB> range_filt(false);
        range_filt.setInputCloud(this->latestPointCloud);
        range_filt.setKeepOrganized(false);

        PCLPointCloudRGB::Ptr clusterCloud(new PCLPointCloudRGB()),
            coloredCloud(new PCLPointCloudRGB());

        for (auto box : boxes.bounding_boxes)
        {
            pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());

            range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::Ptr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GT, box.xmin)));
            range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::Ptr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LT, box.xmax)));

            range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::Ptr(new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::GT, box.ymin)));
            range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::Ptr(new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::LT, box.ymax)));

            range_filt.setCondition(range_cond);
            range_filt.filter(*clusterCloud);

//            std::vector<pcl::PointIndices> clusters;
//            this->segmentationRGRGB.setInputCloud(clusterCloud);
//            this->segmentationRGRGB.extract(clusters);
//
//            auto cloud = getColoredCloud(clusterCloud, clusters);
//
#ifdef DEBUG
            std::cerr << "PointCloud rgb Segmentation Cluster: " << clusterCloud->width * clusterCloud->height
                      << " data points (" << pcl::getFieldsList(*clusterCloud) << ")." << std::endl;
#endif
            int r = rand() % 256, b = rand() % 256, g = rand() % 256; // rng color
            int32_t rgb = (static_cast<uint32_t>(r) << 16 |
                           static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

            for (auto &p : clusterCloud->points)
                p.rgb = rgb;

            *coloredCloud += *clusterCloud;
        }

        publishPointCloud(this->yolorgb_pub, coloredCloud);
    }

public:
    RGBD_Segmentation()
    {
        // Initialize ROS
        this->n = ros::NodeHandle();
        ros::NodeHandle nh("~");

        this->searchAlgorithm = pcl::search::Search<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>());
        this->segmentationRGRGB = pcl::RegionGrowingRGB<pcl::PointXYZRGB>();
        this->segmentationRGRGB.setSearchMethod(this->searchAlgorithm);
        this->segmentationRGRGB.setDistanceThreshold(0.3);
        this->segmentationRGRGB.setPointColorThreshold(6);
        this->segmentationRGRGB.setRegionColorThreshold(10);
        this->segmentationRGRGB.setMinClusterSize(600);

        this->filterVoxelGrid = pcl::VoxelGrid<pcl::PointXYZRGB>();
        this->filterVoxelGrid.setLeafSize(0.025f, 0.025f, 0.025f);
        this->filterVoxelGrid.setFilterFieldName("z");
        this->filterVoxelGrid.setFilterLimits(-8.0f, 8.0f);

        //this->filterVoxelGrid->setMinimumPointsNumberPerVoxel(5); // may not work

        // Read private params
        //nh.param<float>("linear_speed", this->lin_speed, 1.0);

        // Create a publisher object, able to push messages
        this->pc_pub = n.advertise<sensor_msgs::PointCloud2>("yolorgb/filtered", 1);

        // Create a publisher object, able to push messages
        this->yolorgb_pub = n.advertise<sensor_msgs::PointCloud2>("yolorgb/colored", 1);

        // Create a subscriber object, able to get messages
        this->pc_sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 10, &RGBD_Segmentation::pcCallback, this);

        // Create a subscriber object, able to get messages
        this->yolo_sub = n.subscribe<object_detection::BoundingBoxes>("/detection/yolo/bboxes", 10, &RGBD_Segmentation::yoloCallback, this);
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
