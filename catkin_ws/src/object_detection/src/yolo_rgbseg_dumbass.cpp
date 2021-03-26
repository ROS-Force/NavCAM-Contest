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

//#define DEBUG

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

        //#pragma omp parallel for ordered schedule(dynamic)
        for (size_t idx = 0; idx < clusters.size(); idx++)
        {
            auto indices = clusters.at(idx);
            int r = rand() % 256, b = rand() % 256, g = rand() % 256; // rng color
            int32_t rgb = (static_cast<uint32_t>(r) << 16 |
                        static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

            //#pragma omp parallel for
            for (size_t idx_1 = 0; idx_1 < indices.indices.size(); idx_1++){

                auto p = cloud_filtered->at(indices.indices.at(idx));
                p.rgb = rgb;
                currentClusterCloud->push_back(p);
            }
            //#pragma omp ordered
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
//        std::cerr << "PointCloud before voxel filtering: " << cloud->width * cloud->height
//                  << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;
#endif
        this->filterVoxelGrid.setInputCloud(cloud);
        this->filterVoxelGrid.filter(*cloud_filtered);

#ifdef DEBUG
//        std::cerr << "PointCloud after voxel filtering: " << cloud_filtered->width * cloud_filtered->height
//                  << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;
#endif
        this->latestPointCloud = cloud_filtered;

        publishPointCloud(this->pc_pub, cloud_filtered);
    }

    void yoloCallback(const object_detection::BoundingBoxes boxes)
    {


        PCLPointCloudRGB::Ptr coloredCloud(new PCLPointCloudRGB());

        //#pragma omp parallel for ordered schedule(dynamic)
        for (size_t idx = 0; idx < boxes.bounding_boxes.size(); idx++)
        {
            auto box = boxes.bounding_boxes.at(idx);

            PCLPointCloudRGB::Ptr clusterCloud(new PCLPointCloudRGB());

                // Build the filter
            pcl::ConditionalRemoval<pcl::PointXYZRGB> range_filt(false);
            range_filt.setInputCloud(this->latestPointCloud);
            range_filt.setKeepOrganized(false);

            pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
//            std::cerr << "(" << box.bottomright.x << "," << box.bottomright.y << ")" << "(" << box.topleft.x << "," << box.topleft.y << ")" << std::endl;

            range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::Ptr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GT, std::min(box.bottomright.x, box.topleft.x))));
            range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::Ptr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LT, std::max(box.bottomright.x, box.topleft.x))));

            range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::Ptr(new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::GT, std::min(box.bottomright.y, box.topleft.y))));
            range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::Ptr(new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::LT, std::max(box.bottomright.y, box.topleft.y))));
            
            range_filt.setCondition(range_cond);
            range_filt.filter(*clusterCloud);

            std::vector<pcl::PointIndices> clusters;

            //pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
            //pcl::RegionGrowingRGB<pcl::PointXYZRGB> segmentationRGRGB = pcl::RegionGrowingRGB<pcl::PointXYZRGB>();
            //segmentationRGRGB.setSearchMethod(tree);
            //segmentationRGRGB.setDistanceThreshold(1);
            //segmentationRGRGB.setPointColorThreshold(6);
            //segmentationRGRGB.setRegionColorThreshold(10);
            //segmentationRGRGB.setMinClusterSize(600);
            //segmentationRGRGB.setInputCloud(clusterCloud);
            //segmentationRGRGB.extract(clusters);

            this->segmentationRGRGB.setInputCloud(clusterCloud);
            this->segmentationRGRGB.extract(clusters);

//            std::cerr << "after rgbseg" << std::endl;
//            auto cloud = getColoredCloud(clusterCloud, clusters);

#ifdef DEBUG
            std::cerr << "PointCloud rgb Segmentation Cluster: " << clusterCloud->width * clusterCloud->height
                      << " data points (" << pcl::getFieldsList(*clusterCloud) << ")." << std::endl;
#endif
            //int r = rand() % 256, b = rand() % 256, g = rand() % 256; // rng color
            //int32_t rgb = (static_cast<uint32_t>(r) << 16 |
            //               static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
//
            //for (auto &p : clusterCloud->points)
            //    p.rgb = rgb;

            //#pragma omp ordered
            //{
            auto cloud = getColoredCloud(clusterCloud, clusters);
            if (cloud){
                *coloredCloud += *cloud;
            }
            //}
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
        this->segmentationRGRGB.setDistanceThreshold(1);
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
