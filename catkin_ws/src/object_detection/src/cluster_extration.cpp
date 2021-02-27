#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <thread>
#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// The GPU specific stuff here
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.hpp>
#include <pcl/gpu/segmentation/gpu_extract_clusters.h>
#include <pcl/gpu/segmentation/impl/gpu_extract_clusters.hpp>

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

        pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::cuda::PointXYZRGB>);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::cuda::PointXYZRGB>);
        pcl::fromROSMsg(input, *cloud);

        //pcl::IndicesPtr indices(new std::vector<int>);
        //pcl::PassThrough<pcl::PointXYZRGB> pass;
        //pass.setInputCloud(cloud);
        //pass.setFilterFieldName("z");
        //pass.setFilterLimits(0.0, 1.0);
        //pass.filter(*indices);

        //INICIO DO ALGORITMO

        // Read in the cloud data
        pcl::PCDReader reader;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCDWriter writer;
        reader.read(argv[1], *cloud_filtered);

        /////////////////////////////////////////////
        /// CPU VERSION
        /////////////////////////////////////////////

        std::cout << "INFO: PointCloud_filtered still has " << cloud_filtered->size() << " Points " << std::endl;
        clock_t tStart = clock();
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.02); // 2cm
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);

        printf("CPU Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

        int j = 0;
        for (const pcl::PointIndices &cluster : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto &index : (cluster.indices))
                cloud_cluster->push_back((*cloud_filtered)[index]); //*
            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
            std::stringstream ss;
            ss << "cloud_cluster_" << j << ".pcd";
            writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
            j++;
        }

        /////////////////////////////////////////////
        /// GPU VERSION
        /////////////////////////////////////////////

        std::cout << "INFO: starting with the GPU version" << std::endl;

        tStart = clock();

        pcl::gpu::Octree::PointCloud cloud_device;
        cloud_device.upload(cloud_filtered->points);

        pcl::gpu::Octree::Ptr octree_device(new pcl::gpu::Octree);
        octree_device->setCloud(cloud_device);
        octree_device->build();

        std::vector<pcl::PointIndices> cluster_indices_gpu;
        pcl::gpu::EuclideanClusterExtraction gec;
        gec.setClusterTolerance(0.02); // 2cm
        gec.setMinClusterSize(100);
        gec.setMaxClusterSize(25000);
        gec.setSearchMethod(octree_device);
        gec.setHostCloud(cloud_filtered);
        gec.extract(cluster_indices_gpu);
        //  octree_device.clear();

        printf("GPU Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
        std::cout << "INFO: stopped with the GPU version" << std::endl;

        j = 0;
        for (const pcl::PointIndices &cluster : cluster_indices_gpu)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_gpu(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto &index : (cluster.indices))
                cloud_cluster_gpu->push_back((*cloud_filtered)[index]); //*
            cloud_cluster_gpu->width = cloud_cluster_gpu->size();
            cloud_cluster_gpu->height = 1;
            cloud_cluster_gpu->is_dense = true;

            std::cout << "PointCloud representing the Cluster: " << cloud_cluster_gpu->size() << " data points." << std::endl;
            std::stringstream ss;
            ss << "gpu_cloud_cluster_" << j << ".pcd";
            writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster_gpu, false); //*
            j++;
        }

        //FIM DO ALGORITMO

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