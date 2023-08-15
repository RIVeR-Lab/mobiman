/*
This script is used to create a cuboid pointcloud that
will be in mm and exported and saved in JSON format.
*/

// Import libraries
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <nlohmann/json.hpp>


pcl::PointCloud<pcl::PointXYZ>::Ptr cuboid_cloud(new pcl::PointCloud<pcl::PointXYZ>);
double resolution = 4;


int main(char **argv, int argc)
{
    // Create a pointcloud of specified dimensions and resolution
    for (double x = -76; x <= 76; x += resolution) {
        for (double y = -38; y <= 38; y += resolution) {
            for (double z = -76; z <= 0; z += resolution) {
                cuboid_cloud->push_back(pcl::PointXYZ(x, y, z));
            }
        }
    }
    //visualize the cloud
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cuboid_cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer  ->initCameraParameters();
    // Add axes to the viewer. The parameter is the scale of the axes.
    viewer->addCoordinateSystem(40.0); 
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    // Export to JSON format
    //Serializing JSON
    nlohmann::json j;
    for (const auto& point : cuboid_cloud->points) {
        j.push_back({ {"x", point.x}, {"y", point.y}, {"z", point.z} });
    }

    //Write JSON pointcloud to file directory that this scripts is running in
    ofstream file("yellow_box_pointcloud.json");
    if(file.is_open()){
        file << j;
        file.close();
    }

    //All code beyond this point was to verify that the json file could be read from successfully

    //Read the JSON file
    // ifstream inputFile("cuboid_pointcloud.json");
    // if (!inputFile.is_open()) {
    //     cout << "Could not open file" << endl;
    //     return 0;
    // }

    // nlohmann::json j2;
    // inputFile >> j2;
    // inputFile.close();

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // // Parse the JSON data and fill the point cloud
    // for (const auto& pointJson : j2) {
    //     pcl::PointXYZ point;
    //     point.x = pointJson["x"].get<float>();
    //     point.y = pointJson["y"].get<float>();
    //     point.z = pointJson["z"].get<float>();
    //     cloud->push_back(point);
    // }

    // // Visualize the point cloud
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // viewer->setBackgroundColor(0, 0, 0);
    // viewer->addPointCloud<pcl::PointXYZ>(cloud, "loaded cloud");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "loaded cloud");
    // viewer->initCameraParameters();
    // viewer->addCoordinateSystem(40.0); 
    // while (!viewer->wasStopped()) {
    //     viewer->spinOnce(100);
    // }

    // Return 0
    return 0;
}