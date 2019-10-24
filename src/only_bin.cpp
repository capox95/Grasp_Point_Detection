#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <chrono>
#include <ctime>

#include "../include/binsegmentation.h"

//----------------------------------------------------------------------------- //
int main(int argc, char **argv)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *source) == -1)
    {
        PCL_ERROR(" error opening file ");
        return (-1);
    }
    std::cout << "cloud orginal size: " << source->size() << std::endl;

    for (int i = 0; i < source->size(); i++)
    {
        source->points[i].r = 0;
        source->points[i].g = 0;
        source->points[i].b = 0;
    }

    //time computation
    auto start = std::chrono::steady_clock::now();
    //BIN SEGMENTATION -----------------------------------------------------------------------

    BinSegmentation bin;
    bin.setInputCloud(source);
    bin.setNumberLines(4);
    bin.setScaleFactorHullBorders(0.1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_grasp(new pcl::PointCloud<pcl::PointXYZRGB>);
    bin.compute(cloud_grasp);

    pcl::ModelCoefficients::Ptr plane = bin.getPlaneGroundPoints();

    //time computation
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    std::cout << "duration segmentation: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;
    std::cout << std::endl;

    bin.visualize(false, true, true);

    return 0;
}
