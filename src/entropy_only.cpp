#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <chrono>
#include <ctime>

#include "../include/entropy.h"
#include "../include/binsegmentation.h"
#include "../include/pointpose.h"

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

    auto startE = std::chrono::steady_clock::now();
    // ENTROPY FILTER -----------------------------------------------------------------------
    //
    EntropyFilter ef;
    ef.setInputCloud(source);
    ef.setDownsampleLeafSize(0.005);     // size of the leaf for downsampling the cloud, value in meters. Default = 5 mm
    ef.setEntropyThreshold(0.7);         // Segmentation performed for all points with normalized entropy value above this
    ef.setKLocalSearch(500);             // Nearest Neighbour Local Search
    ef.setCurvatureThreshold(0.01);      // Curvature Threshold for the computation of Entropy
    ef.setDepthThreshold(0.03);          // if the segment region has a value of depth lower than this -> not graspable (value in meters)
    ef.setAngleThresholdForConvexity(5); // convexity check performed if the angle btw two normal vectors is larger than this

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZ>);
    bool entropy_result = ef.compute(cloud_result);
    if (entropy_result == false)
        return -1;

    //time computation
    auto endE = std::chrono::steady_clock::now();
    auto diff2 = endE - startE;
    std::cout << "duration entropy filter: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;

    ef.visualizeAll(false);

    return 0;
}
