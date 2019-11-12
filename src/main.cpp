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

    //time computation
    auto start = std::chrono::steady_clock::now();
    //BIN SEGMENTATION -----------------------------------------------------------------------

    BinSegmentation bin;
    bin.setInputCloud(source);
    bin.setNumberLines(4);
    bin.setScaleFactorHullBorders(0.08);
    bin.setMaxBinHeight(0.3);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_grasp(new pcl::PointCloud<pcl::PointXYZRGB>);
    bool bin_result = bin.compute(cloud_grasp);
    if (bin_result == false)
        return -1;

    pcl::ModelCoefficients::Ptr plane = bin.getPlaneGroundPoints();
    pcl::PointCloud<pcl::PointXYZ>::Ptr top_vertices = bin.getVerticesBinContour();

    //time computation
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    std::cout << "duration segmentation: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;
    std::cout << std::endl;

    auto startE = std::chrono::steady_clock::now();
    // ENTROPY FILTER -----------------------------------------------------------------------
    //
    EntropyFilter ef;
    ef.setInputCloud(cloud_grasp);
    ef.setVerticesBinContour(top_vertices);
    ef.setDownsampleLeafSize(0.005);
    ef.setEntropyThreshold(0.65);
    ef.setKLocalSearch(500);        // Nearest Neighbour Local Search
    ef.setCurvatureThreshold(0.01); //Curvature Threshold for the computation of Entropy
    ef.setDepthThreshold(0.03);
    ef.setAngleThresholdForConvexity(5);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZ>);
    bool entropy_result = ef.compute(cloud_result);
    if (entropy_result == false)
        return -1;

    // GRASP POINT --------------------------------------------------------------------------
    PointPose pp;
    pp.setSourceCloud(source);
    pp.setRefPlane(plane);
    pp.setInputCloud(cloud_result);
    Eigen::Affine3d transformation;
    pp.computeGraspPoint(transformation);

    //time computation
    auto endE = std::chrono::steady_clock::now();
    auto diff2 = endE - startE;
    std::cout << "duration entropy filter: " << std::chrono::duration<double, std::milli>(diff2).count() << " ms" << std::endl;

    auto diffAll = endE - start;
    std::cout << "overall processing took: " << std::chrono::duration<double, std::milli>(diffAll).count() << " ms" << std::endl;

    bin.visualize(false, true, false);
    pp.visualizeGrasp();
    ef.visualizeAll(false);

    return 0;
}
