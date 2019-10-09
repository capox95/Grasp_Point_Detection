#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>

#include "../include/pointpose.h"

void PointPose::setSourceCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in) { m_source = cloud_in; }

void PointPose::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) { m_cloud_grasp = cloud; }

void PointPose::setRefPlane(pcl::ModelCoefficients::Ptr &plane) { m_plane = plane; }

Eigen::Vector3f PointPose::getTranslation() { return m_trans; }

Eigen::Quaternionf PointPose::getRotation() { return m_rot; }

void PointPose::computeGraspPoint()
{

    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(m_cloud_grasp);
    proj.setModelCoefficients(m_plane);
    proj.filter(*m_cloud_projected);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*m_cloud_projected, centroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*m_cloud_projected, centroid, covariance);

    Eigen::Vector3f eigenValues;
    Eigen::Matrix3f eigenVectors;
    pcl::eigen33(covariance, eigenVectors, eigenValues);

    std::vector<int> idx = orderEigenvalues(eigenValues);
    Eigen::Matrix3f rotation;
    rotation.col(0) = eigenVectors.col(idx[0]);
    rotation.col(1) = eigenVectors.col(idx[1]);
    rotation.col(2) = eigenVectors.col(idx[2]);

    Eigen::Quaternion<float> quat(rotation);
    quat.normalize();

    m_trans = moveCentroid(centroid); //move centroid from projected cloud to original cloud_grasp
    m_rot = quat;

    //compute point for visualizing coordinate axes
    getCoordinateFrame(m_trans, rotation);

    PCL_INFO("Position: %f, %f, %f\n", m_trans.x(), m_trans.y(), m_trans.z());
    PCL_INFO("Orientation: %f, %f, %f, %f\n", quat.x(), quat.y(), quat.z(), quat.w());
}
void PointPose::visualizeGrasp()
{
    pcl::visualization::PCLVisualizer viz("PCL Cloud Result");
    viz.setBackgroundColor(0.0, 0.0, 0.5);
    viz.addPointCloud<pcl::PointXYZRGB>(m_source, "source");
    viz.addPointCloud<pcl::PointXYZ>(m_cloud_grasp, "cloud_grasp");
    viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_grasp");
    viz.addPointCloud<pcl::PointXYZ>(m_cloud_projected, "cloud_projected");
    viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "cloud_projected");
    viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_projected");

    viz.addSphere(m_pointsCoordinateFrame[0], 0.005, "sphere");
    viz.addArrow(m_pointsCoordinateFrame[1], m_pointsCoordinateFrame[0], 1.0f, 0.0f, 0.0f, false, "x_axis");
    viz.addArrow(m_pointsCoordinateFrame[2], m_pointsCoordinateFrame[0], 0.0f, 1.0f, 0.0f, false, "y_axis");
    viz.addArrow(m_pointsCoordinateFrame[3], m_pointsCoordinateFrame[0], 0.0f, 0.0f, 1.0f, false, "z_axis");

    viz.addSphere(m_origin, 0.005, "origin");

    //viz.addCube(m_trans, m_rot, 0.01, 0.01, 0.01, "cube");
}

std::vector<int> PointPose::orderEigenvalues(Eigen::Vector3f eigenValuesPCA)
{
    std::vector<double> v;
    v.push_back(eigenValuesPCA[0]);
    v.push_back(eigenValuesPCA[1]);
    v.push_back(eigenValuesPCA[2]);

    int maxElementIndex = std::max_element(v.begin(), v.end()) - v.begin();
    double maxElement = *std::max_element(v.begin(), v.end());

    int minElementIndex = std::min_element(v.begin(), v.end()) - v.begin();
    double minElement = *std::min_element(v.begin(), v.end());

    //std::cout << "maxElementIndex:" << maxElementIndex << ", maxElement:" << maxElement << '\n';
    //std::cout << "minElementIndex:" << minElementIndex << ", minElement:" << minElement << '\n';

    int middleElementIndex;
    for (int i = 0; i < 3; i++)
    {
        if (i == maxElementIndex || i == minElementIndex)
            continue;
        middleElementIndex = i;
        //std::cout << "middleElementIndex " << middleElementIndex << std::endl;
    }
    v.clear();
    std::vector<int> result;
    result.push_back(maxElementIndex);
    result.push_back(middleElementIndex);
    result.push_back(minElementIndex);

    return result;
}

void PointPose::getCoordinateFrame(Eigen::Vector3f &centroid, Eigen::Matrix3f &rotation)
{
    pcl::PointXYZ centroidXYZ;
    centroidXYZ.getVector3fMap() = centroid;

    pcl::PointXYZ PointX = pcl::PointXYZ((centroid(0) + rotation.col(0)(0)),
                                         (centroid(1) + rotation.col(0)(1)),
                                         (centroid(2) + rotation.col(0)(2)));

    pcl::PointXYZ PointY = pcl::PointXYZ((centroid(0) + rotation.col(1)(0)),
                                         (centroid(1) + rotation.col(1)(1)),
                                         (centroid(2) + rotation.col(1)(2)));

    pcl::PointXYZ PointZ = pcl::PointXYZ((centroid(0) + rotation.col(2)(0)),
                                         (centroid(1) + rotation.col(2)(1)),
                                         (centroid(2) + rotation.col(2)(2)));

    m_pointsCoordinateFrame.clear();
    m_pointsCoordinateFrame.push_back(centroidXYZ);
    m_pointsCoordinateFrame.push_back(PointX);
    m_pointsCoordinateFrame.push_back(PointY);
    m_pointsCoordinateFrame.push_back(PointZ);
}

Eigen::Vector3f PointPose::moveCentroid(Eigen::Vector4f centroid)
{
    Eigen::Vector3f new_centroid;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(m_cloud_projected);
    pcl::PointXYZ searchPoint;

    searchPoint.x = centroid.x();
    searchPoint.y = centroid.y();
    searchPoint.z = centroid.z();

    int K = 1;
    std::vector<int> indices(K);
    std::vector<float> distances(K);

    if (kdtree.nearestKSearch(searchPoint, K, indices, distances) > 0)
    {
        //std::cout << m_cloud_projected->points[indices[0]].x << "; " << m_cloud_projected->points[indices[0]].y << "; " << m_cloud_projected->points[indices[0]].z << std::endl;
        //std::cout << m_cloud_grasp->points[indices[0]].x << "; " << m_cloud_grasp->points[indices[0]].y << "; " << m_cloud_grasp->points[indices[0]].z << std::endl;
        m_origin = m_cloud_grasp->points[indices[0]];

        new_centroid = m_cloud_grasp->points[indices[0]].getVector3fMap();
    }

    return new_centroid;
}