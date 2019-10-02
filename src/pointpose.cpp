#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/cloud_viewer.h>

#include "../include/pointpose.h"

void PointPose::setSourceCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in) { m_source = cloud_in; }

void PointPose::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) { m_cloud_grasp = cloud; }

void PointPose::setRefPlane(pcl::ModelCoefficients::Ptr &plane) { m_plane = plane; }

Eigen::Vector3f PointPose::getTranslation() { return m_trans; }

Eigen::Quaternionf PointPose::getRotation() { return m_rot; }

void PointPose::computeGraspPoint()
{
    m_pointsAxes = computeTransformation(m_cloud_grasp, m_plane, m_cloud_projected, m_rot, m_trans);
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

    viz.addSphere(m_pointsAxes[0], 0.005, "sphere");
    viz.addArrow(m_pointsAxes[1], m_pointsAxes[0], 1.0f, 0.0f, 0.0f, false, "x_axis");
    viz.addArrow(m_pointsAxes[2], m_pointsAxes[0], 0.0f, 1.0f, 0.0f, false, "y_axis");
    viz.addArrow(m_pointsAxes[3], m_pointsAxes[0], 0.0f, 0.0f, 1.0f, false, "z_axis");

    viz.addCube(m_trans, m_rot, 0.01, 0.01, 0.01, "cube");
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

std::vector<pcl::PointXYZ> PointPose::computeAxes(pcl::PointXYZ &centroid, Eigen::Matrix3f &eigenVectorsPCA,
                                                  std::vector<int> &eigenIdx, float &grasp_depth)
{
    std::vector<pcl::PointXYZ> results;

    // centroid translated along z axis by grasp_depth
    pcl::PointXYZ point;
    int id = eigenIdx[2];
    point.x = centroid.x + grasp_depth * eigenVectorsPCA.col(id).x();
    point.y = centroid.y + grasp_depth * eigenVectorsPCA.col(id).y();
    point.z = centroid.z + grasp_depth * eigenVectorsPCA.col(id).z();

    if (point.z > centroid.z)
    {
        point.x = centroid.x - grasp_depth * eigenVectorsPCA.col(id).x();
        point.y = centroid.y - grasp_depth * eigenVectorsPCA.col(id).y();
        point.z = centroid.z - grasp_depth * eigenVectorsPCA.col(id).z();
    }

    centroid.x = point.x;
    centroid.y = point.y;
    centroid.z = point.z;
    results.push_back(point); // centroid point above

    //points for the axes
    float t = 0.1;
    for (int i = 0; i < eigenIdx.size(); i++)
    {
        int id = eigenIdx[i];
        point.x = centroid.x + t * eigenVectorsPCA.col(id).x();
        point.y = centroid.y + t * eigenVectorsPCA.col(id).y();
        point.z = centroid.z + t * eigenVectorsPCA.col(id).z();

        if (point.z < centroid.z)
        {
            point.x = centroid.x - t * eigenVectorsPCA.col(id).x();
            point.y = centroid.y - t * eigenVectorsPCA.col(id).y();
            point.z = centroid.z - t * eigenVectorsPCA.col(id).z();
        }
        results.push_back(point);
    }
    return results;
}

std::vector<pcl::PointXYZ> PointPose::computeTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                            pcl::ModelCoefficients::Ptr &coefficients,
                                                            pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_projected,
                                                            Eigen::Quaternionf &Quaternion,
                                                            Eigen::Vector3f &Translation)
{

    float z_ref = 10000;
    for (int i = 0; i < cloud->size(); i++)
    {
        if (cloud->points[i].z < z_ref)
            z_ref = cloud->points[i].z;
    }

    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud_projected, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud_projected, pcaCentroid, covariance);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // ---------------------------------------------
    // Axes coordinate
    std::vector<int> eigenvaluesIdx = orderEigenvalues(eigenValuesPCA);

    float grasp_depth = pcaCentroid.z() - z_ref;
    PCL_WARN("Possible grasp travel distance from centroid: %f mm\n", grasp_depth * 1000);

    pcl::PointXYZ centroid;
    centroid.x = pcaCentroid.x();
    centroid.y = pcaCentroid.y();
    centroid.z = pcaCentroid.z();

    std::vector<pcl::PointXYZ> pointAxes = computeAxes(centroid, eigenVectorsPCA, eigenvaluesIdx, grasp_depth);

    // Final transform
    Quaternion = eigenVectorsPCA;
    std::cout << "---------------------------" << std::endl;
    std::cout << "Quaternion: " << Quaternion.x() << ", " << Quaternion.y() << ", " << Quaternion.z() << ", " << Quaternion.w() << std::endl;

    Translation.x() = centroid.x;
    Translation.y() = centroid.y;
    Translation.z() = centroid.z;
    std::cout << "Translation: " << Translation.x() << ", " << Translation.y() << ", " << Translation.z() << std::endl;
    std::cout << "---------------------------" << std::endl;

    return pointAxes;
}
