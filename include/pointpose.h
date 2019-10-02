#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PointPose
{

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_source;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_grasp, m_cloud_projected;
    pcl::ModelCoefficients::Ptr m_plane;

    Eigen::Vector3f m_trans;
    Eigen::Quaternionf m_rot;
    std::vector<pcl::PointXYZ> m_pointsAxes;

public:
    PointPose() : m_source(new pcl::PointCloud<pcl::PointXYZRGB>),
                  m_cloud_grasp(new pcl::PointCloud<pcl::PointXYZ>),
                  m_cloud_projected(new pcl::PointCloud<pcl::PointXYZ>),
                  m_plane(new pcl::ModelCoefficients)

    {
    }

    void setSourceCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in);

    void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    void setRefPlane(pcl::ModelCoefficients::Ptr &plane);

    Eigen::Vector3f getTranslation();

    Eigen::Quaternionf getRotation();

    void computeGraspPoint();

    void visualizeGrasp();

private:
    std::vector<int> orderEigenvalues(Eigen::Vector3f eigenValuesPCA);

    std::vector<pcl::PointXYZ> computeAxes(pcl::PointXYZ &centroid,
                                           Eigen::Matrix3f &eigenVectorsPCA,
                                           std::vector<int> &eigenIdx,
                                           float &grasp_depth);

    std::vector<pcl::PointXYZ> computeTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                     pcl::ModelCoefficients::Ptr &coefficients,
                                                     pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_projected,
                                                     Eigen::Quaternionf &Quaternion,
                                                     Eigen::Vector3f &Translation);
};