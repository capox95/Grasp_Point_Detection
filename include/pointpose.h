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
    std::vector<pcl::PointXYZ> m_pointsCoordinateFrame;
    pcl::PointXYZ m_origin;
    pcl::ModelCoefficients m_line;

    Eigen::Vector3f _directionX, _directionY, _directionZ, _centroid;

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

    Eigen::Vector3f getDirectionWrinkle();

    bool computeGraspPoint(Eigen::Affine3d &transformation_matrix);

    void visualizeGrasp();

private:
    std::vector<int> orderEigenvalues(Eigen::Vector3f eigenValuesPCA);

    void getCoordinateFrame(Eigen::Vector3f &centroid, Eigen::Matrix3f &rotation);

    Eigen::Vector3f moveCentroid(Eigen::Vector4f centroid);

    void directionWrinkle();

    Eigen::Affine3d computeTransformation();
};