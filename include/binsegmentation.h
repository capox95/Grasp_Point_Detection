#include <pcl/ModelCoefficients.h>

class BinSegmentation
{

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_source;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_source_bw;

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_occluding_edges;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m_occluded_edges;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m_boundary_edges;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m_high_curvature_edges;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m_rgb_edges;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_vertices_scaled;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_hull_result;

    pcl::ModelCoefficients::Ptr m_plane;

    int m_num_lines;
    float m_scale_factor;
    std::vector<pcl::ModelCoefficients> m_lines;

public:
    BinSegmentation() : m_source(new pcl::PointCloud<pcl::PointXYZRGB>),
                        m_source_bw(new pcl::PointCloud<pcl::PointXYZ>),
                        m_occluding_edges(new pcl::PointCloud<pcl::PointXYZ>),
                        m_occluded_edges(new pcl::PointCloud<pcl::PointXYZRGBA>),
                        m_boundary_edges(new pcl::PointCloud<pcl::PointXYZRGBA>),
                        m_high_curvature_edges(new pcl::PointCloud<pcl::PointXYZRGBA>),
                        m_rgb_edges(new pcl::PointCloud<pcl::PointXYZRGBA>),
                        m_cloud_vertices_scaled(new pcl::PointCloud<pcl::PointXYZ>),
                        m_hull_result(new pcl::PointCloud<pcl::PointXYZ>),
                        m_plane(new pcl::ModelCoefficients)
    {
    }

    void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

    void setNumberLines(int number);

    void setScaleFactorHullBorders(float scale);

    pcl::ModelCoefficients::Ptr getPlaneGroundPoints();

    void cloudRGBtoXYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_rgb, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_xyz);

    void cloudXYZtoRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_xyz, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_rgb);

    void compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_grasp);

    void visualizeNoSpin(bool showLines, bool showVertices);

    void visualizeWithSpin(bool showLines, bool showVertices);

private:
    void computeEdges(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &occluding_edges,
                      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &occluded_edges, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &boundary_edges,
                      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &high_curvature_edges, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &rgb_edges);

    void ransacLineDetection(pcl::PointCloud<pcl::PointXYZ>::Ptr &occluding_edges, std::vector<pcl::ModelCoefficients> &lines);

    int checkLinesOrthogonal(std::vector<pcl::ModelCoefficients> &lines, std::vector<Eigen::Vector4f> &points);

    void getIntersactions(std::vector<pcl::ModelCoefficients> &lines, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_vertices);

    void scaleHull(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_vertices,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &hull_result);

    void addGroundPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_vertices);

    void convexHullCrop(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_bw,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_vertices,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &hull_result);
};