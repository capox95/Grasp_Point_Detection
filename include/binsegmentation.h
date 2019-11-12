#include <pcl/ModelCoefficients.h>

class BinSegmentation
{

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_source;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_source_bw;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_top_vertices;

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_occluding_edges;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m_occluded_edges;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m_boundary_edges;

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_vertices_scaled, m_cloud_vertices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_hull_result;

    pcl::ModelCoefficients::Ptr m_plane;

    int m_num_lines;
    float m_padding_distance, m_max_bin_height;
    double m_sqr_eps; //  maximum allowable distance to the true solution lineWithLineIntersection (need to be taken squared) and
                      // maximum distance between point and line in checkOrthogonality
    std::vector<pcl::ModelCoefficients> m_lines;

public:
    BinSegmentation() : m_source(new pcl::PointCloud<pcl::PointXYZRGB>),
                        m_source_bw(new pcl::PointCloud<pcl::PointXYZ>),
                        m_top_vertices(new pcl::PointCloud<pcl::PointXYZ>),
                        m_occluding_edges(new pcl::PointCloud<pcl::PointXYZ>),
                        m_occluded_edges(new pcl::PointCloud<pcl::PointXYZRGBA>),
                        m_boundary_edges(new pcl::PointCloud<pcl::PointXYZRGBA>),
                        m_cloud_vertices_scaled(new pcl::PointCloud<pcl::PointXYZ>),
                        m_cloud_vertices(new pcl::PointCloud<pcl::PointXYZ>),
                        m_hull_result(new pcl::PointCloud<pcl::PointXYZ>),
                        m_plane(new pcl::ModelCoefficients)
    {
        m_sqr_eps = 0.03;
        m_max_bin_height = 2;
    }

    void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

    void setNumberLines(int number);

    void setPaddingDistance(float scale);

    void setMaxBinHeight(float value);

    pcl::ModelCoefficients::Ptr getPlaneGroundPoints();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getVerticesBinContour();

    void cloudRGBtoXYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_rgb, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_xyz);

    void cloudXYZtoRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_xyz, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_rgb);

    bool compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_grasp);

    void visualize(bool showLines, bool showVertices, bool spin);

private:
    bool computeEdges(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &occluding_edges,
                      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &occluded_edges,
                      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &boundary_edges);

    bool ransacLineDetection(pcl::PointCloud<pcl::PointXYZ>::Ptr &occluding_edges,
                             std::vector<pcl::ModelCoefficients> &lines);

    bool checkLinesOrthogonal(std::vector<pcl::ModelCoefficients> &lines,
                              std::vector<Eigen::Vector4f> &points);

    bool getIntersactions(std::vector<pcl::ModelCoefficients> &lines,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_vertices);

    void scaleHull(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_vertices,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &hull_result);

    bool addGroundPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_vertices,
                         pcl::ModelCoefficients::Ptr &plane);

    void convexHullCrop(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_bw,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_vertices,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &hull_result);

    pcl::ModelCoefficients::Ptr planeModel(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    void segmentOccludingEdges(pcl::PointCloud<pcl::PointXYZ>::Ptr &occluding_edges,
                               pcl::ModelCoefficients::Ptr &plane);

    void computePlaneBottomBin(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr &vertices,
                               pcl::ModelCoefficients::Ptr &plane, float max_bin_height);
};