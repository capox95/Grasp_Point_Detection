
struct Spherical
{
    float inclination;
    float azimuth;
    float entropy;
    float entropy_normalized;
};

class EntropyFilter
{

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_source, m_cloud_downsample;
    pcl::PointCloud<pcl::PointNormal> m_mls_points;
    pcl::PointCloud<pcl::PointNormal>::Ptr m_convexity_ready;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_mls_cloud, m_cloud_seg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud_depth, m_cloud_convexity, m_cloud_combined;
    pcl::PointCloud<pcl::Normal>::Ptr m_mls_normals;
    pcl::PointCloud<Spherical>::Ptr m_spherical;

    pcl::ModelCoefficients::Ptr m_plane;

    float m_leafsize,
        m_entropy_threshold, m_curvature_threshold, m_depth_interval, m_depth_threshold, m_angle_threshold;
    int m_KNN;

public:
    EntropyFilter() : m_source(new pcl::PointCloud<pcl::PointXYZRGB>),
                      m_cloud_downsample(new pcl::PointCloud<pcl::PointXYZRGB>),
                      m_mls_cloud(new pcl::PointCloud<pcl::PointXYZ>),
                      m_mls_normals(new pcl::PointCloud<pcl::Normal>),
                      m_spherical(new pcl::PointCloud<Spherical>),
                      m_cloud_seg(new pcl::PointCloud<pcl::PointXYZ>),
                      m_cloud_depth(new pcl::PointCloud<pcl::PointXYZI>),
                      m_cloud_convexity(new pcl::PointCloud<pcl::PointXYZI>),
                      m_cloud_combined(new pcl::PointCloud<pcl::PointXYZI>),
                      m_convexity_ready(new pcl::PointCloud<pcl::PointNormal>),
                      m_plane(new pcl::ModelCoefficients)

    {
    }

    void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in);

    void setDownsampleLeafSize(float leaf_size);

    void setEntropyThreshold(float entropy_th);

    void setKLocalSearch(int K);

    void setCurvatureThreshold(float curvature_th);

    void setDepthThreshold(float depth_th);

    void setAngleThresholdForConvexity(float angle_th);

    pcl::PointCloud<pcl::PointXYZ>::Ptr getMLSCloud();

    pcl::PointCloud<pcl::Normal>::Ptr getMLSNormals();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudSeg();

    pcl::ModelCoefficients::Ptr getPlaneForDepth();

    void compute(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);

    //
    //ColorMap functions
    void colorMapEntropy(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_map);

    void colorMapCurvature(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_map);

    void colorMapInclination(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_map);

    void colorMapAzimuth(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_map);

    void visualizeAll(bool flag);

private:
    void downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, float leaf_size,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);

    void computePolyFitting(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointNormal> &mls_points);

    void divideCloudNormals(pcl::PointCloud<pcl::PointNormal> &input, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                            pcl::PointCloud<pcl::Normal>::Ptr &normals);

    void getSpherical(pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals, pcl::PointCloud<Spherical>::Ptr &spherical);

    void normalizeEntropy(pcl::PointCloud<Spherical>::Ptr &spherical);

    //LOCAL HISTOGRAM and entropy calculation at the end.
    //param[in]: point cloud normals in spherical coordinates
    //param[in]: current point index in the cloud
    //param[in]: vector of indeces of neighborhood points of considered on
    void histogram2D(pcl::PointCloud<Spherical>::Ptr &spherical, int id0, std::vector<int> indices);

    // LOCAL SEARCH
    void local_search(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<Spherical>::Ptr &spherical,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_combined);

    void segmentCloudEntropy(pcl::PointCloud<pcl::PointNormal> &cloud, pcl::PointCloud<Spherical>::Ptr &spherical,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr &output, float thresholdEntropy);

    void connectedComponets(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_clusters);

    float getPointPlaneDistanceCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out,
                                     pcl::ModelCoefficients::Ptr &coeff);

    void downsampleCloudAndNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                   pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                   float leaf_size,
                                   pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_out);

    bool checkConvexity(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                        pcl::PointCloud<pcl::Normal>::Ptr &normals,
                        int id0,
                        std::vector<int> ids);

    void splitPointNormal(pcl::PointCloud<pcl::PointNormal>::Ptr &input,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                          pcl::PointCloud<pcl::Normal>::Ptr &normals);

    void localSearchForConvexity(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_map);

    void combineConvexityAndCurvatureInfo(pcl::PointCloud<pcl::PointXYZI>::Ptr &convexity,
                                          pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                          pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_map);
};
