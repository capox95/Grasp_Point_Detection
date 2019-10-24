#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/intersections.h>
#include <pcl/common/geometry.h>
#include <pcl/features/organized_edge_detection.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <string>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include "../include/binsegmentation.h"

void BinSegmentation::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) { m_source = cloud; }

void BinSegmentation::setNumberLines(int number) { m_num_lines = number; }

void BinSegmentation::setScaleFactorHullBorders(float scale) { m_scale_factor = scale; }

pcl::ModelCoefficients::Ptr BinSegmentation::getPlaneGroundPoints() { return m_plane; }

void BinSegmentation::cloudRGBtoXYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_rgb, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_xyz)
{
    cloud_xyz->points.resize(cloud_rgb->size());
    for (int i = 0; i < cloud_rgb->points.size(); i++)
    {
        cloud_xyz->points[i].x = cloud_rgb->points[i].x;
        cloud_xyz->points[i].y = cloud_rgb->points[i].y;
        cloud_xyz->points[i].z = cloud_rgb->points[i].z;
    }
}

void BinSegmentation::cloudXYZtoRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_xyz,
                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_rgb)
{
    cloud_rgb->points.resize(cloud_xyz->size());
    for (int i = 0; i < cloud_xyz->points.size(); i++)
    {
        cloud_rgb->points[i].x = cloud_xyz->points[i].x;
        cloud_rgb->points[i].y = cloud_xyz->points[i].y;
        cloud_rgb->points[i].z = cloud_xyz->points[i].z;
    }
}

bool BinSegmentation::compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_grasp)
{

    cloudRGBtoXYZ(m_source, m_source_bw);
    bool edges_result = computeEdges(m_source, m_occluding_edges, m_occluded_edges, m_boundary_edges);
    if (!edges_result)
    {
        PCL_ERROR("edge detection fails\n");
        return false;
    }
    else
        PCL_INFO("Edge Detection DONE \n");

    pcl::ModelCoefficients::Ptr plane_ref = planeModel(m_source_bw);
    segmentOccludingEdges(m_occluding_edges, plane_ref);

    //RANSAC Lines Detection
    bool ransac_success = ransacLineDetection(m_occluding_edges, m_lines);
    if (!ransac_success)
    {
        PCL_ERROR("ransac line detection fails\n");
        return false;
    }
    else
        PCL_INFO("RANSAC DONE \n");

    //Points Intersaction
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vertices(new pcl::PointCloud<pcl::PointXYZ>);
    bool intersaction_result = getIntersactions(m_lines, cloud_vertices);
    if (!intersaction_result)
    {
        PCL_ERROR("line intersaction fails\n");
        return false;
    }
    else
        PCL_INFO("Line Intersaction DONE \n");

    m_cloud_vertices = cloud_vertices;

    //Get distances and diagonal vector, return 4 vertices inside the original ones
    scaleHull(cloud_vertices, m_cloud_vertices_scaled);

    //Add points on the ground (projection of the 4 vertices)
    bool ground_result = addGroundPoints(m_source_bw, m_cloud_vertices_scaled, plane_ref);
    if (!ground_result)
    {
        PCL_ERROR("ground points fails\n");
        return false;
    }
    //Segmentation based on Convex Hull Crop
    convexHullCrop(m_source_bw, m_cloud_vertices_scaled, m_hull_result);

    cloudXYZtoRGB(m_hull_result, cloud_grasp);

    return true;
}

void BinSegmentation::visualize(bool showLines = true, bool showVertices = true, bool spin = true)
{
    //PointCloud Visualization
    pcl::visualization::PCLVisualizer vizSource("PCL Source");
    vizSource.addCoordinateSystem(0.1, "coord", 0);
    vizSource.setBackgroundColor(1.0f, 1.0f, 1.0f);
    vizSource.addPointCloud(m_source, "m_source");
    vizSource.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.2f, 0.0f, 1.0f, "m_source");

    if (showLines)
    {
        vizSource.addLine(m_lines[0], "line0", 0);
        vizSource.addLine(m_lines[1], "line1", 0);
        vizSource.addLine(m_lines[2], "line2", 0);
        vizSource.addLine(m_lines[3], "line3", 0);
    }

    if (showVertices)
    {
        for (int i = 0; i < m_cloud_vertices->points.size(); i++)
        {
            std::string name = "point" + std::to_string(i);
            vizSource.addSphere(m_cloud_vertices->points[i], 0.01, 1.0f, 0.0f, 0.0f, name, 0);
        }
    }

    pcl::visualization::PCLVisualizer viz("PCL Segmentation");
    //viz.addCoordinateSystem(0.1, "coord", 0);
    viz.setBackgroundColor(1.0f, 1.0f, 1.0f);
    viz.addPointCloud(m_source, "m_source");
    viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.2f, 0.0f, 1.0f, "m_source");

    //viz.addPointCloud(m_boundary_edges, "boundary_edges");
    //viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "boundary_edges");

    viz.addPointCloud(m_occluding_edges, "occluding_edges");
    viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.0f, 1.0f, "occluding_edges");
    viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "occluding_edges");

    if (showLines)
    {
        viz.addLine(m_lines[0], "line0", 0);
        viz.addLine(m_lines[1], "line1", 0);
        viz.addLine(m_lines[2], "line2", 0);
        viz.addLine(m_lines[3], "line3", 0);
    }

    if (showVertices)
    {
        for (int i = 0; i < m_cloud_vertices_scaled->points.size(); i++)
        {
            std::string name = "point" + std::to_string(i);
            viz.addSphere(m_cloud_vertices_scaled->points[i], 0.01, 1.0f, 0.0f, 0.0f, name, 0);
        }
    }

    //viz.addPointCloud(cloud_vertices, "cloud_vertexes");
    //viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "cloud_vertexes");

    viz.addPointCloud(m_hull_result, "hull_result");
    viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 0.0f, "hull_result");

    if (spin)
        viz.spin();
}

bool BinSegmentation::computeEdges(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &occluding_edges,
                                   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &occluded_edges, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &boundary_edges)
{

    pcl::OrganizedEdgeBase<pcl::PointXYZRGB, pcl::Label> oed;
    oed.setInputCloud(cloud);
    oed.setDepthDisconThreshold(0.02); // 2cm
    oed.setMaxSearchNeighbors(50);
    pcl::PointCloud<pcl::Label> labels;
    std::vector<pcl::PointIndices> label_indices;
    oed.compute(labels, label_indices);

    pcl::copyPointCloud(*cloud, label_indices[0].indices, *boundary_edges);
    pcl::copyPointCloud(*cloud, label_indices[1].indices, *occluding_edges);
    pcl::copyPointCloud(*cloud, label_indices[2].indices, *occluded_edges);

    std::cout << "boundary_edges : " << boundary_edges->size() << std::endl;
    std::cout << "occluding_edges : " << occluding_edges->size() << std::endl;
    std::cout << "occluded_edges : " << occluded_edges->size() << std::endl;

    if (occluding_edges->size() == 0)
        return false;
    else
        return true;
}

bool BinSegmentation::ransacLineDetection(pcl::PointCloud<pcl::PointXYZ>::Ptr &occluding_edges, std::vector<pcl::ModelCoefficients> &lines)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr occluding_edges_ransac(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*occluding_edges, *occluding_edges_ransac);

    pcl::ModelCoefficients line;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);

    for (int i = 0; i < m_num_lines; i++)
    {
        seg.setInputCloud(occluding_edges_ransac);
        seg.segment(*inliers, line);

        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a linear model for the given dataset." << std::endl;
            return false;
        }
        else
        {
            //std::cerr << "number inliers " << i << ": " << inliers->indices.size() << std::endl;
        }

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        // Extract the inliers
        extract.setInputCloud(occluding_edges_ransac);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*occluding_edges_ransac);

        lines.push_back(line);
    }
    return true;
}

bool BinSegmentation::checkLinesOrthogonal(std::vector<pcl::ModelCoefficients> &lines, std::vector<Eigen::Vector4f> &points)
{
    // way to find  to which line the point represent the intersaction
    Eigen::Vector4f line_pt, line_dir;
    double sqr_norm, value;

    std::vector<std::vector<int>> idx(4);

    for (int i = 0; i < lines.size(); i++)
    {
        // format the line considered into vector4f

        line_pt[0] = lines[i].values[0];
        line_pt[1] = lines[i].values[1];
        line_pt[2] = lines[i].values[2];
        line_pt[3] = 0;

        line_dir[0] = lines[i].values[3];
        line_dir[1] = lines[i].values[4];
        line_dir[2] = lines[i].values[5];
        line_dir[3] = 0;

        sqr_norm = sqrt(line_dir.norm());

        //check all the 4 points
        //std::vector<int> vtemp;
        for (int k = 0; k < points.size(); k++)
        {
            value = pcl::sqrPointToLineDistance(points[k], line_pt, line_dir, sqr_norm);

            if (value < (m_sqr_eps / 2)) //points[i] IS on the line
            {
                idx[i].push_back(k);
                //std::cout << value << ", " << k << ", " << i << std::endl;
            }
        }
    }

    for (int i = 0; i < lines.size(); i++)
        if (idx[i].size() != 2)
        {
            PCL_ERROR("Found %d intersaction instead of 2\n", idx[i].size());
            return false;
        }

    Eigen::Vector4f point_temp;
    for (int i = 0; i < lines.size(); i++)
    {
        for (int k = 0; k < lines.size(); k++)
        {

            if (i == k)
                continue;

            point_temp.setZero();

            if (idx[i][0] == idx[k][0])
                point_temp = points[idx[i][0]];

            else if (idx[i][1] == idx[k][1])
                point_temp = points[idx[i][1]];

            else if (idx[i][0] == idx[k][1])
                point_temp = points[idx[i][0]];

            else if (idx[i][1] == idx[k][0])
                point_temp = points[idx[i][1]];

            if (!point_temp.isZero())
            {
                Eigen::Vector3f line0, line1;

                line0.x() = lines[i].values[0];
                line0.y() = lines[i].values[1];
                line0.z() = lines[i].values[2];

                line1.x() = lines[k].values[0];
                line1.y() = lines[k].values[1];
                line1.z() = lines[k].values[2];

                Eigen::Vector3f v1 = point_temp.head<3>() - line0;
                Eigen::Vector3f v2 = point_temp.head<3>() - line1;

                double angle = acos(v1.dot(v2));

                //std::cout << "dot value: " << angle << std::endl;

                if (angle < 1.55 || angle > 1.59)
                {
                    PCL_WARN("Angle not orthogonal. value: %f\n", angle);
                    std::cout << angle << std::endl;
                    return false;
                }

                continue;
            }
        }
    }

    return true;
}

bool BinSegmentation::getIntersactions(std::vector<pcl::ModelCoefficients> &lines, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_vertices)
{

    std::vector<Eigen::Vector4f> points;
    Eigen::Vector4f point_temp;
    for (int i = 0; i < lines.size(); i++)
    {
        for (int k = 0; k < lines.size(); k++)
        {
            if (i != k)
            {
                pcl::lineWithLineIntersection(lines[i], lines[k], point_temp, m_sqr_eps * m_sqr_eps); //2 cm max distance from the true solution
                if (point_temp.norm() < 2)
                    points.push_back(point_temp);
            }
        }
    }
    //PCL_INFO("lineWithLineIntersection done\n");

    // remove points that are zero or that are duplicate version of existing ones.
    for (int i = 0; i < points.size(); i++)
    {
        if (points[i].isZero())
            points.erase(points.begin() + i);
    }

    for (int i = 0; i < points.size(); i++)
    {
        for (int k = 0; k < points.size(); k++)
        {
            if (i != k)
            {
                if (points[i].isApprox(points[k], 0.05))
                    points.erase(points.begin() + k);
            }
        }
    }
    //PCL_INFO("purge done\n");

    if (points.size() != 4)
        return false;

    bool success = checkLinesOrthogonal(lines, points);
    if (!success)
        return false;
    //PCL_INFO("checkLinesOrthogonal DONE\n");

    //add points to pointcloud performing conversion from Vector4f to PointXYZ
    pcl::PointXYZ p_temp;
    for (int i = 0; i < points.size(); i++)
    {
        p_temp.getVector4fMap() = points[i];
        cloud_vertices->push_back(p_temp);
        //std::cout << p_temp.x << ", " << p_temp.y << ", " << p_temp.z << std::endl;
    }

    if (cloud_vertices->size() != 4)
    {
        PCL_WARN("Cloud Vertices different than 4, they are: %d\n", cloud_vertices->size());
        return false;
    }
    else
        return true;
}

void BinSegmentation::scaleHull(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_vertices,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &hull_result)
{
    if (cloud_vertices->size() != 4)
    {
        std::cout << "expecting 4 points, instead got: " << cloud_vertices->size() << " points!" << std::endl;
    }

    struct PointDiag
    {
        float distance;
        int index;
        Eigen::Vector3f vector;
    };
    std::vector<PointDiag> vd;

    std::vector<float> distances;
    float distance_temp;
    for (int i = 0; i < cloud_vertices->points.size(); i++) //ref point: "i"
    {
        PointDiag temp;
        //fill vector of distances wrt point "i"
        for (int k = 0; k < cloud_vertices->points.size(); k++)
        {
            distance_temp = pcl::geometry::distance(cloud_vertices->points[i], cloud_vertices->points[k]);
            distances.push_back(distance_temp);
            //std::cout << distance_temp << std::endl;
        }

        //find the max one
        float max_distance = 0;
        int max_index = 0;
        for (int j = 0; j < distances.size(); j++)
        {
            if (distances[j] > max_distance)
            {
                max_distance = distances[j];
                max_index = j;
            }
        }

        //std::cout << "max distance for point " << i << " is: " << max_distance << ", wrt point index: " << max_index << std::endl;
        //std::cout << std::endl;

        temp.distance = max_distance;
        temp.index = max_index;

        vd.push_back(temp);

        distances.clear();
    }

    //compute vector between two points
    Eigen::Vector3f vector_diag;
    int index_diag;
    for (int i = 0; i < vd.size(); i++)
    {
        index_diag = vd[i].index;
        vector_diag.x() = cloud_vertices->points[i].x - cloud_vertices->points[index_diag].x;
        vector_diag.y() = cloud_vertices->points[i].y - cloud_vertices->points[index_diag].y;
        vector_diag.z() = cloud_vertices->points[i].z - cloud_vertices->points[index_diag].z;
        vd[i].vector = vector_diag;
    }

    //new point computation, at 10% distance from original one wrt the diagonal
    pcl::PointXYZ crop_point;
    for (int i = 0; i < cloud_vertices->size(); i++)
    {
        crop_point.x = cloud_vertices->points[i].x - m_scale_factor * vd[i].vector.x();
        crop_point.y = cloud_vertices->points[i].y - m_scale_factor * vd[i].vector.y();
        crop_point.z = cloud_vertices->points[i].z - m_scale_factor * vd[i].vector.z();
        hull_result->points.push_back(crop_point);
    }
}

bool BinSegmentation::addGroundPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_vertices,
                                      pcl::ModelCoefficients::Ptr &plane)
{

    if (plane->values.size() != 4)
    {
        PCL_WARN("Plane coefficients something wrong! size: %d", plane->values.size());
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

    //add some padding to be able to go a little be below the ground plane and segment properly with the convex hull
    if (plane->values[3] < 0)
        plane->values[3] = plane->values[3] - 0.02;
    else
        plane->values[3] = plane->values[3] + 0.02;

    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud_vertices);
    proj.setModelCoefficients(plane);
    proj.filter(*cloud_projected);

    for (int i = 0; i < cloud_projected->size(); i++)
        cloud_vertices->push_back(cloud_projected->points[i]);

    return true;
}

void BinSegmentation::convexHullCrop(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_bw,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_vertices,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr &hull_result)
{
    pcl::CropHull<pcl::PointXYZ> cropHullFilter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_hull(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> hullPolygons;

    // setup hull filter
    pcl::ConvexHull<pcl::PointXYZ> cHull;
    cHull.setInputCloud(cloud_vertices);
    cHull.reconstruct(*points_hull, hullPolygons);

    cropHullFilter.setHullIndices(hullPolygons);
    cropHullFilter.setHullCloud(points_hull);
    //cropHullFilter.setDim(2);
    cropHullFilter.setCropOutside(true);

    //filter points
    cropHullFilter.setInputCloud(cloud_bw);
    cropHullFilter.filter(*hull_result);

    std::cout << std::endl;
    std::cout << "hull result points: " << hull_result->points.size() << std::endl;
    //for (int i = 0; i < hull_result->points.size(); i++)
    //{
    //    std::cout << hull_result->points[i] << std::endl;
    //}
}

pcl::ModelCoefficients::Ptr BinSegmentation::planeModel(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    m_plane = coefficients;

    return coefficients;
}

// find multiple planes parallel to the ground plane and segment the cloud keeping only those points that
// belongs to the plane which is the most distant from the ground.
void BinSegmentation::segmentOccludingEdges(pcl::PointCloud<pcl::PointXYZ>::Ptr &occluding_edges,
                                            pcl::ModelCoefficients::Ptr &plane)
{

    if (plane->values.size() != 4)
    {
        PCL_ERROR("model coefficient plane wrong");
        return;
    }
    // axis wrt we want to find a perpendicular plane
    Eigen::Vector3f plane_vec;
    plane_vec.x() = plane->values[0];
    plane_vec.y() = plane->values[1];
    plane_vec.z() = plane->values[2];

    int cloudSize = (int)occluding_edges->size();
    std::vector<pcl::ModelCoefficients::Ptr> models;
    std::vector<pcl::PointIndices::Ptr> modelsInliers;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*occluding_edges, *filtered);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setAxis(plane_vec);
    seg.setEpsAngle(0.1);
    seg.setDistanceThreshold(0.03);
    seg.setMaxIterations(200);
    while (filtered->size() > 0.1 * cloudSize)
    {
        seg.setInputCloud(filtered);
        seg.segment(*inliers, *coefficients);

        // Extract the inliers
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(filtered);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*filtered);

        models.push_back(coefficients);
        modelsInliers.push_back(inliers);
        coefficients.reset(new pcl::ModelCoefficients);
        inliers.reset(new pcl::PointIndices);
    }

    //std::cout << plane->values[3] << std::endl;
    //for (int i = 0; i < models.size(); i++)
    //    std::cout << models[i]->values[3] << std::endl;

    std::vector<double> diff;
    for (int i = 0; i < models.size(); i++)
    {
        diff.push_back(abs(plane->values[3] - models[i]->values[3]));
    }
    double maxDiffIndex = std::max_element(diff.begin(), diff.end()) - diff.begin();

    // Extract the true contour of the bin
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(occluding_edges);
    extract.setIndices(modelsInliers.at(maxDiffIndex));
    extract.setNegative(false);
    extract.filter(*occluding_edges);
}
