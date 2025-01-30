/**
* This file is part of stairs_detection.
*
* Copyright (C) 2019 Alejandro Pérez Yus <alperez at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/aperezyus/stairs_detection>
*
* stairs_detection is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* stairs_detection is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with stairs_detection. If not, see <http://www.gnu.org/licenses/>.
*/

#include "RGBD/RGBD.h"

void downsampleCloud(float leaf_size, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out) {
    cloud_out.reset(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (cloud_in);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*cloud_out);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloud(float leaf_size, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (cloud_in);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*cloud_out);

    return cloud_out;
}

void filterCloudMinMaxField(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float min, float max, std::string field) {

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName (field);
    pass.setFilterLimits (min, max);
//    pass.filter (*cloud_filtered);
//    pcl::copyPointCloud(*cloud_filtered,*cloud);

    pass.filter(*cloud);
}

void filterCloudMinMaxFieldTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Affine3d T, float min, float max, std::string field) {
    pcl::transformPointCloud(*cloud,*cloud,T);

    filterCloudMinMaxField(cloud,min,max,field);

    pcl::transformPointCloud(*cloud,*cloud,T.inverse());

}


void extractNormalsRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, double radius) {
    normals.reset(new pcl::PointCloud<pcl::Normal>);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setRadiusSearch (radius);
    normal_estimator.useSensorOriginAsViewPoint();

    normal_estimator.compute (*normals);
}

void extractNormalsNeighbors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, int neighbors) {
    normals.reset(new pcl::PointCloud<pcl::Normal>);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (neighbors);
    normal_estimator.useSensorOriginAsViewPoint();

    normal_estimator.compute (*normals);

}


void extractNormalsRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, pcl::search::Search<pcl::PointXYZ>::Ptr &tree, double radius) {
    normals.reset(new pcl::PointCloud<pcl::Normal>);
    tree =  boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setRadiusSearch (radius);
    normal_estimator.useSensorOriginAsViewPoint();

    normal_estimator.compute (*normals);
}

void extractNormalsNeighbors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, pcl::search::Search<pcl::PointXYZ>::Ptr &tree, int neighbors) {
    normals.reset(new pcl::PointCloud<pcl::Normal>);
    tree =  boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (neighbors);
    normal_estimator.useSensorOriginAsViewPoint();

    normal_estimator.compute (*normals);

}

bool sortIndicesBySize(const pcl::PointIndices &lhs, const pcl::PointIndices &rhs) {
    return lhs.indices.size() > rhs.indices.size(); // Large planes to small planes
}

void regionGrowingSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &v_regions, pcl::PointCloud<pcl::PointXYZ>::Ptr &remaining_cloud)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    regionGrowingSegmentation(cloud,cloud_normals,tree,v_regions,remaining_cloud);

}

void regionGrowingSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::search::Search<pcl::PointXYZ>::Ptr tree,  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &v_regions, pcl::PointCloud<pcl::PointXYZ>::Ptr &remaining_cloud)
{
    // Configure region growing:
    const int k_min_cluster_size = 30; // 50
    const int k_num_neighbors = 16; // 8
    const float k_angle_threshold = 6.0; // 10
    const float k_curvature_threshold = 0.5;

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (k_min_cluster_size);
    reg.setSmoothModeFlag(true);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (k_num_neighbors);//8
    reg.setInputCloud (cloud); //fcloud
    reg.setInputNormals (cloud_normals);
    reg.setSmoothnessThreshold (pcl::deg2rad(k_angle_threshold));
    reg.setCurvatureThreshold (k_curvature_threshold);//0.5
    std::vector <pcl::PointIndices> regions;
    reg.extract (regions);

    pcl::PointIndices::Ptr total_indices (new pcl::PointIndices);

    sort(regions.begin(), regions.end(), sortIndicesBySize);

    remaining_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    *remaining_cloud = *cloud;

    for (size_t Q=0; Q < regions.size(); Q++) {
        if (regions[Q].indices.size() > k_min_cluster_size) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>());
            pcl::ExtractIndices<pcl::PointXYZ> ex;
            ex.setInputCloud (cloud);
            pcl::PointIndices::Ptr indices_ (new pcl::PointIndices);
            *indices_ = regions[Q];
            ex.setIndices (indices_);
            ex.setNegative (false);
            ex.filter (*cloud_temp);

            v_regions.push_back(cloud_temp);

            total_indices->indices.insert(total_indices->indices.end(), indices_->indices.begin(), indices_->indices.end());
        }
    }

    pcl::ExtractIndices<pcl::PointXYZ> ex;
    ex.setInputCloud (remaining_cloud);
    ex.setIndices (total_indices);
    ex.setNegative (true);
    ex.filter (*remaining_cloud);

}

bool isPlane (pcl::PointCloud<pcl::PointXYZ>::Ptr region, Eigen::Vector4f &plane_coefficients, float min_ratio)
{
    bool is_plane = false;

    // Create the segmentation object for the planar model and set all the parameters
    const int k_max_iter = 100; // to iterate the RANSAC
    const double k_dist_threshold = 0.04; // To accept points as inliers in the RANSAC //0.02
//    const float k_min_ratio = 0.80f; // Percentage of points of the cloud belonging to the plane

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (k_max_iter);
    seg.setDistanceThreshold (k_dist_threshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (region);
    seg.segment (*inliers, *coefficients);

    float n_points_total = region->points.size();
    float n_points_plane = inliers->indices.size();
    float ratio = n_points_plane/n_points_total;

    if (ratio > min_ratio) {
        is_plane = true;

        plane_coefficients = Eigen::Vector4f(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]);
        if (plane_coefficients[3] < 0) {plane_coefficients = -plane_coefficients;}
    }

    return is_plane;
}

void segmentPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::search::Search<pcl::PointXYZ>::Ptr tree,  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &v_planes,   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &v_clusters, std::vector<Eigen::Vector4f> &v_plane_coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr &remaining_cloud)
{
    const float k_min_ratio = 0.80f; // Percentage of points of the cloud belonging to the plane

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> v_regions;
    regionGrowingSegmentation(cloud, cloud_normals, tree, v_regions, remaining_cloud);

    for (size_t i = 0; i<v_regions.size(); i++)
    {
        Eigen::Vector4f coefficients;
        if (isPlane(v_regions[i], coefficients,k_min_ratio))
        {
            v_planes.push_back(v_regions[i]);
            v_plane_coefficients.push_back(coefficients);
        }
        else {
            v_clusters.push_back(v_regions[i]);
        }
    }
}

void extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &v_clusters) {
    const int k_min_cluster_size = 50;
    const double k_cluster_tolerance = 0.05;

    if (cloud->points.size() > k_min_cluster_size) {

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (k_cluster_tolerance);
        ec.setMinClusterSize (k_min_cluster_size);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract (cluster_indices);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_aux (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                cloud_cluster_aux->points.push_back (cloud->points[size_t(*pit)]);
            cloud_cluster_aux->width = uint32_t(cloud_cluster_aux->points.size());
            cloud_cluster_aux->height = 1;
            cloud_cluster_aux->is_dense = true;

            v_clusters.push_back(cloud_cluster_aux);
        }
    }
}



void scaleDepth(cv::Mat mat_depth, cv::Mat &img_to_plot){
    double min, max;
    cv::minMaxIdx(mat_depth, &min, &max);
    cv::Mat adjMap;
    cv::convertScaleAbs(mat_depth, adjMap, 255.0 / max);
    cv::Mat bgr;
    cv::applyColorMap(adjMap, bgr, cv::COLORMAP_JET);

    bgr.copyTo(img_to_plot);
}