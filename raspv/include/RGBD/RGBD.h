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

#ifndef RGBD_H
#define RGBD_H

#include <boost/filesystem.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

//#include <pcl/kdtree/kdtree_flann.h>

//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/sample_consensus/sac_model_normal_plane.h>
//#include <pcl/features/integral_image_normal.h>
//#include <pcl/sample_consensus/ransac.h>

//#include "RGBD/plane.h"


//// Applies Voxel Grid filter to cloud.
/// in: leaf_size (size of voxel, in meters), cloud (to be filtered)
/// out: fcloud (filtered cloud)
void downsampleCloud(float leaf_size, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloud(float leaf_size, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);

//// Filters cloud given min and max values of certain field. For example, if field is "z", returned cloud will have only points whose "z" is between "min" and "max"
void filterCloudMinMaxField(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float min, float max, std::string field);
//// Same as above, but in this case if the filter needs to be done in other reference, provide transformation T. Cloud will be returned at initial reference frame.
void filterCloudMinMaxFieldTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Affine3d T, float min, float max, std::string field);

//// Extract normals of a cloud either via radius search or neighbor search. Possibility to get the kdtree to be reused afterwards
void extractNormalsRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, double radius);
void extractNormalsNeighbors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, int neighbors);
void extractNormalsRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, pcl::search::Search<pcl::PointXYZ>::Ptr &tree, double radius);
void extractNormalsNeighbors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, pcl::search::Search<pcl::PointXYZ>::Ptr &tree, int neighbors);

//// Helper function to sort an std::vector of pcl::PointIndices by size
bool sortIndicesBySize(const pcl::PointIndices &lhs, const pcl::PointIndices &rhs);

//// Applies region growing algorithm and returns std::vector of pointclouds for each region and remaining points not belonging to any region
void regionGrowingSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &v_regions, pcl::PointCloud<pcl::PointXYZ>::Ptr &remaining_cloud);
void regionGrowingSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::search::Search<pcl::PointXYZ>::Ptr tree,  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &v_regions, pcl::PointCloud<pcl::PointXYZ>::Ptr &remaining_cloud);

//// Given a pointcloud, returns true if this region is a plane (i.e. a ratio of more than min_ratio of points belong to most dominant plane)
/// Also returns plane coefficients if the cloud is a plane.
bool isPlane (pcl::PointCloud<pcl::PointXYZ>::Ptr region, Eigen::Vector4f &plane_coefficients, float min_ratio);

//// Segments the scene in regions via region growing and apply isPlane to return a vector of clouds that are planes (and its coefficients) and a vector of clouds that are not.
void segmentPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::search::Search<pcl::PointXYZ>::Ptr tree,  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &v_planes,   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &v_clusters, std::vector<Eigen::Vector4f> &v_plane_coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr &remaining_cloud);

//// Gets a vector of clusters of points given a pointcloud
void extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &v_clusters);

//// Scales depth map to show it in imshow
void scaleDepth(cv::Mat mat_depth, cv::Mat &img_to_plot);


#endif
