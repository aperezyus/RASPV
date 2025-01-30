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

#include <pcl/filters/conditional_removal.h>

#include "RGBD/current_scene.h"

void CurrentScene::applyVoxelFilter(float leaf_size, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    downsampleCloud(leaf_size, cloud, fcloud);
}

void CurrentScene::transformCloud(Eigen::Affine3d T) {
    pcl::transformPointCloud(*fcloud,*fcloud,T);
}

void CurrentScene::filterFloorAndCeiling(float floor_height, float ceiling_height, std::string field) {
    filterCloudMinMaxField(fcloud,floor_height,ceiling_height,field);
}

void CurrentScene::filterFloorAndCeilingTransform(Eigen::Affine3d T, float floor_height, float ceiling_height, std::string field) {
    filterCloudMinMaxFieldTransform(fcloud,T, floor_height,ceiling_height,field);
}


void CurrentScene::getNormalsRadius(double radius) {
    extractNormalsRadius(fcloud, normals, tree, radius);
}

void CurrentScene::getNormalsNeighbors(int neighbors) {

     extractNormalsNeighbors(fcloud, normals, tree, neighbors);

}


void CurrentScene::regionGrowing() {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> v_planar_clouds;
    std::vector<Eigen::Vector4f> v_planar_coefficients;
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    segmentPlanes(fcloud, normals, tree, v_planar_clouds, vObstacles, v_planar_coefficients, remaining_cloud);

    for (size_t i=0; i<v_planar_clouds.size(); i++)
    {
        Plane plane(v_planar_clouds[i],v_planar_coefficients[i]);
        vPlanes.push_back(plane);
    }
}


void CurrentScene::segmentScene() {

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> v_planar_clouds;
    std::vector<Eigen::Vector4f> v_planar_coefficients;
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    segmentPlanes(fcloud, normals, tree, v_planar_clouds, vObstacles, v_planar_coefficients, remaining_cloud);

    for (size_t i=0; i<v_planar_clouds.size(); i++)
    {
        Plane plane(v_planar_clouds[i],v_planar_coefficients[i]);
        vPlanes.push_back(plane);
    }

    this->getCentroids();
    this->getContours();

    if (remaining_cloud->points.size() > 0)
        extractClusters(remaining_cloud, vObstacles);

}

void CurrentScene::getCentroids() {
    for (size_t Q=0; Q<vPlanes.size(); Q++)
        vPlanes[Q].getCentroid();
}

void CurrentScene::getContours() {
    for (size_t Q=0; Q<vPlanes.size(); Q++) {
        vPlanes[Q].getContour();
        vPlanes[Q].fixHolesConcaveHull();
    }
}

void CurrentScene::getPlaneCoeffs2Floor(Eigen::Affine3d c2f) {
    for (size_t Q=0; Q<vPlanes.size();Q++)
        vPlanes[Q].getCoeffs2Floor(c2f);
}

void CurrentScene::getCentroids2Floor(Eigen::Affine3d c2f) {
    for (size_t Q=0; Q<vPlanes.size();Q++) {
        vPlanes[Q].getCentroid2Floor(c2f);
	}
}

void CurrentScene::getClouds2Floor(Eigen::Affine3d c2f) {
    for (size_t Q = 0; Q<vPlanes.size(); Q++)
        vPlanes[Q].getCloud2Floor(c2f);
}

void CurrentScene::getCentroidsCoeffs2Floor(Eigen::Affine3d c2f)
{
    this->getPlaneCoeffs2Floor(c2f);
    this->getCentroids2Floor(c2f);
}

void CurrentScene::copyPlanesAbsolute()
{
    for (size_t Q = 0; Q<vPlanes.size(); Q++) {
        vPlanes[Q].coeffs2f = vPlanes[Q].coeffs;
        vPlanes[Q].centroid2f = vPlanes[Q].centroid;
        vPlanes[Q].cloud2f.reset(new pcl::PointCloud<pcl::PointXYZ>());
        *vPlanes[Q].cloud2f = *vPlanes[Q].cloud;
        vPlanes[Q].contour2f.reset(new pcl::PointCloud<pcl::PointXYZ>());
        *vPlanes[Q].contour2f = *vPlanes[Q].contour;
    }
    vObstacles2f = vObstacles;

}

void CurrentScene::transformPlanes(Eigen::Affine3d c2f) {

    for (size_t Q = 0; Q<vPlanes.size(); Q++) {
        vPlanes[Q].getCoeffs2Floor(c2f);
        vPlanes[Q].getCentroid2Floor(c2f);
        vPlanes[Q].getCloud2Floor(c2f);
        vPlanes[Q].getContour2Floor(c2f);
    }

}

void CurrentScene::transformObstacles(Eigen::Affine3d c2f) {

    for (size_t Q = 0; Q<vObstacles.size(); Q++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*vObstacles[Q],*obstacle,c2f);
        vObstacles2f.push_back(obstacle);
    }

}

void CurrentScene::transformPlanesAndObstacles(Eigen::Affine3d c2f) {
    transformPlanes(c2f);
    transformObstacles(c2f);
}

void CurrentScene::classifyPlanes() {
    // Thresholds for classification:
    const float k_angle_threshold = 40.0f;
    const float k_dist_floor_threshold = 0.30f;

    for (size_t Q=0; Q<vPlanes.size();Q++) {
        float A = vPlanes[Q].coeffs2f[0];
        float B = vPlanes[Q].coeffs2f[1];
        float C = vPlanes[Q].coeffs2f[2];

        float dot = vPlanes[Q].coeffs2f.head<3>().dot(Eigen::Vector3f::UnitY());
        dot = ( dot < -1.0f ? -1.0f : ( dot > 1.0f ? 1.0f : dot ) ); // to avoid NaNs
        float angle = pcl::rad2deg(acos(dot));

        if (fabs(A)>fabs(B) and fabs(A)>fabs(C)) {         // If component in X is higher (lateral plane)
            if (fabs(90-angle)<k_angle_threshold) // if angle is close to perpendicular to floor normal
                vPlanes[Q].type = 2;
            else
                vPlanes[Q].type = 5;
        }
        else if (fabs(C)>fabs(A) and fabs(C)>fabs(B)) {        // If component in Z is higher (frontal plane)
            if (fabs(90-angle)<k_angle_threshold) // if angle is close to perpendicular to floor normal
                vPlanes[Q].type = 3;
            else
                vPlanes[Q].type = 5;
        }
        else if (fabs(B)>fabs(A) and fabs(B)>fabs(C)) {         // If component in Y is higher (horizontal plane)
            if (angle<k_angle_threshold){ // || fabs(angle-180) < 15) { // if angle is close to floor normal it is horizontal
                if (fabs(vPlanes[Q].centroid2f.y) <= k_dist_floor_threshold) // If close to floor, then it is floor
                    vPlanes[Q].type = 0;
                else
                    vPlanes[Q].type = 1;
            }
            else
                vPlanes[Q].type = 5;
        }
    }
}

void CurrentScene::classifyPlanesAbs() {
    // Thresholds for classification:
    const float k_angle_threshold = 15.0f; // 10
    const float k_dist_floor_threshold = 0.08f;

    for (size_t Q=0; Q<vPlanes.size();Q++) {
        float A = vPlanes[Q].coeffs2f[0];
        float B = vPlanes[Q].coeffs2f[1];
        float C = vPlanes[Q].coeffs2f[2];

        float dot = vPlanes[Q].coeffs2f.head<3>().dot(Eigen::Vector3f::UnitZ());
        dot = ( dot < -1.0f ? -1.0f : ( dot > 1.0f ? 1.0f : dot ) ); // to avoid NaNs
        float angle = pcl::rad2deg(acos(dot));

        if (fabs(B)>fabs(A) and fabs(B)>fabs(C)) {         // If component in Y is higher
            if (fabs(90-angle)<k_angle_threshold) // if angle is close to perpendicular to floor normal
                vPlanes[Q].type = 2;
            else
                vPlanes[Q].type = 5;
        }
        else if (fabs(A)>fabs(B) and fabs(A)>fabs(C)) {        // If component in X is higher
            if (fabs(90-angle)<k_angle_threshold) // if angle is close to perpendicular to floor normal
                vPlanes[Q].type = 3;
            else
                vPlanes[Q].type = 5;
        }
        else if (fabs(C)>fabs(A) and fabs(C)>fabs(B)) {         // If component in Y is higher (horizontal plane)
            if (angle<k_angle_threshold){ // || fabs(angle-180) < 15) { // if angle is close to floor normal it is horizontal
                if (fabs(vPlanes[Q].centroid2f.z) <= k_dist_floor_threshold) // If close to floor, then it is floor
                    vPlanes[Q].type = 0;
                else
                    vPlanes[Q].type = 1;
            }
            else
                vPlanes[Q].type = 5;
        }
    }
}

void CurrentScene::getManhattanDirectionsFromNormalsWithFloor(Eigen::Affine3d f2c, Eigen::Affine3d c2f) {
    // We have one main direction: Y. The idea is to choose another one using existing normals.
    // First, we need to downsample the number of normals, and then we keep choosing one reasonably perpendicular, and compute the third orthogonal one.
    // We compute the system of three orthogonal vectors that most inliers among the rest of normals produce.

    int N_normals = int(normals->points.size());
    if (N_normals < 3)
        return;

    // Parameters
    const int k_N_normals_max = 1000; // Instead of considering all normals, which may be too many
    const int k_attempts = 500; // Attempts to try to find the normal
    const float k_ang_threshold = 10*float(M_PI)/180; // Angular threshold to consider normal inlier
    const float k_percentage_inliers = 0.60f; // Percentage of inliers considered good enough to stop iterating
    const float k_percentage_inliers_rep = 0.30f; // Percentage of inliers considered good enough after k_inliers_rep iterations
    const int k_inliers_rep = 50; // Number of iterations to choose k_percentage_inliers_rep instead of k_percentage_inliers

    // Select up to N_normals_max normals by shuffling the initial normals.
    Eigen::MatrixXf normals_eigen;
    normals_eigen.resize(k_N_normals_max,3);

    std::vector<size_t> indexes(static_cast<size_t>(N_normals));
    std::iota(std::begin(indexes), std::end(indexes), 0);
    std::random_shuffle(indexes.begin(), indexes.end());

    int size_cloud = 0;
    size_t i = 0;

    while (size_cloud < k_N_normals_max) {
        if (normals->points[indexes[i]].curvature < 0.005f && normals->points[indexes[i]].curvature > 0.00f) { // curvature must be small, but normals with 0 curvature come from isolated points
            normals_eigen(size_cloud,0) = normals->points[indexes[i]].normal_x;
            normals_eigen(size_cloud,1) = normals->points[indexes[i]].normal_y;
            normals_eigen(size_cloud,2) = normals->points[indexes[i]].normal_z;
            size_cloud++;
        }
        i++;
        if (i == indexes.size())
            break;
    }

    if (size_cloud < k_N_normals_max) {
        N_normals = size_cloud;
        normals_eigen.resize(N_normals,3);

    }
    else
        N_normals = k_N_normals_max;


    int N_inliers_max = 0;

    Eigen::MatrixXf best_main_dir = Eigen::MatrixXf::Identity(3,3);

    Eigen::Vector3f floor_normal = f2c.rotation().col(1).cast<float>();

    for (int i=0, j=0; i<k_attempts && j<N_normals; i++, j++) {
        // Select normal j
        Eigen::Vector3f selected_normal = normals_eigen.row(j);

        float dot = floor_normal.dot(selected_normal);
        dot = ( dot < -1.0f ? -1.0f : ( dot > 1.0f ? 1.0f : dot ) );
        if (fabs(float(M_PI_2)-acos(dot)) > k_ang_threshold)
            continue;

        Eigen::Vector3f selected_normal2f = c2f.rotation().cast<float>()*selected_normal;
        selected_normal2f(1) = 0; // Now normal it is perpendicular in floor coordinates
        selected_normal2f.normalize(); // Now it is unitary
        selected_normal = f2c.rotation().cast<float>()*selected_normal2f; // Now it is back to camera coordinates

        Eigen::Vector3f orthogonal_normal = (floor_normal.cross(selected_normal)).normalized(); // Now we have three orthogonal components

        // Compute angles of all the normals w.r.t. the three directions and select the minimum
        Eigen::MatrixXf ang1, ang2, ang3;
        ang1 = acos((normals_eigen*floor_normal).array().abs());
        ang2 = acos((normals_eigen*selected_normal).array().abs());
        ang3 = acos((normals_eigen*orthogonal_normal).array().abs());
        Eigen::MatrixXf min_ang;
        min_ang = ang3.array().min(ang1.array().min(ang2.array()));

        // Count inliers
        int N_inliers = int((min_ang.array() < k_ang_threshold).count());

        if (N_inliers > N_inliers_max) {
           N_inliers_max = N_inliers;
           Eigen::Matrix3f current_main_dir;
           current_main_dir << floor_normal, selected_normal, orthogonal_normal;
           best_main_dir = current_main_dir;

           // Exit condition 1: k_percentage_inliers
           if (N_inliers_max > k_percentage_inliers*float(N_normals)) {
               has_manhattan_ = true;
                break;
           }
        }
        // Exit condition 2: k_percentage_inliers_rep after k_inliers_rep iterations
        if (N_inliers_max > k_percentage_inliers_rep*float(N_normals) && i > k_inliers_rep) {
            has_manhattan_ = true;
            break;
        }
    }


    if (has_manhattan_)
        main_dir = sortManhattanDirections(best_main_dir);

}

void CurrentScene::getManhattanDirectionsFromNormals() {
    // This procedure resembles a RANSAC to find three main directions orthogonal that match as many normals as possible

    int N_normals = int(normals->points.size());
    if (N_normals < 3)
        return;

    // Parameters
    const int k_N_normals_max = 1000; // Instead of considering all normals, which may be too many
    const int k_attempts = 500; // Attempts to try to find the normal
    const float k_ang_threshold = 10*float(M_PI)/180; // Angular threshold to consider normal inlier
    const float k_percentage_inliers = 0.60f; // Percentage of inliers considered good enough to stop iterating
    const float k_percentage_inliers_rep = 0.30f; // Percentage of inliers considered good enough after k_inliers_rep iterations
    const int k_inliers_rep = 50; // Number of iterations to choose k_percentage_inliers_rep instead of k_percentage_inliers

    // Select up to N_normals_max normals by shuffling the initial normals.
    Eigen::MatrixXf normals_eigen;
    normals_eigen.resize(k_N_normals_max,3);

    std::vector<size_t> indexes(static_cast<size_t>(N_normals));
    std::iota(std::begin(indexes), std::end(indexes), 0);
    std::random_shuffle(indexes.begin(), indexes.end());

    int size_cloud = 0;
    size_t i = 0;

    while (size_cloud < k_N_normals_max) {
        if (normals->points[indexes[i]].curvature < 0.005f && normals->points[indexes[i]].curvature > 0.00f) { // curvature must be small, but normals with 0 curvature come from isolated points
            normals_eigen(size_cloud,0) = normals->points[indexes[i]].normal_x;
            normals_eigen(size_cloud,1) = normals->points[indexes[i]].normal_y;
            normals_eigen(size_cloud,2) = normals->points[indexes[i]].normal_z;
            size_cloud++;
        }
        i++;
        if (i == indexes.size())
            break;
    }

    if (size_cloud < k_N_normals_max) {
        N_normals = size_cloud;
        normals_eigen.resize(N_normals,3);

    }
    else
        N_normals = k_N_normals_max;

    int N_inliers_max = 0;

    Eigen::MatrixXf best_main_dir = Eigen::MatrixXf::Identity(3,3);

    for (int i=0; i<k_attempts; i++) {
        // Select two random indices
        std::vector<int> indexes(static_cast<size_t>(N_normals));
        std::iota(std::begin(indexes), std::end(indexes), 0);
        std::random_shuffle(indexes.begin(), indexes.end());
        indexes.resize(2);

        // Compute three orthogonal directions
        Eigen::Vector3f v1 = normals_eigen.row(indexes[0]);
        Eigen::Vector3f v_aux = normals_eigen.row(indexes[1]);
        Eigen::Vector3f v2 = v1.cross(v_aux);
        v2.normalize();
        Eigen::Vector3f v3 = v1.cross(v2);

        // Compute angles of all the normals w.r.t. the three directions and select the minimum
        Eigen::MatrixXf ang1, ang2, ang3;
        ang1 = acos((normals_eigen*v1).array().abs());
        ang2 = acos((normals_eigen*v2).array().abs());
        ang3 = acos((normals_eigen*v3).array().abs());
        Eigen::MatrixXf min_ang;
        min_ang = ang3.array().min(ang1.array().min(ang2.array()));

        // Count inliers
        int N_inliers = int((min_ang.array() < k_ang_threshold).count());

        if (N_inliers > N_inliers_max) {
           N_inliers_max = N_inliers;
           Eigen::Matrix3f current_main_dir;
           current_main_dir << v1, v2, v3;
           best_main_dir = current_main_dir;

           // Exit condition 1: k_percentage_inliers
           if (N_inliers_max > k_percentage_inliers*float(N_normals)) {
               has_manhattan_ = true;
                break;
           }
        }
        // Exit condition 2: k_percentage_inliers_rep after k_inliers_rep iterations
        if (N_inliers_max > k_percentage_inliers_rep*float(N_normals) && i > k_inliers_rep) {
            has_manhattan_ = true;
            break;
        }
    }

    if (has_manhattan_)
        main_dir = sortManhattanDirections(best_main_dir);

}



void CurrentScene::getManhattanDirectionsFromPlanesWithFloor(Eigen::Affine3d f2c) {
    // Knowing one direction (floor --> Y in Manhattan)
    // Find a plane between vertical planes to orient the scene so as to maximum number of inliers are aligned
    // Inliers are the points belonging to a plane that is aligned to one of the directions

    // Parameters
    const double k_angle_threshold = 5.0*(M_PI)/180.0;

    int n_inliers = 0;
    int max_n_inliers = 0;
    int plane_selected = 0;
    Eigen::Vector3f best_normal(0,0,0);

    for (size_t P = 0; P < vPlanes.size(); P++) {
        if (vPlanes[P].type == 2 || vPlanes[P].type == 3) {
            Eigen::Vector3f normal;
            normal = vPlanes[P].coeffs2f.head<3>();

            n_inliers = int(vPlanes[P].cloud->points.size());

            for (size_t Q = 0; Q < vPlanes.size(); Q++) {
                if ((Q != P) and (vPlanes[Q].type == 2 || vPlanes[Q].type == 3)) {
                    Eigen::Vector3f current_normal;
                    current_normal = vPlanes[Q].coeffs2f.head<3>();
                    double angle = double(acos(Eigen::Vector2f(normal(0),normal(2)).normalized().dot(Eigen::Vector2f(current_normal(0),current_normal(2)))));

                    if ((fabs(M_PI-angle) < k_angle_threshold) or (fabs(M_PI_2-angle) < k_angle_threshold) or (fabs(M_PI-angle) < k_angle_threshold) or (fabs(3/2*M_PI-angle) < k_angle_threshold) or (angle < k_angle_threshold)) {
                        n_inliers += vPlanes[Q].cloud->points.size();
                    }
                }
            }

            if (n_inliers > max_n_inliers) {
                max_n_inliers = n_inliers;
                best_normal = normal;
                plane_selected = int(P);
            }
        }
    }


    if (max_n_inliers > 0){

        has_manhattan_ = true;

        Eigen::Matrix3f best_main_dir;
        best_main_dir.col(0) = (f2c.rotation().cast<float>())*Eigen::Vector3f(-best_normal(2),0, best_normal(0)).normalized();
        best_main_dir.col(1) = (f2c.rotation().cast<float>())*Eigen::Vector3f(0,1,0);
        best_main_dir.col(2) = (f2c.rotation().cast<float>())*Eigen::Vector3f(best_normal(0),0,best_normal(2)).normalized();

        main_dir = sortManhattanDirections(best_main_dir);


    }
}



void CurrentScene::getManhattanDirectionsFromPlanes()  {
    // Similar to getManhattanDirectionsFromNormals, but using already extracted planes (from vPlanes)
    // Thus, not shuffling, just consider all plane combinations.

    int N_inliers_max = 0;

    // Parameters
    const float k_ang_threshold = 5*float(M_PI)/180; // Angular threshold to consider normal inlier
    const int k_min_N_planes = 2; // Minimum number of planes that must be found to consider good Manhattan directions
    const float k_percentage_inliers = 0.50f; // Percentage of inliers considered good enough to validate

    Eigen::MatrixXf best_main_dir = Eigen::MatrixXf::Identity(3,3);

    Eigen::MatrixXf normals_eigen(vPlanes.size(),3);
    for (size_t P = 0; P < vPlanes.size(); P++)
        normals_eigen.block<1,3>(int(P),0) = vPlanes[P].coeffs.block<3,1>(0,0);

    int N_total_points = 0;

    for (size_t P = 0; P < vPlanes.size(); P++) {
        N_total_points += int(vPlanes[P].cloud->points.size());
        for (size_t Q = 0; Q < vPlanes.size(); Q++)  {
            if ((vPlanes[P].type != vPlanes[Q].type) && !(vPlanes[P].type <= 1 && vPlanes[Q].type <= 1)) {
                int N_inliers = 0;
                int N_planes = 0;

                Eigen::Vector3f normal_1, normal_2;
                normal_1 = normals_eigen.row(int(P));
                normal_2 = normals_eigen.row(int(Q));

                // Check if they are close to perpendicular
                float dot = normal_1.dot(normal_2);
                dot = ( dot < -1.0f ? -1.0f : ( dot > 1.0f ? 1.0f : dot ) );
                if (fabs(float(M_PI_2)-acos(dot)) > pcl::deg2rad(k_ang_threshold))
                    continue;

                Eigen::Vector3f v1 = normals_eigen.row(int(P));
                Eigen::Vector3f v_aux = normals_eigen.row(int(Q));
                Eigen::Vector3f v2 = v1.cross(v_aux);
                v2.normalize();
                Eigen::Vector3f v3 = v1.cross(v2);

                Eigen::MatrixXf ang1, ang2, ang3;
                ang1 = acos((normals_eigen*v1).array().abs());
                ang2 = acos((normals_eigen*v2).array().abs());
                ang3 = acos((normals_eigen*v3).array().abs());
                Eigen::MatrixXf min_ang;
                min_ang = ang3.array().min(ang1.array().min(ang2.array()));

                for (size_t i = 0; i< vPlanes.size(); i++) {
                    if (min_ang(int(i),0) < k_ang_threshold) {
                        N_inliers += vPlanes[i].cloud->points.size();
                        N_planes++;
                    }
                }

                if (N_inliers > N_inliers_max && N_planes >= k_min_N_planes) {
                    N_inliers_max = N_inliers;
                    Eigen::Matrix3f current_main_dir;
                    current_main_dir << v1, v2, v3;
                    best_main_dir = current_main_dir;
                }
            }

        }
    }

    if (N_inliers_max > k_percentage_inliers*N_total_points) {
        main_dir = sortManhattanDirections(best_main_dir);
        has_manhattan_ = true;
    }
}


Eigen::Matrix3f CurrentScene::sortManhattanDirections(Eigen::Matrix3f best_main_dir) {


    Eigen::Vector3f v_floor(0.0f, -sin(float(M_PI_4)), -sin(float(M_PI_4)));
    Eigen::Vector3f v_front(0.0f, 0.0f, 1.0f);

    Eigen::MatrixXf angles_y, angles_z;
    angles_y = acos((v_floor.transpose()*best_main_dir).array().abs());
    angles_z = acos((v_front.transpose()*best_main_dir).array().abs());

    int dummy, index_x, index_y, index_z;
    float min_y, min_z;

    min_y = angles_y.minCoeff(&dummy,&index_y);
    min_z = angles_z.minCoeff(&dummy,&index_z);

    if (index_y == index_z){
        if (min_y < min_z){
            if ((index_y != 0) && (angles_z(0,0) < angles_z(0,1) || angles_z(0,0) < angles_z(0,2)))
                    index_z = 0;
            else if ((index_y != 1) && (angles_z(0,1) < angles_z(0,0) || angles_z(0,1) < angles_z(0,2)))
                    index_z = 1;
            else if ((index_y != 2) && (angles_z(0,2) < angles_z(0,0) || angles_z(0,2) < angles_z(0,1)))
                    index_z = 2;
        }
        else{
            if ((index_z != 0) && (angles_y(0,0) < angles_y(0,1) || angles_y(0,0) < angles_y(0,2)))
                    index_y = 0;
            if ((index_z != 1) && (angles_y(0,1) < angles_y(0,0) || angles_y(0,1) < angles_y(0,2)))
                    index_y = 1;
            if ((index_z != 2) && (angles_y(0,2) < angles_y(0,0) || angles_y(0,2) < angles_y(0,1)))
                    index_y = 2;
        }
    }

    if (index_y == 0)
        index_z == 1 ? index_x = 2 : index_x = 1;
    else if (index_y == 1)
        index_z == 1 ? index_x = 2 : index_x = 0;
    else
        index_z == 0 ? index_x = 1 : index_x = 0;

    Eigen::Vector3f dx = best_main_dir.col(index_x);
    Eigen::Vector3f dy = best_main_dir.col(index_y);
    Eigen::Vector3f dz = best_main_dir.col(index_z);

    if (dy(1) > 0)
        dy = -dy;
    if (dz(2) < 0)
        dz = -dz;
    dx = dy.cross(dz);

    Eigen::MatrixXf final_main_dir(3,3);
    final_main_dir << dx, dy, dz;

    return final_main_dir;
}
