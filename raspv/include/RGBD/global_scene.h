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

#ifndef GLOBAL_SCENE_H
#define GLOBAL_SCENE_H

#include <boost/filesystem.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/angles.h>
#include <pcl/common/common_headers.h>

#include "RGBD/plane.h"
#include "RGBD/current_scene.h"


// CLASS VISUALIZER
class GlobalScene {

  public:

    GlobalScene() {
        c2f.setIdentity();
        main_dir.setIdentity();
		
        initial_floor_ = false;
        initial_reference_ = false;
        new_floor_ = false;
        has_manhattan_ = false;

	}

    //// Resets the object
    void reset();

    //// Finds the floor either from cloud (scene.fcloud) or from already segmented planes (scene.vPlanes)
    /// in: Current scene (including filtered cloud and planes)
    /// out: (in class) floor_normal_ (4-element vector with plane coefficients [A,B,C,D]), initial_floor_ (true if there is floor in the image)
    void findFloor(CurrentScene &scene);

    //// Finds the floor in scene from given point cloud
    /// in: cloud (Pointcloud to find the floor)
    /// out: (in class) floor_normal_ (4-element vector with plane coefficients [A,B,C,D]), initial_floor_ (true if there is floor in the image)
    void findFloorCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    //// Performs RANSAC operation to find the floor, evaluating if first plane found is valid
    /// in: cloud (pointcloud to find the floor)
    /// out: remaining points of the cloud (not in plane found), normal (to become floor_normal_)
    /// return: true (if floor found)
	bool subtractInitialPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Vector4f &normal);

    //// Computes camera to floor transformation matrix given floor_normal_
    /// in: floor_normal (4-element vector with plane coefficients [A,B,C,D])
    /// out: (in class) c2f, f2c (camera to floor transformation matrix and inverse), person_height_
    void computeCamera2FloorMatrix (Eigen::Vector4f floor_normal);

    //// Computes camera to absolute camera reference (i.e. map reference, from gazebo, with Z upwards)
    /// in: f2a (transformation matrix from floor reference frame (Y upwards) to absolute reference frame (Z upwards)
    /// out: c2a, a2c (camera to absolute transformation matrix and inverse)
    void computeCamera2AbsoluteMatrix (Eigen::Affine3d f2a);

    //// Computes camera to absolute camera reference (i.e. map reference, from gazebo, with Z upwards)
    /// in: f2a (computed inside)
    /// out: c2a, a2c (camera to absolute transformation matrix and inverse)
    void computeCamera2AbsoluteMatrix ();

    //// Update camera to absolute matrix with odometry values
    /// in: c2c0 (odometry, i.e. transformation matrix from current camera (c) to initial camera (c0))
    /// out: updated c2a, a2c
    void computeCamera2AbsoluteMatrixWithOdometry (Eigen::Affine3d c2c0);

    //// Update c2f given new floor_normal and compute variation w.r.t. old c2f
    /// in: floor_normal
    /// out: updated c2f, f2c, and f2f (transformation between old and current floor)
    void computeIncrementalCamera2FloorMatrix (Eigen::Vector4f floor_normal);

    //// Find floor quickly given vPlanes
    /// in: vPlanes (vector with Planes in the scene)
    /// out: (in class) floor_normal_, new_floor (true if there is new floor in vPlanes)
    void findFloorFast(std::vector<Plane> vPlanes);

    //// Find Manhattan directions in current scene and updates w.r.t. old Manhattan directions
    /// in: scene (to compute Manhattan directions in current scene)
    /// out: (in class) updated main_dir, floor_normal_, c2f, f2c
    void getManhattanDirections(CurrentScene &scene);

    //// Given Manhattan directons from new scene, update old ones (maintaining orientaitons)
    /// in: manhattan_dirs (new Manhattan directions)
    /// out: main_dir (Principal directions), has_manhattan_ (true if Manhattan has been found)
    void updateManhattanDirections(Eigen::Matrix3f manhattan_dirs);

    //// Updates floor with new Manhattan coordinates (to be used, e.g. when there is no floor in scene)
    /// in: main_dir (Manhattan directions)
    /// out: (in class) floor_normal_, c2f, f2c
    void updateFloorWithManhattan();



	
    //// Variables
    bool initial_floor_; //  true if floor has been found in the scene
    Eigen::Vector4f floor_normal_; // plane coefficients of floor plane
    Eigen::Affine3d c2f; // transformation matrix camera to floor
    Eigen::Affine3d f2c; // transformation matrix floor to camera
    Eigen::Affine3d f2f; // transformation matrix amongst new deteciton of floor
    Eigen::Affine3d c2a; // camera to floor/manhattan in absolute coordinates (Z upwards)
    Eigen::Affine3d a2c; // floor/manhattan in absolute coordinates (Z upwards) to camera
    float person_height_; // estimated height of the camera to floor
    bool new_floor_; // true if floor has been found in current scene
    bool has_manhattan_; // true if manhattan directions have been found
    Eigen::Matrix3f main_dir; // Principal directions (Manhattan directions) on the global scene
    bool initial_reference_; // true if floor/principal directions have been found the first time, establishin camera c0
    Eigen::Affine3d c02a; // Transformation from initial camera (c0) to absolute coordinates

	
};

#endif
