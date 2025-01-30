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

#include "RGBD/plane.h"

void Plane::getCentroid() {
    Eigen::Vector4f vector_centroid;
    pcl::compute3DCentroid(*cloud,vector_centroid);
    centroid = pcl::PointXYZ (vector_centroid[0], vector_centroid[1], vector_centroid[2]);
}

void Plane::getCentroid2Floor(Eigen::Affine3d c2f) {
    centroid2f.x = static_cast<float> (c2f.matrix() (0, 0) * centroid.x + c2f.matrix() (0, 1) * centroid.y + c2f.matrix() (0, 2) * centroid.z + c2f.matrix() (0, 3));
    centroid2f.y = static_cast<float> (c2f.matrix() (1, 0) * centroid.x + c2f.matrix() (1, 1) * centroid.y + c2f.matrix() (1, 2) * centroid.z + c2f.matrix() (1, 3));
    centroid2f.z = static_cast<float> (c2f.matrix() (2, 0) * centroid.x + c2f.matrix() (2, 1) * centroid.y + c2f.matrix() (2, 2) * centroid.z + c2f.matrix() (2, 3));
}

void Plane::getCoeffs2Floor(Eigen::Affine3d c2f) {
  coeffs2f = c2f.inverse().matrix().transpose().cast<float>()*coeffs;
}	

void Plane::getCloud2Floor(Eigen::Affine3d c2f) {
  cloud2f.reset(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*cloud,*cloud2f,c2f);
}

void Plane::getContour() {
    // To obtain the point cloud, first it is necessary to project all points to a plane (using the coeffs from the Plane)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    pcl::ModelCoefficients::Ptr model_coeffs (new pcl::ModelCoefficients);
    model_coeffs->values.resize(4);

    model_coeffs->values[0] = coeffs(0);
    model_coeffs->values[1] = coeffs(1);
    model_coeffs->values[2] = coeffs(2);
    model_coeffs->values[3] = coeffs(3);
    proj.setModelCoefficients (model_coeffs);
    proj.filter (*cloud_projected);

    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setAlpha (0.1); // 0.1 works well in practice
    chull.setInputCloud (cloud_projected);
    contour.reset(new pcl::PointCloud<pcl::PointXYZ>);
    chull.reconstruct (*contour);
}

void Plane::getContour2Floor(Eigen::Affine3d c2f)  {
  contour2f.reset(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*contour,*contour2f,c2f);
}

void Plane::getPrincipalDirections() {
	Eigen::Matrix3f covariance;
	Eigen::Vector4f eigen_centroid(centroid.x,centroid.y,centroid.z,1);
	pcl::computeCovarianceMatrixNormalized(*cloud,eigen_centroid,covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance,Eigen::ComputeEigenvectors);
    main_dir = eigen_solver.eigenvectors();
    main_dir.col(2) = main_dir.col(0).cross(main_dir.col(1));

    // Notice that column 0 has smaller eigenvalues, and thus direction perpendicular to the plane
}

void Plane::getMeasurements() {
    if (main_dir.isZero(0))
        this->getPrincipalDirections();

    // move the points to the that reference frame
    Eigen::Matrix4f w2p(Eigen::Matrix4f::Identity());
    w2p.block<3,3>(0,0) = main_dir.transpose();
    w2p.block<3,1>(0,3) = -1.f * (w2p.block<3,3>(0,0) * centroid.getVector3fMap().head<3>());
    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*cloud, cPoints, w2p);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);

    width = max_pt.z - min_pt.z; // third component, since main_dir has larger eigenvalue in third column
    length = max_pt.y - min_pt.y;
    height = max_pt.x - min_pt.x; // first component, since main_dir has smaller eigenvalue in first column

    // Compute the center (of the bounding rectangle) and put it back in world reference
    Eigen::Affine3d p2w = Eigen::Translation3d(centroid.getVector3fMap().cast<double>()) * Eigen::AngleAxisd(main_dir.cast<double>());
    Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
    pcl::transformPoint(mean_diag,mean_diag,p2w.cast<float>());
    center = pcl::PointXYZ(mean_diag(0),mean_diag(1),mean_diag(2));

}

void Plane::getMeasurements(Eigen::Matrix3f c2m) {

    Eigen::Matrix4f w2p(Eigen::Matrix4f::Identity());
    w2p.block<3,3>(0,0) = c2m.transpose();
    w2p.block<3,1>(0,3) = -1.f * (w2p.block<3,3>(0,0) * centroid.getVector3fMap().head<3>());
    Eigen::Affine3d p2w = Eigen::Translation3d(centroid.getVector3fMap().cast<double>()) * Eigen::AngleAxisd(c2m.cast<double>());

    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*cloud, cPoints, w2p);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);

    // In these projection matrix w/ manhattan directions, the y go upwards (height) and
    // the width should be in X to match the c2f
    width = max_pt.x - min_pt.x;
    height = max_pt.y - min_pt.y;
    length = max_pt.z - min_pt.z;

    Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

    pcl::transformPoint(mean_diag,mean_diag,p2w.cast<float>());
    center = pcl::PointXYZ(mean_diag(0),mean_diag(1),mean_diag(2));
}

void Plane::getMeasurements(Eigen::Affine3d & c2m) {

    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*cloud, cPoints, c2m);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);

    // In these projection matrix w/ manhattan directions, the y go upwards (height) and
    // the width should be in X to match the c2f
    width = max_pt.x - min_pt.x;
    height = max_pt.y - min_pt.y;
    length = max_pt.z - min_pt.z;

    Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

    pcl::transformPoint(mean_diag,mean_diag,c2m.inverse().cast<float>());
    center = pcl::PointXYZ(mean_diag(0),mean_diag(1),mean_diag(2));
}

void Plane::getMeasurements(Eigen::Matrix3f c2m, pcl::PointCloud<pcl::PointXYZ>::Ptr custom_cloud) {
    Eigen::Matrix4f w2p(Eigen::Matrix4f::Identity());
    w2p.block<3,3>(0,0) = c2m.transpose();
    w2p.block<3,1>(0,3) = -1.f * (w2p.block<3,3>(0,0) * centroid.getVector3fMap().head<3>());
    Eigen::Affine3d p2w = Eigen::Translation3d(centroid.getVector3fMap().cast<double>()) * Eigen::AngleAxisd(c2m.cast<double>());

    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*custom_cloud, cPoints, w2p);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);

    // In these projection matrix w/ manhattan directions, the y go upwards (height) and
    // the width should be in X to match the c2f
    width = max_pt.x - min_pt.x;
    height = max_pt.y - min_pt.y;
    length = max_pt.z - min_pt.z;

    Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

    pcl::transformPoint(mean_diag,mean_diag,p2w.cast<float>());
    center = pcl::PointXYZ(mean_diag(0),mean_diag(1),mean_diag(2));
}

void Plane::getVertices() {
    vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);

  // This function uses main_dir, thus, X is perpendicular to the plane and Z along the width

  Eigen::Vector3f pt1, pt2, pt3, pt4;
  pt1 = center.getVector3fMap()  + (width/2)*main_dir.col(2) + (length/2)*main_dir.col(1);
  pt2 = center.getVector3fMap()  - (width/2)*main_dir.col(2) + (length/2)*main_dir.col(1);
  pt3 = center.getVector3fMap()  - (width/2)*main_dir.col(2) - (length/2)*main_dir.col(1);
  pt4 = center.getVector3fMap()  + (width/2)*main_dir.col(2) - (length/2)*main_dir.col(1);

  vertices->push_back(pcl::PointXYZ(pt1(0),pt1(1),pt1(2)));
  vertices->push_back(pcl::PointXYZ(pt2(0),pt2(1),pt2(2)));
  vertices->push_back(pcl::PointXYZ(pt3(0),pt3(1),pt3(2)));
  vertices->push_back(pcl::PointXYZ(pt4(0),pt4(1),pt4(2)));
  vertices->push_back(center);
}

void Plane::getVertices(Eigen::Matrix3f c2m) {
    vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // This function uses dirs from, suposedly, Manhattan.
    // By our convention, X goes with the width and Z with the length

    Eigen::Vector3f pt1, pt2, pt3, pt4;
    pt1 = center.getVector3fMap() + (width/2)*c2m.col(0) + (length/2)*c2m.col(2);
    pt2 = center.getVector3fMap() - (width/2)*c2m.col(0) + (length/2)*c2m.col(2);
    pt3 = center.getVector3fMap() - (width/2)*c2m.col(0) - (length/2)*c2m.col(2);
    pt4 = center.getVector3fMap() + (width/2)*c2m.col(0) - (length/2)*c2m.col(2);

    vertices->push_back(pcl::PointXYZ(pt1(0),pt1(1),pt1(2)));
    vertices->push_back(pcl::PointXYZ(pt2(0),pt2(1),pt2(2)));
    vertices->push_back(pcl::PointXYZ(pt3(0),pt3(1),pt3(2)));
    vertices->push_back(pcl::PointXYZ(pt4(0),pt4(1),pt4(2)));
    vertices->push_back(center);
}

float Plane::getContourArea() {
     if (contour == nullptr) {
        this->getContour();
     }

    float contour_area = pcl::calculatePolygonArea(*contour);

    return contour_area;
}

float Plane::getRectangleArea() {
    this->getMeasurements();

    float rectangle_area = width*length;

    return rectangle_area;
}

float Plane::getRectangleArea(Eigen::Matrix3f c2m) {
    this->getMeasurements(c2m);

    float rectangle_area = width*length;

    return rectangle_area;
}

float Plane::getExtent() {
    float extent = this->getContourArea()/this->getRectangleArea();

    return extent;
}

float Plane::getExtent(Eigen::Matrix3f c2m) {

    float extent = this->getContourArea()/this->getRectangleArea(c2m);

    return extent;
}

bool sortPlanesBySize(const Plane &lhs, const Plane &rhs) {
    return lhs.cloud->points.size() > rhs.cloud->points.size(); //> bigger to smaller
}

std::vector<cv::Point> Plane::projectContourPointsImg (cv::Mat img, cv::Mat K, Eigen::Affine3d T, bool absolute) {
    std::vector<cv::Point> points2D;

    if (absolute) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr contour_aux (new pcl::PointCloud<pcl::PointXYZ>());
      pcl::transformPointCloud(*contour,*contour_aux,T);

      for (size_t i = 0; i<contour_aux->points.size(); i++) {
        cv::Point point2D(int(round(K.at<double>(0,0)*contour_aux->points[i].x/contour_aux->points[i].z + K.at<double>(0,2))),int(round(K.at<double>(1,1)*contour_aux->points[i].y/contour_aux->points[i].z + K.at<double>(1,2))));
        if (point2D.x >= 0 && point2D.x < img.cols && point2D.y >= 0 && point2D.y < img.rows)
          points2D.push_back(point2D);
        else {
            if (point2D.x < 0) point2D.x = 0;
            if (point2D.x >= img.cols) point2D.x = img.cols-1;
            if (point2D.y < 0) point2D.y = 0;
            if (point2D.y >= img.rows) point2D.y = img.rows - 1;
            points2D.push_back(point2D);
        }
      }
    }
    else {
      for (size_t i = 0; i<contour->points.size(); i++) {
        cv::Point point2D(int(round(K.at<double>(0,0)*contour->points[i].x/contour->points[i].z + K.at<double>(0,2))),int(round(K.at<double>(1,1)*contour->points[i].y/contour->points[i].z + K.at<double>(1,2))));
        if (point2D.x >= 0 && point2D.x < img.cols && point2D.y >= 0 && point2D.y < img.rows)
          points2D.push_back(point2D);
        else {
            if (point2D.x < 0) point2D.x = 0;
            if (point2D.x >= img.cols) point2D.x = img.cols-1;
            if (point2D.y < 0) point2D.y = 0;
            if (point2D.y >= img.rows) point2D.y = img.rows - 1;
            points2D.push_back(point2D);
        }
      }
    }

    return points2D;
}

void Plane::plotPolyInImg(cv::Mat img, cv::Mat K, Eigen::Affine3d T, bool absolute, cv::Scalar color) {
    // Get poly
    std::vector<cv::Point> poly = projectContourPointsImg(img,K,T,absolute);

    // Plot poly
    int n_vertices[] = {int(poly.size())};
    cv::Point rook_points[1][int(poly.size())];
    for (size_t i = 0; i < poly.size(); i++)
      rook_points[0][i] = poly[i];

    const cv::Point* ppt[1] = { rook_points[0] };

    cv::fillPoly( img, ppt, n_vertices, 1, color, 8 );
}

void Plane::fixHolesConcaveHull() {
    const float voxel_size = 0.04f;
    const float min_length = 5*voxel_size;

    if (contour->points.size() < 5)
        return;

    std::vector<size_t> ids; // indexes whose distance to the following one is greater than min_length

    for (size_t i = 0; i<contour->points.size(); i++) {
        float distance;
        if (i < contour->points.size()-1) {
            distance =    sqrt(     (contour->points[i].x - contour->points[i+1].x)*(contour->points[i].x - contour->points[i+1].x) +
                                    (contour->points[i].y - contour->points[i+1].y)*(contour->points[i].y - contour->points[i+1].y) +
                                    (contour->points[i].z - contour->points[i+1].z)*(contour->points[i].z - contour->points[i+1].z)
                                );

        }
        else {
            distance =    sqrt(     (contour->points[i].x - contour->points[0].x)*(contour->points[i].x - contour->points[0].x) +
                                    (contour->points[i].y - contour->points[0].y)*(contour->points[i].y - contour->points[0].y) +
                                    (contour->points[i].z - contour->points[0].z)*(contour->points[i].z - contour->points[0].z)
                                );

        }
        if (distance > min_length)
            ids.push_back(i);
    }

    if (ids.size() == 2) {
        contour->insert(contour->points.begin()+ids[1]+1,contour->points[ids[0]+1]);
        contour->insert(contour->points.begin()+ids[1]+2,contour->points[ids[0]]);
    }
}
