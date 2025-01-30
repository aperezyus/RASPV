#include "stair/SPV_stair.h"

void SPV_stair::computeStairFlagSegments(Stair stair, std::vector<int> &phospheneFlag, int intensity)
{
  for (int Q = 1; Q<stair.vLevels.size(); Q++)
  {

    // Convex edge
    Eigen::Vector3d pt1(-stair.step_width/2,	stair.step_height*(Q-1),	stair.step_length*(Q-1));
    Eigen::Vector3d pt2(stair.step_width/2,	stair.step_height*(Q-1),	stair.step_length*(Q-1));

    Eigen::Vector3d pt1_transformed, pt2_transformed;
    pcl::transformPoint(pt1,pt1_transformed,stair.s2i);
    pcl::transformPoint(pt2,pt2_transformed,stair.s2i);

    cv::Point3d pt1_cv(pt1_transformed(0),pt1_transformed(1),pt1_transformed(2));
    cv::Point3d pt2_cv(pt2_transformed(0),pt2_transformed(1),pt2_transformed(2));
    computeSegmentCameraCoordinates(pt1_cv, pt2_cv, phospheneFlag, intensity);

    // Concave edge
    Eigen::Vector3d pt3(-stair.step_width/2,	stair.step_height*(Q-1),	stair.step_length*(Q));
    Eigen::Vector3d pt4(stair.step_width/2,	stair.step_height*(Q-1),	stair.step_length*(Q));

    Eigen::Vector3d pt3_transformed, pt4_transformed;
    pcl::transformPoint(pt3,pt3_transformed,stair.s2i);
    pcl::transformPoint(pt4,pt4_transformed,stair.s2i);

    cv::Point3d pt3_cv(pt3_transformed(0),pt3_transformed(1),pt3_transformed(2));
    cv::Point3d pt4_cv(pt4_transformed(0),pt4_transformed(1),pt4_transformed(2));
    computeSegmentCameraCoordinates(pt3_cv, pt4_cv, phospheneFlag, intensity);

    // Left edge
    computeSegmentCameraCoordinates(pt1_cv, pt3_cv, phospheneFlag, intensity);

    // Right edge
    computeSegmentCameraCoordinates(pt2_cv, pt4_cv, phospheneFlag, intensity);



  }
}
