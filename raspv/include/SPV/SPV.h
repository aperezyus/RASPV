#ifndef PHOSPHENE_FUNCTIONS_H
#define PHOSPHENE_FUNCTIONS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>


#include "RGBD/plane.h"
#include "RGBD/RGBD.h"


#include <iostream>


class SPV
{

public:

  //// PARAMETERS OF THE PROSTHETIC SYSTEM
  double FOV;             // Field of View of the system (in deg)
  int N_fos;              // Number of phosphene (attempted, actual number depends on size of images and distribution)
  int grid_mode;          // How are the phopshenes distributed: 1 = rectangular, 2 = hexagonal
  int map_mode;           // The size of the area of the phosphene map: 1 = rectangular, 2 = circular
  double noise_stddev;    // Noise can be added to the position of the phosphenes (visualization only, rays emanate from initial distribution map)
  double dropout;         // Percentage of phosphenes that are randomly turned off
  double FOV_sprite;      // Field of view of the sprite size. That is, the diameter in angles with respect to visual FOV
  int N_levels;           // Number of levels of intensity allowed by the system

  //// Phosphene Image configuration
  cv::Mat K_c;            // Calibration matrix K of the color camera
  int size_img_x_c;         // Resolution in pixels of the color image in x
  int size_img_y_c;         // Resolution in pixels of the color image in y
  cv::Mat K_d;            // Calibration matrix K of the depth camera
  int size_img_x_d;         // Resolution in pixels of the depth image in x
  int size_img_y_d;         // Resolution in pixels of the depth image in y
  cv::Mat K_p;            // Calibration matrix K from the virtual phosphene camera
  double x0;              // Optical center of the camera (x coordinate)
  double y0;              // Optical center of the camera (y coordinate)
  double f;               // Focal length of the virtual phosphene camera
  int size_img_x;         // Resolution in pixels of the phosphene image in x
  int size_img_y;         // Resolution in pixels of the phosphene image in y

  //// Phosphene matrix configuration
  int deltaX;             // Separation between phosphenes in X
  int deltaY;             // Separation between phosphenes in Y
  double rExt;            // Radius inside which the phosphenes will be displayed
  cv::Mat phosphenesPos;  // The actual pixel position of each phosphene
  cv::Mat phosphenesPos_vis; // The actual pixel position of each phosphene for visualization purposes
  cv::Mat rgbPos;         // The corresponding position of each phosphene in the RGB image
  cv::Mat depthPos;       // The corresponding position of each phosphene in the Depth img
  cv::Mat XiCam;          // Rays emanating from each phosphene into the world
  cv::Mat lookUpTable;  // Labeled cv::Mat, with the size of the phosphene image, where each pixel encodes the value of the closest phosphene in the matrix

  //// Phosphene sprite configuration
  double size_sprite;        // Size of the sprite in pixels
  cv::Mat reference_sprite;  // We built the phosphene sprite once, to be then resized and then vary intensities for each level
  std::vector<cv::Mat> sprites; // Vector of sprites already resized and with varying intensities. Size of the vector = N_levels

  //// Visualization modes (parameters of the system)
  int mode;             // To switch among visualization modes.
  bool segmentation_mode;
  double thDist;        // Distance in pixels to consider that a phosphene has to be light up
  double dChessX;       // In chess-floor representation, square size in X direction
  double dChessY;       // In chess-floor representation, square size in X direction
  double max_depth;     // In depth and depth augmented visualization, maximum value of depth to be displayed and scaled. Controls the range of depth visualization

  //Number of phosphenes columns and rows
  int nPointsX_;
  int nPointsY_;
  //arrow_pos
  int col_up;
  int col2_up;
  int col_down;
  int col2_down;
  //arrow_center_pos
  int col_up_center;
  int col2_up_center;
  int col_down_center;
  int col2_down_center;
  //Index and yaw angle of phosphenes at the second row
  std::vector<int> indexes;
  std::vector<double> yaws;

  SPV() {}

  SPV(ros::NodeHandle &nh)
  {
      // Load the parameters of SPV
      loadParametersYAML(nh);


      bool CAMERA_FROM_TOPIC = false;
      if (nh.hasParam("/SPV_node/CAMERA_FROM_TOPIC"))
          nh.getParam("/SPV_node/CAMERA_FROM_TOPIC",CAMERA_FROM_TOPIC);

      bool SPV_FROM_CAMERA = false;
      if (nh.hasParam("/SPV_node/SPV_FROM_CAMERA"))
          nh.getParam("/SPV_node/SPV_FROM_CAMERA",SPV_FROM_CAMERA);


      if (CAMERA_FROM_TOPIC)
      {
          sensor_msgs::CameraInfo camera_info_rgb, camera_info_depth;

          /* Defining the calibration of the camera */
          std::string camera_rgb_topic = "/camera/rgb/camera_info";
          boost::shared_ptr<sensor_msgs::CameraInfo const> sharedCameraInfo_rgb;
          sharedCameraInfo_rgb = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_rgb_topic, nh);
          if (sharedCameraInfo_rgb != nullptr)
              camera_info_rgb = *sharedCameraInfo_rgb;

          std::string camera_depth_topic = "/camera/depth_registered/camera_info";
          boost::shared_ptr<sensor_msgs::CameraInfo const> sharedCameraInfo_depth;
          sharedCameraInfo_depth = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_depth_topic, nh);
          if (sharedCameraInfo_depth != nullptr)
              camera_info_depth = *sharedCameraInfo_depth;

          loadCameraFromTopics(camera_info_rgb,camera_info_depth);

      }
      else
          loadDefaultCamera(nh);

      if (SPV_FROM_CAMERA)
          useCameraParameters(K_c, size_img_x_c, size_img_y_c);
      else
          useParametersFromFOV();

      initPhosphenesPos();
      initSprites();

      segmentation_mode = false;
  }

  void useCameraParameters(cv::Mat K_c, int size_img_x_c, int size_img_y_c)
  {
      // OPTION 1- Similar to color camera
      size_img_x = int(size_img_x_c);
      size_img_y = int(size_img_y_c);
      K_p = K_c;
      x0 = K_p.at<double>(0, 2);
      y0 = K_p.at<double>(1, 2);
      f = (K_p.at<double>(0, 0) + K_p.at<double>(1, 1))/2;
      FOV = getFOVfromFocalLength(f, size_img_x);
  }

  void useParametersFromFOV()
  {
      f = getFocalLengthFromFOV(FOV, size_img_x);
      x0 = double(size_img_x) / 2.0 - 0.5;
      y0 = double(size_img_y) / 2.0 - 0.5;
      K_p = cv::Mat_<double>::eye(3, 3);
      K_p.at<double>(0, 0) = f;
      K_p.at<double>(1, 1) = f;
      K_p.at<double>(0, 2) = x0;
      K_p.at<double>(1, 2) = y0;
  }

  void loadCameraFromTopics(sensor_msgs::CameraInfo camera_info_rgb, sensor_msgs::CameraInfo camera_info_depth)
  {
      size_img_x_c = int(camera_info_rgb.width);
      size_img_y_c = int(camera_info_rgb.height);
      K_c = loadCalibrationMatrixfromCameraInfo(camera_info_rgb);
      size_img_x_d = int(camera_info_depth.width);
      size_img_y_d = int(camera_info_depth.height);
      K_d = loadCalibrationMatrixfromCameraInfo(camera_info_depth);
  }

  void loadCameraFromTopics(sensor_msgs::CameraInfo camera_info)
  {
      size_img_x_c = int(camera_info.width);
      size_img_y_c = int(camera_info.height);
      K_c = loadCalibrationMatrixfromCameraInfo(camera_info);
  }

  void loadDefaultCamera(ros::NodeHandle &nh){
      int size_img_x_c, size_img_y_c;
      double x0_c, y0_c, fx_c, fy_c;

      // Device parameters
      if (nh.hasParam("/SPV_node/rgb_camera/size_img_x"))
          nh.getParam("/SPV_node/rgb_camera/size_img_x", size_img_x_c);
      if (nh.hasParam("/SPV_node/rgb_camera/size_img_y"))
          nh.getParam("/SPV_node/rgb_camera/size_img_y", size_img_y_c);
      if (nh.hasParam("/SPV_node/rgb_camera/x0"))
          nh.getParam("/SPV_node/rgb_camera/x0", x0_c);
      if (nh.hasParam("/SPV_node/rgb_camera/y0"))
          nh.getParam("/SPV_node/rgb_camera/y0", y0_c);
      if (nh.hasParam("/SPV_node/rgb_camera/fx"))
          nh.getParam("/SPV_node/rgb_camera/fx", fx_c);
      if (nh.hasParam("/SPV_node/rgb_camera/fy"))
          nh.getParam("/SPV_node/rgb_camera/fy", fy_c);

      K_c = cv::Mat_<double>::eye(3, 3);
      K_c.at<double>(0, 0) = fx_c;
      K_c.at<double>(1, 1) = fy_c;
      K_c.at<double>(0, 2) = x0_c;
      K_c.at<double>(1, 2) = y0_c;

      // Depth camera configuration (calibration matrix K_d)
      int size_img_x_d, size_img_y_d;
      double x0_d, y0_d, fx_d, fy_d;

      if (nh.hasParam("/SPV_node/depth_camera/size_img_x"))
          nh.getParam("/SPV_node/depth_camera/size_img_x", size_img_x_d);
      if (nh.hasParam("/SPV_node/depth_camera/size_img_y"))
          nh.getParam("/SPV_node/depth_camera/size_img_y", size_img_y_d);
      if (nh.hasParam("/SPV_node/depth_camera/x0"))
          nh.getParam("/SPV_node/depth_camera/x0", x0_d);
      if (nh.hasParam("/SPV_node/depth_camera/y0"))
          nh.getParam("/SPV_node/depth_camera/y0", y0_d);
      if (nh.hasParam("/SPV_node/depth_camera/fx"))
          nh.getParam("/SPV_node/depth_camera/fx", fx_d);
      if (nh.hasParam("/SPV_node/depth_camera/fy"))
          nh.getParam("/SPV_node/depth_camera/fy", fy_d);

      K_d = cv::Mat_<double>::eye(3, 3);
      K_d.at<double>(0, 0) = fx_d;
      K_d.at<double>(1, 1) = fy_d;
      K_d.at<double>(0, 2) = x0_d;
      K_d.at<double>(1, 2) = y0_d;

  }

  cv::Mat loadCalibrationMatrixfromCameraInfo(sensor_msgs::CameraInfo camera_info){

      boost::array<double, 9> K;
      K = camera_info.K;

      double fx =  K[0];
      double fy = K[4];
      double cx = K[2];
      double cy = K[5];

      cv::Mat K_out = cv::Mat_<double>::eye(3, 3);

      K_out.at<double>(0, 0) = fx;
      K_out.at<double>(1, 1) = fy;
      K_out.at<double>(0, 2) = cx;
      K_out.at<double>(1, 2) = cy;

//      std::cout << K_out << std::endl;


      return K_out;

  }

  void loadParametersYAML(ros::NodeHandle &nh){

      // Display settings
      if (nh.hasParam("/SPV_node/display/size_img_x"))
          nh.getParam("/SPV_node/display/size_img_x", size_img_x);
      else
          size_img_x = 800;

      if (nh.hasParam("/SPV_node/display/size_img_y"))
          nh.getParam("/SPV_node/display/size_img_y", size_img_y);
      else
          size_img_y = 800;

      // Device parameters

      if (nh.hasParam("/SPV_node/device/N_phosphenes"))
          nh.getParam("/SPV_node/device/N_phosphenes", N_fos);
      else
          N_fos = 1000;

      double FOV_deg;
      if (nh.hasParam("/SPV_node/device/FOV_deg"))
          nh.getParam("/SPV_node/device/FOV_deg", FOV_deg);
      else
          FOV_deg = 20.0;
      FOV = FOV_deg*M_PI/180.0;

      if (nh.hasParam("/SPV_node/device/map_mode"))
          nh.getParam("/SPV_node/device/map_mode", map_mode);
      else
          map_mode = 1;

      if (nh.hasParam("/SPV_node/device/grid_mode"))
          nh.getParam("/SPV_node/device/grid_mode", grid_mode);
      else
          grid_mode = 2;

      if (nh.hasParam("/SPV_node/device/noise_stddev"))
          nh.getParam("/SPV_node/device/noise_stddev", noise_stddev);
      else
          noise_stddev = 0.0;

      if (nh.hasParam("/SPV_node/device/dropout_rate"))
          nh.getParam("/SPV_node/device/dropout_rate", dropout);
      else
          dropout = 0.0;

      double FOV_sprite_deg;
      if (nh.hasParam("/SPV_node/device/FOV_phosphene_deg"))
          nh.getParam("/SPV_node/device/FOV_phosphene_deg", FOV_sprite_deg);
      else
          FOV_sprite_deg = 0.5;
      FOV_sprite = FOV_sprite_deg*M_PI/180.0;

      if (nh.hasParam("/SPV_node/device/N_levels"))
          nh.getParam("/SPV_node/device/N_levels", N_levels);
      else
          N_levels = 7;

      if (nh.hasParam("/SPV_node/device/aspect_ratio"))
      {
          double aspect_ratio;
          nh.getParam("/SPV_node/device/aspect_ratio", aspect_ratio);
          size_img_y = int(double(size_img_y)*aspect_ratio);
      }


      // Representation parameters

      if (nh.hasParam("/SPV_node/representation/mode"))
          nh.getParam("/SPV_node/representation/mode", mode);
      else
          mode = 1;

      if (nh.hasParam("/SPV_node/representation/dChessX"))
          nh.getParam("/SPV_node/representation/dChessX", dChessX);
      else
          dChessX = 0.5;

      if (nh.hasParam("/SPV_node/representation/dChessY"))
          nh.getParam("/SPV_node/representation/dChessY", dChessY);
      else
          dChessY = 0.5;

      if (nh.hasParam("/SPV_node/representation/max_depth"))
          nh.getParam("/SPV_node/representation/max_depth", max_depth);
      else
          max_depth = 3.0;





      std::cout << "------ INITIAL SETTINGS OF SPV ------" << std::endl;
      std::cout << "Field of view: " << FOV_deg << " degrees" << std::endl;
      std::cout << "Number of phosphenes: " << N_fos << " phosphenes" << std::endl;
      if (grid_mode == 2)
        std::cout << "Hexagonal grid mode" << std::endl;
      else if (grid_mode == 1)
         std::cout << "Rectangular grid mode" << std::endl;
      if (map_mode == 2)
        std::cout << "Circular map mode" << std::endl;
      else if (map_mode == 1)
         std::cout << "Rectangular map mode" << std::endl;
      std::cout << "Noise std. dev.: " << noise_stddev << std::endl;
      std::cout << "Dropout ratio: " << dropout << std::endl;
      std::cout << "Size of the phosphenes: " << FOV_sprite_deg << " degrees" << std::endl;
      std::cout << "Levels of intensity: " << N_levels << " levels" << std::endl;
      std::cout << "Size of screen image: " << size_img_x << "x" << size_img_y << " pixels" << std::endl;
  }



  void initPhosphenesPos(){
      computeDeltaFromNumberOfPhosphenes(deltaX,deltaY);
      thDist = (deltaX - 1) / 2;

      if (map_mode == 2)
        rExt = std::min(size_img_x/2, size_img_y/2);
      else
        rExt = sqrt(x0*x0+y0*y0); // so big that it does not affect

      genPhosphenesPos();
      applyDropout();
      lookUpTable = genLookUpTable();


  }

  void initSprites(){
      // Phosphene sprite configuration
      double value_border = 0.01;    // The sprite is a 2D gaussian that ends when per-pixel value is value_border
      double visual_factor = 1.5; // If we make value of the border so small (so it looks good when overlap), the visual of the phosphene does not extend all the FOV_sprite. We add this factor to compensate for that. Experimental threshold.
      size_sprite = int(getPixelLengthFromFOV(FOV_sprite*visual_factor, f));


      double sigma = sqrt(-pow((size_sprite-1)/2,2)/(2*log(value_border)));

      reference_sprite = cv::Mat_<cv::Vec3b>::zeros(size_sprite, size_sprite);
      reference_sprite.setTo(cv::Scalar(255, 255, 255));
      genPhoshenSprite(reference_sprite, sigma); // 3.5


      cv::Mat sprite;
      cv::resize(reference_sprite,sprite,cv::Size(size_sprite,size_sprite));

      for (size_t i = 0; i <= N_levels; i++)
      {
        cv::Mat sprite_aux = sprite.clone();
        double dim_factor = double(i)/double(N_levels);
        sprites.push_back(sprite_aux * dim_factor);
      }
      //NAVIGATION
      col_up = round(nPointsX_/2)-1;
      col_up_center = col_up;

      col2_up = round(nPointsX_ + (nPointsX_)/2 -3);
      col2_up_center = col2_up;

      col_down = phosphenesPos.cols - nPointsX_/2;
      col_down_center = col_down;

      col2_down = round(phosphenesPos.cols - nPointsX_ - nPointsX_/2+1);
      col2_down_center = col2_down;
  }

  double getFOVfromFocalLength(double f, double length){
      return 2*atan((length)/(2*f));
  }

  double getPixelLengthFromFOV(double FOV, double f){
      return 2*f*tan(FOV/2);
  }

  double getFocalLengthFromFOV(double FOV, double length){
      return length/(2*tan(FOV/2));
  }

  void computeDeltaFromNumberOfPhosphenes(int &dX, int &dY);
  void genPhosphenesPos();
  void updatePhosphenes();
  void changeDelta(int delta_change);
  void changeDelta(int deltaX_change, int deltaY_change);
  void genPhoshenSprite(cv::Mat &sprite, double sigma);
  void updateSprite(double factor = 1.0);
  cv::Mat genLookUpTable();
  void applyDropout();
  cv::Mat transformCoordinates(cv::Mat &R, cv::Mat &o, cv::Mat &t, double s);
  void computeVanishingLines(Eigen::Affine3d T, cv::Mat G, std::vector<int> & phospheneFlag, int intensity);
  void computeLinePlucker(cv::Mat LCam, std::vector<int> & phospheneFlag, int intensity);
  void computeSegmentPlucker(cv::Point3d X1, cv::Point3d X2, cv::Mat LCam, std::vector<int> & phospheneFlag, std::vector<double> &phospheneDepth, int intensity);
  void computeSegmentCameraCoordinates(cv::Point3d X1, cv::Point3d X2, std::vector<int> &phospheneFlag, int intensity);
  void computeSegmentGlobalCoordinates(cv::Point3d X1, cv::Point3d X2, Eigen::Affine3d T, cv::Mat G, std::vector<int> &phospheneFlag, std::vector<double> &phospheneDepth, int intensity);
  void computeSegmentGlobalCoordinates(std::vector<cv::Point3d> points, Eigen::Affine3d T, cv::Mat G, std::vector<int> &phospheneFlag, std::vector<double> &phospheneDepth, int intensity);
  cv::Mat twoLinesClosestPoint(cv::Mat L, cv::Mat M);
  cv::Mat getPluckerRotation(Eigen::Affine3d T);
  cv::Mat twoPointsSpan(double x0, cv::Mat x, double y0, cv::Mat y);

  void computePlanePolygonIntersection(cv::Mat XiAbs, Plane plane, std::vector<int> & chessFlag, std::vector<double> &phospheneDepth, Eigen::Affine3d T, int intensity);
  void computeFlagObstacles(pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle, cv::Mat lookUpTable, cv::Mat K_d, std::vector<int> & obstacleFlag, int intensity);

  void computeXYChessPatternPolygonConvHull(cv::Mat & XiAbs, std::vector<cv::Point2f> layout2D, double d, double dChessX, double dChessY, std::vector<int> & chessFlag, int intensity_1, int intensity_2);
  void computeFloorPolygonEdges(std::vector<cv::Point2f> points, Eigen::Affine3d T, cv::Mat G, std::vector<int> &phospheneFlag, int intensity);
  void computeChessFlagConvexHull(cv::Mat XiAbs, std::vector<cv::Point2f> layout2D, cv::Mat sidesL, double d, double dChessX, double dChessY, std::vector<int> & chessFlag, int intensity_1, int intensity_2);
  void computeChessFlag(cv::Mat XiAbs, std::vector<cv::Point2f> layout2D, std::vector<int> & chessFlag, int intensity_1, int intensity_2);
  void computeFloorFlag(cv::Mat XiAbs, std::vector<cv::Point2f> layout2D, std::vector<int> & chessFlag, int intensity);



  cv::Mat pointsToPluckerPolygon(std::vector<cv::Point2f> poly);
  cv::Mat pointsToPluckerPolygon(std::vector<int> hullIndex, cv::Mat XLayout);


  cv::Mat normalizeL(cv::Mat L);





  int isFaceSingleRayIntersectionMat(cv::Mat & sidesL, cv::Mat &Xi);

  template<typename T>
  T twoLinesSide4(cv::Mat G, cv::Mat H);

  cv::Mat lineXYPlaneMeetMat(double d, cv::Mat l, cv::Mat lBar);
  cv::Mat linePlaneMeet(double u0, cv::Mat u, cv::Mat l, cv::Mat lBar);

  void plotSprite(cv::Mat &img, cv::Mat& sprite, int xIni, int yIni);

  void visualizeChessFloor(std::vector<cv::Point2f> polygon, cv::Mat XiAbs, cv::Mat G, Eigen::Affine3d T, std::vector<int> & phospheneFlag);
  void visualizeWallObstacles(std::vector<Plane> vPlanes, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vObstacles, cv::Mat XiAbs, Eigen::Affine3d T, std::vector<int> & phospheneFlag, bool absolute);
  void visualizeWallObstaclesChessFloor(std::vector<cv::Point2f> polygon, std::vector<Plane> vPlanes, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vObstacles, cv::Mat XiAbs,  cv::Mat G, Eigen::Affine3d T, std::vector<int> & phospheneFlag, bool absolute);
  void visualizeColor(cv::Mat img, std::vector<int> & phospheneFlag);
  void visualizeDepth(cv::Mat img, std::vector<int> & phospheneFlag);
  void visualizeDepthAugmented(cv::Mat img, std::vector<int> & phospheneFlag, Eigen::Affine3d T);
  void visualizeCanny(cv::Mat img, std::vector<int> & phospheneFlag);
  void visualizeCannyDepth(cv::Mat img, std::vector<int> & phospheneFlag);
  void visualize(std::vector<int> phospheneFlag, cv::Mat &img);
  void visualizeSegmentation(cv::Mat img, cv::Mat img_segmentation, std::vector<int> & phospheneFlag);
  void visualizeOnlySegmentation(cv::Mat img_segmentation, std::vector<int> & phospheneFlag);

  void keyAction(char key);

  void dimPhosphenes(std::vector<int> & phospheneFlag, double factor);

  //navigation functions
  void plotPoint(std::vector<geometry_msgs::PoseStamped> &path, Eigen::Affine3d T, std::vector<int> &phospheneFlag, int intensity);
  bool plotPointWithDepth(std::vector<geometry_msgs::PoseStamped> &path, Eigen::Affine3d T, std::vector<int> &phospheneFlag, int intensity, cv::Mat depth_img);
  bool plotPathOnImage(std::vector<geometry_msgs::PoseStamped> &path,cv::Mat XiAbs, Eigen::Affine3d T, std::vector<int> & phospheneFlag, cv::Mat depth_img);
  void updateDirArrow(std::vector<geometry_msgs::PoseStamped> &path, tf::StampedTransform T, std::vector<int> & phospheneFlag);
  void plotArrow(std::vector<int> & phospheneFlag, int intensity);
  int selectTrajectoryPoint(std::vector<geometry_msgs::PoseStamped> &path, double thold);

  bool plotDoor(cv::Mat depth_img, double door_center_x, double door_center_y, double door_yaw, double door_width, double door_height, Eigen::Affine3d T, cv::Mat G, std::vector<int> & phospheneFlag);
  bool plotObject(cv::Mat depth_img, double object_center_x, double object_center_y, double object_yaw, double object_width, double object_height, Eigen::Affine3d T, cv::Mat G, std::vector<int> & phospheneFlag, cv::Mat XiAbs);

};

#endif
