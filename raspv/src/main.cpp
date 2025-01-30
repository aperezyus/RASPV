#include <math.h>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <time.h>

#include <chrono>  // for high_resolution_clock

#include <boost/filesystem.hpp>
#include <boost/bind.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/topic.h>

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <gazebo_msgs/LinkStates.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <raspv/BumperEvent.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

#include <pcl/PCLHeader.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/time.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "stair/visualizer_stair.h"
#include "stair/global_scene_stair.h"
#include "stair/current_scene_stair.h"
#include "SPV/map.h"
#include "stair/SPV_stair.h"
#include "stair/stair_classes.h"
#include "RGBD/RGBD.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include <sstream>
#include <iostream>
#include <ctime>





struct mainLoop
{
    mainLoop() : gscene()
    {

        c02a.setIdentity();

        // For Gazebo
        computeFloor2Absolute();

        // For SPV
        display_mode = 3; // 1 RGB, 2 depth, 3-0 SPV modes
        display_planes = false;
        display_phosphenes = true;
        display_cloud = false;
        recompute_manhattan = false;
        exit_key = false;
        save_img = false;

        count_time = false;

        bumper_state = 0;

        KEYBOARD = true;

        color_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);


    }

    ~mainLoop() {}

    void imageCallback(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth)
    {
        try {
            mat_depth = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception:  %s", e.what());
            return;
        }

        try {
            mat_rgb = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::TYPE_8UC3)->image;
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception:  %s", e.what());
            return;
        }
    }

    void segmentationCallback(const sensor_msgs::ImageConstPtr& msg_segmentation)
    {
        try {
            mat_segmentation = cv_bridge::toCvCopy(*msg_segmentation, sensor_msgs::image_encodings::TYPE_8UC1)->image;
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception:  %s", e.what());
            return;
        }

    }

    void keyCallback(const std_msgs::String::ConstPtr& msg)
    {
        std::string last_key_pressed = msg->data;
        teleop_key_pressed = last_key_pressed[0];
    }

    void bumperCallback(const raspv::BumperEventConstPtr msg)
    {
        if (msg->state == raspv::BumperEvent::PRESSED)
            bumper_state = 1;
        else
            bumper_state = 0;
    }

    void cloudCallback(const sensor_msgs::PointCloud2 &cloud_msg)
    {
        pcl::fromROSMsg(cloud_msg,*color_cloud);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
      current_path_ = msg->poses;
    }

    void goalCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
    {
      if (!msg->status_list.empty())
        status_ = msg->status_list[0].status;
    }

    bool setOdom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
      gscene.computeCamera2AbsoluteMatrix();
    }

    void startMainLoop(int argc, char* argv[])
    {

        ros::init(argc, argv, "main_node");
        ros::NodeHandle nh;
        
        //////// Parse launch cfg ////////

        bool MODE_RGBD = true; if (nh.hasParam("/SPV_node/MODE_RGBD")) nh.getParam("/SPV_node/MODE_RGBD",MODE_RGBD);
        bool MODE_GAZEBO = false; if (nh.hasParam("/SPV_node/MODE_GAZEBO")) nh.getParam("/SPV_node/MODE_GAZEBO",MODE_GAZEBO);
        bool MODE_SLAM = false; if (nh.hasParam("/SPV_node/MODE_SLAM")) nh.getParam("/SPV_node/MODE_SLAM",MODE_SLAM);
        bool MODE_STAIR = false; if (nh.hasParam("/SPV_node/MODE_STAIR")) nh.getParam("/SPV_node/MODE_STAIR",MODE_STAIR);
        bool MODE_NAVIGATION = false; if (nh.hasParam("/SPV_node/MODE_NAVIGATION")) nh.getParam("/SPV_node/MODE_NAVIGATION",MODE_NAVIGATION);
        // bool MODE_DOOR = false; if (nh.hasParam("/SPV_node/MODE_DOOR")) nh.getParam("/SPV_node/MODE_DOOR",MODE_DOOR);
        ASSISTANT_MODE = 0; if (nh.hasParam("/SPV_node/ASSISTANT_MODE")) nh.getParam("/SPV_node/ASSISTANT_MODE",ASSISTANT_MODE);
        int prev_ASSISTANT_MODE = -1;
        std::string SAVE_FOLDER = "/home/aleks"; if (nh.hasParam("/SPV_node/SAVE_IMG_PATH")) nh.getParam("/SPV_node/SAVE_IMG_PATH",SAVE_FOLDER);

        //////// Subscribers/Publishers ////////

        // Subscribe to camera. Even if MODE_RGBD = 0, a camera is assumed. For simplicity, an RGBD camera is used, but only the images and not the clouds.
        message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
        message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
        boost::shared_ptr<sensor_msgs::Image const> sharedPtr_img;
        sharedPtr_img  = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_raw", ros::Duration(5));
        if (sharedPtr_img == nullptr) {
            ROS_INFO("Could not find RGB subscribers");
        }
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy_imgs;
        message_filters::Synchronizer<MySyncPolicy_imgs> sync_imgs(MySyncPolicy_imgs(10), rgb_sub, depth_sub);
        sync_imgs.registerCallback(boost::bind(&mainLoop::imageCallback, this, _1, _2));

        // Subscribe to Point Cloud
        ros::Subscriber cloud_sub = nh.subscribe("/camera/depth_registered/points", 1, &mainLoop::cloudCallback, this);
        boost::shared_ptr<sensor_msgs::PointCloud2 const> sharedPtr;
        sharedPtr  = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points", ros::Duration(5));
        if (sharedPtr == nullptr) {
            ROS_INFO("Could not find cloud subscribers");
            // MODE_RGBD = false;
        }

        // Subscribe to key topic (from blindbot_teleop_key)
        ros::Subscriber key_sub = nh.subscribe("/key", 1, &mainLoop::keyCallback, this);
        boost::shared_ptr<std_msgs::String const> key_sharedPtr;
        key_sharedPtr  = ros::topic::waitForMessage<std_msgs::String>("/key", ros::Duration(1));
        if (key_sharedPtr == nullptr) {
            ROS_INFO("Could not find /key subscribers");
            KEYBOARD = false; // If blindbot_teleop_key is not launched, it won't use this topic
        }

        // Subscribe to bumper
        ros::Subscriber bumper_sub = nh.subscribe("/mobile_base/events/bumper", 1, &mainLoop::bumperCallback, this);
        // Publisher (of phopspheneFlags for using in HMD)
        ros::Publisher chatter_pub = nh.advertise<std_msgs::Int16MultiArray>("chatter", spv.XiCam.cols); 
        // Subscribe to semantic segmentation result
        ros::Subscriber sub = nh.subscribe("/sem", 1, &mainLoop::segmentationCallback, this);

        //////// Navigation config ////////

        navigation_mode_path = true; // true for path, false for compass
        // Odometry frame is different with Gazebo or SLAM
        if(MODE_GAZEBO){odom_frame_ = "world";}else{odom_frame_ = "odom";}      
        // To get the ground truth pose (and other tfs)
        tf::TransformListener pose_listener;
        // if (MODE_GAZEBO)
        //     pose_listener.waitForTransform("/world", "/camera_depth_optical_frame", ros::Time::now(), ros::Duration(0.5));

        if(MODE_NAVIGATION){
          path_sub_ = nh.subscribe("/move_base/NavfnROS/plan",1000,&mainLoop::pathCallback,this);   // Subscribe to the plan path
          goal_sub_ = nh.subscribe("/move_base/status",1000,&mainLoop::goalCallback,this); // Subscribe to status (1/true if there is a goal)
        }
        // set_odom_srv_ = nh.advertiseService("set_odom",&mainLoop::setOdom,this);
        oTc0_pub_ = nh.advertise<geometry_msgs::Transform>("/odom_C0_tf",1); // Publish current pose of the initial camera pose to /odomm
        old_oTc0_pub_ = nh.advertise<geometry_msgs::Transform>("/old_odom_C0_tf",1); // Publish previous pose of the initial camera pose to /odomm
        goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1); // Publish current goal
        // // Get pose from camera link to optical frame
        // pose_listener.waitForTransform("/camera_link", "/camera_depth_optical_frame", ros::Time::now(), ros::Duration(0.5));
        // try{
        //   pose_listener.lookupTransform("camera_link", "camera_depth_optical_frame",  ros::Time(0), cTcopt_);
        // }catch (tf::TransformException ex){
        //   ROS_ERROR("%s",ex.what());
        //   ros::Duration(1.0).sleep();
        // }

        //////// SPV ////////

        // Define spv object
        spv = SPV_stair(nh);

        // Window to show phosphene visualization
        cv::namedWindow("Display window", cv::WINDOW_NORMAL);
        mat_rgb=cv::Mat::zeros(480, 640, CV_8UC3);
        mat_segmentation=cv::Mat::zeros(480, 640, CV_8U);
        cv::Mat phospheneImg = cv::Mat_<cv::Vec3b>::zeros(spv.size_img_y, spv.size_img_x);
        cv::imshow("Display window", phospheneImg);
        cv::waitKey(10);


        //////// RESULTS FILE ////////

        std::vector<double> vTimes;
        std::ofstream file_;
        file_.open("/home/aleks/catkin_ws/src/RASPV_code/raspv/results.txt"); // create a file

        //////// ROS ////////

        capture_.reset(new ros::AsyncSpinner(0));
        capture_->start();
        ros::Rate r(100);

        //////// MAIN LOOP ////////

        while (nh.ok() && !exit_key)
        {
            // Record start time
            auto start = std::chrono::high_resolution_clock::now();

            cv::Mat img_to_plot;

            if (display_mode == 3) // That is, phosphene visualization
            {
                // Image where the phosphenes will be displayed
                phospheneImg = cv::Mat_<cv::Vec3b>::zeros(spv.size_img_y, spv.size_img_x);
                // Flags that will tell, for each phosphene, whether they must be turn on or off, and what intensity
                std::vector<int> phospheneFlag(size_t(spv.XiCam.cols), 0);
                std::vector<int> phospheneFlag_highlight(size_t(spv.XiCam.cols), 0); // To mark which phosphenes to "highlight"

                ros::Time tiempo = ros::Time::now();

                if (spv.mode == 4 && !MODE_NAVIGATION) // That is merely downsampling
                {
                    spv.visualizeColor(mat_rgb, phospheneFlag);
                }
                else
                {
                    if (MODE_GAZEBO){
                        try
                        {
                            pose_listener.lookupTransform("world", "camera_depth_optical_frame",  ros::Time(0), tf_gt_);

                            tf::transformTFToEigen(tf_gt_,c2w);
                            // std::cout << c2w.translation()[0] << " " << c2w.translation()[1] << std::endl;

                            if(MODE_NAVIGATION)
                                pose_listener.lookupTransform("world", "camera_link",  ros::Time(0), tf_cL2w_);

                        }
                        catch (tf::TransformException ex)
                        {
                            ROS_ERROR("%s",ex.what());
                            ros::Duration(1.0).sleep(); 
                        }

                        gscene.c2f = f2a.inverse()*c2w;
                        gscene.c2a = c2w;

                        gscene.has_manhattan_ = true; gscene.initial_floor_ = true; gscene.initial_reference_ = true;


                    }
                    // TODO: else

                    // Get cloud
                    pcl::copyPointCloud(*color_cloud,*cloud);
                    std::vector<int> indices;
                    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

                    // For plucker computations, get absolute transformations
                    cv::Mat G = spv.getPluckerRotation(c2w);
                    cv::Mat XiAbs = G * spv.XiCam;

                    if (cloud->points.size() > 0) // If there is a cloud
                    {
                        CurrentSceneStair scene(cloud,0.04f); // Defines current scene and filters the cloud 


                        if (MODE_GAZEBO)
                            scene.filterFloorAndCeilingTransform(gscene.c2f, 0.1f, 1.80f, "y");
                        // TODO: else

                        scene.getNormalsNeighbors(16);
                        scene.segmentScene();

                        if (gscene.initial_floor_)
                        {
                            scene.getCentroidsCoeffs2Floor(gscene.c2f);
                            scene.classifyPlanes();
                        }
                        else
                        {
                            std::cout << "NO FLOOR" << std::endl;
                        }

                        if (display_cloud) // Display PCL viewer
                        {
                            if (!viewer_created)
                            {
                                viewer_created = true;
                                viewer = new ViewerStair();
                            }

                            viewer->cloud_viewer_.removeAllPointClouds();
                            viewer->cloud_viewer_.removeAllShapes();
                            viewer->drawPlaneTypesContour(scene.vPlanes);
                            viewer->drawCloudsRandom(scene.vObstacles);
                            viewer->drawColorCloud(color_cloud,1);

                            viewer->cloud_viewer_.spinOnce();
                        }

                        if (gscene.initial_reference_)
                            scene.transformPlanesAndObstacles(c2w);


                        //// Base phosphene visualization ////
                        // spv.mode == 1 : Chess-Floor
                        // spv.mode == 2 : Wall-Obstacles
                        // spv.mode == 3 : Chess-Floor + Wall-Obstacles
                        // spv.mode == 4 : Grayscale Downsampling
                        // spv.mode == 5 : Depthmap
                        // spv.mode == 6 : Depthmap (no floor)
                        // spv.mode == 7 : Canny
                        // spv.mode == 8 : Canny depth

						if (spv.mode == 1 && (gscene.initial_reference_ || (!MODE_SLAM && gscene.initial_floor_)))
						{      std::vector<cv::Point2f> final_vertices = map.getPolygon(c2w, scene.vPlanes, scene.vObstacles2f);
							   spv.visualizeChessFloor(final_vertices, XiAbs, G, c2w, phospheneFlag);    }
						else if (spv.mode == 2 && (gscene.initial_reference_ || (!MODE_SLAM && gscene.initial_floor_)))
						{
							spv.visualizeWallObstacles(scene.vPlanes, scene.vObstacles, XiAbs, c2w, phospheneFlag, true);
						}
						else if (spv.mode == 3 && (gscene.initial_reference_ || (!MODE_SLAM && gscene.initial_floor_)))
						{      std::vector<cv::Point2f> final_vertices = map.getPolygon(c2w, scene.vPlanes, scene.vObstacles2f);
							   spv.visualizeWallObstaclesChessFloor(final_vertices, scene.vPlanes, scene.vObstacles, XiAbs, G, c2w, phospheneFlag, true);    }
						if (spv.mode == 4) {
							spv.visualizeColor(mat_rgb, phospheneFlag);
						}
						else if (spv.mode == 5) {
							spv.visualizeDepth(mat_depth, phospheneFlag);
						}
						else if (spv.mode == 6 && gscene.initial_floor_)
						{      spv.visualizeDepthAugmented(mat_depth, phospheneFlag, c2w);    }
						else if (spv.mode == 7) {
							spv.visualizeCanny(mat_rgb, phospheneFlag);
						}
						else if (spv.mode == 8) {
							spv.visualizeCannyDepth(mat_depth, phospheneFlag);
						}
                        // else if (spv.mode == 9){
						// 	spv.visualizeSegmentation(mat_rgb, mat_segmentation, phospheneFlag);
						// }
					}
                }

                if (MODE_NAVIGATION)
                {
                    bool plot_goal_ = false;
                    // For plucker computations, get absolute transformations
                    cv::Mat G = spv.getPluckerRotation(c2w);
                    cv::Mat XiAbs = G * spv.XiCam;

                    // Save odometry in file
                    file_ << c2w.translation()[0] << " " << c2w.translation()[1] << " " << bumper_state << " " << tiempo << std::endl;


                    double goal_x = 0.0; double goal_y = 0.0; double goal_yaw = 0.0; 
                    double goal_width = 0.0; double goal_height = 0.0; 

                    if (ASSISTANT_MODE == 1) // Kitchen
                    {
                        if (nh.hasParam("/SPV_node/goal_x_1")) nh.getParam("/SPV_node/goal_x_1",goal_x);
                        if (nh.hasParam("/SPV_node/goal_y_1")) nh.getParam("/SPV_node/goal_y_1",goal_y);
                        if (nh.hasParam("/SPV_node/goal_yaw_1")) nh.getParam("/SPV_node/goal_yaw_1",goal_yaw); // 0 if aligned in X, M_PI/2 otherwise
                        if (nh.hasParam("/SPV_node/goal_width_1")) nh.getParam("/SPV_node/goal_width_1",goal_width);
                        if (nh.hasParam("/SPV_node/goal_height_1")) nh.getParam("/SPV_node/goal_height_1",goal_height);
                        plot_goal_ = spv.plotDoor(mat_depth, goal_x, goal_y, goal_yaw, goal_width, goal_height, c2w, G, phospheneFlag_highlight);

                    }
                    else if (ASSISTANT_MODE == 2) // Bathroom
                    {
                        if (nh.hasParam("/SPV_node/goal_x_2")) nh.getParam("/SPV_node/goal_x_2",goal_x);
                        if (nh.hasParam("/SPV_node/goal_y_2")) nh.getParam("/SPV_node/goal_y_2",goal_y);
                        if (nh.hasParam("/SPV_node/goal_yaw_2")) nh.getParam("/SPV_node/goal_yaw_2",goal_yaw); // 0 if aligned in X, M_PI/2 otherwise
                        if (nh.hasParam("/SPV_node/goal_width_2")) nh.getParam("/SPV_node/goal_width_2",goal_width);
                        if (nh.hasParam("/SPV_node/goal_height_2")) nh.getParam("/SPV_node/goal_height_2",goal_height);
                        plot_goal_ = spv.plotDoor(mat_depth, goal_x, goal_y, goal_yaw, goal_width, goal_height, c2w, G, phospheneFlag_highlight);

                    }
                    else if (ASSISTANT_MODE == 3) // Bedroom
                    {
                        if (nh.hasParam("/SPV_node/goal_x_3")) nh.getParam("/SPV_node/goal_x_3",goal_x);
                        if (nh.hasParam("/SPV_node/goal_y_3")) nh.getParam("/SPV_node/goal_y_3",goal_y);
                        if (nh.hasParam("/SPV_node/goal_yaw_3")) nh.getParam("/SPV_node/goal_yaw_3",goal_yaw); // 0 if aligned in X, M_PI/2 otherwise
                        if (nh.hasParam("/SPV_node/goal_width_3")) nh.getParam("/SPV_node/goal_width_3",goal_width);
                        if (nh.hasParam("/SPV_node/goal_height_3")) nh.getParam("/SPV_node/goal_height_3",goal_height);
                        plot_goal_ = spv.plotDoor(mat_depth, goal_x, goal_y, goal_yaw, goal_width, goal_height, c2w, G, phospheneFlag_highlight);

                    }
                    else if (ASSISTANT_MODE == 4) // Table
                    {
                    
                        if (nh.hasParam("/SPV_node/goal_x_4")) nh.getParam("/SPV_node/goal_x_4",goal_x);
                        if (nh.hasParam("/SPV_node/goal_y_4")) nh.getParam("/SPV_node/goal_y_4",goal_y);
                        if (nh.hasParam("/SPV_node/goal_yaw_4")) nh.getParam("/SPV_node/goal_yaw_4",goal_yaw); // 0 if aligned in X, M_PI/2 otherwise
                        if (nh.hasParam("/SPV_node/goal_width_4")) nh.getParam("/SPV_node/goal_width_4",goal_width);
                        if (nh.hasParam("/SPV_node/goal_height_4")) nh.getParam("/SPV_node/goal_height_4",goal_height);
                        plot_goal_ = spv.plotObject(mat_depth, goal_x, goal_y, goal_yaw, goal_width, goal_height, c2w, G, phospheneFlag_highlight, XiAbs);

                    }
                    else if (ASSISTANT_MODE == 5) // Office chair
                    {
                    
                        if (nh.hasParam("/SPV_node/goal_x_5")) nh.getParam("/SPV_node/goal_x_5",goal_x);
                        if (nh.hasParam("/SPV_node/goal_y_5")) nh.getParam("/SPV_node/goal_y_5",goal_y);
                        if (nh.hasParam("/SPV_node/goal_yaw_5")) nh.getParam("/SPV_node/goal_yaw_5",goal_yaw); // 0 if aligned in X, M_PI/2 otherwise
                        if (nh.hasParam("/SPV_node/goal_width_5")) nh.getParam("/SPV_node/goal_width_5",goal_width);
                        if (nh.hasParam("/SPV_node/goal_height_5")) nh.getParam("/SPV_node/goal_height_5",goal_height);
                        plot_goal_ = spv.plotObject(mat_depth, goal_x, goal_y, goal_yaw, goal_width, goal_height, c2w, G, phospheneFlag_highlight, XiAbs);

                    }
                    else if (ASSISTANT_MODE == 6) // Bed
                    {
                    
                        if (nh.hasParam("/SPV_node/goal_x_6")) nh.getParam("/SPV_node/goal_x_6",goal_x);
                        if (nh.hasParam("/SPV_node/goal_y_6")) nh.getParam("/SPV_node/goal_y_6",goal_y);
                        if (nh.hasParam("/SPV_node/goal_yaw_6")) nh.getParam("/SPV_node/goal_yaw_6",goal_yaw); // 0 if aligned in X, M_PI/2 otherwise
                        if (nh.hasParam("/SPV_node/goal_width_6")) nh.getParam("/SPV_node/goal_width_6",goal_width);
                        if (nh.hasParam("/SPV_node/goal_height_6")) nh.getParam("/SPV_node/goal_height_6",goal_height);
                        plot_goal_ = spv.plotObject(mat_depth, goal_x, goal_y, goal_yaw, goal_width, goal_height, c2w, G, phospheneFlag_highlight, XiAbs);

                    }



                    // Check proximity to goal
                    double distance = sqrt(pow(goal_x - c2w.translation()[0],2) + pow(goal_y - c2w.translation()[1],2));

                    // Publish pose goal to navigation package
                    // if (prev_ASSISTANT_MODE != ASSISTANT_MODE || prev_ASSISTANT_MODE == -1){

                        geometry_msgs::PoseStamped pose_goal;
                        pose_goal.header.frame_id="map";
                        pose_goal.header.stamp = ros::Time::now();
                        pose_goal.pose.position.x = goal_x;
                        pose_goal.pose.position.y = goal_y;
                        pose_goal.pose.position.z = 0;
                        // pose_goal.pose.orientation.w = 1.0;
                        pose_goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_yaw);
                        goal_pub.publish(pose_goal);

                        prev_ASSISTANT_MODE = ASSISTANT_MODE;
                    // }


                    bool plot_path_ = false;
                    if(status_ && MODE_NAVIGATION && navigation_mode_path)
                        plot_path_ = spv.plotPathOnImage(current_path_,XiAbs,c2w, phospheneFlag_highlight, mat_depth);
                    else if (status_ && MODE_NAVIGATION && !navigation_mode_path)
                        spv.updateDirArrow(current_path_,tf_cL2w_,phospheneFlag_highlight);

                    if (MODE_NAVIGATION && (plot_path_ || plot_goal_))
                    {
                        spv.dimPhosphenes(phospheneFlag,0.5);

                        for (int k = 0; k<phospheneFlag_highlight.size(); k++)
                            if (phospheneFlag_highlight[k] > 0)
                            phospheneFlag[k] = spv.N_levels;
                    }
                }

                

                if (spv.segmentation_mode)
                {

                    spv.visualizeOnlySegmentation(mat_segmentation, phospheneFlag_highlight);

                    spv.dimPhosphenes(phospheneFlag,0.5);

                    for (int k = 0; k<phospheneFlag_highlight.size(); k++)
                      if (phospheneFlag_highlight[k] > 0)
                        phospheneFlag[k] = spv.N_levels;
                }


                
                spv.visualize(phospheneFlag, phospheneImg);
                phospheneImg.copyTo(img_to_plot);

				//Send data to the chatter topic (new)
                std::vector<int> phospheneFlag_msg; 
                phospheneFlag_msg = phospheneFlag;
                int N_fos = phospheneFlag_msg.size();
                phospheneFlag_msg.insert(phospheneFlag_msg.begin(), N_fos);
                phospheneFlag_msg.insert(phospheneFlag_msg.begin(), 8);

				std_msgs::Int16MultiArray msg;

				msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
				msg.layout.dim[0].size =phospheneFlag_msg.size();
				msg.layout.dim[0].stride = 1;
				msg.layout.dim[0].label = "x";
				msg.data.clear();
				msg.data.insert(msg.data.end(), phospheneFlag_msg.begin(), phospheneFlag_msg.end());
				chatter_pub.publish(msg);

                
            }
            else if (display_mode == 1) // Show RGB
            {
                mat_rgb.copyTo(img_to_plot);
                if (display_phosphenes)
                    displayPhosphenesPos(img_to_plot, spv.rgbPos, spv.K_c.at<double>(0,0), spv.FOV_sprite);
            }
            else if (display_mode == 2) // Show Depthmap scaled
            {
                scaleDepth(mat_depth, img_to_plot);
                if (display_phosphenes)
                    displayPhosphenesPos(img_to_plot, spv.depthPos, spv.K_d.at<double>(0,0), spv.FOV_sprite);
            }

            // If display_planes is active, show them overlaid
            if (display_planes && MODE_RGBD)
                displayPlanes(img_to_plot, color_cloud, spv, display_mode);


            // Save image
            if (save_img) { saveImg(img_to_plot, SAVE_FOLDER); save_img = false; }
            // Show image
            cv::namedWindow("Display window", cv::WINDOW_NORMAL);
            cv::imshow("Display window", img_to_plot);

            // Grab keyboard button press and perform action
            char key_phosphenes = char(cv::waitKey(10));
            if (KEYBOARD)  { key_phosphenes = teleop_key_pressed; }
            keyAction(key_phosphenes, map.debug_mode);
            teleop_key_pressed = '\0';


            // Record end time
            auto finish = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = finish - start;
            if (count_time)
            {
                // std::cout << "Elapsed time: " << elapsed.count() << " s\n";
                vTimes.push_back(elapsed.count());
            }

            r.sleep(); // end loop
        }
        file_.close(); // once you create a file you need to close it
        capture_->stop();

        double average = std::accumulate( vTimes.begin(), vTimes.end(), 0.0)/vTimes.size();

        std::cout << vTimes[0] << std::endl;


    }

    void saveImg(cv::Mat img, std::string folder)
    {
        char filename[40];
        struct tm *timenow;

        time_t now = time(nullptr);
        timenow = gmtime(&now);

        strftime(filename, sizeof(filename), "%Y-%m-%d_%H:%M:%S", timenow);

        std::string filename_str(filename);
        filename_str = folder + filename_str + ".jpg";

        cv::imwrite(filename_str, img);

        std::cout << "Image saved as " << filename_str << std::endl;


    }

    void displayPhosphenesPos(cv::Mat &img_to_plot, cv::Mat phosphenesPos, double f, double FOV_sprite)
    {
        int radius = int(2*f*tan(FOV_sprite/2)/2);

        for (int k = 0; k < phosphenesPos.cols; k++)
        {
          int iIndex = int(round(phosphenesPos.at<double>(1, k)));
          int jIndex = int(round(phosphenesPos.at<double>(0, k)));

          if (iIndex < 0 || iIndex >= img_to_plot.rows || jIndex < 0 || jIndex >= img_to_plot.cols )
              continue;

          cv::circle(img_to_plot, cv::Point(jIndex,iIndex), radius, cv::Scalar(0,255,0));
        }
    }

    void displayPlanes(cv::Mat &img_to_plot, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &color_cloud, SPV spv, int display_mode)
    {

        pcl::copyPointCloud(*color_cloud,*cloud);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        if (cloud->points.size() > 0)
        {
            CurrentScene scene_temp(cloud,0.04f);
            scene_temp.getNormalsNeighbors(4);
            scene_temp.regionGrowing();
            scene_temp.getContours();

            for (size_t p = 0; p < scene_temp.vPlanes.size(); p++)
            {
                if (display_mode == 1)
                    scene_temp.vPlanes[p].plotPolyInImg(img_to_plot, spv.K_c, Eigen::Affine3d::Identity(), false, cv::Scalar(double(rand() % 255), double(rand() % 255), double(rand() % 255)));
                else if (display_mode == 2)
                    scene_temp.vPlanes[p].plotPolyInImg(img_to_plot, spv.K_d, Eigen::Affine3d::Identity(), false, cv::Scalar(double(rand() % 255), double(rand() % 255), double(rand() % 255)));
                else if (display_mode == 3)
                    scene_temp.vPlanes[p].plotPolyInImg(img_to_plot, spv.K_p, Eigen::Affine3d::Identity(), false, cv::Scalar(double(rand() % 255), double(rand() % 255), double(rand() % 255)));

            }
        }
    }

    void computeFloor2Absolute(){
        f2a = Eigen::Matrix4d::Zero(); // this matrix allow to change reference with y upwards (rgbd processing) to z upwards (gazebo)
        f2a(1, 0) = 1;    f2a(2, 1) = 1;    f2a(0, 2) = 1;    f2a(3, 3) = 1;
    }


    void keyAction(char key, bool &set_debug)
    {
    //   std::cout << key << std::endl;
      switch (key)
      {
          case 27: // Esc
            exit_key = true;
            break;
          case 32: // Spacebar
             save_img = true;
          break;

          // OTHER
          case 'i':
            recompute_manhattan = true;
            break;
          case 'm':
            set_debug = true;
            cv::namedWindow("map_debug", cv::WINDOW_NORMAL);
            break;
          case 'b':
            display_cloud = !display_cloud;
            std::cout << "Toggle PCL live visualization" << std::endl;
//            if (display_cloud == 0)
//              display_cloud = 1;
            break;
          case 'l':
            navigation_mode_path = !navigation_mode_path;
          break;

          // CHOOSE VISUALIZATION
          case '1':
            display_mode = 1;
            std::cout << "Visualizing RGB" << std::endl;
            break;
          case '2':
            display_mode = 2;
            std::cout << "Visualizing Depth" << std::endl;
            break;
          case 'p':
            display_planes = !display_planes;
            std::cout << "Toggle overlay planes in visualization" << std::endl;
            break;
          case 96: // `
            display_phosphenes = !display_phosphenes;
            std::cout << "Toggle phosphenes overlay in color/depth" << std::endl;
            break;
        //   case 'k':
            // count_time = true;
        //   case 186: // º
          case 'k': // º
            ASSISTANT_MODE += 1;
            if (ASSISTANT_MODE > 3)
                ASSISTANT_MODE = 1;

          default:
            if ((key == '0' || key == '3' || key == '4' || key == '5' || key == '6' || key == '7' || key == '8' || key == '9'))
            {
                display_mode = 3;
                std::cout << "Visualizing Phosphenes" << std::endl;

            }
            spv.keyAction(key);
      }


    }


    // ROS/RGB-D
    boost::mutex data_ready_mutex_;
    boost::condition_variable data_ready_cond_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud;
    cv::Mat mat_rgb;
    cv::Mat mat_depth;
    cv::Mat mat_segmentation;
    boost::shared_ptr<ros::AsyncSpinner> capture_;


    // RGB-D
    ViewerStair *viewer;
    GlobalSceneStair gscene;

    //Navigation
    ros::Subscriber path_sub_;      //Navigation path subscriber
    ros::Subscriber goal_sub_;    //Navigation status
    ros::Publisher oTc0_pub_;
    ros::Publisher old_oTc0_pub_;
    // ros::ServiceServer get_floor_srv_;
    // ros::ServiceServer set_odom_srv_;
    std::vector<geometry_msgs::PoseStamped> current_path_;  //Current Path
    int status_;                    //Navigation status
    bool navigation_mode_path;          //Navigation representation mode
    std::string odom_frame_;        //Odom frame name
    bool count_time;

    ros::Publisher goal_pub;
    bool current_goal_door;

    // Gazebo
//    ros::Publisher vel_pub;
    ros::Publisher head_pub;
    std_msgs::Float64 msg_head;
//    geometry_msgs::Twist msg_vel;

    // Transformations
    Eigen::Affine3d c02a;
    Eigen::Affine3d c2w; // Camera to World: in this case, to floor reference (oriented with Manhattan) at first iteration, kept with odometry
    Eigen::Affine3d f2a; // Fix references: Reference in RGB-D computations has y upwards, in SPV has z upwards.
//    Eigen::Affine3d w2c0; // World coordinates to Camera coordinates at first iteration (C0)
    Eigen::Affine3d c2c0; // Camera (current) to Camera (initial) --> odometry!
//    Eigen::Affine3d optical_frame2camera_frame; // Transformation from camera optical frame to camera link (odometry in gazebo comes from camera link to world)
    tf::StampedTransform tf_gt_;
    tf::StampedTransform tf_cL2w_;
    // tf::StampedTransform cTcopt_;
    //tf::TransformBroadcaster odom_broadcaster_;


    // SPV
//    bool absolute; // True when all computations all done in woorld coordinates (e.g. given by gazebo)
    int display_mode; // 1 fosfenos, 2 RGB, 3 Depth, 4 lookUpTable
    bool display_planes; // If true, overlays planes in display window
    bool display_phosphenes; // If true, overlays of phosphenes in display window (in color or depth visualization)
    bool display_cloud; // If true, plot stuff in PCL viewer
    bool viewer_created; // To track if the PCL viewer has been created
    bool recompute_manhattan; // If true, all Manhattan computations are performed (i.e. they are not kept by odometry)
    bool exit_key;
    bool save_img;

    char teleop_key_pressed;

    int bumper_state;


    Map map;
    SPV_stair spv;

    bool KEYBOARD;
    int ASSISTANT_MODE;

};


int main(int argc, char* argv[])
{

    std::cout << "KEYBOARD SETTINGS" << std::endl << "(click when focus is on Display Window or keyboard_teleop)" << std::endl << std::endl;

    std::cout << "---- VISUALIZATION ----" << std::endl;
    std::cout << "1 - Visualize RGB" << std::endl;
    std::cout << "2 - Visualize Depthmap" << std::endl << std::endl;

    std::cout << "-- Phosphene patterns --" << std::endl;
    std::cout << "3 - Visualize phosphenes: Chess-Floor mode" << std::endl;
    std::cout << "4 - Visualize phosphenes: Wall-Obstacles mode" << std::endl;
    std::cout << "5 - Visualize phosphenes: Wall-Obstacles-Chess-Floor mode" << std::endl;
    std::cout << "6 - Visualize phosphenes: Black and white image intensity mode" << std::endl;
    std::cout << "7 - Visualize phosphenes: Depth mode" << std::endl;
    std::cout << "8 - Visualize phosphenes: Augmented depth mode" << std::endl;
    std::cout << "9 - Visualize phosphenes: Canny mode" << std::endl << std::endl;

    std::cout << "-- SPV Configuration --" << std::endl;
    std::cout << "*// - Increase/decrease size of phosphene" << std::endl;
    std::cout << "+/- - Increase/decrease number of phosphenes" << std::endl;
    std::cout << "v/V - Increase/decrease field of view" << std::endl;
    std::cout << "n/N - Increase/decrease noise of phosphene position" << std::endl;
    std::cout << "o/O - Increase/decrease dropout ratio" << std::endl;
    std::cout << "l/L - Increase/decrease number of levels of intensity of phosphenes" << std::endl;
    std::cout << " .  - Toggle grid mode: hexagonal/rectangular" << std::endl;
    std::cout << "j/J - Visualization modifier:" << std::endl;
    std::cout << "     if Chess-Floor or Wall-Obstacles-Chess-Floor, increase/decrease size of tiles of chessboard" << std::endl;
    std::cout << "     if Depth or augmented depth, increase/decrease max depth" << std::endl << std::endl;

    std::cout << "---- other ----" << std::endl;
    std::cout << "p - Toggle overlay planes in visualization" << std::endl;
    std::cout << "` - Toggle overlay phosphene positions in visualization (color or depth)" << std::endl;
    std::cout << "b - Display Point Cloud with segmented planes/obstacles" << std::endl << std::endl;
    std::cout << "i - Force recompute Manhattan directions" << std::endl;
    std::cout << "m - Map debug mode (opens 2D map window)" << std::endl;
    std::cout << "space - Save a screenshot of current Display window" << std::endl << std::endl;


    mainLoop app;

    try
    {
        app.startMainLoop(argc, argv);
    }
    catch (const std::bad_alloc& /*e*/)
    {
        cout << "Bad alloc" << endl;
    }
    catch (const std::exception& /*e*/)
    {
        cout << "Exception" << endl;
    }

    return 0;
}
