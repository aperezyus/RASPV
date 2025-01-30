//#include <math.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "RGBD/plane.h"

#define COLOR_RED  cv::Scalar(0,0,255)
#define COLOR_BLACK  cv::Scalar(0,0,0)
#define COLOR_WHITE  cv::Scalar(255,255,255)
#define COLOR_BLUE  cv::Scalar(255,0,0)
#define COLOR_GREEN  cv::Scalar(0,255,0)
#define COLOR_YELLOW  cv::Scalar(0,255,255)
#define COLOR_PINK  cv::Scalar(200,0,255)

struct Vertex
{
    cv::Point2f point2f;
    cv::Point point;
    float angle;
    Eigen::Vector2f dir;
    float dist;
//    bool extended;
    bool BB_int;
    int BB;
    bool extreme;
};

struct BoundingBox
{
    int index;
    std::vector<std::vector<cv::Point2f> > segments;
    std::vector<std::vector<cv::Point> > segments_map;
};

class Map
{
//public:

    // initialized at start
    int img_size; // Size of the img map size
    float map_size; // Size of the map. in meters
    std::string default_map_filename;
    bool plot_map;
    float FOV_h_deg;
    float FOV_v_deg;

    // initialized during execution
    cv::Point2f robot_pos;
    cv::Point robot_pos_img;
    cv::Point old_robot_pos_img;
    cv::Point2f old_robot_pos;
    Eigen::Vector3f v_front;
    std::vector<cv::Point2f> frustrum;
    std::vector<cv::Point> frustrum_img;
    std::vector<std::vector<cv::Point2f> > segments_frustrum_square;
    std::vector<std::vector<cv::Point2f> > segments_frustrum_rays;


public:
    cv::Mat map;
    cv::Mat map_to_plot;
    bool debug_mode;


    Map() : img_size(1001), map_size(10.0), default_map_filename("map.jpg"), plot_map(false), debug_mode(false), FOV_h_deg(57.0), FOV_v_deg(45.0)
    {
        createMap();
        map_to_plot = map.clone();

        if (plot_map)
            cv::namedWindow("map",cv::WINDOW_NORMAL);
        if (debug_mode)
          cv::namedWindow("map_debug",cv::WINDOW_NORMAL);

        frustrum.resize(5);
        frustrum_img.resize(5);
        segments_frustrum_square.resize(4);
        segments_frustrum_rays.resize(4);
    }

    Map(int img_size_in, float map_size_in, std::string default_map_filename_in, bool plot_map_in) :
        img_size(img_size_in), map_size(map_size_in), default_map_filename(default_map_filename_in), plot_map(plot_map_in), debug_mode(false), FOV_h_deg(57.0), FOV_v_deg(45.0)
    {
        createMap();
        map_to_plot = map.clone();

        if (plot_map)
            cv::namedWindow("map",cv::WINDOW_NORMAL);
//        if (debug_mode)
//          cv::namedWindow("map_debug",cv::WINDOW_NORMAL);

        frustrum.resize(5);
        frustrum_img.resize(5);
        segments_frustrum_square.resize(4);
        segments_frustrum_rays.resize(4);

    }

    ~Map()
    {
      if (plot_map)
        cv::destroyWindow("map");
      if (debug_mode)
        cv::destroyWindow("map_debug");
    }

    void setPlot(bool activated){plot_map = activated;}
    void setDebug(bool activated){debug_mode = activated;}

    void createMap();
    void getRobotPosition(Eigen::Affine3f T);
    cv::Point getImgPoint(cv::Point2f point);
    cv::Point getImgPoint(pcl::PointXYZ point);
    void plotCloudInMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Scalar color);
    void plotPoseIncrement();
    void getFrustrum(Eigen::Affine3f T);
    Vertex createVertexFromPos(cv::Point2f point2f);
    float angleRelativePose(cv::Point2f pos);
    float angleRelativePose(cv::Point2f pos, Eigen::Vector2f vertex_dir);
    std::vector<cv::Point2f> getVerticesOrConvexHull2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
    cv::Mat getPoints2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    BoundingBox BBFromVertices(std::vector<cv::Point2f> vertices2D, int index);
    bool pointsOnSegment(cv::Point2f p, cv::Point2f q, cv::Point2f r);
    int pointOrientation(cv::Point2f p, cv::Point2f q, cv::Point2f r);
    bool pointsIntersect(cv::Point2f p1, cv::Point2f q1, cv::Point2f p2, cv::Point2f q2);
    bool pointsIntersect(std::vector<cv::Point2f> v1, std::vector<cv::Point2f> v2);
    cv::Point2f getIntersectionPoint(std::vector<cv::Point2f> v1, std::vector<cv::Point2f> v2);
    bool getIntersectionPoint(std::vector<cv::Point2f> v1, std::vector<cv::Point2f> v2, cv::Point2f & intersection_point);
    void getBBSegmentsMap(BoundingBox &BB);
    void plotSegmentInMap(std::vector<cv::Point> segment, cv::Scalar color);
    void plotBBInMap(BoundingBox BB, cv::Scalar color);
    void plotVisibilityLine(cv::Point p, cv::Scalar color);
    void plotFrustrum();
    void plotPoint(cv::Point p, cv::Scalar color);
    void plotPoint(Vertex vertex, cv::Scalar color);
    void plotPoly(std::vector<Vertex> poly, cv::Scalar color);
    void plotDebugAndStop();
    void plotDebug();
    std::vector<cv::Point2f> getPolygon(Eigen::Affine3d T, std::vector<Plane> vPlanes, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vObstacles);

};
