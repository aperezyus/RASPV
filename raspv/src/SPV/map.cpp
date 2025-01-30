#include "SPV/map.h"

bool sortVertices(const Vertex &v1, const Vertex &v2){return v1.angle < v2.angle;}

void Map::createMap()
{
//  map = cv::imread(default_map_filename, cv::IMREAD_COLOR);
//  if (!map.data)
//  {
    map = cv::Mat(img_size,img_size,CV_8UC3,COLOR_WHITE);

//    cv::Point center((img_size-1)/2,(img_size-1)/2);
//    cv::Point center_east(center.x+(img_size-1)/10,center.y);
//    cv::Point center_north(center.x,center.y-(img_size-1)/10);
//    cv::line(map, center, center_east, COLOR_RED,2);
//    cv::line(map, center, center_north, COLOR_GREEN,2);
//  }
//  else
//  {
//    if (map.cols != img_size)
//    {
//      cv::Mat map_aux;
//      cv::resize(map, map_aux, cv::Size(img_size,img_size));
//      map = map_aux.clone();
//    }
//  }
}


void Map::getRobotPosition(Eigen::Affine3f T)
{
  old_robot_pos = robot_pos;
  old_robot_pos_img = robot_pos_img;
  robot_pos = cv::Point2f(T.translation()[0], T.translation()[1]);
  robot_pos_img = getImgPoint(robot_pos);

  // TODO: verify if bool f(cv::Point &p); works to check if cv::Point is empty
  if (!old_robot_pos.x)
  {
    old_robot_pos = robot_pos;
    old_robot_pos_img = robot_pos_img;
  }

  v_front = T.rotation().col(2).head<3>().normalized();
}


cv::Point Map::getImgPoint(cv::Point2f point)
{
  cv::Point point_img;
  point_img.x = int( (point.x*float(img_size-1))/map_size + float(img_size-1)/2);
  point_img.y = int( -(point.y*float(img_size-1))/map_size + float(img_size-1)/2);

  return point_img;
}


cv::Point Map::getImgPoint(pcl::PointXYZ point)
{
  cv::Point point_img;
  point_img.x = int( (point.x*float(img_size-1))/map_size + float(img_size-1)/2);
  point_img.y = int( -(point.y*float(img_size-1))/map_size + float(img_size-1)/2);

  return point_img;
}

void Map::plotCloudInMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Scalar color)
{
  for (int i = 0; i < cloud->points.size(); i++)
  {
    cv::Point obstacle = getImgPoint(cloud->points[i]);
    map_to_plot.at<cv::Vec3b>(obstacle.y,obstacle.x) = cv::Vec3b(color[0],color[1],color[2]);
  }
}

void Map::plotPoseIncrement()
{
  cv::line(map,old_robot_pos_img,robot_pos_img,COLOR_BLUE,1);
}

void Map::getFrustrum(Eigen::Affine3f T)
{

  float FOV_h = fmod((FOV_h_deg),360) * M_PI /180;
  float FOV_v = fmod((FOV_v_deg),360) * M_PI /180;

  Eigen::Vector3f v_front_cam(0.0,0.0,1.0);

  Eigen::Matrix3f rot_pos_x;
  rot_pos_x << 1, 0, 0, 0, cos(FOV_v/2), -sin(FOV_v/2), 0, sin(FOV_v/2), cos(FOV_v/2);
  Eigen::Matrix3f rot_neg_x;
  rot_neg_x << 1, 0, 0, 0, cos(-FOV_v/2), -sin(-FOV_v/2), 0, sin(-FOV_v/2), cos(-FOV_v/2);
  Eigen::Matrix3f rot_neg_y;
  rot_neg_y << cos(FOV_h/2), 0,  sin(FOV_h/2), 0, 1, 0, -sin(FOV_h/2),0, cos(FOV_h/2);
  Eigen::Matrix3f rot_pos_y;
  rot_pos_y << cos(-FOV_h/2), 0,  sin(-FOV_h/2), 0, 1, 0, -sin(-FOV_h/2),0, cos(-FOV_h/2);

  Eigen::Vector3f v_front_1_3d = T.rotation()*rot_neg_x*rot_neg_y*v_front_cam; //v_front_1_3d.normalize();
  Eigen::Vector3f v_front_2_3d = T.rotation()*rot_pos_x*rot_neg_y*v_front_cam; //v_front_2_3d.normalize();
  Eigen::Vector3f v_front_3_3d = T.rotation()*rot_pos_x*rot_pos_y*v_front_cam; //v_front_3_3d.normalize();
  Eigen::Vector3f v_front_4_3d = T.rotation()*rot_neg_x*rot_pos_y*v_front_cam; //v_front_4_3d.normalize();

  const float distance = 10.0; // distance of ray

  Eigen::Vector3f translation = T.translation();
  Eigen::Vector3f v_distance_1_3d = v_front_1_3d*distance + translation;
  Eigen::Vector3f v_distance_2_3d = v_front_2_3d*distance + translation;
  Eigen::Vector3f v_distance_3_3d = v_front_3_3d*distance + translation;
  Eigen::Vector3f v_distance_4_3d = v_front_4_3d*distance + translation;

  cv::Point2f p_1_m, p_2_m, p_3_m, p_4_m;

  if (v_distance_1_3d[2] < 0)
  {
    p_1_m.x = translation[0]-translation[2]/v_front_1_3d[2]*v_front_1_3d[0];
    p_1_m.y = translation[1]-translation[2]/v_front_1_3d[2]*v_front_1_3d[1];
  }
  else    {p_1_m.x = v_distance_1_3d[0]; p_1_m.y = v_distance_1_3d[1]; }
  if (v_distance_2_3d[2] < 0)
  {
    p_2_m.x = translation[0]-translation[2]/v_front_2_3d[2]*v_front_2_3d[0];
    p_2_m.y = translation[1]-translation[2]/v_front_2_3d[2]*v_front_2_3d[1];
  }
  else    {p_2_m.x = v_distance_2_3d[0]; p_2_m.y = v_distance_2_3d[1]; }
  if (v_distance_3_3d[2] < 0)
  {
    p_3_m.x = translation[0]-translation[2]/v_front_3_3d[2]*v_front_3_3d[0];
    p_3_m.y = translation[1]-translation[2]/v_front_3_3d[2]*v_front_3_3d[1];
  }
  else    {p_3_m.x = v_distance_3_3d[0]; p_3_m.y = v_distance_3_3d[1]; }
  if (v_distance_4_3d[2] < 0)
  {
    p_4_m.x = translation[0]-translation[2]/v_front_4_3d[2]*v_front_4_3d[0];
    p_4_m.y = translation[1]-translation[2]/v_front_4_3d[2]*v_front_4_3d[1];
  }
  else    {p_4_m.x = v_distance_4_3d[0]; p_4_m.y = v_distance_4_3d[1]; }


      cv::Point p_1 = getImgPoint(p_1_m);
      cv::Point p_2 = getImgPoint(p_2_m);
      cv::Point p_3 = getImgPoint(p_3_m);
      cv::Point p_4 = getImgPoint(p_4_m);

  std::vector< cv::Point2f > segment_1; segment_1.push_back(robot_pos); segment_1.push_back(p_1_m);
  std::vector< cv::Point2f > segment_2; segment_2.push_back(robot_pos); segment_2.push_back(p_2_m);
  std::vector< cv::Point2f > segment_3; segment_3.push_back(robot_pos); segment_3.push_back(p_3_m);
  std::vector< cv::Point2f > segment_4; segment_4.push_back(robot_pos); segment_4.push_back(p_4_m);
  std::vector< cv::Point2f > segment_12; segment_12.push_back(p_1_m); segment_12.push_back(p_2_m);
  std::vector< cv::Point2f > segment_23; segment_23.push_back(p_2_m); segment_23.push_back(p_3_m);
  std::vector< cv::Point2f > segment_34; segment_34.push_back(p_3_m); segment_34.push_back(p_4_m);
  std::vector< cv::Point2f > segment_41; segment_41.push_back(p_4_m); segment_41.push_back(p_1_m);


  frustrum[0] = p_1_m; frustrum[1] = p_2_m; frustrum[2] = p_3_m; frustrum[3] = p_4_m; frustrum[4] = robot_pos;
  frustrum_img[0] = p_1; frustrum_img[1] = p_2; frustrum_img[2] = p_3; frustrum_img[3] = p_4; frustrum_img[4] = robot_pos_img;
  segments_frustrum_rays[0] = segment_1; segments_frustrum_rays[1] = segment_2; segments_frustrum_rays[2] = segment_3; segments_frustrum_rays[3] = segment_4;
  segments_frustrum_square[0] = segment_12; segments_frustrum_square[1] = segment_23; segments_frustrum_square[2] = segment_34; segments_frustrum_square[3] = segment_41;

}

float Map::angleRelativePose(cv::Point2f pos)
{
  Eigen::Vector2f vertex_dir = Eigen::Vector2f(pos.x-robot_pos.x,pos.y-robot_pos.y).normalized();
  float angle = asin(vertex_dir(0)*v_front(1) - vertex_dir(1)*v_front(0))*180/M_PI+90.0f;

//  float angle = asin((pos.x-robot_pos.x)*v_front(1) - (pos.y-robot_pos.y)*v_front(1))*180.0f/M_PI+90.0f;

  return angle;
}

float Map::angleRelativePose(cv::Point2f pos, Eigen::Vector2f vertex_dir)
{
  float angle = asin(vertex_dir(0)*v_front(1) - vertex_dir(1)*v_front(0))*180/M_PI+90.0f;

//  float angle = asin((pos.x-robot_pos.x)*v_front(1) - (pos.y-robot_pos.y)*v_front(1))*180.0f/M_PI+90.0f;

  return angle;
}

Vertex Map::createVertexFromPos(cv::Point2f pos)
{
  Eigen::Vector2f vertex_dir = Eigen::Vector2f(pos.x-robot_pos.x,pos.y-robot_pos.y).normalized();
  float angle = angleRelativePose(pos,vertex_dir);//asin(vertex_dir(0)*v_front(1) - vertex_dir(1)*v_front(0))*180/M_PI+90;
  float dist = cv::norm(robot_pos-pos);
  Vertex vertex = {pos,    // point2f
                   getImgPoint(pos),  //point
                   angle, // angle
                   vertex_dir, //dir
                   dist, // dist
                   false, // BB_int
                   0, // BB
                   false}; // extreme

  return vertex;
}

std::vector<cv::Point2f> Map::getVerticesOrConvexHull2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
  cv::Mat points2D;
  points2D = cv::Mat(cloud_in->points.size(),2,CV_32FC1,cv::Scalar(0.0));

  for (int i = 0; i<cloud_in->points.size(); i++)
  {
    points2D.at<float>(i,0) = cloud_in->points[i].x;
    points2D.at<float>(i,1) = cloud_in->points[i].y;
  }

  cv::PCA pca_analysis(points2D, cv::Mat(), cv::PCA::DATA_AS_ROW);
  //Store the center of the object
  cv::Point2f cntr = cv::Point2f(pca_analysis.mean.at<float>(0, 0),pca_analysis.mean.at<float>(0, 1));
  //Store the eigenvalues and eigenvectors
  std::vector<cv::Point2f> eigen_vecs(2);
  std::vector<float> eigen_val(2);
  for (int i = 0; i < 2; ++i)
  {
    eigen_vecs[i] = cv::Point2f(pca_analysis.eigenvectors.at<float>(i, 0),
                                pca_analysis.eigenvectors.at<float>(i, 1));
    eigen_val[i] = pca_analysis.eigenvalues.at<float>(0, i);
  }

  if (eigen_val[0]/eigen_val[1] > 100)
  {
    cv::Mat aux_cols = cv::Mat(1, points2D.rows, CV_32FC1, cntr.x);    // 3 cols, 4 rows
    cv::Mat aux_rows = cv::Mat(1, points2D.rows, CV_32FC1, cntr.y);
    aux_cols.push_back(aux_rows);
    cv::Mat aux = aux_cols.t();

    cv::Mat points2D_center = points2D - aux;
    cv::Mat points2D_rotated = pca_analysis.eigenvectors*points2D_center.t();

    double min_x, max_x;
    cv::minMaxLoc(points2D_rotated.row(0), &min_x, &max_x);

    cv::Mat vertices_mat = (cv::Mat_<double>(2,2) << min_x, max_x, 0.0, 0.0);
    vertices_mat.convertTo(vertices_mat, CV_32FC1);
    cv::Mat vertices_mat_desrot = pca_analysis.eigenvectors.t()*vertices_mat;

    std::vector<cv::Point2f> vertices2D_vector;

    for (int i = 0; i<2; i++)
    {
      cv::Point2f vertex = cv::Point2f(vertices_mat_desrot.at<float>(0,i) + cntr.x,vertices_mat_desrot.at<float>(1,i) + cntr.y );
      vertices2D_vector.push_back(vertex);

    }

    return vertices2D_vector;
  }
  else
  {


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud_in);
    pcl::ModelCoefficients::Ptr model_coeffs (new pcl::ModelCoefficients);
    model_coeffs->values.resize(4);

    model_coeffs->values[0] = 0.0;
    model_coeffs->values[1] = 0.0;
    model_coeffs->values[2] = 1.0;
    model_coeffs->values[3] = 0.0;
    proj.setModelCoefficients (model_coeffs);
    proj.filter (*cloud_projected);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_chull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;

    chull.setInputCloud (cloud_projected);
    chull.setComputeAreaVolume(true);
    chull.reconstruct (*cloud_chull);
    double area_chull = chull.getTotalArea();

    cv::RotatedRect rectangle = cv::minAreaRect(points2D);

    cv::Point2f vertices2D[4];
    rectangle.points(vertices2D);

    std::vector<cv::Point2f> vertices2D_vector(vertices2D, vertices2D + sizeof vertices2D / sizeof vertices2D[0]);
//    std::vector<cv::Point2d> vertices2D_vector;
//    for (size_t i=0 ; i<vertices2D_vector_f.size(); i++)
//      vertices2D_vector.push_back( cv::Point2d( (double)vertices2D_vector_f[i].x, (double)vertices2D_vector_f[i].y  ) );

    double area_rect = (double) cv::norm(vertices2D_vector[0]-vertices2D_vector[1])*cv::norm(vertices2D_vector[1]-vertices2D_vector[2]);

    if ((area_chull/area_rect > 0.9) or (area_chull < 0.1))
    {
      return vertices2D_vector;
    }
    else
    {
      // TODO approxPolyDP function para aproximar curvas con menos puntos, con el Ramer–Douglas–Peucker algorithm

      //        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reduced (new pcl::PointCloud<pcl::PointXYZ>);

      //        float leaf_size = 0.1;

      //        pcl::VoxelGrid<pcl::PointXYZ> vg;
      //        vg.setInputCloud (cloud_chull);
      //        vg.setLeafSize (leaf_size, leaf_size, leaf_size);
      //        vg.filter (*cloud_reduced);

      //        chull.setInputCloud (cloud_reduced);
      //        chull.reconstruct (*cloud_chull);

      cv::Mat points2D_chull = getPoints2D(cloud_chull);
      std::vector<cv::Point2f> vertices2D_vector_chull;
      for (int i = 0; i<points2D_chull.rows; i++)
        vertices2D_vector_chull.push_back(cv::Point2f(points2D_chull.at<float>(i,0),points2D_chull.at<float>(i,1)));


      return vertices2D_vector_chull;
    }
  }

}

cv::Mat Map::getPoints2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

  cv::Mat points2D;
  points2D = cv::Mat(cloud->points.size(),2,CV_32FC1,cv::Scalar(0.0));

  for (int i = 0; i<cloud->points.size(); i++)
  {
    points2D.at<float>(i,0) = cloud->points[i].x;
    points2D.at<float>(i,1) = cloud->points[i].y;
  }

  return points2D;
}

BoundingBox Map::BBFromVertices(std::vector<cv::Point2f> vertices2D, int index)
{
  BoundingBox BB;
  BB.index = index;

  for (int j = 0; j<vertices2D.size()-1; j++)
  {
    std::vector<cv::Point2f> segment_BB; segment_BB.push_back(vertices2D[j]); segment_BB.push_back(vertices2D[j+1]);
    BB.segments.push_back(segment_BB);
  }
  if (vertices2D.size() > 2)
  {
    std::vector<cv::Point2f> segment_BB; segment_BB.push_back(vertices2D.back()); segment_BB.push_back(vertices2D[0]);
    BB.segments.push_back(segment_BB);
  }

  return BB;
}


// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool Map::pointsOnSegment(cv::Point2f p, cv::Point2f q, cv::Point2f r)
{
  if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
      q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
    return true;

  return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int Map::pointOrientation(cv::Point2f p, cv::Point2f q, cv::Point2f r)
{
  // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
  // for details of below formula.
  float val = (q.y - p.y) * (r.x - q.x) -
      (q.x - p.x) * (r.y - q.y);

  //    std::cout << "val:" << val << std::endl;

  if (val == 0) return 0;  // colinear

  return (val > 0)? 1: 2; // clock or counterclock wise
}

// The main function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool Map::pointsIntersect(cv::Point2f p1, cv::Point2f q1, cv::Point2f p2, cv::Point2f q2)
{
  // https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
  // Find the four orientations needed for general and
  // special cases
  int o1 = pointOrientation(p1, q1, p2);
  int o2 = pointOrientation(p1, q1, q2);
  int o3 = pointOrientation(p2, q2, p1);
  int o4 = pointOrientation(p2, q2, q1);

  // General case
  if (o1 != o2 && o3 != o4)
    return true;

  // Special Cases
  // p1, q1 and p2 are colinear and p2 lies on segment p1q1
  if (o1 == 0 && pointsOnSegment(p1, p2, q1)) return true;

  // p1, q1 and q2 are colinear and q2 lies on segment p1q1
  if (o2 == 0 && pointsOnSegment(p1, q2, q1)) return true;

  // p2, q2 and p1 are colinear and p1 lies on segment p2q2
  if (o3 == 0 && pointsOnSegment(p2, p1, q2)) return true;

  // p2, q2 and q1 are colinear and q1 lies on segment p2q2
  if (o4 == 0 && pointsOnSegment(p2, q1, q2)) return true;

  return false; // Doesn't fall in any of the above cases
}

// The main function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool Map::pointsIntersect(std::vector<cv::Point2f> v1, std::vector<cv::Point2f> v2)
{

  cv::Point2f p1 = v1[0];
  cv::Point2f q1 = v1[1];
  cv::Point2f p2 = v2[0];
  cv::Point2f q2 = v2[1];

  return pointsIntersect(p1, q1, p2, q2);

}

cv::Point2f Map::getIntersectionPoint(std::vector<cv::Point2f> v1, std::vector<cv::Point2f> v2)
{
  // https://www.topcoder.com/community/data-science/data-science-tutorials/geometry-concepts-line-intersection-and-its-applications/
  cv::Point2f intersection_point;


  cv::Point2f p1v1 = v1[0];
  cv::Point2f p2v1 = v1[1];
  cv::Point2f p1v2 = v2[0];
  cv::Point2f p2v2 = v2[1];

  float A1 = p2v1.y - p1v1.y;
  float B1 = p1v1.x - p2v1.x;
  float C1 = A1*p1v1.x + B1*p1v1.y;
  float A2 = p2v2.y - p1v2.y;
  float B2 = p1v2.x - p2v2.x;
  float C2 = A2*p1v2.x + B2*p1v2.y;

  float det = A1*B2 - A2*B1;

  if (det == 0)
  {
//    std::cout << "lineas paralelas, igual sí que tienes que hacer algo" << std::endl;
  }
  else
  {
    intersection_point.x = (B2*C1-B1*C2)/det;
    intersection_point.y = (A1*C2 - A2*C1)/det;
  }

  return intersection_point;

}

bool Map::getIntersectionPoint(std::vector<cv::Point2f> v1, std::vector<cv::Point2f> v2, cv::Point2f &intersection_point)
{
  // https://www.topcoder.com/community/data-science/data-science-tutorials/geometry-concepts-line-intersection-and-its-applications/
  //    cv::Point2f intersection_point;

  bool intersects = true;

  cv::Point2f p1v1 = v1[0];
  cv::Point2f p2v1 = v1[1];
  cv::Point2f p1v2 = v2[0];
  cv::Point2f p2v2 = v2[1];

  float A1 = p2v1.y - p1v1.y;
  float B1 = p1v1.x - p2v1.x;
  float C1 = A1*p1v1.x + B1*p1v1.y;
  float A2 = p2v2.y - p1v2.y;
  float B2 = p1v2.x - p2v2.x;
  float C2 = A2*p1v2.x + B2*p1v2.y;

  float det = A1*B2 - A2*B1;
  if (det == 0)
  {
    //        std::cout << "lineas paralelas, igual sí que tienes que hacer algo" << std::endl;
    intersects = false;
  }
  else
  {
    intersection_point.x = (B2*C1-B1*C2)/det;
    intersection_point.y = (A1*C2 - A2*C1)/det;
  }

  return intersects;

}

void Map::getBBSegmentsMap(BoundingBox &BB)
{
  for (int j = 0; j<BB.segments.size(); j++)
  {
    cv::Point p1 = getImgPoint(BB.segments[j][0]);
    cv::Point p2 = getImgPoint(BB.segments[j][1]);
    std::vector<cv::Point> segment_map;
    segment_map.push_back(p1); segment_map.push_back(p2);
    BB.segments_map.push_back(segment_map);
  }
}

void Map::plotSegmentInMap(std::vector<cv::Point> segment, cv::Scalar color)
{
  for (int i = 0; i < segment.size()-1; i++)
    cv::line(map_to_plot,segment[i],segment[i+1],color,1);
}

void Map::plotBBInMap(BoundingBox BB, cv::Scalar color)
{
  if (BB.segments_map.size() < 1)
    getBBSegmentsMap(BB);

  for (int i = 0; i<BB.segments_map.size(); i++)
    plotSegmentInMap(BB.segments_map[i],color);
}

void Map::plotVisibilityLine(cv::Point p, cv::Scalar color)
{
  cv::line(map_to_plot,robot_pos_img,p,color,1);
}

void Map::plotFrustrum()
{
  cv::line(map_to_plot,frustrum_img[0],frustrum_img[1],COLOR_PINK,1);
  cv::line(map_to_plot,frustrum_img[1],frustrum_img[2],COLOR_PINK,1);
  cv::line(map_to_plot,frustrum_img[2],frustrum_img[3],COLOR_PINK,1);
  cv::line(map_to_plot,frustrum_img[3],frustrum_img[0],COLOR_PINK,1);
  plotVisibilityLine(frustrum_img[0],COLOR_PINK);
  plotVisibilityLine(frustrum_img[1],COLOR_PINK);
  plotVisibilityLine(frustrum_img[2],COLOR_PINK);
  plotVisibilityLine(frustrum_img[3],COLOR_PINK);

//  std::cout << robot_pos_img << std::endl << frustrum_img << std::endl << std::endl;
}

void Map::plotPoint(cv::Point p, cv::Scalar color)
{
  cv::circle(map_to_plot,p,2,color,1);
}

void Map::plotPoint(Vertex vertex, cv::Scalar color)
{
  cv::circle(map_to_plot,vertex.point,2,color,1);
}

void Map::plotPoly(std::vector<Vertex> poly, cv::Scalar color)
{
    int n_vertices[] = {poly.size()};
    cv::Point rook_points[1][poly.size()];
    for (int i = 0; i < poly.size(); i++)
    {
        rook_points[0][i] = poly[i].point;
    }

    const cv::Point* ppt[1] = { rook_points[0] };

    cv::fillPoly( map_to_plot, ppt, n_vertices, 1, color, 8 );
}

void Map::plotDebugAndStop()
{
  if (debug_mode)
  {
    cv::imshow("map_debug", map_to_plot);
    cv::waitKey();
    //  cv::waitKey(30);
  }
}

void Map::plotDebug()
{
  if (debug_mode)
  {
    
    cv::imshow("map_debug", map_to_plot);
    //          cv::waitKey();
    cv::waitKey(30);
  }
}



std::vector<cv::Point2f> Map::getPolygon(Eigen::Affine3d T, std::vector<Plane> vPlanes, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vObstacles)
{

  std::vector<cv::Point2f> polygon_points;

  if (debug_mode)
    map_to_plot = map.clone();

  Eigen::Affine3f Tf = T.cast<float>();

  getRobotPosition(Tf);
  getFrustrum(Tf);

  if (debug_mode)
    plotFrustrum();

  plotPoseIncrement();



  std::vector<Vertex> vertices;
  std::vector<BoundingBox> BBs;

  // Add frustrum vertices to the vertices vector
  for (size_t nV=0; nV<4; nV++)
  {
    Vertex vertex = createVertexFromPos(frustrum[nV]);
    vertices.push_back(vertex);
  }

  int n_planes = 0;

  cv::RNG rng(12345);

  for (size_t Q = 0; Q<vPlanes.size(); Q++)
  {
    if (vPlanes[Q].type > 0)
    {
      std::vector<cv::Point2f> vertices2D = getVerticesOrConvexHull2D(vPlanes[Q].cloud2f);

      BoundingBox BB = BBFromVertices(vertices2D,Q+1);
      BBs.push_back(BB);

      float max_angle = 0.0f, min_angle = 360.0f;
      int index_min = 0, index_max = 0;

      for (int i = 0; i < vertices2D.size(); i++)
      {

        Vertex vertex = createVertexFromPos(vertices2D[i]);
        vertex.BB = Q+1;
        vertices.push_back(vertex);

        if (vertex.angle > max_angle)
        {
          max_angle = vertex.angle;
          index_max = vertices.size()-1;
        }
        if (vertex.angle < min_angle)
        {
          min_angle = vertex.angle;
          index_min = vertices.size()-1;
        }
      }

      vertices[index_min].extreme = true;
      vertices[index_max].extreme = true;
      n_planes++;


      if (debug_mode)
        plotCloudInMap(vPlanes[Q].cloud2f, COLOR_BLACK);//cv::Vec3b(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255)));
    }
  }

  for (int Q = 0; Q<vObstacles.size(); Q++)
  {

    std::vector<cv::Point2f> vertices2D = getVerticesOrConvexHull2D(vObstacles[Q]);
    BoundingBox BB = BBFromVertices(vertices2D,n_planes + Q+1);
    BBs.push_back(BB);

    float max_angle = 0.0f, min_angle = 360.0f;
    int index_min = 0, index_max = 0;

    for (int i = 0; i < vertices2D.size(); i++)
    {

      Vertex vertex = createVertexFromPos(vertices2D[i]);
      vertex.BB = n_planes + Q + 1;
      vertices.push_back(vertex);

      if (vertex.angle > max_angle)
      {
        max_angle = vertex.angle;
        index_max = vertices.size()-1;
      }
      if (vertex.angle < min_angle)
      {
        min_angle = vertex.angle;
        index_min = vertices.size()-1;
      }
    }

    vertices[index_min].extreme = true;
    vertices[index_max].extreme = true;

    if (debug_mode)
      plotCloudInMap(vObstacles[Q], COLOR_BLACK);//cv::Vec3b(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255)));
  }








  // En este caso, comprobamos si son o no visibles los 4 vértices, y calculamos intersecciones de los segmentos entre vértices, y nos quedamos con todas (ya comprobaremos si son o no visibles)
  for (int b = 0; b<BBs.size(); b++)
  {
    for (int s = 0; s<BBs[b].segments.size(); s++)
    {
      for (int f = 0; f<segments_frustrum_square.size()-1; f++) // not intersections with fourth frustrum segment (i.e. bottom one)
      {
        if (pointsIntersect(segments_frustrum_square[f],BBs[b].segments[s]))
        {
          cv::Point2f intersection_point;
          if (getIntersectionPoint(segments_frustrum_square[f],BBs[b].segments[s],intersection_point))
          {
            Vertex vertex = createVertexFromPos(intersection_point);
            vertex.BB = 100;
            vertices.push_back(vertex);
          }
        }
      }
    }
  }



  // Intersecciones de BB
  for (int b1 = 0; b1<BBs.size(); b1++)
  {
    for (int b2 = b1+1; b2<BBs.size(); b2++)
    {
      for (int i1 = 0; i1<BBs[b1].segments.size(); i1++)
      {
        for (int i2 = 0; i2<BBs[b2].segments.size(); i2++)
        {
          if (pointsIntersect(BBs[b1].segments[i1],BBs[b2].segments[i2]))
          {
            cv::Point2f intersection_point;
            if (getIntersectionPoint(BBs[b1].segments[i1],BBs[b2].segments[i2],intersection_point))
            {
              Vertex vertex = createVertexFromPos(intersection_point);
              vertex.BB = 100;
              vertex.BB_int = true;
              vertices.push_back(vertex);

            }
          }
        }
      }
    }
  }

  std::vector<Vertex> visible_vertices;

  for (int v = 0; v<vertices.size(); v++)
  {

//    double inPolygon = cv::pointPolygonTest(frustrum, vertices[v].point2f, false);

//    std::cout << inPolygon << std::endl;

//    if (inPolygon >= 0)
//    {
      std::vector<cv::Point2f> visibility_segment;
      visibility_segment.push_back(cv::Point2f(vertices[v].point2f.x-0.00001*vertices[v].dir[0],vertices[v].point2f.y-0.00001*vertices[v].dir[1]));
      visibility_segment.push_back(robot_pos);
      bool is_visible = true;
      int n_intersections = 0;
      for (int b = 0; b<BBs.size(); b++)
      {
        for (int s = 0; s<BBs[b].segments.size(); s++)
        {
          if (pointsIntersect(visibility_segment,BBs[b].segments[s]))
          {
            n_intersections++;
            if (n_intersections > 0)
            {
              is_visible = false;
              break   ;
            }
          }
        }
        //                                                            }
      }
      if (is_visible)
      {
        visible_vertices.push_back(vertices[v]);

//        if (debug_mode)
//        {
//          plotPoint(vertices[v],COLOR_GREEN);
//        }
      }
//      else
//      {
//        if (debug_mode)
//        {
//          plotPoint(vertices[v],COLOR_RED);
//        }
//      }
////    }
  }







  std::sort(visible_vertices.begin(), visible_vertices.end(), sortVertices);

  std::vector<Vertex> final_vertices;

  for (int i = 0; i<visible_vertices.size()-1; i++)
  {

    // Check if infinite extension of mediatrix between current and next vertex intersects with any BB
    Eigen::Vector2f direction_mediatrix = (visible_vertices[i].dir + visible_vertices[i+1].dir).normalized();
    cv::Point2f infinite_point = cv::Point2f(robot_pos.x+10.0*direction_mediatrix[0],robot_pos.y+10.0*direction_mediatrix[1]);
    std::vector<cv::Point2f> segment; segment.push_back(robot_pos); segment.push_back(infinite_point);

    int n_intersections = 0;
    float distance_BB = 10.0;

    for (int b = 0; b<BBs.size(); b++)
    {
      for (int s = 0; s<BBs[b].segments.size(); s++)
      {
        if (pointsIntersect(segment,BBs[b].segments[s]))
        {
          cv::Point2f intersection_point;
          if (getIntersectionPoint(segment,BBs[b].segments[s],intersection_point))
          {
            n_intersections++;

            float distance_point = cv::norm(intersection_point-robot_pos);
            if (distance_point < distance_BB)
            {
              distance_BB = distance_point;
            }
          }
        }
      }
    }

    if (n_intersections == 0) // If it does not intersect with anything
    {
      const float min_angle = 3;

      if (fabs(visible_vertices[i].angle - visible_vertices[i+1].angle) < min_angle) // If angle between vertices is too small
      {
        if (final_vertices.size() > 0)
        {
          if ((final_vertices.back().dist != visible_vertices[i].dist) or (final_vertices.back().angle != visible_vertices[i].angle))
            final_vertices.push_back(visible_vertices[i]);

        }
        else
          final_vertices.push_back(visible_vertices[i]);

        final_vertices.push_back(visible_vertices[i+1]);

      }
      else // If angles between vertices is large, extend!
      {
        // Create extension of vertex [i] and [i + 1]
        for (int ii = i; ii<=i+1; ii++)
        {
          Vertex vertex_ii;
          cv::Point2f infinite_point_ii = cv::Point2f(robot_pos.x+10.0*visible_vertices[ii].dir[0],robot_pos.y+10.0*visible_vertices[ii].dir[1]);
          if (visible_vertices[ii].BB != 0)
          {
            std::vector<cv::Point2f> segment_ii; segment_ii.push_back(robot_pos); segment_ii.push_back(infinite_point_ii);

            for (int f = 0; f<segments_frustrum_square.size() -1; f++) // Attention, for all except the fourth
            {
              if (pointsIntersect(segments_frustrum_square[f],segment_ii))
              {
                infinite_point_ii = getIntersectionPoint(segments_frustrum_square[f],segment_ii);
                break;
              }
            }

            vertex_ii = createVertexFromPos(infinite_point_ii);
            vertex_ii.BB = 100;
          }
          else
            vertex_ii = visible_vertices[ii];


          final_vertices.push_back(vertex_ii);
        }
      }
    }
    else // If there are intersections with BBs of the mediatrix
    {
      // Get segment that joints consecutive vertices and intersect with mediatrix
      std::vector<cv::Point2f> two_points_link_segment; two_points_link_segment.push_back(visible_vertices[i].point2f); two_points_link_segment.push_back(visible_vertices[i+1].point2f);
      cv::Point2f intersection_point_mediatrix = getIntersectionPoint(two_points_link_segment,segment);
      float distance_two_points_mediatrix = cv::norm(intersection_point_mediatrix - robot_pos);

      if ((fabs(distance_two_points_mediatrix) - fabs(distance_BB) > 0.001) ||  (fabs(distance_two_points_mediatrix - distance_BB) < 0.001)   )  // If continuity (small distance, on polygon)
      {

        if (final_vertices.size() > 0)
        {

          if ((final_vertices.back().dist != visible_vertices[i].dist) or (final_vertices.back().angle != visible_vertices[i].angle))
            final_vertices.push_back(visible_vertices[i]);

        }
        else
          final_vertices.push_back(visible_vertices[i]);

        final_vertices.push_back(visible_vertices[i+1]);
      }
      else // If discontinuity (definitely extend)
      {
        for (int ii = i; ii<=i+1; ii++)
        {
          if (visible_vertices[ii].extreme)
          {
            cv::Point2f infinite_point_ii = cv::Point2f(robot_pos.x+10.0*visible_vertices[ii].dir[0],robot_pos.y+10.0*visible_vertices[ii].dir[1]);
            std::vector<cv::Point2f> segment_ii; segment_ii.push_back(robot_pos); segment_ii.push_back(infinite_point_ii);
            cv::Point2f closest_intersection_point;

            float distance_vertex = 10.0;
            int n_intersections = 0;
            for (int b = 0; b<BBs.size(); b++)
            {
              if (BBs[b].index != visible_vertices[ii].BB)
              {
                for (int s = 0; s < BBs[b].segments.size(); s++)
                {
                  if (pointsIntersect(segment_ii,BBs[b].segments[s]))
                  {
                    cv::Point2f intersection_point;
                    if (getIntersectionPoint(segment_ii,BBs[b].segments[s],intersection_point))
                    {
                      n_intersections++;
                      float distance_point = cv::norm(intersection_point-robot_pos);
                      if (distance_point < distance_vertex)
                      {
                        closest_intersection_point = intersection_point;
                        distance_vertex = distance_point;
                      }
                    }
                  }
                }
              }
            }
            if (n_intersections > 0) // if intersects with BB
            {

//              std::cout << closest_intersection_point << std::endl;

//              plotPoint(visible_vertices[i].point,COLOR_GREEN);
//              plotPoint(visible_vertices[i+1].point,COLOR_RED);

//              plotPoint(getImgPoint(closest_intersection_point), COLOR_BLUE);
//              plotVisibilityLine(getImgPoint(closest_intersection_point),COLOR_BLACK);

//              double inPolygon = cv::pointPolygonTest(frustrum, closest_intersection_point, false);

//              std::cout << inPolygon << std::endl;
//              plotDebugAndStop();

//              if (inPolygon >= 0) // Verify if in frustrum and then just add
//              {
                Vertex vertex_ii = createVertexFromPos(closest_intersection_point);
                vertex_ii.BB = 100;

                final_vertices.push_back(vertex_ii);
//              }
//              else // if it is outside the frustrum, compute intersection point
//              {
//                std::vector<cv::Point2f> segment_cip; segment_cip.push_back(robot_pos); segment_cip.push_back(closest_intersection_point);

//                cv::Point2f frustrum_point_ii;

//                for (int f = 0; f<segments_frustrum_square.size() -1; f++) // Attention, for all except the fourth
//                {
//                  if (pointsIntersect(segments_frustrum_square[f],segment_cip))
//                  {
//                    frustrum_point_ii = getIntersectionPoint(segments_frustrum_square[f],segment_ii);
//                    break;
//                  }
//                }

//                Vertex vertex_ii = createVertexFromPos(frustrum_point_ii);
//                vertex_ii.BB = 100;

//                    /*
//                    // Beware, there might be a frustrum point in between
//                    double angle_f2 = angleRelativePose(frustrum[1]);
//                    if (angle_f2 < vertex_ii.angle && angle_f2 > final_vertices.back().angle)
//                    {

//                    }*/

//                final_vertices.push_back(vertex_ii);
//              }


            }
            else // if it does not intersect with any BB, just add (does it make any sense?)
            {
              if (final_vertices.size() > 0)
              {
                if ((final_vertices.back().dist != visible_vertices[ii].dist) or (final_vertices.back().angle != visible_vertices[ii].angle))
                  final_vertices.push_back(visible_vertices[ii]);

              }
              else
                final_vertices.push_back(visible_vertices[ii]);
            }

          }
          else // if point is not extreme
          {
            if (final_vertices.size() > 0)
            {
              if ((final_vertices.back().dist != visible_vertices[ii].dist) or (final_vertices.back().angle != visible_vertices[ii].angle))
                final_vertices.push_back(visible_vertices[ii]);

            }
            else
              final_vertices.push_back(visible_vertices[ii]);
          }
        }
      }
    }
  }

  std::reverse(final_vertices.begin(),final_vertices.end());
  Vertex robot_pos_vertex = createVertexFromPos(robot_pos);
  final_vertices.push_back(robot_pos_vertex);



  for (int i = 0; i < final_vertices.size(); i++)
  {
      polygon_points.push_back(final_vertices[i].point2f);
  }



  if (debug_mode)
  {
    plotPoly(final_vertices,COLOR_YELLOW);


    for (size_t Q = 0; Q<vPlanes.size(); Q++)
    {
      if (vPlanes[Q].type > 0)
      {
//        if (debug_mode)
          plotCloudInMap(vPlanes[Q].cloud2f, COLOR_BLACK);
      }
    }

    for (int Q = 0; Q<vObstacles.size(); Q++)
    {
//      if (debug_mode)
        plotCloudInMap(vObstacles[Q], COLOR_BLACK);
    }

//    if (debug_mode)
//    {
      for (int i = 0; i<BBs.size(); i++)
      {
        getBBSegmentsMap(BBs[i]);
        plotBBInMap(BBs[i],COLOR_BLUE);
      }

      for (int i = 0; i < visible_vertices.size(); i++)
      {
        plotPoint(visible_vertices[i].point,COLOR_GREEN);
//        std::cout << visible_vertices[i].extreme << std::endl;
//        plotDebugAndStop();
      }

//    }

    plotDebug();
//    plotDebugAndStop();
  }

  return polygon_points;
}

