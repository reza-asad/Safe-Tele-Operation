//
// Created by rakesh on 17/08/18.
//

#include <icp_slam/utils.h>

namespace icp_slam
{
namespace utils
{

cv::Mat laserScanToPointMat(const sensor_msgs::LaserScanConstPtr &scan)
{
  cv::Mat scan_matrix(scan->ranges.size(), 2, CV_32F);
  double angle_min = scan->angle_min;
  float x = 0;
  float y = 0;
  for (int i=0; i<scan->ranges.size(); ++i) {
    double scan_val = scan->ranges[i];
    if (scan->ranges[i] == 0) scan_val = 2.0;
    polarToCartesian(scan_val, angle_min, x, y);
    scan_matrix.at<float>(i, 0) = x;
    scan_matrix.at<float>(i, 1) = y;
    angle_min += scan->angle_increment;
  }
  return scan_matrix;
}

cv::Mat transformPointMat(const tf::Transform transform, cv::Mat &point_mat)
{
  assert(point_mat.data);
  assert(!point_mat.empty());

  cv::Mat point_mat_homogeneous(3, point_mat.rows, CV_32F, cv::Scalar(1.0f));

  cv::Mat(point_mat.t()).copyTo(point_mat_homogeneous.rowRange(0, 2));

  auto T = transformToMatrix(transform);
  // std::cout << T << std::endl; 
  cv::Mat transformed_point_mat =  T * point_mat_homogeneous;
  return cv::Mat(transformed_point_mat.t()).colRange(0, 2);
}

} // namespace utils
} // namespace icp_slam