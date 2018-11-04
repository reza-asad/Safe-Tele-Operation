//
// Created by rakesh on 13/08/18.
//
#include <cmath>
#include <map>
#include <numeric>
#include <chrono>

#include <boost/atomic/atomic.hpp>
#include <opencv2/flann/miniflann.hpp>

#include <icp_slam/icp_slam.h>
#include <icp_slam/utils.h>
#include <icp_slam/config.h>

#define TIME_DIFF(tic, toc) ((std::chrono::duration<double, std::milli>((toc) - (tic))).count())

namespace icp_slam
{

ICPSlam::ICPSlam(tfScalar max_keyframes_distance, tfScalar max_keyframes_angle, double max_keyframes_time)
  : max_keyframes_distance_(max_keyframes_distance),
    max_keyframes_angle_(max_keyframes_angle),
    max_keyframes_time_(max_keyframes_time),
    last_kf_laser_scan_(new sensor_msgs::LaserScan()),
    is_tracker_running_(false)
{
  last_kf_tf_odom_laser_.stamp_ = ros::Time(0);
}

bool ICPSlam::track(const sensor_msgs::LaserScanConstPtr &laser_scan,
                    const tf::StampedTransform &current_frame_tf_odom_laser,
                    tf::StampedTransform &tf_map_laser)
{
  if (is_tracker_running_)
  {
    ROS_WARN_THROTTLE(1.0, "Couldn't track frame, tracker already running");
    return false;
  }
  is_tracker_running_ = true;
  // Initialize the last key frame wrt odom 
  if (last_kf_laser_scan_->ranges.size() == 0) 
  {
    last_kf_tf_odom_laser_.frame_id_ = current_frame_tf_odom_laser.frame_id_;
    last_kf_tf_odom_laser_.child_frame_id_ = current_frame_tf_odom_laser.child_frame_id_;
    last_kf_tf_odom_laser_.setOrigin(current_frame_tf_odom_laser.getOrigin());
    last_kf_tf_odom_laser_.setRotation(current_frame_tf_odom_laser.getRotation());

    tf_map_laser.frame_id_ = "map";
    tf_map_laser.child_frame_id_ = current_frame_tf_odom_laser.frame_id_;
    tf_map_laser.setOrigin(current_frame_tf_odom_laser.getOrigin());
    tf_map_laser.setRotation(current_frame_tf_odom_laser.getRotation());

    // copy the laser scan
    last_kf_laser_scan_->ranges = laser_scan->ranges;
    last_kf_laser_scan_->angle_min = laser_scan->angle_min;
    last_kf_laser_scan_->angle_max = laser_scan->angle_max;
    last_kf_laser_scan_->angle_increment = laser_scan->angle_increment;
    last_scan_matrix = utils::laserScanToPointMat(last_kf_laser_scan_);
  }
  // Find the conversion from map 
  bool is_key_frame = isCreateKeyframe(current_frame_tf_odom_laser, last_kf_tf_odom_laser_); 
  if (is_key_frame) 
  {
    // convert the laser scans to matrix
    cv::Mat current_scan_matrix = utils::laserScanToPointMat(laser_scan);

    // Find correspondences between previous and current laser scan.

    // run ICP between current scan and last laser scan
    tf::Transform icp_transform_ = icpIteration(last_scan_matrix, current_scan_matrix);

    // refine the icp transform and align the current scan
    icp_transform_ = icpRegistration(last_scan_matrix, current_scan_matrix, icp_transform_);
    last_scan_matrix = utils::transformPointMat(icp_transform_, current_scan_matrix);

    // update the robot's last pose and laser scan wrt odom
    last_kf_tf_odom_laser_.stamp_ = ros::Time(0);
    last_kf_tf_odom_laser_.setOrigin(current_frame_tf_odom_laser.getOrigin());
    last_kf_tf_odom_laser_.setRotation(current_frame_tf_odom_laser.getRotation());

    // update the robot's pose wrt to map
    // tf_map_laser = last_kf_tf_odom_laser_;

  } else {
    // obtain laser pose in map based on odometry update
    last_kf_tf_map_laser_ = current_frame_tf_odom_laser * tf_map_laser.inverse();
  }
  is_tracker_running_ = false;

  // return is_key_frame;
  // TODO: find the pose of laser in map frame
  // if a new keyframe is created, run ICP
  // if not a keyframe, obtain the laser pose in map frame based on odometry update
}

bool ICPSlam::isCreateKeyframe(const tf::StampedTransform &current_frame_tf, const tf::StampedTransform &last_kf_tf) const
{
  assert(current_frame_tf.frame_id_ == last_kf_tf.frame_id_);
  assert(current_frame_tf.child_frame_id_ == last_kf_tf.child_frame_id_);

  // compute distance between frames
  tf::Vector3 current_orig_ = current_frame_tf.getOrigin();
  tf::Vector3 last_orig_ = last_kf_tf.getOrigin();
  tfScalar keyframes_dist_ = sqrt(pow(current_orig_.x() - last_orig_.x(), 2) + pow(current_orig_.y() - last_orig_.y(), 2));

  // compute the angle diff between frames
  double keyframes_time_ = (current_frame_tf.stamp_.toSec() - last_kf_tf.stamp_.toSec()) < max_keyframes_time_;

  // compute the angle difference between frames
  tfScalar keyframes_angle_ = abs(current_frame_tf.getRotation().getAngle() - last_kf_tf.getRotation().getAngle());

  return ((keyframes_dist_ > max_keyframes_distance_) || 
          (keyframes_time_ > max_keyframes_time_) ||
          (keyframes_angle_ > max_keyframes_angle_));

  // TODO: check whether you want to create keyframe (based on max_keyframes_distance_, max_keyframes_angle_, max_keyframes_time_)
}

tf::Transform ICPSlam::icpIteration(cv::Mat &point_mat1,
                                    cv::Mat &point_mat2) 
{
  // Find the means and normalize
  cv::Mat mu_x;
  cv::reduce(point_mat1, mu_x, 0, CV_REDUCE_SUM);
  mu_x /= point_mat1.rows;

  cv::Mat mu_p;
  cv::reduce(point_mat2, mu_p, 0, CV_REDUCE_SUM);
  mu_p /= point_mat2.rows;

  // calculate the covarianc matrix
  cv::Mat mu_x_transpose;
  cv::transpose(mu_x, mu_x_transpose);
  cv::Mat w = mu_x_transpose * mu_p;
  
  // compute the SVD.
  cv::SVD svd(w);

  // compute the rotation and translation
  cv::Mat R = svd.u * svd.vt;
  cv::Mat t = mu_x - R * mu_p;

  // create the transform form the rotation and translation
  tf::Transform icp_transform_;
  icp_transform_.setOrigin(tf::Vector3(t.at<float>(0), t.at<float>(1), 0.0));
  double theta;
  double cos_theta = R.at<float>(0,0);
  double sin_theta = R.at<float>(1,0);
  if (cos_theta == 0) cos_theta += 1e-6;
  if (sin_theta >= 0) 
  {
    if (cos_theta > 0) 
    {
      theta = atan(atan(sin_theta/cos_theta));   
    } 
    else 
    {
      theta = M_PI - atan(sin_theta/cos_theta);
    }
  } 
  else 
  {
    if (cos_theta > 0)
    {
      theta = atan(atan(sin_theta/cos_theta));
    }
    else
    {
      theta = atan(atan(sin_theta/cos_theta)) - M_PI; 
    }
  }

  icp_transform_.setRotation(tf::createQuaternionFromYaw(theta));

  return icp_transform_;
}

tf::Transform ICPSlam::icpRegistration(cv::Mat last_scan_matrix,
                                       cv::Mat current_scan_matrix,
                                       const tf::Transform &T_2_1)
{
  // apply the transformation and compute the error.
  cv::Mat current_scan_transformed = utils::transformPointMat(T_2_1, current_scan_matrix);
  cv::Mat diff = current_scan_transformed - last_scan_matrix;
  cv::multiply(diff, diff, diff);
  std::vector<float> errors;
  cv::reduce(diff, errors, 1, CV_REDUCE_SUM);

  // only take the top %99 points wrt error
  std::vector<float> sorted_errors = errors;
  std::sort(sorted_errors.begin(), sorted_errors.end());
  int max_error_idx = 0.99 * errors.size();
  float max_error = sorted_errors[max_error_idx];
  cv::Mat last_scan_trimmed;
  cv::Mat current_scan_trimmed;
  for (int i=0; i<errors.size(); ++i)
  {
    if (errors[i] < max_error)
    {
      last_scan_trimmed.push_back(last_scan_matrix.row(i));
      current_scan_trimmed.push_back(current_scan_matrix.row(i));
    }
  }
  tf::Transform icp_transform_ = icpIteration(last_scan_trimmed, current_scan_trimmed); 
  return icp_transform_;
}

void ICPSlam::closestPoints(cv::Mat &point_mat1,
                            cv::Mat &point_mat2,
                            std::vector<int> &closest_indices,
                            std::vector<float> &closest_distances_2)
{
  // uses FLANN for K-NN e.g. http://www.morethantechnical.com/2010/06/06/iterative-closest-point-icp-with-opencv-w-code/
  closest_indices = std::vector<int>(point_mat1.rows, -1);
  closest_distances_2 = std::vector<float>(point_mat1.rows, -1);


  cv::Mat multi_channeled_mat1;
  cv::Mat multi_channeled_mat2;

  point_mat1.convertTo(multi_channeled_mat1, CV_32FC2);
  point_mat2.convertTo(multi_channeled_mat2, CV_32FC2);

  cv::flann::Index flann_index(multi_channeled_mat2, cv::flann::KDTreeIndexParams(2));  // using 2 randomized kdtrees

  cv::Mat mat_indices(point_mat1.rows, 1, CV_32S);
  cv::Mat mat_dists(point_mat1.rows, 1, CV_32F);
  flann_index.knnSearch(multi_channeled_mat1, mat_indices, mat_dists, 1, cv::flann::SearchParams(64) );

  int* indices_ptr = mat_indices.ptr<int>(0);
  //float* dists_ptr = mat_dists.ptr<float>(0);
  for (int i=0;i<mat_indices.rows;++i) {
    closest_indices[i] = indices_ptr[i];
  }

  mat_dists.copyTo(cv::Mat(closest_distances_2));

  // ---------------------------- naive version ---------------------------- //
  // max allowed distance between corresponding points
//  const float max_distance = 0.5;
//
//  for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
//  {
//    int closest_point_idx = -1;
//    float closest_distance_2 = std::pow(max_distance, 2.0f);
//
//    for (size_t j = 0, len_j = (size_t)point_mat2.rows; j < len_j; j++)
//    {
//      auto distance2 =
//        std::pow(point_mat2.at<float>(j, 0) - point_mat1.at<float>(i, 0), 2.0f)
//        + std::pow(point_mat2.at<float>(j, 1) - point_mat1.at<float>(i, 1), 2.0f);
//
//      if (distance2 < closest_distance_2)
//      {
//        closest_distance_2 = distance2;
//        closest_point_idx = (int)j;
//      }
//    }
//
//    if (closest_point_idx >= 0)
//    {
//      closest_indices[i] = closest_point_idx;
//      closest_distances_2[i] = closest_distance_2;
//    }
//  }
}

void ICPSlam::vizClosestPoints(cv::Mat &point_mat1,
                               cv::Mat &point_mat2,
                               const tf::Transform &T_2_1)
{
  assert(point_mat1.size == point_mat2.size);

  const float resolution = 0.005;

  float *float_array = (float*)(point_mat1.data);
  float size_m = std::accumulate(
    float_array, float_array + point_mat1.total(), std::numeric_limits<float>::min(),
    [](float max, float current)
    {
      return current > max ? current : max;
    }
  );
  // add some slack
  size_m += 0.5;

  int size_pix = (int)(size_m / resolution);

  cv::Mat img(
    size_pix,
    size_pix,
    CV_8UC3,
    cv::Scalar(0, 0, 0)
  );

  auto meters_to_pix = [&size_pix, resolution](float meters) {
    int pix = (int)(meters / resolution + size_pix / 2.0f);
    pix = std::max(0, pix);
    pix = std::min(size_pix - 1, pix);
    return pix;
  };

  cv::Mat transformed_point_mat2 = utils::transformPointMat(T_2_1.inverse(), point_mat2);

  for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
  {
    float x1 = point_mat1.at<float>(i, 0);
    float y1 = point_mat1.at<float>(i, 1);
    float x2 = transformed_point_mat2.at<float>(i, 0);
    float y2 = transformed_point_mat2.at<float>(i, 1);

    auto pix_x1 = meters_to_pix(x1);
    auto pix_y1 = meters_to_pix(y1);
    auto pix_x2 = meters_to_pix(x2);
    auto pix_y2 = meters_to_pix(y2);

    cv::Point point1(pix_x1, pix_y1);
    cv::Point point2(pix_x2, pix_y2);

    cv::circle(img, point1, 5, cv::Scalar(0, 0, 255), -1);
    cv::circle(img, point2, 5, cv::Scalar(255, 0, 0), -1);

    cv::line(img, point1, point2, cv::Scalar(0, 255, 0), 2);
  }

  cv::Mat tmp;
  cv::flip(img, tmp, 0);
  cv::imwrite("/tmp/icp_laser.png", img);
}

} // namespace icp_slam

