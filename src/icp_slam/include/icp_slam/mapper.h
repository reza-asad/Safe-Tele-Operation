/** @file mapper.h */
//
// Created by rakesh on 27/08/18.
//

#ifndef ICP_SLAM_MAPPER_H
#define ICP_SLAM_MAPPER_H

// stdlib includes
#include <map>

// misc includes
#include <boost/thread/recursive_mutex.hpp>
#include <boost/atomic.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>

#include <omp.h>
#include <atomic>
#include <nav_msgs/OccupancyGrid.h>
#include <icp_slam/utils.h>
#include <algorithm>

namespace icp_slam
{
// cost values for different occupancy tyes
static const int8_t NO_INFORMATION = -1;
static const int8_t FREE_SPACE = 0;
static const int8_t LETHAL_OBSTACLE = 100;

static const int8_t FREE_SPACE_THRESHOLD = 20;
static const int8_t OBSTACLE_THRESHOLD = 70;
static const int8_t VOTE_WEIGHT = 10;

class Mapper
{
public:
  Mapper(unsigned int width, unsigned int height, float resolution);
  ~Mapper();

  cv::Mat getMapMatrix() {return map_matrix_;}
  void vizOccupancyGrid(const nav_msgs::OccupancyGrid &occupancy_grid);
  void updateMap(const tf::StampedTransform &tf_map_laser, const sensor_msgs::LaserScanConstPtr &laser_msg, nav_msgs::OccupancyGrid &occupancy_grid);

  void mapToGridCoordinate(float map_x, float map_y, unsigned int &grid_x_, unsigned int &grid_y_);
  void gridToMapCoordinate(unsigned int grid_x_, unsigned int grid_y_, float &map_x, float &map_y);

protected:
  cv::Mat map_matrix_;

  unsigned int width_;
  unsigned int height_;
  float resolution_;  

  float map_origin_x_;	// map origin in meters
  float map_origin_y_;

  unsigned int grid_origin_x_;
  unsigned int grid_origin_y_;

  boost::recursive_mutex map_matrix_mutex_;
  bool first_update_;
};

} // namespace icp_slam

#endif //ICP_SLAM_MAPPER_H
