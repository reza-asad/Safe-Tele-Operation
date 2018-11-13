//
// Created by rakesh on 27/08/18.
//

#include <icp_slam/mapper.h>
using namespace std;

namespace icp_slam
{
  Mapper::Mapper(unsigned int width, unsigned int height, float resolution)
  {
    width_ = width;
    height_ = height;
    resolution_ = resolution;

    grid_origin_x_ = width_ / 2;
    grid_origin_y_ = height_ / 2;

    map_matrix_ = cv::Mat::ones(height, width, CV_8SC1) * ((icp_slam::FREE_SPACE + icp_slam::LETHAL_OBSTACLE)/2);
    first_update_ = true;
  }

  Mapper::~Mapper()
  {

  }

  void Mapper::mapToGridCoordinate(float map_x, float map_y, unsigned int &grid_x_, unsigned int &grid_y_)
  {
    grid_x_ = grid_origin_x_ + (map_x - map_origin_x_) / resolution_;
    grid_y_ = grid_origin_y_ + (map_y - map_origin_y_) / resolution_;
  }

  void Mapper::gridToMapCoordinate(unsigned int grid_x_, unsigned int grid_y_, float &map_x, float &map_y)
  {
    map_x = map_origin_x_ + static_cast<float>(grid_x_ - grid_origin_x_) * resolution_;
    map_y = map_origin_y_ + static_cast<float>(grid_y_ - grid_origin_y_) * resolution_;
  }

  void Mapper::updateMap(const tf::StampedTransform &tf_map_laser, const sensor_msgs::LaserScanConstPtr &laser_msg, nav_msgs::OccupancyGrid &occupancy_grid)
  {
    boost::unique_lock<boost::recursive_mutex> scoped_lock(map_matrix_mutex_);

    if(first_update_)
    {
      // initialize map origin(center) to current robot position
      map_origin_x_ = tf_map_laser.getOrigin().getX();
      map_origin_y_ = tf_map_laser.getOrigin().getY();
    
      occupancy_grid.info.origin.position.x = map_origin_x_ - static_cast<float>(grid_origin_x_) * resolution_;
      occupancy_grid.info.origin.position.y = map_origin_y_ - static_cast<float>(grid_origin_y_) * resolution_;

      first_update_ = false;
    }

    unsigned int robot_cell_x = 0, robot_cell_y = 0;
    mapToGridCoordinate(tf_map_laser.getOrigin().getX(), tf_map_laser.getOrigin().getY(), robot_cell_x, robot_cell_y);
    float robot_yaw = tf::getYaw(tf_map_laser.getRotation()); 	

    for(int i = 0; i < laser_msg->ranges.size(); i++)
    {
      // filter invalid scans
      float cur_range = laser_msg->ranges[i];
      if(cur_range < laser_msg->range_min || cur_range > laser_msg->range_max)
      {
          continue;
      }
      cur_range = cur_range / resolution_;

      // for each laser beam, calculate the end cell
      float cur_range_angle = utils::normalizeAngle(robot_yaw + laser_msg->angle_min + laser_msg->angle_increment * i);
      float rel_laser_pos_x = 0, rel_laser_pos_y = 0;
      utils::polarToCartesian(cur_range, cur_range_angle, rel_laser_pos_x, rel_laser_pos_y);
      unsigned int laser_cell_x = robot_cell_x + rel_laser_pos_x;
      unsigned int laser_cell_y = robot_cell_y + rel_laser_pos_y;

      // iterate through the points along robot cell and laser cell and update the map
      // ignoring the case where a point goes out of bound of the map; realistically map should be expanded
      cv::LineIterator it(map_matrix_, cv::Point(robot_cell_x, robot_cell_y), cv::Point(laser_cell_x, laser_cell_y));
      for(int j = 0; j < it.count; ++j, ++it) 
      {
        cv::Point point = it.pos();
        // vote obstacle for the laser end cells, otherwise vote free space
        int8_t delta = (j == it.count - 1) ? icp_slam::VOTE_WEIGHT : -icp_slam::VOTE_WEIGHT;

        // if unknown point, set val to 50 + delta, otherwise set to old value + delta; truncate val to between 0 and 100
        int8_t old_val = map_matrix_.at<int8_t>(point.y, point.x);
        int8_t new_val = 0;
        new_val = old_val + delta;
        new_val = fmax(fmin(new_val, icp_slam::LETHAL_OBSTACLE), icp_slam::FREE_SPACE); // clamp
        map_matrix_.at<int8_t>(point.y, point.x) = new_val;
      }
    }

    // convert map matrix to occupancy grid
    for(int i = 1; i < height_-1 ; i++)
    {
      for(int j = 1; j < width_-1; j++)
      {
        int8_t grid_val = map_matrix_.at<int8_t>(i ,j);
        if(grid_val >= icp_slam::OBSTACLE_THRESHOLD)
        {
          grid_val = 100;
          occupancy_grid.data[(i-1) * width_ + (j-1)] = grid_val;
          occupancy_grid.data[(i-1) * width_ + j] = grid_val;
          occupancy_grid.data[i * width_ + (j-1)] = grid_val;
          occupancy_grid.data[i * width_ + (j+1)] = grid_val;
          occupancy_grid.data[(i+1) * width_ + j] = grid_val;
          occupancy_grid.data[(i+1) * width_ + (j+1)] = grid_val;
          occupancy_grid.data[(i+1) * width_ + (j-1)] = grid_val;
          occupancy_grid.data[(i-1) * width_ + (j+1)] = grid_val;
        }
        else if(grid_val <= icp_slam::FREE_SPACE_THRESHOLD)
        {
          grid_val = 0;
        }
        else
        {
          grid_val = -1;
        }
        occupancy_grid.data[i * width_ + j] = grid_val;
      }
    }
  }

} // namespace icp_slam
