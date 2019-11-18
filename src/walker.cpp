/**
 * @file walker.cpp
 * @brief Algorithm to control a Turtlebot to avoid obstacles. The class can get laser scan sensor readings and choose to either move forward or turn in place if an obstacle is detected.
 * @author Ryan Cunningham
 * @copyright 2019
 * Distributed under the BSD License (license terms found in LICENSE or at https://www.freebsd.org/copyright/freebsd-license.html)
 */

#include "walker.hpp"

#include <geometry_msgs/Twist.h>
#include <math.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>

Walker::Walker(const double detectionDistance) : detectionDistance(detectionDistance) {
  /// initialize the velocity to zero
  velocity.linear.x = 0.0;
  velocity.linear.y = 0.0;
  velocity.linear.z = 0.0;
  velocity.angular.x = 0.0;
  velocity.angular.y = 0.0;
  velocity.angular.z = 0.0;
}

void Walker::sense(const sensor_msgs::LaserScan::ConstPtr& msg) {
  if (isObstacleDetected(msg)) {
    /// if an obstacle is detected then turn in place
    velocity.linear.x = 0.0;
    velocity.linear.y = 0.0;
    velocity.linear.z = 0.0;
    velocity.angular.x = 0.0;
    velocity.angular.y = 0.0;
    velocity.angular.z = 0.5;
  } else {
    /// if no obstacle detected then move straight ahead
    velocity.linear.x = 0.2;
    velocity.linear.y = 0.0;
    velocity.linear.z = 0.0;
    velocity.angular.x = 0.0;
    velocity.angular.y = 0.0;
    velocity.angular.z = 0.0;
  }
}

geometry_msgs::Twist Walker::getVelocity() {
  return velocity;
}

bool Walker::isObstacleDetected(const sensor_msgs::LaserScan::ConstPtr& msg) {
  for (auto range : msg->ranges) {
    /// disregard laser scan readings that are NaN
    if (!std::isnan(range)) {
      /// if the laser range is less than the defined detection distance then
      /// return signifying an obstacle was detected
      if (range < detectionDistance) {
        ROS_INFO_STREAM("Obstacle detected");
        return true;
      }
    } else {
      ROS_WARN_STREAM("Laser scan value is NaN");
    }
  }

  /// if no laser ranges are less than the defined detection distance then
  /// return signifying no obstacle detected
  return false;
}
