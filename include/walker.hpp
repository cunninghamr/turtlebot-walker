/**
 * @file walker.hpp
 * @brief Algorithm to control a Turtlebot to avoid obstacles. The class can get laser scan sensor readings and choose to either move forward or turn in place if an obstacle is detected.
 * @author Ryan Cunningham
 * @copyright 2019
 * Distributed under the BSD License (license terms found in LICENSE or at https://www.freebsd.org/copyright/freebsd-license.html)
 */

#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class Walker {
 public:
  /**
   * @brief Constructor
   * @param detectionDistance the distance that qualifies an object as an obstacle.
   */
  explicit Walker(const double detectionDistance = 0.5);

  /**
   * @brief Receive a laser scan sensor reading and assign an appropriate velocity.
   * @param msg the laser scan message
   */
  void sense(const sensor_msgs::LaserScan::ConstPtr& msg);

  /**
   * @brief Getter for the calculated velocity.
   * @return velocity message
   */
  geometry_msgs::Twist getVelocity();

 private:
  /// the minimum detection distance for an obstacle
  double detectionDistance;

  /// the velocity to command the Turtlebot
  geometry_msgs::Twist velocity;

  /**
   * @brief Determines whether a laser scan sensor reading has detected an obstacle. An obstacle is detected if any reading is less than the detectionDistance value.
   * @param msg the laser scan message
   * @return true if an obstacle is detected, false otherwise
   */
  bool isObstacleDetected(const sensor_msgs::LaserScan::ConstPtr& msg);
};

#endif  // INCLUDE_WALKER_HPP_
