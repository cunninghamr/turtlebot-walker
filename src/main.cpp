/**
 * @file main.cpp
 * @brief ROS node to control Turtlebot so that it does not collide with obstacles.
 * @author Ryan Cunningham
 * @copyright 2019
 * Distributed under the BSD License (license terms found in LICENSE or at https://www.freebsd.org/copyright/freebsd-license.html)
 */

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <iostream>

#include "walker.hpp"

int main(int argc, char **argv) {
  ROS_INFO_STREAM("Walker node started");

  /// initialize ROS node
  ros::init(argc, argv, "walker_node");

  /// get node handle to interface with ROS
  ros::NodeHandle nh;

  /// create walker object to control the Turtlebot
  Walker walker;

  /// subscriber to the Turtlebot's laser scan sensor
  /// the callback takes the laser scan and figures out the new velocity to use
  ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1000, &Walker::sense, &walker);

  /// publisher to the Turtlebot's velocity topic
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1000);

  ros::Rate rate(20);
  while (ros::ok()) {
    /// publish the calculated velocity from the walker alogrithm
    pub.publish(walker.getVelocity());

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
