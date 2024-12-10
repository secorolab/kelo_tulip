/******************************************************************************
 * Copyright (c) 2021
 * KELO Robotics GmbH
 *
 * Author:
 * Sushant Chavan
 * Walter Nowak
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/
#ifndef PLATFORM_CONTROLLER_HPP
#define PLATFORM_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "tf2_ros/buffer.h"

#include <kelo_tulip/VelocityPlatformController.h>
#include <kelo_tulip/KeloDrive.h>

/**
 * @brief Velocity controller for gazebo simulations of arbitrary
 * KELO platform configurations
 */
class PlatformController : public rclcpp::Node {
public:
    /**
     * @brief Construct a new KELO Platform Controller object
     */
    PlatformController(const std::string& node_name);

    /**
     * @brief Set the linear and angular velocities for the KELO platform
     *
     * @param vx Linear velocity (in m/s) along the positive x axis in robot frame
     * @param vy Linear velocity (in m/s) along the positive y axis in robot frame
     * @param va Angular velocity in (rad/s) around the positive z axis in robot frame
     */
    void setCmdVel(double vx, double vy, double va);

    /**
     * @brief Set the maximum linear and angular velocities for the KELO platform
     *
     * @param linearVel Max linear velocity the KELO platform could achieve
     * @param angularVel Max angular velocity the KELO platform could achieve
     */
    void setMaxPlatformVelocity(double linearVel, double angularVel);

    /**
     * @brief Controller step which computes and sets the desired hub wheel velocities
     */
    void step();

    /**
     * @brief Publish RViz markers for each active wheel's pivot pose
     */
    void publishPivotMarkers() const;

    /**
     * @brief Set the frequency at which odometry messages are published
     *
     * @param frequency Frequency (in Hz) at which odometry messages should be published
     */
    void setOdomFrequency(double frequency);

    /**
     * @brief Publish the latest odometry received from Gazebo on '/odom' topic
     */
    void publishOdom();

    /**
     * @brief Publish the latest transform from odom to base_link on the '/tf' topic
     */
    void publishOdomToBaseLinkTF();

    /**
     * @brief Set the wheel setpoint velocities for all the hub wheels in the KELO platform
     */
    void setAllHubWheelVelocities(const std_msgs::msg::Float64MultiArray& msg);


protected:
    /**
     * @brief Callback function to receive and process the robot joint states from Gazebo
     *
     * @param msg ROS message from Gazebo with the Joint state information
     */
    void jointStatesCallBack(const sensor_msgs::msg::JointState::SharedPtr msg);

    /**
     * @brief Initialize the Kelo drive data structures and the Velocity controller
     *
     * @param pivotJointData Map of Kelo drive name vs its latest pivot orientation
     */
    void initDrives(const std::map<std::string, double>& pivotJointData);

    /**
     * @brief Set the Pivot Orientations for every Kelo drive in the KELO platform
     *
     * @param pivotJointData Map of Kelo drive name vs the latest pivot orientation
     */
    void setPivotOrientations(const std::map<std::string, double>& pivotJointData);

    /**
     * @brief Extract the pivot name from the pivot joint name
     *
     * @param jointName Joint name of the Kelo drive pivot
     * @return std::string Name of the pivot
     */
    std::string getPivotName(const std::string& jointName);

    // ROS2 Specific Members
    std::unique_ptr<tf2_ros::Buffer> _tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> _tfListener;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _jointStatesSubscriber;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr base_velocity_control_pub_;

    // Existing Members
    std::map<std::string, KeloDrive> _drives;
    bool _initialized{false};

    double _cmdVelX{0.0};
    double _cmdVelY{0.0};
    double _cmdVelA{0.0};

    std::map<std::string, int> drives_map = {
    // Note: the order of the drives should be same as the order of controllers in eddie_controllers.yaml (eddie_gazebo)
    // Assumption: for each wheel unit, first the left wheel and then the right wheel is considered in the controller configuration file

        {"eddie_front_left", 0},
        {"eddie_rear_left", 1},
        {"eddie_rear_right", 2},
        {"eddie_front_right", 3}
    };

    kelo::VelocityPlatformController _controller;
    std::map<std::string, kelo::WheelConfig> _wheelConfigs;
};

#endif // PLATFORM_CONTROLLER_HPP
