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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "kelo_tulip/PlatformController.h"

class KeloGazeboController : public rclcpp::Node {
private:
    std::shared_ptr<PlatformController> platformController;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber;
    rclcpp::TimerBase::SharedPtr controlTimer;
    rclcpp::executors::MultiThreadedExecutor* executor;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (platformController) {
            platformController->setCmdVel(msg->linear.x, msg->linear.y, msg->angular.z);
        }
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) {
        if (platformController) {
            platformController->setCmdVel(joy->axes[1], joy->axes[0], joy->axes[2]);
        }
    }

    void controlLoop() {
        platformController->step();
    }

public:
    KeloGazeboController(const std::string& node_name, rclcpp::executors::MultiThreadedExecutor* exec)
    : Node(node_name), executor(exec) {

        cmdVelSubscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&KeloGazeboController::cmdVelCallback, this, std::placeholders::_1)
        );

        joySubscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 1000, 
            std::bind(&KeloGazeboController::joyCallback, this, std::placeholders::_1)
        );

        // Load max linear velocity
        double platformMaxLinVel = this->declare_parameter("platform_max_lin_vel", 1.0);

        // Load max angular velocity
        double platformMaxAngVel = this->declare_parameter("platform_max_ang_vel", 1.0);

        // Initialize platform controller
        platformController = std::make_shared<PlatformController>("platform_controller");
        if (executor) {
            executor->add_node(platformController);
        }
        platformController->setMaxPlatformVelocity(platformMaxLinVel, platformMaxAngVel);

        // Create timer for control loop (50Hz)
        controlTimer = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&KeloGazeboController::controlLoop, this)
        );
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<KeloGazeboController>("kelo_gazebo_controller", &executor);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}