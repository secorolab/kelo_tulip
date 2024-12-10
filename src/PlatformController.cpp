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
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>

#include "kelo_tulip/PlatformController.h"

PlatformController::PlatformController(const std::string& node_name) 
    : Node(node_name),
    _cmdVelX(0.0),
    _cmdVelY(0.0),
    _cmdVelA(0.0),
    _initialized(false) {
        _tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        _tfListener = std::make_unique<tf2_ros::TransformListener>(*_tfBuffer);

        // Setup subscribers
        _jointStatesSubscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::QoS(10).reliable().transient_local(),
        std::bind(&PlatformController::jointStatesCallBack, this, std::placeholders::_1)
        );

        // Setup publishers
        base_velocity_control_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/base_velocity_controller/commands", 10);
    }

    void PlatformController::setCmdVel(double vx, double vy, double va) {
        _cmdVelX = vx;
        _cmdVelY = vy;
        _cmdVelA = va;
    }

    void PlatformController::setMaxPlatformVelocity(double linearVel, double angularVel) {
        _controller.setPlatformMaxLinVelocity(linearVel);
        _controller.setPlatformMaxAngVelocity(angularVel);
    }

    void PlatformController::step() {
        if (_initialized) {
            _controller.setPlatformTargetVelocity(_cmdVelX, _cmdVelY, _cmdVelA);
            _controller.calculatePlatformRampedVelocities();
            std_msgs::msg::Float64MultiArray message;
            size_t array_size = 8;
            message.data.resize(array_size);

            for (const auto& drive: _drives) {
                const std::string& driveName = drive.first;
                int wheelNumber = _wheelConfigs[driveName].ethercatNumber;
                float left_whl_sp, right_whl_sp;
                _controller.calculateWheelTargetVelocity(wheelNumber, drive.second.getPivotOrientation(), left_whl_sp, right_whl_sp);
                message.data[2*wheelNumber] = left_whl_sp;
                message.data[2*wheelNumber+1] = -right_whl_sp;
            }
            setAllHubWheelVelocities(message);
        }
    }

    void PlatformController::initDrives(const std::map<std::string, double>& pivotJointData) {
        if (!_drives.empty())
            return;

        auto now = this->get_clock()->now();
        for (auto& joint: pivotJointData) {
            std::string driveName = getPivotName(joint.first); 
            // Eg: from "eddie_rear_right_drive_pivot_joint" to "eddie_rear_right_drive_struct_link"
            std::string pivotLink = driveName + "_drive_struct_link";
            
            geometry_msgs::msg::TransformStamped transform;
            try {
                transform = _tfBuffer->lookupTransform("eddie_base_link", pivotLink, now, rclcpp::Duration::from_seconds(2.0));
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
                _drives.clear();
                return;
            }

            _drives.insert(std::make_pair(driveName,
                                          KeloDrive(driveName, 
                                                    transform.transform.translation.x, 
                                                    transform.transform.translation.y, 
                                                    transform.transform.translation.z,
                                                    joint.second)));
        }

        std::vector<kelo::WheelConfig> wheelConfigsVector;
        int wheelNumber = 0;
        double zDummy = 0;
        for (const auto& drive: _drives) {
            kelo::WheelConfig wc;
            wc.ethercatNumber = drives_map[drive.first];
            drive.second.getPos(wc.x, wc.y, zDummy);
            wc.a = 0; // assume zero in simulation
            _wheelConfigs[drive.first] = wc;
            wheelConfigsVector.push_back(wc);
            wheelNumber++;
        }
        _controller.initialise(wheelConfigsVector);

        _initialized = true;
        RCLCPP_INFO(this->get_logger(), "Initialized %lu Kelo drives", _drives.size());
    }

    void PlatformController::jointStatesCallBack(const sensor_msgs::msg::JointState::SharedPtr msg) {
        const std::vector<std::string>& jointNames = msg->name;
        const std::vector<double>& jointPositions = msg->position;

        if (jointNames.empty() || jointNames.size() != jointPositions.size()) {
            return;
        }

        std::map<std::string, double> pivotJointData;
        std::string pivotJointNameIdentifier = "pivot_joint";
        for (size_t i = 0; i < jointNames.size(); ++i) {
            std::string jointName = jointNames[i];
            if (jointName.find(pivotJointNameIdentifier) != std::string::npos) {
                double pivotAngle = jointPositions[i] - (int(jointPositions[i] / (2*M_PI)) * 2 * M_PI);
                pivotJointData[jointName] = pivotAngle;
            }
        }

        if (_drives.empty()) {
            initDrives(pivotJointData);
        } else {
            setPivotOrientations(pivotJointData);
        }
    }

    void PlatformController::setPivotOrientations(const std::map<std::string, double>& pivotJointData) {
        for (const auto& joint: pivotJointData) {
            std::string driveName = getPivotName(joint.first);
            if (_drives.find(driveName) == _drives.end()) {
                RCLCPP_ERROR(this->get_logger(), "Cannot set pivot orientation for drive %s", driveName.c_str());
                continue;
            }
            KeloDrive& drive = _drives.at(driveName);
            drive.setPivotOrientation(joint.second);
            // std::cout << "Drive: " << driveName << " Pivot: " << joint.second << std::endl;
        }
    }

    std::string PlatformController::getPivotName(const std::string& jointName) {
        return jointName.substr(0, jointName.find("_drive_"));
    }

    void PlatformController::setAllHubWheelVelocities(const std_msgs::msg::Float64MultiArray& msg) {
        if (!base_velocity_control_pub_) {
            RCLCPP_WARN(this->get_logger(), "Publisher is not initialized!");
            return;
        }
        base_velocity_control_pub_->publish(msg);        
    }
