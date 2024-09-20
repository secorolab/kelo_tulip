/******************************************************************************
 * Copyright (c) 2021
 * KELO Robotics GmbH
 *
 * Author:
 * Walter Nowak
 * Sebastian Blumenthal
 * Dharmin Bakaraniya
 * Nico Huebel
 * Arthur Ketels
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

#include "kelo_tulip/modules/RobileMasterBatteryROS.h"
#include <memory>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.h>
#include "rclcpp/rclcpp.hpp"

namespace kelo {

RobileMasterBatteryROS::RobileMasterBatteryROS(std::shared_ptr<rclcpp::Node> node)
: EtherCATModuleROS(node), battery(nullptr), topicPrefix("robile_master_battery/") {
	battery = NULL;
	topicPrefix = "robile_master_battery/";
}

RobileMasterBatteryROS::~RobileMasterBatteryROS() {
}

bool RobileMasterBatteryROS::init(const std::string& configPrefix) {
    // get EtherCAT slave number from parameters
    int ethercatNumber = node_->declare_parameter<int>(configPrefix + "ethercat_number", 0);
    if (ethercatNumber <= 0) {
        ethercatNumber = node_->declare_parameter<int>("robile_master_battery_ethercat_number", 0);
        if (ethercatNumber <= 0) {
            RCLCPP_ERROR(node_->get_logger(), "EtherCAT number for robile master battery not set.");
            return false;
        }
    }

    // Create EtherCAT module
    battery = std::make_unique<RobileMasterBattery>(ethercatNumber);
    if (!battery) {
        return false;
    }

    // Setup publishers and subscribers
    batteryPublisher = node_->create_publisher<std_msgs::msg::Float32>(topicPrefix + "voltage", 10);
    processDataInputPublisher = node_->create_publisher<std_msgs::msg::Float64MultiArray>(topicPrefix + "ethercat_input", 10);

    resetErrorSubscriber = node_->create_subscription<std_msgs::msg::Empty>(
        topicPrefix + "reset_error", 10,
        [this](const std_msgs::msg::Empty::SharedPtr msg) { this->callbackResetError(msg); });

    shutdownSubscriber = node_->create_subscription<std_msgs::msg::Int32>(
        topicPrefix + "shutdown", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) { this->callbackShutdown(msg); });

    chargerStartSubscriber = node_->create_subscription<std_msgs::msg::Int32>(
        topicPrefix + "charger_start", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) { this->callbackChargerStart(msg); });

    chargerStopSubscriber = node_->create_subscription<std_msgs::msg::Int32>(
        topicPrefix + "charger_stop", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) { this->callbackChargerStop(msg); });

    return true;
}

bool RobileMasterBatteryROS::step() {
    // Publish voltage
    auto msgBattery = std_msgs::msg::Float32();
    msgBattery.data = battery->getVoltage();
    batteryPublisher->publish(msgBattery);

    publishEthercatInput();

    return true;
}

std::string RobileMasterBatteryROS::getType() {
    return "robile_master_battery";
}

EtherCATModule* RobileMasterBatteryROS::getEtherCATModule() {
    return battery.get();
}

void RobileMasterBatteryROS::callbackResetError(const std_msgs::msg::Empty::SharedPtr msg) {
    battery->resetError();
}

void RobileMasterBatteryROS::callbackShutdown(const std_msgs::msg::Int32::SharedPtr msg) {
    battery->shutdown(msg->data);
}

void RobileMasterBatteryROS::callbackChargerStart(const std_msgs::msg::Int32::SharedPtr msg) {
    battery->startCharge();
}

void RobileMasterBatteryROS::callbackChargerStop(const std_msgs::msg::Int32::SharedPtr msg) {
    battery->stopCharge();
}

void RobileMasterBatteryROS::publishEthercatInput() {
    auto msg = std_msgs::msg::Float64MultiArray();
    const auto* input = battery->getProcessDataInput();

    std_msgs::msg::MultiArrayDimension dim;
    dim.size = 1;
    dim.label = "timestamp";
    msg.data.push_back(input->TimeStamp);
    msg.layout.dim.push_back(dim);

    dim.label = "status";
    dim.stride++;
    msg.data.push_back(input->Status);
    msg.layout.dim.push_back(dim);

    dim.label = "error";
    dim.stride++;
    msg.data.push_back(input->Error);
    msg.layout.dim.push_back(dim);

    dim.label = "warning";
    dim.stride++;
    msg.data.push_back(input->Warning);
    msg.layout.dim.push_back(dim);

    dim.label = "output_current";
    dim.stride++;
    msg.data.push_back(input->OutputCurrent);
    msg.layout.dim.push_back(dim);

    dim.label = "output_voltage";
    dim.stride++;
    msg.data.push_back(input->OutputVoltage);
    msg.layout.dim.push_back(dim);

    dim.label = "output_power";
    dim.stride++;
    msg.data.push_back(input->OutputPower);
    msg.layout.dim.push_back(dim);

    dim.label = "aux_port_current";
    dim.stride++;
    msg.data.push_back(input->AuxPortCurrent);
    msg.layout.dim.push_back(dim);

    dim.label = "generic_data1";
    dim.stride++;
    msg.data.push_back(input->GenericData1);
    msg.layout.dim.push_back(dim);

    dim.label = "generic_data2";
    dim.stride++;
    msg.data.push_back(input->GenericData2);
    msg.layout.dim.push_back(dim);

    dim.label = "bmsm_PwrDeviceId";
    dim.stride++;
    msg.data.push_back(input->bmsm_PwrDeviceId);
    msg.layout.dim.push_back(dim);

    dim.label = "bmsm_Status";
    dim.stride++;
    msg.data.push_back(input->bmsm_Status);
    msg.layout.dim.push_back(dim);

    dim.label = "bmsm_Voltage";
    dim.stride++;
    msg.data.push_back(input->bmsm_Voltage);
    msg.layout.dim.push_back(dim);

    dim.label = "bmsm_Current";
    dim.stride++;
    msg.data.push_back(input->bmsm_Current);
    msg.layout.dim.push_back(dim);

    dim.label = "bmsm_Temperature";
    dim.stride++;
    msg.data.push_back(input->bmsm_Temperature);
    msg.layout.dim.push_back(dim);

    dim.label = "bmsm_SOC";
    dim.stride++;
    msg.data.push_back(input->bmsm_SOC);
    msg.layout.dim.push_back(dim);

    dim.label = "bmsm_SN";
    dim.stride++;
    msg.data.push_back(input->bmsm_SN);
    msg.layout.dim.push_back(dim);

    dim.label = "bmsm_BatData1";
    dim.stride++;
    msg.data.push_back(input->bmsm_BatData1);
    msg.layout.dim.push_back(dim);

    dim.label = "bmsm_BatData2";
    dim.stride++;
    msg.data.push_back(input->bmsm_BatData2);
    msg.layout.dim.push_back(dim);

    processDataInputPublisher->publish(msg);
}

} // namespace kelo
