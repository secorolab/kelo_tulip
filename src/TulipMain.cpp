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

#include "kelo_tulip/EtherCATMaster.h"
#include "kelo_tulip/PlatformDriverROS.h"
#include "kelo_tulip/modules/PowerManagementUnitROS.h"
#include "kelo_tulip/modules/RobileMasterBatteryROS.h"
#include <rclcpp/rclcpp.hpp>

class PlatformDriver : public rclcpp::Node
{
public:
	PlatformDriver() : Node("platform_driver")
	{
	}

	void initializeModules()
	{
		std::vector<kelo::EtherCATModuleROS *> rosModules;

		std::string configModulesTag = "modules";
		std::string moduleType;

		// list of modules to be created

		std::string platform_driver_module_name = "platform_driver";
		std::string power_management_unit_module_name = "power_management_unit";

		this->declare_parameter("modules." + platform_driver_module_name + ".type", rclcpp::ParameterType::PARAMETER_STRING);
		this->declare_parameter("modules." + power_management_unit_module_name + ".type", rclcpp::ParameterType::PARAMETER_STRING);

		if (!this->get_parameter("modules." + platform_driver_module_name + ".type", moduleType))
		{
			RCLCPP_ERROR(this->get_logger(), "Missing 'modules.%s.type' parameter", platform_driver_module_name.c_str());
			return;
		}
		RCLCPP_INFO(this->get_logger(), "Type of platform_driver_module_name: %s", moduleType.c_str());

		std::string configTag = configModulesTag + "." + platform_driver_module_name + ".";
		kelo::EtherCATModuleROS *module = createModule(moduleType, platform_driver_module_name, configTag);
		if (module)
		{
			rosModules.push_back(module);
		}

		if (!this->get_parameter("modules." + power_management_unit_module_name + ".type", moduleType))
		{
			RCLCPP_ERROR(this->get_logger(), "Missing 'modules.%s.type' parameter", power_management_unit_module_name.c_str());
			return;
		}
		RCLCPP_INFO(this->get_logger(), "Type of power_management_unit_module_name: %s", moduleType.c_str());

		configTag = configModulesTag + "." + power_management_unit_module_name + ".";
		module = createModule(moduleType, power_management_unit_module_name, configTag);
		if (module)
		{
			rosModules.push_back(module);
		}

		if (rosModules.empty())
		{
			RCLCPP_ERROR(this->get_logger(), "No valid modules were created. Shutting down.");
			rclcpp::shutdown();
			return;
		}

		// Legacy config mode for master battery
		int robileMasterBatteryEthercatNumber = this->declare_parameter<int>("robile_master_battery_ethercat_number", 0);

		if (robileMasterBatteryEthercatNumber > 0)
		{
			kelo::EtherCATModuleROS *module = new kelo::RobileMasterBatteryROS(this->shared_from_this());

			// Assuming the `init` function needs a node interface. Pass "this" (the current node).
			if (!module || !module->init(""))
			{
				RCLCPP_ERROR(this->get_logger(), "Failed to initialize RobileMasterBatteryROS module");
				rclcpp::shutdown();
				return;
			}

			rosModules.push_back(module);
		}

		// Collect EtherCAT modules
		std::vector<kelo::EtherCATModule *> etherCATmodules;
		for (size_t i = 0; i < rosModules.size(); i++)
		{
			etherCATmodules.push_back(rosModules[i]->getEtherCATModule());
		}

		// Create and configure EtherCAT master
		std::string device;
		this->declare_parameter("device", rclcpp::ParameterType::PARAMETER_STRING);

		if (!this->get_parameter("device", device))
		{
			RCLCPP_ERROR_STREAM(this->get_logger(), "Missing 'device' parameter");
		}

		double delayRetry =
			this->declare_parameter<double>("start_retry_delay", 1.0);
		int maxRetries = this->declare_parameter<int>("max_retries", 5);

		kelo::EtherCATMaster *master = new kelo::EtherCATMaster(device, etherCATmodules);
		if (!master)
		{
			std::cout << "Failed to create EtherCAT master." << std::endl;
			rclcpp::shutdown();
			return;
		}

		int retryCount = 0;
		while (!master->initEthercat() && retryCount < maxRetries)
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to initialize EtherCAT, attempt %d of %d. Retrying in %.2f s.", retryCount + 1, maxRetries, delayRetry);
			std::this_thread::sleep_for(std::chrono::duration<double>(delayRetry));
			retryCount++;
		}

		if (retryCount >= maxRetries)
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to initialize EtherCAT after %d attempts. Shutting down.", maxRetries);
			rclcpp::shutdown();
			return;
		}

		// Create the executor
		rclcpp::executors::SingleThreadedExecutor executor;
		executor.add_node(this->get_node_base_interface());

		// ROS2 main loop (use an executor)
		rclcpp::Rate rate(100.0); // 20 Hz
		while (rclcpp::ok())
		{
			executor.spin_some();
			for (size_t i = 0; i < rosModules.size(); i++)
			{
				rosModules[i]->step();
			}
			rate.sleep();
		}

		// Delete and close everything
		for (size_t i = 0; i < rosModules.size(); i++)
		{
			delete rosModules[i];
		}

		delete master;
	}

private:

	kelo::EtherCATModuleROS *createModule(const std::string &moduleType, const std::string &moduleName, const std::string &configTag)
	{
		kelo::EtherCATModuleROS *module = nullptr;

		if (moduleType == "robile_master_battery")
		{
			module = new kelo::RobileMasterBatteryROS(this->shared_from_this());
		}
		else if (moduleType == "platform_driver")
		{
			module = new kelo::PlatformDriverROS(this->shared_from_this());
		}
		else if (moduleType == "power_management_unit")
		{
			module = new kelo::PowerManagementUnitROS(this->shared_from_this());
		}
		else
		{
			std::cout << "Unknown module type: " << moduleType << std::endl;
			return nullptr;
		}

		if (!module)
		{
			std::cout << "Module " << moduleName << " could not be created." << std::endl;
			return nullptr;
		}

		if (!module->init(configTag))
		{
			std::cout << "Failed to initialize module " << moduleName << "." << std::endl;
			delete module;
			return nullptr;
		}

		return module;
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	auto platform_driver_node = std::make_shared<PlatformDriver>();

	platform_driver_node->initializeModules();

	rclcpp::spin(platform_driver_node);

	rclcpp::shutdown();
	return 0;
}
