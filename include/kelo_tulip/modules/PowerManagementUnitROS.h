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

#ifndef MODULES_POWERMANAGEMENTUNITROS_H
#define MODULES_POWERMANAGEMENTUNITROS_H

#include "kelo_tulip/modules/PowerManagementUnit.h"
#include "kelo_tulip/EtherCATModuleROS.h"
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <rclcpp/rclcpp.hpp>

namespace kelo {

//! ROS2 interface for PowerManagementUnit

class PowerManagementUnitROS : public EtherCATModuleROS {
public:
    PowerManagementUnitROS(std::shared_ptr<rclcpp::Node> node);

    virtual ~PowerManagementUnitROS();

    virtual bool init(const std::string& configPrefix);

    virtual bool step();

    virtual std::string getType();

    virtual EtherCATModule* getEtherCATModule();
    
protected:
    PowerManagementUnit* pmu;

};

} // namespace kelo

#endif // MODULES_POWERMANAGEMENTUNITROS_HPP