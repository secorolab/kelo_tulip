/******************************************************************************
 * Copyright (c) 2021
 * KELO Robotics GmbH
 *
 * Author:
 * Sushant Chavan
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

#ifndef KELO_DRIVE_HPP
#define KELO_DRIVE_HPP

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

/**
 * @brief Class to store information about and manage an individual Kelo drive
 * that is part of a KELO platform
 *
 */
class KeloDrive {
  public:
    /**
     * @brief Construct a new Kelo Drive object
     *
     * @param name Name of the Kelo drive
     * @param xPos Position of the kelo drive along the x-axis of the base_link
     * @param yPos Position of the kelo drive along the y-axis of the base_link
     * @param zPos Position of the kelo drive along the z-axis of the base_link
     * @param pivotOrientation Initial orientation of the kelo drive pivot with
     * respect to the base_link
     */
    KeloDrive(const std::string& name, double xPos,
              double yPos, double zPos, double pivotOrientation);

    /**
     * @brief Destroy the Kelo Drive object
     *
     */
    virtual ~KeloDrive() {}

    /**
     * @brief Get the position of the kelo drive w.r.t base_link
     *
     * @param xPos
     * @param yPos
     * @param zPos
     */
    void getPos(double& xPos, double& yPos, double& zPos) const;

    /**
     * @brief Set the latest pivot orientation of the kelo drive
     *
     * @param orientation Latest pivot orientation w.r.t base_link
     */
    void setPivotOrientation(double orientation);

    /**
     * @brief Get the latest pivot orientation of the kelo drive
     *
     * @return double Latest pivot orientation w.r.t base_link
     */
    double getPivotOrientation() const { return pivot_orientation_; }

  protected:
    /**
     * @brief Name of the kelo drive
     * 
     */
    std::string name_;

    /**
     * @brief Position of the kelo drive along the x-axis of the base_link
     * 
     */
    double x_pos_;

    /**
     * @brief Position of the kelo drive along the y-axis of the base_link
     * 
     */
    double y_pos_;

    /**
     * @brief Position of the kelo drive along the z-axis of the base_link
     * 
     */
    double z_pos_;

    /**
     * @brief Orientation of the kelo drive pivot with respect to the base_link
     * 
     */
    double pivot_orientation_;
};

#endif // KELO_DRIVE_HPP
