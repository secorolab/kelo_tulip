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



#ifndef VELOCITY_PLATFORM_CONTROLLER_H
#define VELOCITY_PLATFORM_CONTROLLER_H

#include <kelo_tulip/Utils.h>
#include <kelo_tulip/Structs.h>
#include <kelo_tulip/WheelConfig.h>
#include <boost/thread.hpp>
#include <iostream>

extern "C" {
#include <kelo_tulip/KELORobotKinematics.h>
#include <kelo_tulip/PlatformToWheelInverseKinematicsSolver.h>
#include <kelo_tulip/SmartWheelKinematics.h>
#include <gsl/gsl_matrix_double.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_sf_trig.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_vector_double.h>
#include <gsl/gsl_multifit.h>
}

namespace kelo
{

    class VelocityPlatformController
    {

        public:
            VelocityPlatformController();

            virtual ~VelocityPlatformController();

            void initialise(const std::vector<WheelConfig>& wheel_configs);

            void setPlatformMaxLinVelocity(float max_vel_linear);
            void setPlatformMaxAngVelocity(float max_vel_angular);
            void setPlatformMaxLinAcceleration(float max_acc_linear);
            void setPlatformMaxAngAcceleration(float max_acc_angular);
            void setPlatformMaxLinDeceleration(float max_dec_linear);
            void setPlatformMaxAngDeceleration(float max_dec_angular);
            
            void calculatePlatformRampedVelocities();

            void calculateWheelTargetTorques(double *wheel_torques,
                                            double *pivot_angles,
                                            double *wheel_coordinates,
                                            double *pivot_angles_deviation,
                                            double *measured_platform_velocity,
                                            double *platform_damping_parameters,
                                            const gsl_matrix *K,
                                            const gsl_matrix *W,
                                            const unsigned int M,
                                            const unsigned int N);

            void getPivotError(const size_t &wheel_index,
                                const float &raw_pivot_angle,
                                float &pivot_error);

            void calculateWheelTargetVelocity(const size_t &wheel_index,
                                              const float &pivot_angle,
                                              float &target_ang_vel_l,
                                              float &target_ang_vel_r);

            void setPlatformTargetVelocity(const float &vel_x,
                                           const float &vel_y,
                                           const float &vel_a);

            friend std::ostream& operator << (std::ostream &out,
                                              const VelocityPlatformController& controller);

        private:

            std::vector<WheelParamVelocity> wheel_params_;

            Attitude2D platform_target_vel_;
            Attitude2D platform_ramped_vel_;           

        	  struct PlatformLimits {
                float max_vel_linear;
                float max_vel_angular;
                float max_acc_linear;
                float max_acc_angular;
                float max_dec_linear;
                float max_dec_angular;
        	  } platform_limits_;

        	  boost::posix_time::ptime time_last_ramping;
        	  bool first_ramping_call;
    };

} /* namespace kelo */

#endif /* VELOCITY_PLATFORM_CONTROLLER_H */
