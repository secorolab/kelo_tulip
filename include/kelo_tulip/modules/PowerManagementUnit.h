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


#ifndef MODULES_POWERMANAGEMENTUNIT_H
#define MODULES_POWERMANAGEMENTUNIT_H

#include "kelo_tulip/EtherCATModule.h"
#include <boost/thread.hpp>

namespace kelo {

class PowerManagementUnit : public EtherCATModule {
public:
	PowerManagementUnit(int slaveNumber);
	virtual ~PowerManagementUnit();
	
	bool initEtherCAT(ec_slavet* ecx_slaves, int ecx_slavecount);
	bool initEtherCAT2(ecx_contextt* ecx_context, int ecx_slavecount);
	bool step();
	
	const struct PowerManagementUnitProcessDataInput* getProcessDataInput();

	void shutdown(int seconds);
	
private:
	ec_slavet* ecx_slaves;
	int slaveNumber;
	uint16_t status;
	float current;
	float voltage;
	float power;
};

struct __attribute__((packed)) PowerManagementUnitProcessDataInput {
	uint16_t STATUS;     	 	 // Status bits
	uint64_t TIME_STAMP;  	 	 // EtherCAT timestamp ms
	float    CURRENT;  // Total current consumption
	float    VOLTAGE;  // System Voltage
	float    POWER;    // Total power consumption of the system
	float    PARAM1; // Generic data, might be used for different purposes
	uint32_t  PARAM2;	 // Generic data, might be used for different purposes
};

struct __attribute__((packed)) PowerManagementUnitProcessDataOutput {
	uint16_t      SHUTDOWN;
	uint32_t      COMMAND;
};

} // namespace kelp

#endif // MODULES_POWERMANAGEMENTUNIT_H
