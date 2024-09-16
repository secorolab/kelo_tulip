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

#include <iostream>
#include "kelo_tulip/modules/PowerManagementUnit.h"

namespace kelo {

PowerManagementUnit::PowerManagementUnit(int slaveNumber) : EtherCATModule() {
	this->slaveNumber = slaveNumber;

	ecx_slaves = 0;

	voltage = 0;
	current = 0;
	power = 0;
}

PowerManagementUnit::~PowerManagementUnit() {
}

bool PowerManagementUnit::initEtherCAT2(ecx_contextt* ecx_context, int ecx_slavecount)
{
	return true;
}

bool PowerManagementUnit::initEtherCAT(ec_slavet* ecx_slaves, int ecx_slavecount)  {
	this->ecx_slaves = ecx_slaves;

	if (!(slaveNumber > 0))
		return false;
		
	std::cout << "Power Management Unit is slave #" << slaveNumber << std::endl; 
	if (slaveNumber > ecx_slavecount) { // slaves start index 1
		std::cout << "Found only " << ecx_slavecount << " EtherCAT slaves, but config requires at least " << slaveNumber << std::endl; 
		return false;
	}
	if (ecx_slaves[slaveNumber].eep_id != 2415923201 && ecx_slaves[slaveNumber].eep_id != 0) {
		std::cout << "EtherCAT slave #" << slaveNumber << " has wrong id: " << ecx_slaves[slaveNumber].eep_id << std::endl;
		return false;
	}

	return true;
}

bool PowerManagementUnit::step() {
	if (slaveNumber == 0)
		return false;

 	PowerManagementUnitProcessDataInput* input = (PowerManagementUnitProcessDataInput*) ecx_slaves[slaveNumber].inputs;
	status = input->STATUS;
	current = input->CURRENT;
	voltage = input->VOLTAGE;
	power = input->POWER;

	PowerManagementUnitProcessDataOutput data;

	data.SHUTDOWN = 0;
	data.COMMAND = 32;
	
	*((PowerManagementUnitProcessDataOutput*) ecx_slaves[slaveNumber].outputs) = data;

	return true;
}

const PowerManagementUnitProcessDataInput* PowerManagementUnit::getProcessDataInput() {
	return (PowerManagementUnitProcessDataInput*) ecx_slaves[slaveNumber].inputs;
}

void PowerManagementUnit::shutdown(int seconds) {
	
}

} //namespace kelo
