/*
* The MySensors Arduino library handles the wireless radio link and protocol
* between your home built sensors/actuators and HA controller of choice.
* The sensors forms a self healing radio network with optional repeaters. Each
* repeater and gateway builds a routing tables in EEPROM which keeps track of the
* network topology allowing messages to be routed to nodes.
*
* Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
* Copyright (C) 2013-2017 Sensnology AB
* Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
*
* Documentation: http://www.mysensors.org
* Support Forum: http://forum.mysensors.org
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*/
#ifndef SensorRain_h
#define SensorRain_h

/*
SensorRain
*/

#include "SensorAnalogInput.h"

class SensorRain: public SensorAnalogInput {
public:
	SensorRain(int pin, int child_id = -255): SensorAnalogInput(pin, child_id) {
		_name = "RAIN";
		children.get(1)->setPresentation(S_RAIN);
		children.get(1)->setType(V_RAINRATE);
		children.get(1)->setDescription(_name);
	};	

	// define what to do during setup
	void onSetup() {
		setReference(DEFAULT);
		setOutputPercentage(true);
		setReverse(true);
		setRangeMin(100);
	};
};
#endif