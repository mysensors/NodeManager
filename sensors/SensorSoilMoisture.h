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
#ifndef SensorSoilMoisture_h
#define SensorSoilMoisture_h

/*
SensorSoilMoisture
*/

#include "SensorAnalogInput.h"

class SensorSoilMoisture: public SensorAnalogInput {
public:
	SensorSoilMoisture(int8_t pin, uint8_t child_id = 0): SensorAnalogInput(pin, child_id) {
		_name = "SOIL";
		children.get()->setPresentation(S_MOISTURE);
		children.get()->setType(V_LEVEL);
		children.get()->setDescription(_name);
		setReverse(true);
		setRangeMin(100);
		setOutputPercentage(true);
	};

};
#endif