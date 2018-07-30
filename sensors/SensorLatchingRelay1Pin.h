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
#ifndef SensorLatchingRelay1Pin_h
#define SensorLatchingRelay1Pin_h

/*
SensorLatchingRelay1Pin
*/

#include "SensorRelay.h"

class SensorLatchingRelay1Pin: public SensorRelay {
public:
	SensorLatchingRelay1Pin(NodeManager& node_manager, int pin, int child_id = -255): SensorRelay(node_manager, pin, child_id) {
		_name = "LATCHING1PIN";
		children.get(1)->setDescription(_name);
		// set pulse duration
		_pulse_width = 50;
	};
};
#endif