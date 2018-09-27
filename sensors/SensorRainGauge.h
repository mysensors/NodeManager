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
#ifndef SensorRainGauge_h
#define SensorRainGauge_h

/*
	SensorRainGauge
*/

#include "SensorPulseMeter.h"

class SensorRainGauge: public SensorPulseMeter {
public:
	SensorRainGauge(int8_t pin, uint8_t child_id = 255): SensorPulseMeter(pin, child_id) {
		_name = "RAIN_GAUGE";
		children.get()->setFormat(FLOAT);
		children.get()->setPresentation(S_RAIN);
		children.get()->setType(V_RAIN);
		children.get()->setDescription(_name);
		setPulseFactor(9.09);
	};
	
	// what to do when receiving an interrupt
	void onInterrupt() {
		// increment the accumulated value
		children.get()->setValue(1 / _pulse_factor);
	};
	
	// what to do as the main task when receiving a message
	void onReceive(MyMessage* message) {
		Child* child = getChild(message->sensor);
		if (child == nullptr) return;
		if (message->getCommand() == C_REQ && message->type == child->getType()) {
			// send the accumulated value so far
			children.get()->sendValue();
		}
	};
};
#endif