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
#ifndef SensorParent_h
#define SensorParent_h

/*
SensorParent: report parent node to the gateway
*/
class SensorParent: public Sensor {
public:
	SensorParent(uint8_t child_id = 254): Sensor(-1) {
		_name = "PARENT";
		children.allocateBlocks(1);
		new Child(this,INT,child_id, S_CUSTOM,V_VAR1,_name);
		setReportIntervalMinutes(60);
	};
		
	// define what to do during loop
	void onLoop(Child* child) {
		int16_t value = transportGetParentNodeId();
		child->setValue(value);
	};
};

#endif