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
#ifndef SensorMCP9808_h
#define SensorMCP9808_h

/*
SensorMCP9808
*/

#include <Wire.h>
#include <Adafruit_MCP9808.h>

class SensorMCP9808: public Sensor {
protected:
	Adafruit_MCP9808* _mcp;
	
public:
	SensorMCP9808(NodeManager& node_manager, int child_id = -255): Sensor(node_manager) {
		_name = "MCP9808";
		children.allocateBlocks(1);
		new ChildFloat(this,_node->getAvailableChildId(child_id),S_TEMP,V_TEMP,_name);
	};

	// define what to do during setup
	void onSetup() {
		_mcp = new Adafruit_MCP9808();
	};

	// define what to do during loop
	void onLoop(Child* child) {
		float temperature = _mcp->readTempC();
		// convert it
		temperature = _node->celsiusToFahrenheit(temperature);
		// store the value
		((ChildFloat*)child)->setValue(temperature);
	};
};
#endif