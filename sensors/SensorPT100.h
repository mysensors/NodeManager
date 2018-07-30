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
#ifndef SensorPT100_h
#define SensorPT100_h

/*
	SensorPT100
*/

#include <DFRobotHighTemperatureSensor.h>

class SensorPT100: public Sensor {
protected:
	DFRobotHighTemperature* _PT100;
	float _voltageRef = 3.3;
	
public:
	SensorPT100(NodeManager& node_manager, int pin, int child_id = -255): Sensor(node_manager, pin) {
	_name = "PT100";
	children.allocateBlocks(1);
	new ChildFloat(this,_node->getAvailableChildId(child_id),S_TEMP,V_TEMP,_name);
};

	// [101] set the voltageRef used to compare with analog measures
	void setVoltageRef(float value) {
	_voltageRef = value;
};

	// define what to do during setup
	void onSetup() {
	_PT100 = new DFRobotHighTemperature(_voltageRef); 
	// set the pin as input
	pinMode(_pin, INPUT);
};

// define what to do during loop
	void onLoop(Child* child) {
	// read the PT100 sensor
	int temperature = _PT100->readTemperature(_pin);  
	// store the value
	((ChildFloat*)child)->setValue(temperature);
};

#ifdef USE_CONFIGURATION
	// define what to do when receiving an OTA configuration request
	void onConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 101: setVoltageRef(request->getValueFloat()); break;
		default: return;
		}
	};
#endif
};
#endif