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
#ifndef SensorHCSR04_h
#define SensorHCSR04_h

/*
SensorHCSR04
*/

#include <NewPing.h>

class SensorHCSR04: public Sensor {
protected:
	int _trigger_pin;
	int _echo_pin;
	bool _report_if_invalid = true;
	int _max_distance = 300;
	NewPing* _sonar;
	
public:
	SensorHCSR04(NodeManager& node_manager, int echo_pin, int trigger_pin, int child_id = -255): Sensor(node_manager) {
		_name = "HCSR04";
		_echo_pin = echo_pin;
		_trigger_pin = trigger_pin;
		children.allocateBlocks(1);
		new ChildInt(this,_node->getAvailableChildId(child_id),S_DISTANCE,V_DISTANCE,_name);
	};

	// [103] Maximum distance we want to ping for (in centimeters) (default: 300)
	void setMaxDistance(int value) {
		_max_distance = value;
	};
	// [104] Report the measure even if is invalid (e.g. 0) (default: true)
	void setReportIfInvalid(bool value) {
		_report_if_invalid = value;
	};

	// define what to do during setup
	void onSetup() {
		// initialize the library
		_sonar = new NewPing(_trigger_pin,_echo_pin,_max_distance);
	};

	// define what to do during loop
	void onLoop(Child* child) {
		int distance = _node->getIsMetric() ? _sonar->ping_cm() : _sonar->ping_in();
		if (! _report_if_invalid && distance == 0) return;
		((ChildInt*)child)->setValue(distance);
	};
	
#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 103: setMaxDistance(request->getValueInt()); break;
		case 104: setReportIfInvalid(request->getValueInt()); break;
		default: return;
		}
	};
#endif
};
#endif