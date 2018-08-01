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
#ifndef SensorExample_h
#define SensorExample_h

/*
* SensorExample: example for new sensors
*/

// include here required libraries, dependencies or other sensor's header file (if inheriting from another class)
#include <Wire.h>

// declare the name of the class and its inheritance (Sensor or its subclasses)
class SensorExample: public Sensor {
protected:
	// declare here private variables (no functions). As per naming convention they should start with "_"
	float _temperature_offset = -1;

public:
	// contructor, takes at least a NodeManager object. Calls its superclass
	SensorExample(int child_id = -255): Sensor(child_id) {
		// set the sensor name. Useful when reviewing the logs and used as description of the child 
		_name = "EXAMPLE";
		// allocate a fixed number of blocks for the list containing the children
		children.allocateBlocks(1);
		// create one or multiple child. See Child class for the supported formats.
		new ChildFloat(this,nodeManager.getAvailableChildId(child_id),S_TEMP,V_TEMP,_name);
	};

	// implement any function the user can call from the main sketch for configuring the sensor
	void setOffset(float temperature) {
		_temperature_offset = temperature;
	};

	// optionally define what the sensor should do during setup
	void onSetup() {
		Wire.begin();
	};
	
	// optionally define what the sensor should do during loop
	void onLoop(Child* child) {
		// if there are multiple children, identify which one the loop function has to be executed for
		if (child->getType() == V_TEMP) {
			float temperature = 30;
			temperature = _temperatureOffset(temperature);
			// store the value into the child object. NodeManager will take care of sending it back to the gateway
			((ChildFloat*)child)->setValue(temperature);
		}
	};
	
	// optionally define what the sensor should do when receiving a message
	void onReceive(MyMessage* message) {
		// the recipient child id and the payload sent by the user are all available within the MyMessage data structure
	};
	
	// optionally define what the sensor should do when receiving an interrupt
	void onInterrupt() {
		// interrupt has to be registered with NodeManager::setInterruptMode()
	};
	
#if NODEMANAGER_OTA_CONFIGURATION == ON
	// optionally define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		// requested function and value are available from within the ConfigurationRequest data structure
		switch(request->getFunction()) {
		case 101: setOffset((request->getValueFloat()); break;
		default: return;
		}
	};
#endif

protected:
	// add here private functions (private variables have to be decleared above). As per naming convention they should start with "_"
	float _temperatureOffset(float value) {
		return value + _temperature_offset;
	};
};
#endif