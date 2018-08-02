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
#ifndef SensorSI7021_h
#define SensorSI7021_h

/*
SensorSI7021: temperature and humidity sensor
*/

#include <Wire.h>
#include "SparkFun_Si7021_Breakout_Library.h"

class SensorSI7021: public Sensor {
protected:
	Weather* _si7021;
	
public:
	SensorSI7021(int child_id = -255): Sensor(-1) {
		_name = "SI7021";
		children.allocateBlocks(2);
		new Child(this,FLOAT,nodeManager.getAvailableChildId(child_id),S_TEMP,V_TEMP,_name);
		new Child(this,FLOAT,nodeManager.getAvailableChildId(child_id+1),S_HUM,V_HUM,_name);
	};
	
	// define what to do during setup
	void onSetup() {
		_si7021 = new Weather();
		_si7021->begin();
	};

	// define what to do during loop
	void onLoop(Child* child) {
		// temperature sensor
		if child->getType() == V_TEMP) {
			// read the temperature
			float temperature = _si7021->getTemp();
			// convert it
			temperature = nodeManager.celsiusToFahrenheit(temperature);
			// store the value
			child->setValue(temperature);
		}
		// Humidity Sensor
		else if (child->getType() == V_HUM) {
			// read humidity
			float humidity = _si7021->getRH();
			// store the value
			child->setValue(humidity);
		}
	};
};
#endif