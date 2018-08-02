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
#ifndef SensorSHT21_h
#define SensorSHT21_h

/*
SensorSHT21: temperature and humidity sensor
*/

#include <Wire.h>
#include <Sodaq_SHT2x.h>

class SensorSHT21: public Sensor {
public:
	SensorSHT21(int child_id = -255): Sensor(-1) {
		_name = "SHT21";
		children.allocateBlocks(2);
		new Child(this,FLOAT,nodeManager.getAvailableChildId(child_id),S_TEMP,V_TEMP,_name);
		new Child(this,FLOAT,nodeManager.getAvailableChildId(child_id+1),S_HUM,V_HUM,_name);
	};
	
	// what to do during setup
	void onSetup() {
		// initialize the library
		Wire.begin();
	};

	// what to do during loop
	void onLoop(Child* child) {
		// temperature sensor
		if child->getType() == V_TEMP) {
			// read the temperature
			float temperature = SHT2x.GetTemperature();
			// convert it
			temperature = nodeManager.celsiusToFahrenheit(temperature);
			// store the value
			child->setValue(temperature);
		}
		// Humidity Sensor
		else if child->getType() == V_HUM) {
			// read humidity
			float humidity = SHT2x.GetHumidity();
			// store the value
			child->setValue(humidity);
		}
	};
};
#endif