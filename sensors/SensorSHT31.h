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
#ifndef SensorSHT31_h
#define SensorSHT31_h

/*
SensorSHT31: temperature and humidity sensor
*/

#include <Wire.h>
#include "Adafruit_SHT31.h"

class SensorSHT31: public Sensor {
protected:
	Adafruit_SHT31* _sht31;
	
public:
	SensorSHT31(int child_id = -255): Sensor(-1) {
		_name = "SHT31";
		children.allocateBlocks(2);
		new Child(this,FLOAT,nodeManager.getAvailableChildId(child_id),S_TEMP,V_TEMP,_name);
		new Child(this,FLOAT,nodeManager.getAvailableChildId(child_id+1),S_HUM,V_HUM,_name);
	};

	// define what to do during setup
	void onSetup() {
		_sht31 = new Adafruit_SHT31();
		// Set to 0x45 for alternate i2c addr
		_sht31->begin(0x44);
	};

	// define what to do during loop
	void onLoop(Child* child) {
		// temperature sensor
		if child->getType() == V_TEMP) {
			// read the temperature
			float temperature = _sht31->readTemperature();
			// convert it
			temperature = nodeManager.celsiusToFahrenheit(temperature);
			// store the value
			child->setValue(temperature);
		}
		// Humidity Sensor
		else if (child->getType() == V_HUM) {
			// read humidity
			float humidity = _sht31->readHumidity();
			// store the value
			child->setValue(humidity);
		}
	};
};
#endif