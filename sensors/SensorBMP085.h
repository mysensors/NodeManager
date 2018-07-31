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
#ifndef SensorBMP085_h
#define SensorBMP085_h

/*
SensorBMP085
*/

#include <Wire.h>
#include <Adafruit_BMP085.h>

#include "SensorBosch.h"

class SensorBMP085: public SensorBosch {
protected:
	Adafruit_BMP085* _bm;
	
public:
	SensorBMP085(int child_id = -255): SensorBosch(child_id) {
		_name = "BMP085";
		children.allocateBlocks(3);
		new ChildFloat(this,nodeManager.getAvailableChildId(child_id),S_TEMP,V_TEMP,_name);
		new ChildFloat(this,nodeManager.getAvailableChildId(child_id+1),S_BARO,V_PRESSURE,_name);
		new ChildString(this,nodeManager.getAvailableChildId(child_id+2),S_BARO,V_FORECAST,_name);
	};

	// define what to do during setup
	void onSetup() {
		_bm = new Adafruit_BMP085();
		_bm->begin(detectI2CAddress(0x55));
	};

	// define what to do during loop
	void onLoop(Child* child) {
		// temperature sensor
		if (child->getType() == V_TEMP) {
			// read the temperature
			float temperature = _bm->readTemperature();
			// convert it
			temperature = nodeManager.celsiusToFahrenheit(temperature);
			// store the value
			((ChildFloat*)child)->setValue(temperature);
		}
		// Pressure Sensor
		else if (child->getType() == V_PRESSURE) {
			// read pressure
			float pressure = _bm->readPressure() / 100.0F;
			// store the value
			((ChildFloat*)child)->setValue(pressure);
		}
		// Forecast Sensor
		else if (child->getType() == V_FORECAST) {
			float pressure = _bm->readPressure() / 100.0F;
			((ChildString*)child)->setValue(_forecast(pressure));
		}
	};
};
#endif