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
#ifndef SensorAM2320_h
#define SensorAM2320_h

/*
SensorAM2320
*/

#include <Wire.h>
#include <AM2320.h>

class SensorAM2320: public Sensor {
protected:
	AM2320* _th;
	
public:
	SensorAM2320(int child_id = -255): Sensor(-1) {
		_name = "AM2320";
		children.allocateBlocks(2);
		new Child(this,FLOAT,nodeManager.getAvailableChildId(child_id),S_TEMP,V_TEMP,_name);
		new Child(this,FLOAT,nodeManager.getAvailableChildId(child_id+1),S_HUM,V_HUM,_name);
	};

	// define what to do during setup
	void onSetup() {
		_th = new AM2320();
	};

	// define what to do during loop
	void onLoop(Child* child) {
		// read data from the sensor
		int status = _th->Read();
		if (status != 0) return;
		// temperature sensor
		if child->getType() == V_TEMP) {
			float temperature = _th->t;
			// store the value
			child->setValue(temperature);
		}
		// Humidity Sensor
		else if (child->getType() == V_HUM) {
			// read humidity
			float humidity = _th->h;
			// store the value
			child->setValue(humidity);
		}
	};
};
#endif