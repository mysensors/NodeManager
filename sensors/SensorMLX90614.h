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
#ifndef SensorMLX90614_h
#define SensorMLX90614_h

/*
SensorMLX90614
*/

#include <Wire.h>
#include <Adafruit_MLX90614.h>

class SensorMLX90614: public Sensor {
protected:
	Adafruit_MLX90614* _mlx;
	int _sensor_type;
	
public:
	SensorMLX90614(uint8_t child_id = 0): Sensor(-1) {
		_name = "MLX90614";
		children.allocateBlocks(2);
		new Child(this,FLOAT,nodeManager.getAvailableChildId(child_id),S_TEMP,V_TEMP,_name);
		new Child(this,FLOAT,child_id > 0 ? nodeManager.getAvailableChildId(child_id+1) : nodeManager.getAvailableChildId(child_id),S_TEMP,V_TEMP,_name);
	};

	// define what to do during setup
	void onSetup() {
		// initialize the library
		_mlx = new Adafruit_MLX90614();
		_mlx->begin();
	};

	// define what to do during loop
	void onLoop(Child* child) {
		float temperature;
		// the first child is the ambient temperature, the second the object temperature
		if (children.get(1) == child) temperature = _mlx->readAmbientTempC();
		else temperature = _mlx->readObjectTempC();
		// convert it
		temperature = nodeManager.celsiusToFahrenheit(temperature);
		child->setValue(temperature);
	};
};
#endif