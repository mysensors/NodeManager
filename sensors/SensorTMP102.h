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
#ifndef SensorTMP102_h
#define SensorTMP102_h

/*
SensorTMP102: temperature and humidity sensor
*/

#include <Wire.h>
#include <Sensor_TMP102.h>

class SensorTMP102: public Sensor {
protected:
	Sensor_TMP102* _tmp102;
	uint8_t _address;
	bool _single_shot;

public:
	SensorTMP102(uint8_t address = 0x48, bool single_shot = false, uint8_t child_id = 0): Sensor(-1) {
		_name = "TMP102";
		children.allocateBlocks(1);
		new Child(this,FLOAT,nodeManager.getAvailableChildId(child_id),S_TEMP,V_TEMP,_name);
		_address = address;
		_single_shot = single_shot;
	}

	/* Can be:
	RATE_SINGLE_SHOT (reads temperature only when requested) takes about 50 ms
	RATE_025HZ (reads temperature at 0.25Hz intervals) faster than single shot
	RATE_1HZ (reads temperature at 1Hz intervals)
	etc.
	*/
	void setConversionRate(Sensor_TMP102::conversion_rate rate) {
		_tmp102->setConversionRate(rate);
	}

	// Set the TMP102 to extended mode (temperatures above 128*C)
	void setExtended(bool extended) {
		_tmp102->setExtended(extended);
	}

	// define what to do during setup
	void onSetup() {
		_tmp102 = new Sensor_TMP102(_single_shot);
		_tmp102->begin(_address);
	};

	// define what to do during loop
	void onLoop(Child* child) {
		// temperature sensor
		if (child->getType() == V_TEMP) {
			// read the temperature
			float temperature = _tmp102->readTemperature();
			// convert it
			temperature = nodeManager.celsiusToFahrenheit(temperature);
			// store the value
			child->setValue(temperature);
		}
	};
};
#endif
