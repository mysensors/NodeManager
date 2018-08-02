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
#ifndef SensorVL53L0X_h
#define SensorVL53L0X_h

/*
* VL53L0X Laser distance sensor
*/

#include <Wire.h>
#include <VL53L0X.h>

class SensorVL53L0X: public Sensor {
protected:
	VL53L0X *_lox;
	
public:
	SensorVL53L0X(int xshut_pin, int child_id = -255): Sensor(xshut_pin) {
		_name = "VL53L0X";
		children.allocateBlocks(1);
		new Child(this,INT,nodeManager.getAvailableChildId(child_id), S_DISTANCE, V_DISTANCE,_name);
	};

	// define what to do during setup
	void onSetup() {
		if (_pin > 0) {
			pinMode(_pin, OUTPUT); // Put sensor in deep sleep until the loop
			digitalWrite(_pin, LOW);
		}
		_lox = new VL53L0X();
		if (_lox) {
			Wire.begin();
		}
	};

	// define what to do during loop
	void onLoop(Child* child) {
		int val = _getDistance();
		child->setValue(val);
	};
protected:
	// measure the distance
	int _getDistance() {
		int distance = -1;
		if (_lox) {
			// The XSHUT pin puts the sensor into deep sleep when pulled to LOW;
			// To wake up, do NOT write HIGH (=3.3V or 5V) to the pin, as the sensor
			// uses only 2.8V and is not 5V-tolerant. Instead, set the pin to INPUT.
			if (_pin >= 0) {
				pinMode(_pin, INPUT);
				sleep(5); // Transition from HW standby to SW standby might take up to 1.5 ms => use 5ms to be on the safe side
			}
			_lox->init();
			_lox->setTimeout(500);
			distance = _lox->readRangeSingleMillimeters();
			if (_pin >= 0) {
				digitalWrite(_pin, LOW);
				pinMode(_pin, OUTPUT);
			}
		}
		//  if (measure.RangeStatus == 0) {  // only 0  data
		if (_lox->timeoutOccurred()) {
			distance = -1;
		}
		return distance;
	};
};
#endif