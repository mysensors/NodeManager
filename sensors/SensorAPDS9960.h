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
#ifndef SensorAPDS9960_h
#define SensorAPDS9960_h

/*
* SparkFun RGB and Gesture Sensor
*/

#include <Wire.h>
#include <SparkFun_APDS9960.h>

class SensorAPDS9960: public Sensor {
protected:
	SparkFun_APDS9960* _apds;
	
public:
	SensorAPDS9960(int8_t pin, uint8_t child_id = 255): Sensor(pin) {
		_name = "APDS9960";
		children.allocateBlocks(1);
		new Child(this,STRING,nodeManager.getAvailableChildId(child_id),S_INFO,V_TEXT,_name);
		setPinInitialValue(HIGH);
		setInterruptMode(FALLING);
	};

	// define what to do during setup
	void onSetup() {
		// do not average the value
		children.get()->setValueProcessing(NONE);
		_apds = new SparkFun_APDS9960();
		pinMode(_pin, INPUT);
		// report immediately
		setReportTimerMode(IMMEDIATELY);
		// initialize the library
		_apds->init();
		_apds->enableGestureSensor(true);
	};

	// what to do on interrupt
	void onInterrupt() {
		const char* gesture = "";
		if ( _apds->isGestureAvailable() ) {
			switch ( _apds->readGesture() ) {
			case DIR_UP: gesture = "UP"; break;
			case DIR_DOWN: gesture = "DOWN"; break;
			case DIR_LEFT: gesture = "LEFT"; break;
			case DIR_RIGHT: gesture = "RIGHT"; break;
			case DIR_NEAR: gesture = "NEAR"; break;
			case DIR_FAR: gesture = "FAR"; break;
			default: gesture = "NONE"; break;
			}
			// store it in the child so it will be sent back 
			children.get()->setValue(gesture);
		}
	};
};
#endif