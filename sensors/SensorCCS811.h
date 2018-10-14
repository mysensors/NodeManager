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
#ifndef SensorCCS811_h
#define SensorCCS811_h

/*
SensorCCS811
*/

#include "Adafruit_CCS811.h"

class SensorCCS811: public Sensor {
protected:
	Adafruit_CCS811* _ccs = new Adafruit_CCS811();
	bool _temperature_sensor = false;

public:
	SensorCCS811(uint8_t child_id = 0): Sensor(-1) {
		_name = "CCS811";
		children.allocateBlocks(2);
		new Child(this,INT,nodeManager.getAvailableChildId(child_id),S_AIR_QUALITY,V_LEVEL,_name);
		new Child(this,INT,child_id > 0 ? nodeManager.getAvailableChildId(child_id+1) : nodeManager.getAvailableChildId(child_id),S_AIR_QUALITY,V_LEVEL,_name);
	};
	
	// [101] set the temperature for calibrating the sensor
	void setTemperature(float value) {
		_ccs->setTempOffset(value - 25.0);
	};
	
	// Set to true if the board has a temperature sensor embedded that can be used for calibration (default: false)
	void setTemperatureSensor(bool value) {
		_temperature_sensor = value;
	};
	
	// what to do during setup
	void onSetup() {
		// initialize the library
		if(!_ccs->begin()) debug(PSTR(LOG_SENSOR "%s: KO\n"),_name);
		if (_temperature_sensor) {
			// use the on-board temperature sensor for calibration
			wait(500);
			setTemperature(_ccs->calculateTemperature());
		}
	};

	// what to do during loop
	void onLoop(Child* child) {
		if (_ccs->available() && ! _ccs->readData()) {
			if (child == children.get(1)) {
				// eCO2
				child->setValue((int)_ccs->geteCO2());
			}
			if (child == children.get(2)) {
				// TVOC
				child->setValue((int)_ccs->getTVOC());
			}
		} else {
			debug(PSTR(LOG_SENSOR "%s: ERROR\n"),_name);
		}
		// we need to wait otherwise during loop, two consecutive readData() would result in an error
		wait(500);
	};
	
#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 101: setTemperature(request->getValueFloat()); break;
		default: return;
		}
	};
#endif
};
#endif