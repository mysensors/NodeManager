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
#ifndef SensorBH1750_h
#define SensorBH1750_h

/*
SensorBH1750
*/

#include <BH1750.h>
#include <Wire.h>

class SensorBH1750: public Sensor {
		
protected:
	BH1750* _lightSensor;
	
public:
	SensorBH1750(uint8_t child_id = 0): Sensor(-1) {
		_name = "BH1750";
		children.allocateBlocks(1);
		new Child(this,INT,nodeManager.getAvailableChildId(child_id),S_LIGHT_LEVEL,V_LEVEL,_name);
	};

	// [101] set sensor reading mode, e.g. BH1750_ONE_TIME_HIGH_RES_MODE
	void setMode(uint8_t mode) {
		_lightSensor->configure(mode);
	};

	// define what to do during setup
	void onSetup() {
		_lightSensor = new BH1750();
		_lightSensor->begin();
	};

	// define what to do during loop
	void onLoop(Child* child) {
		// request the light level
		int value = _lightSensor->readLightLevel();
		child->setValue(value);
	};
	
#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 101: setMode(request->getValueInt()); break;
		default: return;
		}
	};
#endif
};
#endif