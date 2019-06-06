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
#ifndef SensorMCP9808_h
#define SensorMCP9808_h

/*
SensorMCP9808
*/

#include <Wire.h>
#include <Adafruit_MCP9808.h>

class SensorMCP9808: public Sensor {
protected:
	Adafruit_MCP9808* _mcp;
	uint8_t _i2caddress = MCP9808_I2CADDR_DEFAULT;
	uint8_t _resolution = 3;
	bool _sleep = true;

public:
	SensorMCP9808(uint8_t child_id = 0): Sensor(-1) {
		_name = "MCP9808";
		children.allocateBlocks(1);
		new Child(this,FLOAT,nodeManager.getAvailableChildId(child_id),S_TEMP,V_TEMP,_name);
	};
	// set I2C address (default: 0x18)
	void setI2CAddress(uint8_t i2caddress) {
		_i2caddress = i2caddress;
	};
	// set temperature resolution (default: 3)
	void setResolution(uint8_t resolution) {
		_resolution = resolution;
	};
	// sleep sensor after measurment (default: true)
	void setSleep(bool value){
		_sleep = value;
	}
	// define what to do during setup
	void onSetup() {
		_mcp = new Adafruit_MCP9808();
		_mcp->begin(_i2caddress);
		_mcp->setResolution(_resolution);
	};
	// define what to do during loop
	void onLoop(Child* child) {
		// wake up
		_mcp->shutdown_wake(false);
		// sleep based on sample time
		if(_sleep) nodeManager.sleepOrWait(33 << _resolution);
		// get data
		float temperature = _mcp->readTempC();
		// sleep
		if(_sleep) _mcp->shutdown_wake(true);
		// convert it
		temperature = nodeManager.celsiusToFahrenheit(temperature);
		// store the value
		child->setValue(temperature);
	};
};
#endif