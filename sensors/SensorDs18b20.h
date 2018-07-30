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
#ifndef SensorDs18b20_h
#define SensorDs18b20_h

/*
SensorDs18b20
*/

#include <OneWire.h>
#include <DallasTemperature.h>

class SensorDs18b20: public Sensor {
protected:
	bool _sleep_during_conversion = false;
	DallasTemperature* _sensors;

public:
	SensorDs18b20(NodeManager& node_manager, int pin, int child_id = -255): Sensor(node_manager, pin) {
		_name = "DS18B20";
		// initialize the library
		OneWire* oneWire = new OneWire(_pin);
		_sensors = new DallasTemperature(oneWire);
		// initialize the sensors
		_sensors->begin();
		// register a new child for each sensor on the bus
		for(int i = 0; i < _sensors->getDeviceCount(); i++) {
			new ChildFloat(this,_node->getAvailableChildId(child_id+i),S_TEMP,V_TEMP,_getAddress(i));
		}
	};

	// returns the sensor's resolution in bits
	int getResolution() {
		return _sensors->getResolution();
	};
	// [101] set the sensor's resolution in bits
	void setResolution(int value) {
		_sensors->setResolution(value);
	};
	// [102] sleep while DS18B20 calculates temperature (default: false)
	void setSleepDuringConversion(bool value) {
		_sleep_during_conversion = value;
	};
	
	// define what to do during setup
	void onSetup() {
		for (int i = 1; i <= children.size(); i++) {
			children.get(i);
			_node->sendMessage(children.get(i)->getChildId(),V_ID,_getAddress(i-1));
		}
	};

	// define what to do during loop
	void onLoop(Child* child) {
		int index = -1;
		// get the index of the requested child
		for (int i = 1; i <= children.size(); i++) {
			if (children.get(i) == child) index = i-1;
		}
		// do not wait for conversion, will sleep manually during it
		if (_sleep_during_conversion) _sensors->setWaitForConversion(false);
		// request the temperature
		_sensors->requestTemperaturesByIndex(index);
		if (_sleep_during_conversion) {
			// calculate conversion time and sleep
			int16_t conversion_time = _sensors->millisToWaitForConversion(_sensors->getResolution());
			sleep(conversion_time);
		}
		// read the temperature
		float temperature = _sensors->getTempCByIndex(index);
		if (temperature == -127.00 || temperature == 85.00) return;
		// convert it
		temperature = _node->celsiusToFahrenheit(temperature);
		// store the value
		((ChildFloat*)child)->setValue(temperature);
	};
	
#if FEATURE_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 101: setResolution(request->getValueInt()); break;
		case 102: setSleepDuringConversion(request->getValueInt()); break;
		default: return;
		}
	};
#endif

protected:
	// return the address of a device
	char* _getAddress(int index){
		char* charAddr = new char[17];
		DeviceAddress device_address;
		_sensors->getAddress(device_address,index);
		String strAddr = String(device_address[0], HEX);
		for (uint8_t i = 1; i < 8; i++) {
			if (device_address[i] < 16) strAddr = strAddr + 0;
			strAddr = strAddr + String(device_address[i], HEX);
			strAddr.toUpperCase();
		}
		for (int j = 0; j < 16; j++) charAddr[j] = strAddr[j];
		charAddr[16] = 0;
		return charAddr;
	};
};
#endif