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
#ifndef SensorACS712_h
#define SensorACS712_h

/*
	SensorACS712
*/

class SensorACS712: public Sensor {
protected:
	int _ACS_offset = 2500;
	int _mv_per_amp = 185;
	
public:
	SensorACS712(int pin, int child_id = -255): Sensor(pin) {
		_name = "ACS712";
		children.allocateBlocks(1);
		new Child(this,FLOAT,nodeManager.getAvailableChildId(child_id),S_MULTIMETER,V_CURRENT,_name);
	};

	// [101] set how many mV are equivalent to 1 Amp. The value depends on the module (100 for 20A Module, 66 for 30A Module) (default: 185);
	void setmVPerAmp(int value) {
		_mv_per_amp = value;
	};
	// [102] set ACS offset (default: 2500);
	void setOffset(int value) {
		_ACS_offset = value;
	};

	// define what to do during setup
	void onSetup() {
		// set the pin as input
		pinMode(_pin, INPUT);
	};

	// define what to do during loop
	void onLoop(Child* child) {
		int value = analogRead(_pin);
		// convert the analog read in mV
		double voltage = (value / 1024.0) * 5000; 
		// convert voltage in amps
		float value_float = ((voltage - _ACS_offset) / _mv_per_amp);
		child->setValue(value_float);
	};
};
#endif