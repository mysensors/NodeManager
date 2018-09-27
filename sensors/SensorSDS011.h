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
#ifndef SensorSDS011_h
#define SensorSDS011_h

/*
SensorSDS011
*/

#include <SDS011.h>

class SensorSDS011: public Sensor {
protected:
	SDS011* _sds;
	float _p10 = 0.;
	float _p25 = 0.;
	int8_t _rx_pin = 6;
	int8_t _tx_pin = 7;
	bool _slp = true;
	
public:
	SensorSDS011(int8_t rxpin, int8_t txpin, uint8_t child_id = 255): Sensor(rxpin){
		_name = "SDS011";
		_rx_pin = rxpin;
		_tx_pin = txpin;
		children.allocateBlocks(2);
		new Child(this,INT,nodeManager.getAvailableChildId(child_id), S_AIR_QUALITY, V_LEVEL, _name);
		new Child(this,INT,nodeManager.getAvailableChildId(child_id+1), S_AIR_QUALITY, V_LEVEL, _name);
	};

	// Sleep sensor during measurment aka stop fan.
	void setSleep(bool value) {
		_slp = value;
	};

	// define what to do during setup
	void onSetup() {
		_sds = new SDS011();
		_sds->begin(_rx_pin, _tx_pin);
		wait(2000);
	};

	// define what to do during loop
	void onLoop(Child* child) {
		if (children.get(1) == child){
			int error;
			// Make sure sensor is running
			_sds -> wakeup();
			// Powering up the fan needs some time.
			if (_slp) wait(4000);
			// Read the particle concentration values
			error = _sds->read(&_p25,&_p10);
			// Stop fan to keep it clean
			if (_slp) _sds->sleep();
			child->setValue(_p10);
		}
		if (children.get(2) == child){
			child->setValue(_p25);
		}
	};
};
#endif