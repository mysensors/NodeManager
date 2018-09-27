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
#ifndef SensorDigitalInput_h
#define SensorDigitalInput_h

/*
SensorDigitalInput: read the digital input of the configured pin
*/
class SensorDigitalInput: public Sensor {
public:
	SensorDigitalInput(int8_t pin, uint8_t child_id = 255): Sensor(pin) {
		_name = "DIGITAL_I";
		children.allocateBlocks(1);
		new Child(this,INT,nodeManager.getAvailableChildId(child_id),S_CUSTOM,V_CUSTOM,_name);
	};

	// define what to do during setup
	void onSetup() {
		// set the pin for input
		pinMode(_pin, INPUT);
	};

	// define what to do during loop
	void onLoop(Child* child) {
		// read the value
		int value = digitalRead(_pin);
		// store the value
		child->setValue(value);
	};
};
#endif