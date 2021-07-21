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
#ifndef PowerManager_h
#define PowerManager_h

/******************************************
PowerManager: helper class to power sensors on-demand through the board's pins
*/

class PowerManager {
public:
	PowerManager(int8_t ground_pin, int8_t vcc_pin, unsigned long wait_time = 50);
	// to save battery the sensor can be optionally connected to two pins which will act as vcc and ground and activated on demand
	virtual void setPowerPins(int8_t ground_pin, int8_t vcc_pin, unsigned long wait_time = 50);
	// if enabled the pins will be automatically powered on while awake and off during sleeping
	// turns the power pins on
	virtual void powerOn();
	// turns the power pins on
	virtual void powerOff();
protected:
	int8_t _vcc_pin = -1;
	int8_t _ground_pin = -1;
	unsigned long _wait = 0;
};

#endif
