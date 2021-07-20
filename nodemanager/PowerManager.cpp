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

/******************************************
PowerManager: helper class to power sensors on-demand through the board's pins
*/

#include "PowerManager.h"

PowerManager::PowerManager(int8_t ground_pin, int8_t vcc_pin, unsigned long wait_time) {
	setPowerPins(ground_pin, vcc_pin, wait_time);
}

// set the vcc and ground pin the sensor is connected to
void PowerManager::setPowerPins(int8_t ground_pin, int8_t vcc_pin, unsigned long wait_time) {
	_ground_pin = ground_pin;
	_vcc_pin = vcc_pin;
	if (_ground_pin > 0) {
		// configure the ground pin as output and initialize to low
		pinMode(_ground_pin, OUTPUT);
		digitalWrite(_ground_pin, LOW);
	}
	if (_vcc_pin > 0) {
		// configure the vcc pin as output and initialize to high (power on)
		pinMode(_vcc_pin, OUTPUT);
		digitalWrite(_vcc_pin, HIGH);
	}
	// save wait time
	_wait = wait_time;
}

// turn on the sensor by activating its power pins
void PowerManager::powerOn() {
	if (_vcc_pin == -1) return;
	debug(PSTR(LOG_POWER "ON p=%d\n"),_vcc_pin);
	// power on the sensor by turning high the vcc pin
	digitalWrite(_vcc_pin, HIGH);
	// wait a bit for the device to settle down
	if (_wait > 0) wait(_wait);
}

// turn off the sensor
void PowerManager::powerOff() {
	if (_vcc_pin == -1) return;
	debug(PSTR(LOG_POWER "OFF p=%d\n"),_vcc_pin);
	// power off the sensor by turning low the vcc pin
	digitalWrite(_vcc_pin, LOW);
}
