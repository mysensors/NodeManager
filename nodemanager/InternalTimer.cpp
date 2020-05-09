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
InternalTimer: helper class to keep track of the elapsed time
*/

#include "InternalTimer.h"

InternalTimer::InternalTimer() {
	nodeManager.registerTimer(this);
}

// set the timer mode
void InternalTimer::setMode(timer_mode mode) {
	_mode = mode;
}

// get the timer mode
timer_mode InternalTimer::getMode() {
	return _mode;
}

// set the timer value in seconds
void InternalTimer::setValue(unsigned long value) {
	_value = value;
}

// get the timer value in seconds
unsigned long InternalTimer::getValue() {
	return _value;
}

// start/restart the timer
void InternalTimer::start() {
#if NODEMANAGER_TIME == ON
	// target will be the current unix timestamp plus the requested
	if (_mode == TIME_INTERVAL) _target = now() + _value;
	// target will be the next minute/hour/day
	if (_mode == EVERY_MINUTE) _target = minute();
	if (_mode == EVERY_HOUR) _target = hour();
	if (_mode == EVERY_DAY) _target = day();
#else
	// target will be current millis() plus the requested value (in milliseconds)
	_target = millis() + _value * 1000UL;
#endif
	_is_running = true;
}

// stop the timer
void InternalTimer::stop() {
	_is_running = false;
}

// update the timer and keep track of the elapsed time
void InternalTimer::update() {
#if NODEMANAGER_TIME == OFF && NODEMANAGER_SLEEP == ON
	// if a sleeping node and time is not reliable take out from target the time slept in the previous cycle
	if (_mode == TIME_INTERVAL && nodeManager.isSleepingNode()) _target -= nodeManager.getSleepSeconds()*1000UL;
#endif
}

// return true if the time is over
bool InternalTimer::isOver() {
	// time is never over if not configured or instructed to never report
	if (_mode == DO_NOT_REPORT || _mode == NOT_CONFIGURED) return false;
	// timer is always over when reporting immediately
	if (_mode == IMMEDIATELY) return true;
	// timer is never over if not running
	if (! _is_running) return false;
#if NODEMANAGER_TIME == ON
	if (_mode == TIME_INTERVAL) {
		// check if the current unix timestamp is greater than the target
		if (now() >= _target) return true;
		return false;
	}
	// if the minute/hour/day has changed, the timer is over
	if (_mode == EVERY_MINUTE && minute() != _target) return true;
	if (_mode == EVERY_HOUR && hour() != _target) return true;
	if (_mode == EVERY_DAY && day() != _target) return true;
	// if we are in the requested minute/hour/day and not already reported, timer is over
	if (_mode == AT_MINUTE && minute() >= _value && ! _already_reported) {
		_already_reported = true;
		return true;
	}
	if (_mode == AT_HOUR && hour() >= _value && ! _already_reported) {
		_already_reported = true;
		return true;
	}
	if (_mode == AT_DAY && day() >= _value && ! _already_reported) { 
		_already_reported = true;
		return true;
	}
#else
	if (_mode == TIME_INTERVAL) {
		// check for a millis() rollover. The difference between millis() and the target (in seconds) cannot be less than value
		if ( (_target - millis()) >= (_value * 1000UL) ) return true;
		return false;
	}
#endif
	return false;
}
