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
Timer: helper class to keep track of the elapsed time
*/

#include "Timer.h"

Timer::Timer() {
}

void Timer::setMode(timer_mode mode) {
	_mode = mode;
}
timer_mode Timer::getMode() {
	return _mode;
}

void Timer::setValue(int value) {
	_value = value;
}

int Timer::getValue() {
	return _value;
}

// start the timer
void Timer::start() {
	// reset the timer
	_last = 0;
	_is_running = true;
#if NODEMANAGER_TIME == ON
	// save the current timestamp (which is sync'ed when sleeping or not sleeping)
	if (_mode == TIME_INTERVAL) _last = now();
	// keep track of the current minute/hour/day
	if (_mode == EVERY_MINUTE) _last = minute();
	if (_mode == EVERY_HOUR) _last = hour();
	if (_mode == EVERY_DAY) _last = day();
#else
	// keep track of millis() for calculating the difference
	_last = millis();
#endif
}

// stop the timer
void Timer::stop() {
	_is_running = false;
}

// return true if the time is over
bool Timer::isOver() {
	if (_mode == DO_NOT_REPORT || _mode == NOT_CONFIGURED) return false;
	if (_mode == IMMEDIATELY) return true;
	if (_mode == TIME_INTERVAL) {
		if (! _is_running) return false;
		long elapsed = getElapsed();
		// check if time has elapsed or millis has started over
		if (elapsed >= _value || elapsed < 0) return true;
		return false;
	}
#if NODEMANAGER_TIME == ON
	// if the minute/hour/day has changed, the timer is over
	if (_mode == EVERY_HOUR && hour() != _last) return true;
	if (_mode == EVERY_HOUR && hour() != _last) return true;
	if (_mode == EVERY_DAY && day() != _last) return true;
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
#endif
	return false;
}

// return elapsed seconds so far
long Timer::getElapsed() {
	// calculate the elapsed time
	long elapsed = 0;
#if NODEMANAGER_TIME == ON
	// system time is available so use now() to calculated the elapsed time
	elapsed = (long)(now() - _last);
#else
	// system time is not available
#if NODEMANAGER_SLEEP == ON
	// millis() is not reliable while sleeping so calculate how long a sleep cycle would last in seconds and update the elapsed time
	if (nodeManager.isSleepingNode()) elapsed += nodeManager.getSleepSeconds();
#endif
	// use millis() to calculate the elapsed time in seconds
	if (! nodeManager.isSleepingNode()) elapsed = (long)((millis() - _last)/1000);
#endif
	return elapsed;
}