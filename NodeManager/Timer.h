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
#ifndef Timer_h
#define Timer_h

/*
Timer
*/

class NodeManager;

class Timer {
public:
	Timer(NodeManager* node_manager);
	void setMode(timer_mode mode);
	timer_mode getMode();
	void setValue(int value);
	int getValue();
	// start the timer
	void start();
	// stop the timer
	void stop();
	// return true if the time is over
	bool isOver();
	// return elapsed time in seconds
	long getElapsed();
private:
	NodeManager* _node;
	timer_mode _mode = NOT_CONFIGURED;
	int _value = 0;
	bool _is_running = false;
	long _last = 0;
#if NODEMANAGER_TIME == ON
	bool _already_reported = false;
#endif
};

#endif