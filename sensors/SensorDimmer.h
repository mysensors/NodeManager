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
#ifndef SensorDimmer_h
#define SensorDimmer_h

/*
	SensorDimmer
*/

#include <math.h>

class SensorDimmer: public Sensor {
protected:
	enum _easing_list {
		EASE_LINEAR,
		EASE_INSINE,
		EASE_OUTSINE,
		EASE_INOUTSINE,
	};
	int _percentage = 100;
	int _status = OFF;
	int _easing = EASE_LINEAR;
	int _duration = 1000;
	int _step_duration = 100;
	int _reverse = false;
	
public:
	SensorDimmer(int pin, int child_id = -255): Sensor(pin) {
		_name = "DIMMER";
		children.allocateBlocks(2);
		new ChildInt(this,nodeManager.getAvailableChildId(child_id),S_DIMMER,V_STATUS,_name);
		new ChildInt(this,nodeManager.getAvailableChildId(child_id),S_DIMMER,V_PERCENTAGE,_name);
	};

	// [101] set the effect to use for a smooth transition, can be one of SensorDimmer::EASE_LINEAR, SensorDimmer::EASE_INSINE, SensorDimmer::EASE_OUTSINE, SensorDimmer::EASE_INOUTSINE (default: EASE_LINEAR)
	void setEasing(int value) {
		_easing = value;
	};
	// [102] the duration of entire the transition in seconds (default: 1)
	void setDuration(int value) {
		_duration = value*1000;
	};
	// [103] the duration of a single step of the transition in milliseconds (default: 100)
	void setStepDuration(int value) {
		_duration = value;
	};
	// [104] reverse cathod and anode (default: false)
	void setReverse(bool value) {
		_reverse = value;
	};

	// set the status of the dimmer
	void setStatus(int value) {
		// get the V_STATUS child
		Child* child = children.get(1);
		if (value == ON) {
			// fade the dimmer to the percentage last set
			_fadeTo(child,_percentage);
		}
		else if (value == OFF) {
			// fade the dimmer to 0
			_fadeTo(child,0);
		}
		else return;
		// send the status back
		_status = value;
		((ChildInt*)child)->setValue(_status);
	};

	// set the percentage of the dimmer
	void setPercentage(int value) {
		// get the V_PERCENTAGE child
		Child* child = children.get(2);
		int percentage = value;
		// normalize the provided percentage
		if (percentage < 0) percentage = 0;
		if (percentage > 100) percentage = 100;
		// fade to it
		_fadeTo(child,percentage);
		_percentage = percentage;
		((ChildInt*)child)->setValue(_percentage);
	};

	// define what to do during setup
	void onSetup() {
		pinMode(_pin, OUTPUT);
		// report immediately
		setReportTimerMode(IMMEDIATELY);
	};

	// what to do as the main task when receiving a message
	void onReceive(MyMessage* message) {
		Child* child = getChild(message->sensor);
		if (child == nullptr) return;
		// heandle a SET command
		if (message->getCommand() == C_SET && message->type == child->getType()) {
			// if changing the status
			if (child->getType() == V_STATUS) setStatus(message->getInt());
			// if changing the percentage of the dimmer
			if (child->getType() == V_PERCENTAGE) setPercentage(message->getInt());
		}
		// handle REQ command
		if (message->getCommand() == C_REQ && message->type == child->getType()) {
			// return the current status
			if (child->getType() == V_STATUS) ((ChildInt*)child)->setValue(_status);
			if (child->getType() == V_PERCENTAGE) ((ChildInt*)child)->setValue(_percentage);
		}
	};
	
#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 101: setEasing(request->getValueInt()); break;
		case 102: setDuration(request->getValueInt()); break;
		case 103: setStepDuration(request->getValueInt()); break;
		case 104: setReverse(request->getValueInt()); break;
		default: return;
		}
	};
#endif
protected:
	// fade to the provided value
	void _fadeTo(Child* child, int target_percentage) {
		// count how many steps we need to do
		int steps = _duration / _step_duration;
		int start_from = _percentage;
		if (_status == OFF) start_from = 0;
		// for each step
		for (int current_step = 1; current_step <= steps; current_step++) {
			// calculate the delta between the target value and the current
			int delta = target_percentage - start_from;
			// calculate the smooth transition and adjust it in the 0-255 range
			int value_to_write = (int)(_getEasing(current_step,start_from,delta,steps) / 100. * 255);
			// write to the PWM output
			if (_reverse) analogWrite(_pin,255 - value_to_write);
			else analogWrite(_pin,value_to_write);
			// wait at the end of this step
			wait(_step_duration);
		}
	};

	// for smooth transitions. t: current time, b: beginning value, c: change in value, d: duration
	float _getEasing(float t, float b, float c, float d) {
		if (_easing == EASE_INSINE) return -c * cos(t/d * (M_PI/2)) + c + b;
		else if (_easing == EASE_OUTSINE) return c * sin(t/d * (M_PI/2)) + b;
		else if (_easing == EASE_INOUTSINE) return -c/2 * (cos(M_PI*t/d) - 1) + b;
		else return c*t/d + b;
	};
};
#endif