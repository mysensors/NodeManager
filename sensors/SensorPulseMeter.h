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
#ifndef SensorPulseMeter_h
#define SensorPulseMeter_h

/*
	SensorPulseMeter
*/

class SensorPulseMeter: public Sensor {
protected:
	long _count = 0;
	float _pulse_factor;
	
public:
	SensorPulseMeter(NodeManager& node_manager, int pin, int child_id = -255): Sensor(node_manager, pin) {
		_name = "PULSE";
		setPinInitialValue(HIGH);
		setInterruptMode(FALLING);
	};

	// [102] set how many pulses for each unit (e.g. 1000 pulses for 1 kwh of power, 9 pulses for 1 mm of rain, etc.)
	void setPulseFactor(float value) {
		_pulse_factor = value;
	};

	// what to do during loop
	void onLoop(Child* child) {
		// do not report anything if called by an interrupt
		if (_node->getLastInterruptPin() == _interrupt_pin) return;
		// time to report the accumulated value so far
		_reportTotal(child);
		// reset the counter
		_count = 0;
	};

	// what to do as the main task when receiving a message
	void onReceive(MyMessage* message) {
		Child* child = getChild(message->sensor);
		if (child == nullptr) return;
		if (message->getCommand() == C_REQ && message->type == child->getType()) {
			// report the total the last period
			_reportTotal(child);
		}
	};

	// what to do when receiving an interrupt
	void onInterrupt() {
		// increase the counter
		_count++;
		debug(PSTR(LOG_SENSOR "%s:INT++"),_name);
	};

#ifdef USE_CONFIGURATION
	// define what to do when receiving an OTA configuration request
	void onConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 102: setPulseFactor(request->getValueFloat()); break;
		default: return;
		}
	};
#endif

protected:
	// return the total based on the pulses counted
	virtual void _reportTotal(Child* child) {
		((ChildFloat*)child)->setValue(_count / _pulse_factor);
	};
};
#endif