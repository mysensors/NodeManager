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
#ifndef SensorInterrupt_h
#define SensorInterrupt_h

/*
* SensorInterrupt
*/

class SensorInterrupt: public Sensor {
protected:
	bool _invert_value_to_report = false;
#if NODEMANAGER_TIME == ON
	int _threshold = 1;
	int _counter = 0;
	int _current_minute = minute();
#endif

public:
	SensorInterrupt(int pin, int child_id = -255): Sensor(pin) {
		_name = "INTERRUPT";
		children.allocateBlocks(1);
		new Child(this,INT,nodeManager.getAvailableChildId(child_id),S_CUSTOM,V_CUSTOM,_name);
		setPinInitialValue(HIGH);
		setInterruptMode(CHANGE);
	};

	// [105] Invert the value to report. E.g. if FALLING and value is LOW, report HIGH (default: false) 
	void setInvertValueToReport(bool value) {
		_invert_value_to_report = value;
	};
#if NODEMANAGER_TIME == ON
	// [107] when keeping track of the time, trigger only after X consecutive interrupts within the same minute (default: 1)
	void setThreshold(int value) {
		_threshold = value;      
	};
#endif

	// define what to do during setup
	void onSetup() {
		// report immediately
		setReportTimerMode(IMMEDIATELY);
	};

	// what to do as the main task when receiving a message
	void onReceive(MyMessage* message) {
		Child* child = getChild(message->sensor);
		if (child == nullptr) return;
		if (message->getCommand() == C_REQ && message->type == V_STATUS) {
			// return current status
			child->setValue(digitalRead(_pin));
		}
	};

	// what to do when receiving an interrupt
	void onInterrupt() {
#if NODEMANAGER_TIME == ON
		// if this interrupt came in a new minute, reset the counter
		if (minute() != _current_minute) _counter = 0;
		// increase the counter
		_counter = _counter + 1;
#endif
		Child* child = children.get(1);
		// read the value of the pin
		int value = nodeManager.getLastInterruptValue();
		// invert the value if needed
		if (_invert_value_to_report) value = !value;
#if NODEMANAGER_TIME == ON
		// report only when there are at least _threshold triggers
		if (_counter < _threshold) return;
#endif
		child->setValue(value);
	};

#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 105: setInvertValueToReport(request->getValueInt()); break;
		default: return;
		}
	};
#endif
};
#endif