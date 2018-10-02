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
#ifndef SensorServo_h
#define SensorServo_h

/*
* Servo motor sensor
*/

#include <Servo.h>

class SensorServo: public Sensor {
protected:
	Servo _servo;
	int _value;
	
public:
	SensorServo(int8_t pin, uint8_t child_id = 255): Sensor(pin) {
		_name = "SERVO";
		children.allocateBlocks(1);
		new Child(this,INT,nodeManager.getAvailableChildId(child_id), S_DIMMER, V_PERCENTAGE ,_name);
	};
	
	// set the servo to the given percentage
	void setPercentage(int value) {
		_value = value;
		// set the servo to the given value
		_servo.write(map(_value,0,100,0,180));
		// set the value so to send it back
		Child* child = children.get(1);
		if (child == nullptr) return;
		child->setValue(_value);
	};
	
	// define what to do during setup
	void onSetup() {
		// do not average the value
		children.get()->setValueProcessing(NONE);
		_servo.attach(_pin);
		// report immediately
		setReportTimerMode(IMMEDIATELY);
	};

	// what to do as the main task when receiving a message
	void onReceive(MyMessage* message) {
		Child* child = getChild(message->sensor);
		if (child == nullptr) return;
		if (message->getCommand() == C_SET) setPercentage(message->getInt());
		if (message->getCommand() == C_REQ) child->setValue(_value);
	};
};
#endif