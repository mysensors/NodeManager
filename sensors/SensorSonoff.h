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
#ifndef SensorSonoff_h
#define SensorSonoff_h

/*
SensorSonoff
*/

#include <Bounce2.h>

class SensorSonoff: public Sensor {
protected:
	Bounce _debouncer = Bounce();
	int _button_pin = 0;
	int _relay_pin = 12;
	int _led_pin = 13;
	int _old_value = 0;
	bool _state = false;
	int _relay_on = 1;
	int _relay_off = 0;
	int _led_on = 0;
	int _led_off = 1;
	
public:
	SensorSonoff(NodeManager& node_manager, int child_id = -255): Sensor(node_manager) {
		_name = "SONOFF";
		children.allocateBlocks(1);
		new ChildInt(this,_node->getAvailableChildId(child_id),S_BINARY,V_STATUS,_name);
	};

	// [101] set the button's pin (default: 0)
	void setButtonPin(int value) {
		_button_pin = value;
	};
	// [102] set the relay's pin (default: 12)
	void setRelayPin(int value) {
		_relay_pin = value;
	};
	// [103] set the led's pin (default: 13)
	void setLedPin(int value) {
		_led_pin = value;
	};

	// what to do during setup
	void onSetup() {
		// Setup the button
		pinMode(_button_pin, INPUT_PULLUP);
		// After setting up the button, setup debouncer
		_debouncer.attach(_button_pin);
		_debouncer.interval(5);
		// Make sure relays and LED are off when starting up
		digitalWrite(_relay_pin, _relay_off);
		digitalWrite(_led_pin, _led_off);
		// Then set relay pins in output mode
		pinMode(_relay_pin, OUTPUT);
		pinMode(_led_pin, OUTPUT);
		_blink();
	};

	// what to do during loop
	void onLoop(Child* child) {
		_debouncer.update();
		// Get the update value from the button
		int value = _debouncer.read();
		if (value != _old_value && value == 0) {
			// button pressed, toggle the state
			_toggle(child);
		}
		_old_value = value;
	};

	// what to do as the main task when receiving a message
	void onReceive(MyMessage* message) {
		Child* child = getChild(message->sensor);
		if (child == nullptr) return;
		if (message->getCommand() == C_SET) {
			// retrieve from the message the value to set
			int value = message->getInt();
			if ((value != 0 && value != 1) || value == _state) return;
			// toggle the state
			_toggle(child);
		}
		if (message->getCommand() == C_REQ) {
			// return the current state
			((ChildInt*)child)->setValue(_state);
		}
	};
	
#ifdef USE_CONFIGURATION
	// define what to do when receiving an OTA configuration request
	void onConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 101: setButtonPin(request->getValueInt()); break;
		case 102: setRelayPin(request->getValueInt()); break;
		case 103: setLedPin(request->getValueInt()); break;
		default: return;
		}
	};
#endif

protected:
	// blink the led
	void _blink() {
		digitalWrite(_led_pin, digitalRead(_led_pin) ? _led_on : _led_off);
		wait(200);
		digitalWrite(_led_pin, digitalRead(_led_pin) ? _led_on : _led_off);
		wait(200);
		digitalWrite(_led_pin, digitalRead(_led_pin) ? _led_on : _led_off);
		wait(200);
		digitalWrite(_led_pin, digitalRead(_led_pin) ? _led_on : _led_off);
	};

	// toggle the state
	void _toggle(Child* child) {
		// toggle the state
		_state = _state ? false : true;
		// Change relay state
		digitalWrite(_relay_pin, _state? _relay_on: _relay_off);
		// Change LED state
		digitalWrite(_led_pin, _state? _led_on: _led_off);
		((ChildInt*)child)->setValue(_state);
	};
};
#endif