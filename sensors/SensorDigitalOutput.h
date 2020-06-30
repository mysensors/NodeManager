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
#ifndef SensorDigitalOutput_h
#define SensorDigitalOutput_h

/*
SensorDigitalOutput: control a digital output of the configured pin
*/
class SensorDigitalOutput: public Sensor {
protected:
	int _status = OFF;
	int8_t _pin_off = -1;
	bool _legacy_mode = false;
	bool _input_is_elapsed = false;
	int _wait_after_set = 0;
	int _pulse_width = 0;
	bool _invert_value_to_write = false;
	InternalTimer* _safeguard_timer = new InternalTimer();
	
public:
	SensorDigitalOutput(int8_t pin, uint8_t child_id = 0): Sensor(pin) {
		_name = "DIGITAL_O";
		children.allocateBlocks(1);
		new Child(this,INT,nodeManager.getAvailableChildId(child_id),S_CUSTOM,V_CUSTOM,_name);
	};
	
	// [104] when legacy mode is enabled expect a REQ message to trigger, otherwise the default SET (default: false)
	void setLegacyMode(bool value) {
		_legacy_mode = value;
	};
	// [105] automatically turn the output off after the given number of minutes
	void setSafeguard(int value) {
		_safeguard_timer->setMode(TIME_INTERVAL);
		_safeguard_timer->setValue(value*60);
	};
	// [106] if true the input value becomes a duration in minutes after which the output will be automatically turned off (default: false)
	void setInputIsElapsed(bool value) {
		_input_is_elapsed = value;
	};
	// [107] optionally wait for the given number of milliseconds after changing the status (default: 0)
	void setWaitAfterSet(int value) {
		_wait_after_set = value;
	};
	// [108] when switching on, turns the output off after the given number of milliseconds. For latching relay controls the pulse width (default: 0)
	void setPulseWidth(int value) {
		_pulse_width = value;
	};
	// [109] Invert the value to write. E.g. if ON is received, write LOW (default: false) 
	void setInvertValueToWrite(bool value) {
		_invert_value_to_write = value;
	};
	// [110] for a 2-pins latching relay, set the pin which turns the relay off (default: -1)
	void setPinOff(int8_t value) {
		_pin_off = value;
	};

	// manually switch the output to the provided status (ON or OFF)
	void setStatus(int requested_status) {
		// the input provided is an elapsed time
		if (_input_is_elapsed) {
			int elapsed = requested_status;
			// received a request to turn it off immediately, stop the timer
			if (elapsed == 0) _safeguard_timer->stop();
			else {
				// configure the timer with the provided elapsed time and start it
				_safeguard_timer->setValue(elapsed*60);
				_safeguard_timer->start();
				// set the status just to ON, not to the requested elapsed
				requested_status = ON;
			}
		} else {
			// if turning the output on and a safeguard timer is configured, start it
			if (requested_status == ON && _safeguard_timer->getMode() == TIME_INTERVAL) _safeguard_timer->start();
		}
		// switch the actual output
		_switchOutput(requested_status);
		// store the new status so it will be sent to the controller
		_status = requested_status;
		children.get()->setValue(_status);
		// wait if needed for relays drawing a lot of current
		if (_wait_after_set > 0) nodeManager.sleepOrWait(_wait_after_set);
	};

	// toggle the status
	void toggleStatus() {
		setStatus(!_status);
	};

	// get the current state
	int getStatus() {
		return _status;
	};

	// what to do during setup
	void onSetup() {
		// do not average the value
		children.get()->setValueProcessing(NONE);
		// setup the pin
		pinMode(_pin, OUTPUT);
		// setup the off pin if needed
		if (_pin_off > 0) pinMode(_pin_off, OUTPUT);
		// report immediately
		setReportTimerMode(IMMEDIATELY);
#if NODEMANAGER_EEPROM == ON
		// keep track of the value in EEPROM so to restore it upon a reboot
		children.get()->setPersistValue(true);
#else
		// turn the output off by default
		setStatus(OFF);
#endif
	};

	// what to do during loop
	void onLoop(Child* child) {
#if NODEMANAGER_EEPROM == ON
		// if this is the first time running loop and a value has been restored from EEPROM, turn the output according the last value
		if (_first_run) setStatus(children.get()->getValueInt());
#endif
		// if the time is over, turn the output off
		if (_safeguard_timer->isOver()) {
			setStatus(OFF);
			_safeguard_timer->stop();
		}
	};

	// what to do when receiving a message
	void onReceive(MyMessage* message) {
		Child* child = getChild(message->sensor);
		if (child == nullptr) return;
		// by default handle a SET message but when legacy mode is set when a REQ message is expected instead
		if ( (message->getCommand() == C_SET && ! _legacy_mode) || (message->getCommand() == C_REQ && _legacy_mode)) {
			// switch the output
			setStatus(message->getInt());
		}
		if (message->getCommand() == C_REQ && ! _legacy_mode) {
			// just return the current status
			child->setValue(_status);
		}
	};
	
#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 104: setLegacyMode(request->getValueInt()); break;
		case 105: setSafeguard(request->getValueInt()); break;
		case 106: setInputIsElapsed(request->getValueInt()); break;
		case 107: setWaitAfterSet(request->getValueInt()); break;
		case 108: setPulseWidth(request->getValueInt()); break;
		case 109: setInvertValueToWrite(request->getValueInt()); break;
		case 110: setPinOff(request->getValueInt()); break;
		default: return;
		}
	};
#endif
	
protected:
	// switch to the requested status
	virtual void _switchOutput(int requested_status) {
		// select the right pin
		int pin = _pin;
		if (_pin_off > 0 && requested_status == OFF) pin = _pin_off;
		// set the value to write. If pulse (latching relay), value is always ON, otherwise is the requested status
		int value = _pulse_width > 0 ? ON : requested_status;
		// invert the value to write if needed. E.g. if ON is received, write LOW, if OFF write HIGH
		if (_invert_value_to_write) value = !value;
		// write the value to the pin
		digitalWrite(pin, value);
		// if pulse is set wait for the given timeframe before restoring the value to the original value
		if (_pulse_width > 0) {
			nodeManager.sleepOrWait(_pulse_width);
			digitalWrite(pin, ! value);
		}
	};
};
#endif