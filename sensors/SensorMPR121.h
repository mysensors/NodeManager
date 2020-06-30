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
#ifndef SensorMPR121_h
#define SensorMPR121_h

#include <Wire.h>
#include "Adafruit_MPR121.h"

#ifndef _BV
#define _BV(bit) (1 << (bit)) 
#endif

/*
SensorMPR121: capacitive Touch control sensor
*/

class SensorMPR121: public Sensor {
protected:
	Adafruit_MPR121 *_cap = new Adafruit_MPR121();
	List<int> _passcode;
	int _passcode_length = 4;
	uint8_t _i2c_addr = 0x5A;
	int _wait_code_for_seconds = 0;
	bool _code_is_valid = false;
	uint16_t _lasttouched = 0;
	uint16_t _currtouched = 0;
	
public:
	SensorMPR121(uint8_t child_id = 0): Sensor(-1) {
		_name = "MPR121";
		children.allocateBlocks(1);
		new Child(this,INT,nodeManager.getAvailableChildId(child_id),S_INFO,V_TEXT,_name);
	};

	// set the passcode length. Passcode will be sent to the controller only after this number of digits have been pressed (default: 4)
	void setPasscodeLength(int value) {
		_passcode_length = value;
	};
	
	// [101] wait for a valid code for the given amount of seconds. Useful when battery powered (default: 0)
	void setWaitCodeForSeconds(int value) {
		_wait_code_for_seconds = value;
	};
	
	// return true if the code was recognized successfully, false otherwise. Useful when a hook function needs to act upon the result
	bool getCodeIsValid() {
		return _code_is_valid;
	};

	// define what to do during setup
	void onSetup() {
		// do not average the value
		children.get()->setValueProcessing(NONE);
		// report immediately
		setReportTimerMode(IMMEDIATELY);
		if (!_cap->begin(_i2c_addr)) debug(PSTR(LOG_SENSOR "%s: KO\n"),_name);
		// setup passcode array  
		_passcode.allocateBlocks(_passcode_length);
	};
	
	// define what to do during loop
	void onLoop(Child* child) {
		_code_is_valid = false;
		// start the timer
		long start_millis = millis();
		while(true) {
			// if a timer is set, leave the cycle if over
			if (_wait_code_for_seconds > 0 && ((millis() - start_millis) > (unsigned long)_wait_code_for_seconds*1000)) break;
			 // Get the currently touched pads
			_currtouched = _cap->touched();
			bool value_set = false;
			for (uint8_t i=0; i<12; i++) {
				if ((_currtouched & _BV(i)) && !(_lasttouched & _BV(i)) ) {
					// pad i touched
					debug(PSTR(LOG_SENSOR "%s(%d):READ v=%d\n"),_name,child->getChildId(),i);
					// add the value to the passcode array
					_passcode.push(i);
					// time to send the passcode back
					if (_passcode.size() == _passcode_length) {
						int passcode = 0;
						// build up the passcode
						for (int i = 1; i <= _passcode.size(); i++) {
							passcode *= 10;
							passcode += (int) _passcode.get(i);
						}
						// store it in the child so it will be sent back 
						child->setValue(passcode);
						// clear the passcode array
						_passcode.clear();
						value_set = true;
						break;
					}
				}
			}
			// value was captured, leave the while cycle
			if (value_set) break;
			// reset state
			_lasttouched = _currtouched;
			wait(100);
		}
	};

#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 101: setWaitCodeForSeconds(request->getValueInt()); break;
		default: return;
		}
	};
#endif
};
#endif