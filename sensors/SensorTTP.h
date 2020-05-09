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
#ifndef SensorTTP_h
#define SensorTTP_h

/*
SensorTTP: TTP226/TTP229 Touch control sensor
*/

class SensorTTP: public Sensor {
protected:
	int8_t _clock_pin = 6;
	int8_t _sdo_pin = 5;
	int8_t _dv_pin = 3;
	int8_t _rst_pin = 4;
	int key = 0;
	int count = 0;
	List<int> _passcode;
	int _passcode_length = 4;
	
public:
	SensorTTP(uint8_t child_id = 0): Sensor(-1) {
		_name = "TTP";
		children.allocateBlocks(1);
		new Child(this,INT,nodeManager.getAvailableChildId(child_id),S_INFO,V_TEXT,_name);
		setPinInitialValue(LOW);
		setInterruptMode(RISING);
	};

	// set the passcode length. Passcode will be sent to the controller only after this number of digits have been pressed (default: 4)
	void setPasscodeLength(int value) {
		_passcode_length = value;
	};
	// set the clock pin (default: 6)
	void setClockPin(int8_t value) {
		_clock_pin = value;
	};
	// set the SDO pin (default: 5)
	void setSdoPin(int8_t value) {
		_sdo_pin = value;
	};
	// set the DV pin (default: 3)
	void setDvPin(int8_t value) {
		_dv_pin = value;
	};
	// set the RST pin (default: 4)
	void setRstPin(int8_t value) {
		_rst_pin = value;
	};

	// define what to do during setup
	void onSetup() {
		// do not average the value
		children.get()->setValueProcessing(NONE);
		// setup passcode array  
		_passcode.allocateBlocks(_passcode_length);
		// initialize pins
		pinMode(_dv_pin, INPUT);
		pinMode(_sdo_pin, INPUT);
		pinMode(_rst_pin, OUTPUT); 
		pinMode(_clock_pin, OUTPUT);
		// this will allow to register the interrupt
		_pin = _dv_pin;
		// report immediately
		setReportTimerMode(IMMEDIATELY);
		digitalWrite(_rst_pin, LOW);
	};

	// what to do when receiving an interrupt
	void onInterrupt() {
		Child* child = children.get(1);
		// fetch the key pressed from the keypad
		int value = _fetchData();
		// invalid value, return
		if (value == 0) return;
		debug(PSTR(LOG_SENSOR "%s(%d):READ v=%d\n"),_name,child->getChildId(),value);
		// add the value to the passcode array
		_passcode.push(value);
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
		}
	};
protected:
	// Send 8 clock pulses and check each data bit as it arrives
	int _fetchData() {
		count = 0;
		for(int i = 1; i < 9; i++) {       
			digitalWrite(_clock_pin,1);
			delayMicroseconds(1000);
			// If data bit, high, then that key was pressed.
			if(digitalRead(_sdo_pin) == HIGH)  
			key=i; 
			else 
			count++;
			digitalWrite(_clock_pin,0);
			// Don't use delay(1) as it will mess up interrupts
			delayMicroseconds(1000);  
		}
		if(key > 0 && count == 7) return key;
		return 0;
	};
};
#endif