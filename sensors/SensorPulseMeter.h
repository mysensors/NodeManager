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
	float _pulse_factor;
	
public:
	SensorPulseMeter(int8_t pin, uint8_t child_id = 0): Sensor(pin) {
		_name = "PULSE";
		children.allocateBlocks(1);
		new Child(this,DOUBLE,nodeManager.getAvailableChildId(child_id),S_CUSTOM,V_CUSTOM,_name);
		// enable pullup and capture interrupt on the falling edge
		setPinInitialValue(HIGH);
		setInterruptMode(FALLING);
		// since the change in the value can be very short, this ensure we don't miss any interrupt
		setInterruptStrict(false);
#if NODEMANAGER_TIME == ON
		// report at the beginning of the hour the accumulated value of the previous hour
		setReportTimerMode(EVERY_HOUR);
#else
		// report every 60 minutes, assuming we are not running on batteries
		setReportIntervalMinutes(60);
#endif
	};
	
	// [102] set how many pulses for each unit (e.g. 1000 pulses for 1 kwh of power, 9 pulses for 1 mm of rain, etc.)
	void setPulseFactor(float value) {
		_pulse_factor = value;
	};

	// what to do during setup
	void onSetup() {
		// sum up the values of consecutive setValue() calls
		children.get()->setValueProcessing(SUM);
		// send the value even if no interrupt has happened at the end of the reporting interval
		children.get()->setSendWithoutValue(true);
#if NODEMANAGER_EEPROM == ON
		// keep track of the value in EEPROM so to restore it upon a reboot
		children.get()->setPersistValue(true);
#endif
	};

#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 102: setPulseFactor(request->getValueFloat()); break;
		default: return;
		}
	};
#endif
};
#endif