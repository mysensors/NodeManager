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
#ifndef SensorDSM501A_h
#define SensorDSM501A_h

/*
SensorDSM501A
*/

class SensorDSM501A: public Sensor {

protected:
	int8_t _pin_10;
	int8_t _pin_25;
	int _temperature = 20;
	
public:
	SensorDSM501A(int8_t pin_10 = 3, int8_t pin_25 = 6, uint8_t child_id = 0): Sensor(-1) {
		_name = "DSM501A";
		_pin_10 = pin_10;
		_pin_25 = pin_25;
		children.allocateBlocks(2);
		new Child(this,INT,nodeManager.getAvailableChildId(child_id),S_DUST,V_LEVEL,_name);
		new Child(this,INT,child_id > 0 ? nodeManager.getAvailableChildId(child_id+1) : nodeManager.getAvailableChildId(child_id),S_DUST,V_LEVEL,_name);
	};
	
	// [101] set the reference temperature for calculating PM1.0
	void setTemperature(int value) {
		_temperature = value;
	}

	// define what to do during setup
	void onSetup() {
		pinMode(_pin_10,INPUT);
		pinMode(_pin_25,INPUT);
	};
	
	// define what to do during loop
	void onLoop(Child* child) {
		if (child == children.get(1)) {
			// get PM 1.0 - density of particles over 1 µm.
			child->setValue((int)((_getPM(_pin_10)*0.0283168/100/1000) *  (0.08205*_temperature)/0.01));
		}
		if (child == children.get(2)) {
			// get PM 2.5 density of particles over 2.5 µm.
			child->setValue((int)_getPM(_pin_25));
		}
	};
	
	// return PM concentration
	long _getPM(int pin) {
		uint32_t duration;
		uint32_t starttime = millis();
		uint32_t endtime;
		uint32_t sampletime_ms = 30000;
		uint32_t lowpulseoccupancy = 0;
		float ratio = 0;
		while (1) {
			duration = pulseIn(pin, LOW);
			lowpulseoccupancy += duration;
			endtime = millis();
			if ((endtime-starttime) > sampletime_ms) {
				ratio = (lowpulseoccupancy-endtime+starttime)/(sampletime_ms*10.0);  // Integer percentage 0=>100
				long concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
				debug_verbose(PSTR(LOG_SENSOR "l=%u r=%d.%02d c=%d\n"),lowpulseoccupancy,(int)ratio, (int)(ratio*100)%100,concentration);
				return(concentration);
			}
		}
	};
	
#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 101: setTemperature(request->getValueInt()); break;
		default: return;
		}
	};
#endif
};
#endif