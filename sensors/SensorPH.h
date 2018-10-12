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
#ifndef SensorPH_h
#define SensorPH_h

/*
* SensorPH: read the ph from an analog signal amplifier + ph probe
*/

class SensorPH: public Sensor {
protected:
	float _voltage_ref = 5.0;
	float _ph7_voltage = 2.52;
	float _ph4_voltage = 3.04;
	float _ph_step;

public:
	SensorPH(int8_t pin, uint8_t child_id = 255): Sensor(pin) {
		_name = "PH";
		children.allocateBlocks(1);
		new Child(this,FLOAT,nodeManager.getAvailableChildId(child_id),S_WATER_QUALITY,V_PH,_name);
	};
	
	// setter/getter
	void setVoltageRef(float value) {
		_voltage_ref = value;
	};
	void setPH7Voltage(float value) {
		_ph7_voltage = value;
	};
	void setPH4Voltage(float value) {
		_ph4_voltage = value;
	};

	// what the sensor should do during setup
	void onSetup() {
		// set the pin as input
		_ph_step = 1 - ( 1 + (_ph7_voltage - _ph4_voltage) / (7 - 4));
		pinMode(_pin, INPUT);
	};
	
	// what the sensor should do during loop
	void onLoop(Child* child) {
		int buf[10],temp;
		// read the voltage across the amplifier and store 10 measurements
		for (int i=0; i<10; i++) { 
			float adc = analogRead(_pin);
			// calculate the ph
			double reading = _voltage_ref / 1024.0 * adc; 
			float ph = 7 + ((_ph7_voltage - reading) / _ph_step);
			buf[i]= ph * 1000;
			delay(10);
		}
		// ordering : higher to lower values
		for (int i=0; i<9; i++) {
			for (int j=i+1; j<10; j++) {
				if (buf[i] > buf[j]) {
					temp = buf[i];
					buf[i] = buf[j];
					buf[j] = temp;
				}
			}
		}
		// discard the 2 highest and lowest walues and calculate mean of 6 remaining values
		unsigned long int avgValue = 0;
		float ph = 0; 
		for (int i=2; i<8; i++) {
			avgValue += buf[i];
			ph = (float)avgValue/1000/6;  
		}
		child->setValue(ph);
	};
};
#endif