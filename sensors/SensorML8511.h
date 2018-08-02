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
#ifndef SensorML8511_h
#define SensorML8511_h

/*
	SensorML8511
*/

class SensorML8511: public Sensor {
public:
	SensorML8511(int pin, int child_id = -255): Sensor(pin) {
		_name = "ML8511";
		children.allocateBlocks(1);
		new Child(this,FLOAT,nodeManager.getAvailableChildId(child_id),S_UV,V_UV,_name);
	};

	// define what to do during setup
	void onSetup() {
		// set the pin as input
		pinMode(_pin, INPUT);
	};

	// define what to do during loop
	void onLoop(Child* child) {
		// read the voltage 
		int uvLevel = analogRead(_pin);
		int refLevel = nodeManager.getVcc()*1024/3.3;
		//Use the 3.3V power pin as a reference to get a very accurate output value from sensor
		float outputVoltage = 3.3 / refLevel * uvLevel;
		//Convert the voltage to a UV intensity level
		float uvIntensity = _mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); 
		// store the value
		child->setValue(uvIntensity);
	};
	
protected:
	
	// The Arduino Map function but for floats
	float _mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	};
};
#endif