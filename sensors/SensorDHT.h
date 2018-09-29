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
#ifndef SensorDHT_h
#define SensorDHT_h

/*
SensorDHT
*/

#include <DHT.h>

class SensorDHT: public Sensor {
	
protected:
	DHT* _dht;
	int _dht_type;
	float _offset = 0;
	
public:
	SensorDHT(int8_t pin, uint8_t child_id = 255): Sensor(pin) {
		_name = "DHT";
		_dht_type = DHT::DHT11;
		children.allocateBlocks(2);
		new Child(this,FLOAT,nodeManager.getAvailableChildId(child_id),S_TEMP,V_TEMP,_name);
		new Child(this,FLOAT,nodeManager.getAvailableChildId(child_id+1),S_HUM,V_HUM,_name);
	};

	// define what to do during setup
	void onSetup() {
		// store the dht object
		_dht = new DHT();
		// initialize the dht library
		_dht->setup(_pin,(DHT::DHT_MODEL_t)_dht_type);
	};
	
	// define what to do during setup
	void onLoop(Child* child) {
		nodeManager.sleepOrWait(_dht->getMinimumSamplingPeriod());
		_dht->readSensor(true);
		// temperature sensor
		if (child->getType() == V_TEMP) {
			// read the temperature
			float temperature = _dht->getTemperature();
			if (!nodeManager.getIsMetric()) temperature = _dht->toFahrenheit(temperature);
			// store the value
			child->setValue(temperature);
		}
		// humidity sensor
		else if (child->getType() == V_HUM) {
			// read humidity
			float humidity = _dht->getHumidity();
			// store the value
			child->setValue(humidity);
		}
	};
};
#endif