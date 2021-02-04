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
#ifndef SensorAHT10_h
#define SensorAHT10_h

/*
SensorAHT10: temperature and humidity sensor
*/

#include <Wire.h>
#include <AHT10.h>

class SensorAHT10: public Sensor {
protected:
	AHT10* _aht10;

public:
	SensorAHT10(uint8_t child_id = 0): Sensor(-1) {
		_name = "AHT10";
		children.allocateBlocks(2);
		new Child(this,FLOAT,nodeManager.getAvailableChildId(child_id),S_TEMP,V_TEMP,_name);
		new Child(this,FLOAT,child_id > 0 ? nodeManager.getAvailableChildId(child_id+1) : nodeManager.getAvailableChildId(child_id),S_HUM,V_HUM,_name);
	};
	
	// what to do during setup
	void onSetup() {

		_aht10 = new AHT10(AHT10_ADDRESS_0X38);

		while (_aht10->begin() != true)
  	{
			debug(PSTR(LOG_SETUP "AHT10 not connected or fail to load calibration coefficient\n"));
			setIndication(INDICATION_ERR_HW_INIT);
  	  sleep(5000);
  	}
	};

	// what to do during loop
	void onLoop(Child* child) {
		// temperature sensor
		if (child->getType() == V_TEMP) {
			// read the temperature
			float temperature = _aht10->readTemperature();
			if (temperature != AHT10_ERROR )
			{
				// convert it
				temperature = nodeManager.celsiusToFahrenheit(temperature);
				// store the value
				child->setValue(temperature);
			}
		}
		// Humidity Sensor
		else if (child->getType() == V_HUM) {
			// read humidity
			float humidity = _aht10->readHumidity();
			if (humidity != AHT10_ERROR )
			{
				// store the value
				child->setValue(humidity);
			}
		}
	};
};
#endif
