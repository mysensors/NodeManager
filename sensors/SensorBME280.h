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
#ifndef SensorBME280_h
#define SensorBME280_h

/*
SensorBME280
*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include "SensorBosch.h"

class SensorBME280: public SensorBosch {
protected:
	Adafruit_BME280* _bm;
	
public:
	SensorBME280(uint8_t child_id = 0): SensorBosch(child_id) {
		_name = "BME280";
#if !defined(NODEMANAGER_SENSOR_BOSCH_LITE)
		children.allocateBlocks(4);
#else
		children.allocateBlocks(3);
#endif
		new Child(this,FLOAT,nodeManager.getAvailableChildId(child_id),S_TEMP,V_TEMP,_name);
		new Child(this,FLOAT,child_id > 0 ? nodeManager.getAvailableChildId(child_id+1) : nodeManager.getAvailableChildId(child_id),S_HUM,V_HUM,_name);
		new Child(this,FLOAT,child_id > 0 ? nodeManager.getAvailableChildId(child_id+2) : nodeManager.getAvailableChildId(child_id),S_BARO,V_PRESSURE,_name);
#if !defined(NODEMANAGER_SENSOR_BOSCH_LITE)
		new Child(this,STRING,child_id > 0 ? nodeManager.getAvailableChildId(child_id+3) : nodeManager.getAvailableChildId(child_id),S_BARO,V_FORECAST,_name);
#endif
	};
	
	// set custom sampling to the sensor
	void setSampling(Adafruit_BME280::sensor_mode mode, Adafruit_BME280::sensor_sampling tempSampling, Adafruit_BME280::sensor_sampling pressSampling, Adafruit_BME280::sensor_sampling humSampling, Adafruit_BME280::sensor_filter filter, Adafruit_BME280::standby_duration duration) {
		_bm->setSampling(mode,tempSampling,pressSampling,humSampling,filter,duration);
	}

	// define what to do during setup
	void onSetup() {
		_bm = new Adafruit_BME280();
		_bm->begin(detectI2CAddress(0x60));
	};

	// define what to do during loop
	void onLoop(Child* child) {
		// temperature sensor
		if (child->getType() == V_TEMP) {
			// read the temperature
			float temperature = _bm->readTemperature();
			// convert it
			temperature = nodeManager.celsiusToFahrenheit(temperature);
			// store the value
			child->setValue(temperature);
		}
		// Humidity Sensor
		else if (child->getType() == V_HUM) {
			// read humidity
			float humidity = _bm->readHumidity();
			// store the value
			child->setValue(humidity);
		}
		// Pressure Sensor
		else if (child->getType() == V_PRESSURE) {
			// read pressure
			float pressure = _bm->readPressure() / 100.0F;
			// store the value
			child->setValue(pressure);
		}
#if !defined(NODEMANAGER_SENSOR_BOSCH_LITE)
		// Forecast Sensor
		else if (child->getType() == V_FORECAST) {
			float pressure = _bm->readPressure() / 100.0F;
			child->setValue(_forecast(pressure));
		}
#endif
	};
};
#endif