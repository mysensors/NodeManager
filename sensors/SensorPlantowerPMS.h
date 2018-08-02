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
#ifndef SensorPlantowerPMS_h
#define SensorPlantowerPMS_h

/*
SensorPlantowerPMS
*/

#include <PMS.h>
#include <SoftwareSerial.h> 

class SensorPlantowerPMS: public Sensor {
protected:
	int _readSensorValues();
	SoftwareSerial* _ser;
	int _tx_pin = 4;
	int _rx_pin = 3;
	PMS *_pms;
	PMS::DATA _data;
	bool _valuesRead = false;
	bool _valuesReadError = false;
	
public:
	SensorPlantowerPMS(int rxpin, int txpin, int child_id = -255): Sensor(rxpin) {
		_name = "PMS";
		_rx_pin = rxpin;
		_tx_pin = txpin;
		children.allocateBlocks(3);
		// register the child
		new Child(this,INT,nodeManager.getAvailableChildId(child_id), S_DUST, V_LEVEL, "PM1.0");
		new Child(this,INT,nodeManager.getAvailableChildId(child_id+1), S_DUST, V_LEVEL, "PM2.5");
		new Child(this,INT,nodeManager.getAvailableChildId(child_id+2), S_DUST, V_LEVEL, "PM10.0");
	};

	// define what to do during setup
	void onSetup() {
		_ser = new SoftwareSerial(_rx_pin, _tx_pin);
		_pms = new PMS(*_ser);
		_ser->begin(9600);
	};

	// define what to do during loop
	void onLoop(Child* child) {
		if (child == children.get(1)) {
			_valuesRead = false;
			_valuesReadError = false;
		}
		// Read the ppm values
		if (!_valuesRead || _valuesReadError) {
			_valuesReadError = !_pms->readUntil(_data, 1000);
			if (_valuesReadError) {
				debug(PSTR("!" LOG_SENSOR "%s:READ\n"),_name);
				return;
			}
			_valuesRead = true;
		}
		int val = 0;
		if (child == children.get(1)) {
			// PM1.0 values
			val = _data.PM_AE_UG_1_0;
		} else if (child == children.get(2)) {
			// PM 2.5 values
			val = _data.PM_AE_UG_2_5;
		} else if (child == children.get(3)) {
			// PM 10.0 values
			val = _data.PM_AE_UG_10_0;
		} else {
			debug(PSTR("!" LOG_SENSOR "%s:CHILD\n"),_name);
			return;
		}
		// store the value
		child->setValue(val);
	};
};
#endif