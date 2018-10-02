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
#ifndef SensorMHZ19_h
#define SensorMHZ19_h

/*
SensorMHZ19
*/

#include <SoftwareSerial.h>

class SensorMHZ19: public Sensor {
protected:
	SoftwareSerial* _ser;
	int8_t _tx_pin = 6;
	int8_t _rx_pin = 7;
	
public:
	SensorMHZ19(int8_t rxpin, int8_t txpin, uint8_t child_id = 255): Sensor(rxpin) {
		_name = "MHZ19";
		_rx_pin = rxpin;
		_tx_pin = txpin;
		children.allocateBlocks(1);
		new Child(this,INT,nodeManager.getAvailableChildId(child_id),S_AIR_QUALITY,V_LEVEL,_name);
	};

	// define what to do during setup
	void onSetup() {
		_ser = new SoftwareSerial(_rx_pin, _tx_pin);
		_ser->begin(9600);
		delay(2000);
		// clear CO2 buffer
		while (_ser->read()!=-1) {};  
	};

	// define what to do during setup
	void onLoop(Child* child) {
		// Read the ppm value
		int co2ppm = _readCO2(); 
		// store the value
		child->setValue(co2ppm);
	};

protected:
	// Read out the CO2 data
	int _readCO2() {
		while (_ser->read() != -1) {};  //clear serial buffer
		unsigned char response[9]; // for answer
		byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
		// Command to ask for data.
		_ser->write(cmd, 9); //request PPM CO2
		// Then for 1 second listen for 9 bytes of data.
		_ser->readBytes(response, 9);
		for (int i=0; i<9; i++) debug(PSTR(LOG_SENSOR "%s:READ i=%d v=0x%x\n"),_name,i,response[i]);
		if (response[0] != 0xFF) {
			debug(PSTR("!" LOG_SENSOR "%s:BYTE\n"),_name);
			return -1;
		}
		if (response[1] != 0x86) {
			debug(PSTR("!" LOG_SENSOR "%s:CMD\n"),_name);
			return -1;
		}
		int responseHigh = (int) response[2];
		int responseLow = (int) response[3];
		int ppm = (256 * responseHigh) + responseLow;
		return ppm;
	};
};
#endif