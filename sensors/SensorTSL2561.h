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
#ifndef SensorTSL2561_h
#define SensorTSL2561_h

/*
SensorTSL2561
*/

#include <TSL2561.h>
#include <Wire.h>

class SensorTSL2561: public Sensor {
protected:
	TSL2561* _tsl;
	int _tsl_address = 0;
	int _tsl_gain = 1;
	int _tsl_timing = 0;
	int _tsl_spectrum = 0;
public:
	// constants
	const static int ADDR_FLOAT = 0;
	const static int ADDR_LOW = 1;
	const static int ADDR_HIGH = 2;
	const static int GAIN_0X = 0;
	const static int GAIN_16X = 1;
	const static int INTEGRATIONTIME_13MS = 0;
	const static int INTEGRATIONTIME_101MS = 1;
	const static int INTEGRATIONTIME_402MS = 2;
	const static int VISIBLE = 0;
	const static int FULLSPECTRUM = 1;
	const static int INFRARED = 2;
	const static int FULL = 3;
	
	SensorTSL2561(int child_id = -255): Sensor(-1) {
		_name = "TSL2561";
		children.allocateBlocks(1);
		new ChildInt(this,nodeManager.getAvailableChildId(child_id),S_LIGHT_LEVEL,V_LEVEL,_name);
	};

	// [101] set the gain, possible values are SensorTSL2561::GAIN_0X (0), SensorTSL2561::GAIN_16X (1) (default 16x)
	void setGain(int value) {
		_tsl_gain = value;
	};
	// [102] set the timing, possible values are SensorTSL2561::INTEGRATIONTIME_13MS (0), SensorTSL2561::INTEGRATIONTIME_101MS (1), SensorTSL2561::INTEGRATIONTIME_402MS (2) (default: 13ms)
	void setTiming(int value) {
		_tsl_timing = value;
	};
	// [103] set the spectrum, possible values are SensorTSL2561::VISIBLE (0), SensorTSL2561::FULLSPECTRUM (1), SensorTSL2561::INFRARED (2), SensorTSL2561::FULL (3) (default: visible)
	void setSpectrum(int value) {
		_tsl_spectrum = value;
	};
	// [104] set the i2c address values are SensorTSL2561::ADDR_FLOAT, SensorTSL2561::ADDR_LOW, SensorTSL2561::ADDR_HIGH
	void setAddress(int value) {
		_tsl_address = value;
	};

	// define what to do during setup
	void onSetup() {
		switch (_tsl_address) {
		case SensorTSL2561::ADDR_FLOAT:
			_tsl = new TSL2561(TSL2561_ADDR_FLOAT);
			break;
		case SensorTSL2561::ADDR_LOW:
			_tsl = new TSL2561(TSL2561_ADDR_LOW);
			break;   
		case SensorTSL2561::ADDR_HIGH:
			_tsl = new TSL2561(TSL2561_ADDR_HIGH);
			break;   
		}
		if (_tsl->begin()) {
			switch (_tsl_gain) {
			case SensorTSL2561::GAIN_0X:
				_tsl->setGain(TSL2561_GAIN_0X);
				break; 
			case SensorTSL2561::GAIN_16X:
				_tsl->setGain(TSL2561_GAIN_16X);
				break;      
			}
			switch (_tsl_timing) {
			case SensorTSL2561::INTEGRATIONTIME_13MS:
				_tsl->setTiming(TSL2561_INTEGRATIONTIME_13MS);
				break; 
			case SensorTSL2561::INTEGRATIONTIME_101MS:
				_tsl->setTiming(TSL2561_INTEGRATIONTIME_101MS); 
				break; 
			case SensorTSL2561::INTEGRATIONTIME_402MS:
				_tsl->setTiming(TSL2561_INTEGRATIONTIME_402MS); 
				break;
			}
		}
		else debug(PSTR("!" LOG_SENSOR "%s:STP"),_name);
	};

	// define what to do during loop
	void onLoop(Child* child) {
		// request the light level
		switch (_tsl_spectrum) {
		case SensorTSL2561::VISIBLE:
			((ChildInt*)child)->setValue(_tsl->getLuminosity(TSL2561_VISIBLE));
			break; 
		case SensorTSL2561::FULLSPECTRUM:
			((ChildInt*)child)->setValue(_tsl->getLuminosity(TSL2561_FULLSPECTRUM));
			break; 
		case SensorTSL2561::INFRARED:
			((ChildInt*)child)->setValue(_tsl->getLuminosity(TSL2561_INFRARED));
			break; 
		case SensorTSL2561::FULL:
			// request the full light level
			uint32_t lum = _tsl->getFullLuminosity(); 
			uint16_t ir, full;
			ir = lum >> 16;
			full = lum & 0xFFFF;
			((ChildInt*)child)->setValue(_tsl->calculateLux(full, ir));
			break; 
		}
	};
	
#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 101: setGain(request->getValueInt()); break;
		case 102: setTiming(request->getValueInt()); break;
		case 103: setSpectrum(request->getValueInt()); break;
		case 104: setAddress(request->getValueInt()); break;
		default: return;
		}
	};
#endif
};
#endif