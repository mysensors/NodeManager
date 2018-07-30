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
#ifndef SensorChirp_h
#define SensorChirp_h

/*
SensorChirp: Chirp soil moisture sensor (includes temperature and light sensors)  
*/

#include <Wire.h>
#include <I2CSoilMoistureSensor.h>

class SensorChirp: public Sensor {
protected:
	I2CSoilMoistureSensor* _chirp;
	int _chirp_moistureoffset = 0;
	int _chirp_moisturerange = 0;
	bool _chirp_moisturenormalized = false;  
	bool _chirp_lightreversed = true;
	
public:
	SensorChirp(NodeManager& node_manager, int child_id = -255): Sensor(node_manager) {
		_name = "CHIRP";
		children.allocateBlocks(3);
		new ChildFloat(this,_node->getAvailableChildId(child_id),S_HUM,V_HUM,_name);
		new ChildFloat(this,_node->getAvailableChildId(child_id+1),S_TEMP,V_TEMP,_name);
		new ChildFloat(this,_node->getAvailableChildId(child_id+2),S_LIGHT_LEVEL,V_LIGHT_LEVEL,_name);
	};
	
	// [101] set the soil moisture offset (default: 0)
	void setMoistureOffset(int value) {
		_chirp_moistureoffset = value;
	};
	// [102] set the soil moisture range (default: 0)
	void setMoistureRange(int value) {
		_chirp_moisturerange = value;
	};
	// [103] return the soil moisture normalized (default: false)
	void setReturnMoistureNormalized(bool value) {
		_chirp_moisturenormalized = value;  
	} ;
	// [104] reverse the light value (default: true)
	void setReturnLightReversed(bool value) {
		_chirp_lightreversed = value;  
	} ; 
	
	// define what to do during setup
	void onSetup() {
		// initialize the library
		Wire.begin();
		_chirp->begin();
		wait(1000);
		debug(PSTR(LOG_SENSOR "%s:STP a=0x%x v=%d\n"),_name,_chirp->getAddress(),_chirp->getVersion());
	};
	
	// define what to do during loop
	void onLoop(Child* child) {
		while (_chirp->isBusy()) wait(50);
		// temperature sensor
		if (child->getType() == V_TEMP) {
			// read the temperature
			float temperature = _chirp->getTemperature()/(float)10;
			// convert it
			temperature = _node->celsiusToFahrenheit(temperature);
			// store the value
			((ChildFloat*)child)->setValue(temperature);
		}
		// Humidity Sensor
		else if (child->getType() == V_HUM) {
			// request the SoilMoisturelevel
			float capacitance = _chirp->getCapacitance();
			float cap_offsetfree = capacitance - _chirp_moistureoffset;
			if (cap_offsetfree < 0) cap_offsetfree = 0;
			if (_chirp_moisturenormalized == true && _chirp_moistureoffset > 0 && _chirp_moisturerange > 0) {
				capacitance = ((cap_offsetfree/_chirp_moisturerange)*100);
				if (capacitance > 100) { capacitance = 100; }
				int tmp_cap = (int)(capacitance+0.5);
				capacitance = (float)tmp_cap;
			}    
			// store the value
			((ChildFloat*)child)->setValue(capacitance);
		}
		else if (child->getType() == V_LIGHT_LEVEL) {
			// read light
			float light = _chirp->getLight(true);
			if ( _chirp_lightreversed ) light = 65535 - light;
			// store the value
			((ChildFloat*)child)->setValue(light);
		}
	};
	
#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 101: setMoistureOffset(request->getValueInt()); break;
		case 102: setMoistureRange(request->getValueInt()); break;
		case 103: setReturnMoistureNormalized(request->getValueInt()); break;
		case 104: setReturnLightReversed(request->getValueInt()); break;
		default: return;
		}
	};
#endif
};
#endif