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
#ifndef SensorThermistor_h
#define SensorThermistor_h

/*
SensorThermistor: read the temperature from a thermistor
*/

class SensorThermistor: public Sensor {
protected:
	long _nominal_resistor = 10000;
	int _nominal_temperature = 25;
	int _b_coefficient = 3950;
	long _series_resistor = 10000;
	float _offset = 0;
	
public:
	SensorThermistor(NodeManager& node_manager, int pin, int child_id = -255): Sensor(node_manager, pin) {
		_name = "THERMISTOR";
		children.allocateBlocks(1);
		new ChildFloat(this,_node->getAvailableChildId(child_id),S_TEMP,V_TEMP,_name);
	};

	// [101] resistance at 25 degrees C (default: 10000)
	void setNominalResistor(long value) {
		_nominal_resistor = value;
	};
	// [102] temperature for nominal resistance (default: 25)
	void setNominalTemperature(int value) {
		_nominal_temperature = value;
	};
	// [103] The beta coefficient of the thermistor (default: 3950)
	void setBCoefficient(int value) {
		_b_coefficient = value;
	};
	// [104] the value of the resistor in series with the thermistor (default: 10000)
	void setSeriesResistor(long value) {
		_series_resistor = value;
	};
	// [105] set a temperature offset
	void setOffset(float value) {
		_offset = value;
	};

	// define what to do during setup
	void onSetup() {
		// set the pin as input
		pinMode(_pin, INPUT);
	};

	// define what to do during setup
	void onLoop(Child* child) {
		// read the voltage across the thermistor
		float adc = analogRead(_pin);
		// calculate the temperature
		float reading = (1023 / adc)  - 1;
		reading = _series_resistor / reading;
		float temperature;
		temperature = reading / _nominal_resistor;     // (R/Ro)
		temperature = log(temperature);                  // ln(R/Ro)
		temperature /= _b_coefficient;                   // 1/B * ln(R/Ro)
		temperature += 1.0 / (_nominal_temperature + 273.15); // + (1/To)
		temperature = 1.0 / temperature;                 // Invert
		temperature -= 273.15;                         // convert to C
		temperature = _node->celsiusToFahrenheit(temperature);
		// store the value
		((ChildFloat*)child)->setValue(temperature);
	};

#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	virtual void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 101: setNominalResistor((long)request->getValueInt()); break;
		case 102: setNominalTemperature(request->getValueInt()); break;
		case 103: setBCoefficient(request->getValueInt()); break;
		case 104: setSeriesResistor((long)request->getValueInt()); break;
		case 105: setOffset(request->getValueFloat()); break;
		default: return;
		}
	};
#endif
};
#endif