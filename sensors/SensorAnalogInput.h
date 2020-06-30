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

#ifndef SensorAnalogInput_h
#define SensorAnalogInput_h

/*
SensorAnalogInput: read the analog input of a configured pin
*/
class SensorAnalogInput: public Sensor {
protected:
	int _reference = -1;
	bool _reverse = false;
	bool _output_percentage = true;
	int _range_min = 0;
#if defined(ARDUINO_ARCH_STM32F1)
	int _range_max = 4096;
#else
	int _range_max = 1024;
#endif
	
public:
	SensorAnalogInput(int8_t pin, uint8_t child_id = 0): Sensor(pin) {
		_name = "ANALOG_I";
		children.allocateBlocks(1);
		new Child(this,INT,nodeManager.getAvailableChildId(child_id),S_CUSTOM,V_CUSTOM,_name);
	};

	// [101] the analog reference to use (default: not set, can be either INTERNAL or DEFAULT)
	void setReference(int value) {
		_reference = value;
	};

	// [102] reverse the value or the percentage (e.g. 70% -> 30%) (default: false)
	void setReverse(bool value) {
		_reverse = value;
	};

	// [103] when true returns the value as a percentage (default: true)
	void setOutputPercentage(bool value) {
		_output_percentage = value;
	};

	// [104] minimum value for calculating the percentage (default: 0)
	void setRangeMin(int value) {
		_range_min = value;
	};

	// [105] maximum value for calculating the percentage (default: 1024)
	void setRangeMax(int value) {
		_range_max = value;
	};

	// define what to do at each stage of the sketch
	void onSetup() {
		// prepare the pin for input
		pinMode(_pin, INPUT);
	};

	void onLoop(Child* child) {
		// read the input
		int adc = _getAnalogRead();
		// calculate the percentage
		int percentage = 0;
		if (_output_percentage) percentage = _getPercentage(adc);
		// store the result
		child->setValue(_output_percentage ? percentage : adc);
	};

#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	virtual void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 101: setReference(request->getValueInt()); break;
		case 102: setReverse(request->getValueInt()); break;
		case 103: setOutputPercentage(request->getValueInt()); break;
		case 104: setRangeMin(request->getValueInt()); break;
		case 105: setRangeMax(request->getValueInt()); break;
		default: return;
		}
	};
#endif

protected:
	// return a percentage from an analog value
	int _getPercentage(int adc) {
		float value = (float)adc;
		// restore the original value
		if (_reverse) value = _range_max - value;
		// scale the percentage based on the range provided
		float percentage = ((value - _range_min) / (_range_max - _range_min)) * 100;
		if (_reverse) percentage = 100 - percentage;
		if (percentage > 100) percentage = 100;
		if (percentage < 0) percentage = 0;
		return (int)percentage;
	};
	
	// read the analog input
	int _getAnalogRead() {
#ifdef CHIP_AVR
		// set the reference
		if (_reference != -1) nodeManager.setAnalogReference(_reference);
#endif
		// read and return the value
		int value = analogRead(_pin);
		if (_reverse) value = _range_max - value;
		return value;
	};
};
#endif