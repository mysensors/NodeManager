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
#ifndef SensorBattery_h
#define SensorBattery_h
 
/*
SensorBattery: report battery level
*/

#define BATTERY_CHILD_ID 201

class SensorBattery: public Sensor {
	
protected:
	float _battery_min = 2.6;
	float _battery_max = 3.3;
	bool _battery_internal_vcc = true;
	int8_t _battery_pin = -1;
	float _battery_volts_per_bit = 0.003363075;
	float _battery_adj_factor = 1.0;
	
public:
	SensorBattery(uint8_t child_id = BATTERY_CHILD_ID): Sensor(-1) {
		_name = "BATTERY";
		children.allocateBlocks(1);
		new Child(this,FLOAT,child_id,S_MULTIMETER,V_VOLTAGE,_name);
		// report battery level every 60 minutes by default
		setReportIntervalMinutes(60);
	};
	
	// [102] the expected vcc when the batter is fully discharged, used to calculate the percentage (default: 2.7)
	void setMinVoltage(float value) {
		_battery_min = value;
	};
	
	// [103] the expected vcc when the batter is fully charged, used to calculate the percentage (default: 3.3)
	void setMaxVoltage(float value) {
		_battery_max = value;
	};
	
	// [104] if true, the battery level will be evaluated by measuring the internal vcc without the need to connect any pin, if false the voltage divider methon will be used (default: true)
	void setBatteryInternalVcc(bool value) {
		_battery_internal_vcc = value;
	};
	
	// [105] if setBatteryInternalVcc() is set to false, the analog pin to which the battery's vcc is attached (https://www.mysensors.org/build/battery) (default: -1)
	void setBatteryPin(int8_t value) {
		_battery_pin = value;
	};
	
	// [106] if setBatteryInternalVcc() is set to false, the volts per bit ratio used to calculate the battery voltage (default: 0.003363075)
	void setBatteryVoltsPerBit(float value) {
		_battery_volts_per_bit = value;
	};
	
	// [107] change battery voltage calibration factor
	void setBatteryCalibrationFactor(float value) {
		_battery_adj_factor = value;
	};
	
	// define what to do during setup
	void onSetup() {
#ifdef CHIP_AVR
		// when measuring the battery from a pin, analog reference must be internal
		if (! _battery_internal_vcc && _battery_pin > -1)
#ifdef CHIP_MEGA
		analogReference(INTERNAL1V1);
#else
		analogReference(INTERNAL);
#endif
#endif
	};
	
	// define what to do during loop
	void onLoop(Child* child) {
		// measure the board vcc
		float volt = 0;
		if (_battery_internal_vcc || _battery_pin == -1) volt = (float)hwCPUVoltage()/1000;
		else volt = analogRead(_battery_pin) * _battery_volts_per_bit;
		volt = volt * _battery_adj_factor;
		child->setValue(volt);
		// calculate the percentage
		int percentage = ((volt - _battery_min) / (_battery_max - _battery_min)) * 100;
		if (percentage > 100) percentage = 100;
		if (percentage < 0) percentage = 0;
		// report battery level percentage
		sendBatteryLevel(percentage);
		nodeManager.sleepBetweenSend();
	};

#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 102: setMinVoltage(request->getValueFloat()); break;
		case 103: setMaxVoltage(request->getValueFloat()); break;
		case 104: setBatteryInternalVcc(request->getValueInt()); break;
		case 105: setBatteryPin(request->getValueInt()); break;
		case 106: setBatteryVoltsPerBit(request->getValueFloat()); break;
		case 107: setBatteryCalibrationFactor(request->getValueFloat()); break;
		default: return;
		}
	};
#endif
};
#endif