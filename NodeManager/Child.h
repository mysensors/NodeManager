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
#ifndef Child_h
#define Child_h

/******************************************
Child: data structure for representing a Child of a Sensor
*/

class Sensor;
#include "Timer.h"

// how many slots to leave for the user before starting using them for Child
#define EEPROM_CHILD_OFFSET	10
// define the number of slots of the EEPROM needed to store a Child's value
#define EEPROM_CHILD_SIZE 6
// define the relative positions of each information
#define EEPROM_CHILD_CHECKSUM 0
#define EEPROM_CHILD_SIGN 1
#define EEPROM_CHILD_INT_1 2
#define EEPROM_CHILD_INT_2 3
#define EEPROM_CHILD_DEC_1 4
#define EEPROM_CHILD_DEC_2 5

class Child {
public:
	Child();
	Child(Sensor* sensor, int child_id, int presentation, int type, const char* description = "");
	// set child id used to communicate with the gateway/controller
	void setChildId(int value);
	int getChildId();
	// set sensor presentation (default: S_CUSTOM)
	void setPresentation(int value);
	int getPresentation();
	// set sensor type (default: V_CUSTOM)
	void setType(int value);
	int getType();
	// set how many decimal digits to use (default: 2 for ChildFloat, 4 for ChildDouble)
	void setFloatPrecision(int value);
	// set sensor description
	void setDescription(const char* value);
	const char* getDescription();
	// configure the behavior of the child when setValue() is called multiple times. It can be NONE (ignore the previous values but the last one),  AVG (averages the values), SUM (sum up the values) (default: AVG)
	void setValueProcessing(child_processing value);
	// send the current value to the gateway
	virtual void sendValue(bool force = 0);
	// print the current value on a LCD display
	virtual void print(Print& device);
	// reset all the counters
	virtual void reset();
#if NODEMANAGER_CONDITIONAL_REPORT == ON
	// force to send an update after the configured number of minutes
	void setForceUpdateTimerValue(int value);
	// never report values below this threshold (default: FLT_MIN)
	void setMinThreshold(float value);
	// never report values above this threshold (default: FLT_MAX)
	void setMaxThreshold(float value);
	// do not report values if too close to the previous one (default: 0)
	void setValueDelta(float value);
#endif
#if NODEMANAGER_EEPROM == ON
	// persist the child's value in EEPROM. The value will be saved at each update and loaded at boot time (default: false)
	void setPersistValue(bool value);
	bool getPersistValue();
	// load old value from EEPROM
	virtual void loadValue();
	// load current value to EEPROM
	virtual void saveValue();
#endif
protected:
	Sensor* _sensor;
	int _child_id;
	int _presentation = S_CUSTOM;
	int _type = V_CUSTOM;
	int _float_precision;
	const char* _description = "";
	int _samples = 0;
	child_processing _value_processing = AVG;
#if NODEMANAGER_CONDITIONAL_REPORT == ON
	Timer* _force_update_timer;
	float _min_threshold = FLT_MIN;
	float _max_threshold = FLT_MAX;
	float _value_delta = 0;
#endif
#if NODEMANAGER_EEPROM == ON
	bool _persist_value = false;
	int _eeprom_address;
	bool _saveValueInt(int value);
	bool _saveValueFloat(float value);
	bool _saveValueDouble(double value);
	int _loadValueInt();
	float _loadValueFloat();
	double _loadValueDouble();
#endif
};

class ChildInt: public Child {
public:
	ChildInt(Sensor* sensor, int child_id, int presentation, int type, const char* description = "");
	void setValue(int value);
	int getValue();
	void sendValue(bool force = 0);
	void print(Print& device);
	void reset();
#if NODEMANAGER_EEPROM == ON
	void loadValue();
	void saveValue();
#endif
private:
	int _value;
	int _total = 0;
#if NODEMANAGER_CONDITIONAL_REPORT == ON
	int _last_value = -256;
#endif
};

class ChildFloat: public Child {
public:
	ChildFloat(Sensor* sensor, int child_id, int presentation, int type, const char* description = "");
	void setValue(float value);
	float getValue();
	void sendValue(bool force = 0);
	void print(Print& device);
	void reset();
#if NODEMANAGER_EEPROM == ON
	void loadValue();
	void saveValue();
#endif
private:
	float _value = 0;
	float _total = 0;
#if NODEMANAGER_CONDITIONAL_REPORT == ON
	float _last_value = 0;
#endif
};

class ChildDouble: public Child {
public:
	ChildDouble(Sensor* sensor, int child_id, int presentation, int type, const char* description = "");
	void setValue(double value);
	double getValue();
	void sendValue(bool force = 0);
	void print(Print& device);
	void reset();
#if NODEMANAGER_EEPROM == ON
	void loadValue();
	void saveValue();
#endif
private:
	double _value;
	double _total = 0;
#if NODEMANAGER_CONDITIONAL_REPORT == ON
	double _last_value = -256;
#endif
};

class ChildString: public Child {
public:
	ChildString(Sensor* sensor, int child_id, int presentation, int type, const char* description = "");
	void setValue(const char* value);
	const char* getValue();
	void sendValue(bool force = 0);
	void print(Print& device);
	void reset();
#if NODEMANAGER_EEPROM == ON
	void loadValue();
	void saveValue();
#endif
private:
	const char* _value = "";
#if NODEMANAGER_CONDITIONAL_REPORT == ON
	const char* _last_value = "";
#endif
};

#endif