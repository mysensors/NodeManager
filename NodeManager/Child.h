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

/***************************************
Child: child class
*/

class Sensor;
#include "Timer.h"

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
#if FEATURE_CONDITIONAL_REPORT == ON
	// force to send an update after the configured number of minutes
	void setForceUpdateTimerValue(int value);
	// never report values below this threshold (default: FLT_MIN)
	void setMinThreshold(float value);
	// never report values above this threshold (default: FLT_MAX)
	void setMaxThreshold(float value);
	// do not report values if too close to the previous one (default: 0)
	void setValueDelta(float value);
#endif
	// send the current value to the gateway
	virtual void sendValue(bool force);
	// print the current value on a LCD display
	virtual void print(Print& device);
	// reset all the counters
	virtual void reset();
protected:
	int _samples = 0;
	Sensor* _sensor;
	int _child_id;
	int _presentation = S_CUSTOM;
	int _type = V_CUSTOM;
	int _float_precision;
	const char* _description = "";
#if FEATURE_CONDITIONAL_REPORT == ON
	Timer* _force_update_timer;
	float _min_threshold = FLT_MIN;
	float _max_threshold = FLT_MAX;
	float _value_delta = 0;
#endif
};

class ChildInt: public Child {
public:
	ChildInt(Sensor* sensor, int child_id, int presentation, int type, const char* description = "");
	void setValue(int value);
	int getValue();
	void sendValue(bool force);
	void print(Print& device);
	void reset();
private:
	int _value;
#if FEATURE_CONDITIONAL_REPORT == ON
	int _last_value = -256;
#endif
	int _total = 0;
};

class ChildFloat: public Child {
public:
	ChildFloat(Sensor* sensor, int child_id, int presentation, int type, const char* description = "");
	void setValue(float value);
	float getValue();
	void sendValue(bool force);
	void print(Print& device);
	void reset();
private:
	float _value;
#if FEATURE_CONDITIONAL_REPORT == ON
	float _last_value = -256;
#endif
	float _total = 0;
};

class ChildDouble: public Child {
public:
	ChildDouble(Sensor* sensor, int child_id, int presentation, int type, const char* description = "");
	void setValue(double value);
	double getValue();
	void sendValue(bool force);
	void print(Print& device);
	void reset();
private:
	double _value;
#if FEATURE_CONDITIONAL_REPORT == ON
	double _last_value = -256;
#endif
	double _total = 0;
};

class ChildString: public Child {
public:
	ChildString(Sensor* sensor, int child_id, int presentation, int type, const char* description = "");
	void setValue(const char* value);
	const char* getValue();
	void sendValue(bool force);
	void print(Print& device);
	void reset();
private:
	const char* _value = "";
#if FEATURE_CONDITIONAL_REPORT == ON
	const char* _last_value = "";
#endif
};

#endif