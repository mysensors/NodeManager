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
#include "InternalTimer.h"

// how many slots to leave for the user before starting using them for Child
#define EEPROM_CHILD_OFFSET	10
// define the number of slots of the EEPROM needed to store a Child's value
#define EEPROM_CHILD_SIZE 6
// define the relative positions of each information
#define EEPROM_CHILD_TYPE 0
#define EEPROM_CHILD_SIGN 1
#define EEPROM_CHILD_INT_1 2
#define EEPROM_CHILD_INT_2 3
#define EEPROM_CHILD_DEC_1 4
#define EEPROM_CHILD_DEC_2 5

class Child {
public:
	Child();
	Child(Sensor* sensor, value_format format, uint8_t child_id, uint8_t presentation, uint8_t type, const char* description = "");
	// set child id used to communicate with the gateway/controller
	void setChildId(uint8_t value);
	uint8_t getChildId();
	// set sensor format
	void setFormat(value_format value);
	value_format getFormat();
	// set sensor presentation (default: S_CUSTOM)
	void setPresentation(uint8_t value);
	uint8_t getPresentation();
	// set sensor type (default: V_CUSTOM)
	void setType(uint8_t value);
	uint8_t getType();
	// set how many decimal digits to use (default: 2 for ChildFloat, 4 for ChildDouble)
	void setFloatPrecision(uint8_t value);
	// set sensor description
	void setDescription(const char* value);
	const char* getDescription();
	// configure the behavior of the child when setValue() is called multiple times. It can be NONE (ignore the previous values but the last one),  AVG (averages the values), SUM (sum up the values) (default: AVG)
	void setValueProcessing(child_processing value);
	// send the value to the gateway even if there have been no samples collected (default: false)
	void setSendWithoutValue(bool value);
	// set the value of the child
	void setValue(int value);
	void setValue(float value);
	void setValue(double value);
	void setValue(const char* value);
	// get the value of the child
	int getValueInt();
	float getValueFloat();
	double getValueDouble();
	const char* getValueString();
	// check if the value must be sended back to the controller
	bool valueReadyToSend();
	// send the current value to the gateway
	void sendValue(bool force = 0);
	// print the current value on a LCD display
	void print(Print& device);
	// reset all the counters
	void reset();
#if NODEMANAGER_CONDITIONAL_REPORT == ON
	// force to send an update after the configured number of minutes
	void setForceUpdateTimerValue(unsigned long value);
	// never report values below this threshold (default: -FLT_MAX)
	void setMinThreshold(float value);
	// never report values above this threshold (default: FLT_MAX)
	void setMaxThreshold(float value);
	// do not report values if too close to the previous one (default: 0)
	void setValueDelta(float value);
	// set when the last value is updated. Possible values are UPDATE_ALWAYS (at every cycle), UPDATE_ON_SEND (only after sending) (default: UPDATE_ON_SEND)
	void setUpdateLastValue(last_value_mode value);
	// get the last value of the child
	int getLastValueInt();
	float getLastValueFloat();
	double getLastValueDouble();
	const char* getLastValueString();
#endif
#if NODEMANAGER_EEPROM == ON
	// persist the child's value in EEPROM. The value will be saved at each update and loaded at boot time (default: false)
	void setPersistValue(bool value);
	bool getPersistValue();
	// load old value from EEPROM
	void loadValue();
	// load current value to EEPROM
	void saveValue();
#endif
protected:
	Sensor* _sensor;
	value_format _format;
	uint8_t _child_id;
	uint8_t _presentation = S_CUSTOM;
	uint8_t _type = V_CUSTOM;
	const char* _description = "";
	unsigned int _samples = 0;
	uint8_t _float_precision;
	child_processing _value_processing = AVG;
	double _value = 0;
	const char* _value_string = "";
	double _total = 0;
	bool _send_without_value = false;
	void _setValueNumber(double value);
#if NODEMANAGER_CONDITIONAL_REPORT == ON
	double _last_value = 0;
	const char* _last_value_string = "";
	InternalTimer* _force_update_timer = new InternalTimer();
	float _min_threshold = -FLT_MAX;
	float _max_threshold = FLT_MAX;
	float _value_delta = 0;
	last_value_mode _last_value_mode = UPDATE_ON_SEND;
#endif
#if NODEMANAGER_EEPROM == ON
	bool _persist_value = false;
	int _eeprom_address;
#endif
};


#endif