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

/******************************************
Child: data structure for representing a Child of a Sensor
*/

#include "Child.h"

Child::Child() {
}

// constructor
Child::Child(Sensor* sensor, value_format format, uint8_t child_id, uint8_t presentation, uint8_t type, const char* description, const char* unit_prefix, bool request_initial_value) {
	_sensor = sensor;
	setFormat(format);
	_child_id = child_id;
	_presentation = presentation;
	_type = type;
	_description = description;
	_unit_prefix = unit_prefix;
	_request_initial_value = request_initial_value;
	// register the child with the sensor
	_sensor->registerChild(this);
#if NODEMANAGER_EEPROM == ON
	// define the EEPROM starting address for this child
	_eeprom_address = EEPROM_CHILD_OFFSET+_child_id*EEPROM_CHILD_SIZE;
#endif
}

// setter/getter
void Child::setChildId(uint8_t value) {
	_child_id = value;
}
uint8_t Child::getChildId() {
	return _child_id;
}
void Child::setFormat(value_format value) {
	_format = value;
	// set default float precision
	if (_format == FLOAT) _float_precision = 2;
	if (_format == DOUBLE) _float_precision = 4;
}
value_format Child::getFormat() {
	return _format;
}
void Child::setPresentation(uint8_t value) {
	_presentation = value;
}
uint8_t Child::getPresentation() {
	return _presentation;
}
void Child::setType(uint8_t value) {
	_type = value;
}
uint8_t Child::getType() {
	return _type;
}
void Child::setFloatPrecision(uint8_t value) {
	_float_precision = value;
}
void Child::setDescription(const char* value) {
	_description = value;
}
const char* Child::getDescription() {
	return _description;
}
void Child::setValueProcessing(child_processing value) {
	_value_processing = value;
}
void Child::setSendWithoutValue(bool value) {
	_send_without_value = value;
}

// set the value to the child
void Child::setValue(int value) {
	_setValueNumber(value);
}
void Child::setValue(float value) {
	_setValueNumber(value);
}
void Child::setValue(double value) {
	_setValueNumber(value);
}
void Child::setValue(const char* value) {
	_value_string = value;
	_samples = 1;
	debug(PSTR(LOG_LOOP "%s(%d):SET t=%d v=%s\n"),_description,_child_id,_type,_value_string);
}
void Child::setUnitPrefix(const char* value) {
	_unit_prefix = value;
}
const char* Child::getUnitPrefix() {
	if (_unit_prefix == NULL) return nodeManager.getDefaultUnitPrefix(_presentation,_type);
	return _unit_prefix;
}
void Child::setRequestInitialValue(bool value) {
	_request_initial_value = value;
}
bool Child::getRequestInitialValue() {
	return _request_initial_value;
}

// store a new value and update the total
void Child::_setValueNumber(double value) {
	if (isnan(value)) return;
	// this is the first measure after a send(), reset _value and _total
	if (_samples == 0) {
		_value = 0; 
		_total = 0;
	}
	if (_value_processing != NONE) _total = _total + value;
	// keep track of the samples
	_samples++;
	// process the value
	if (_value_processing == AVG) _value = _total / _samples;
	if (_value_processing == SUM) _value = _total;
	if (_value_processing == NONE) _value = value;
	// print out a debug message
	if (_format == INT) { debug(PSTR(LOG_LOOP "%s(%d):SET t=%d v=%d\n"),_description,_child_id,_type,(int)_value); }
	if (_format == FLOAT) { debug(PSTR(LOG_LOOP "%s(%d):SET t=%d v=%d.%02d\n"),_description,_child_id,_type,(int)_value,(int)((_value-(int)_value)*100)*(_value<0.0?-1:1)); }
	if (_format == DOUBLE) { debug(PSTR(LOG_LOOP "%s(%d):SET t=%d v=%d.%04d\n"),_description,_child_id,_type,(int)_value,(int)((_value-(int)_value)*10000)*(_value<0.0?-1:1)); }
#if NODEMANAGER_EEPROM == ON
	// if the value is supposed to be persisted in EEPROM, save it
	if (_persist_value) saveValue();
#endif
}

// return the value of the child in the requested format
int Child::getValueInt() {
	return (int)_value;
}
float Child::getValueFloat() {
	return (float)_value;
}
double Child::getValueDouble(){
	return _value;
}
const char* Child::getValueString(){
	return _value_string;
}


// check if the value must be sended back to the controller
bool Child::valueReadyToSend() {
	// do not send if no samples have been collected, unless instructed otherwise
	if (!_send_without_value && _samples == 0) return false;
#if NODEMANAGER_CONDITIONAL_REPORT == ON
	// ignore conditional reporting settings if it is the first run
	if (!_sensor->getFirstRun()) {
		// if the value is a number
		if (_format != STRING) {
			// if below or above the thresholds, do not send the value
			if (_value < _min_threshold || _value > _max_threshold) return false;
			// if the force update timer is over, send the value
			if (!_force_update_timer->isOver()) {
				// if the value does not differ enough from the previous one, do not send the value
				if (_value > (_last_value - _value_delta) && _value < (_last_value + _value_delta)) {
					return false;
				}
			}
		}
		// if the value is a string
		else {
			// if a delta is configured, do not report if the string is the same as the previous one
			if (_value_delta > 0 && strcmp(_value_string, _last_value_string) == 0) {
				return false;
			}
		}
	}
#endif
	return true;
}


// send the value back to the controller
void Child::sendValue(bool force) {

#if NODEMANAGER_CONDITIONAL_REPORT == ON
	// update last value if mode = UPDATE_ALWAYS
	if (_last_value_mode == UPDATE_ALWAYS) _last_value = _value;
#endif
	// ignore conditional reporting settings if forced to report
	if (!force && !valueReadyToSend()) return;
	// we report value
#if NODEMANAGER_CONDITIONAL_REPORT == ON
	// if the force update timer is over restart it
	if (_force_update_timer->isOver()) _force_update_timer->start();
	// keep track of the previous value
	if (_format != STRING) _last_value = _value;
	else _last_value_string = _value_string;
#endif
	// send unit prefix if configured to do so
	if (nodeManager.getSendUnitPrefix() && ! _unit_prefix_sent) {
		nodeManager.sendMessage(_child_id,V_UNIT_PREFIX,getUnitPrefix());
		_unit_prefix_sent = true;
	}
	// send the value to the gateway
	if (_format == INT) nodeManager.sendMessage(_child_id,_type,(int)_value);
	if (_format == FLOAT) nodeManager.sendMessage(_child_id,_type,(float)_value,_float_precision);
	if (_format == DOUBLE) nodeManager.sendMessage(_child_id,_type,_value,_float_precision);
	if (_format == STRING) nodeManager.sendMessage(_child_id,_type,_value_string);
	// reset the counters
	reset();
}

// print the child value to a device
void Child::print(Print& device) { 
	if (_format == INT) device.print((int)_value);
	if (_format == FLOAT) device.print((float)_value,_float_precision);
	if (_format == DOUBLE) device.print((double)_value,_float_precision);
	if (_format == STRING) device.print(_value_string);
}

// reset the counters
void Child::reset() { 
	if (_format != STRING) {
		if (_value_processing != NONE) {
#if NODEMANAGER_EEPROM == ON
			// if the value is supposed to be persisted in EEPROM, save it
			if (_persist_value) saveValue();
#endif
		}
	}
	_samples = 0;
}

#if NODEMANAGER_CONDITIONAL_REPORT == ON
// setter/getter
void Child::setForceUpdateTimerValue(unsigned long value) {
	_force_update_timer->setMode(TIME_INTERVAL);
	_force_update_timer->setValue(value*60);
	_force_update_timer->start();
}
void Child::setMinThreshold(float value) {
	_min_threshold = value;
}
void Child::setMaxThreshold(float value) {
	_max_threshold = value;
}
void Child::setValueDelta(float value) {
	_value_delta = value;
}
void Child::setUpdateLastValue(last_value_mode value) {
	_last_value_mode = value;
}

// return the last value of the child in the requested format
int Child::getLastValueInt() {
	return (int)_last_value;
}
float Child::getLastValueFloat() {
	return (float)_last_value;
}
double Child::getLastValueDouble(){
	return _last_value;
}
const char* Child::getLastValueString(){
	return _last_value_string;
}
#endif

#if NODEMANAGER_EEPROM == ON
// setter/getter
void Child::setPersistValue(bool value) {
	_persist_value = value;
}
bool Child::getPersistValue() {
	return _persist_value;
}

// save value to EEPROM - subclass needs to implement
void Child::saveValue() {
	if (_format == STRING) return;
	// number is too large to be saved or we run out of EEPROM slots
	if (_value >= 10000 || ((_eeprom_address+EEPROM_CHILD_SIZE) > 255) ) return;
	debug(PSTR(LOG_EEPROM "%s(%d):SAVE\n"),_description,_child_id);
	// save the type
	nodeManager.saveToMemory(_eeprom_address+EEPROM_CHILD_TYPE,_type);
	// encode the sign (e.g. 0 if > 0, 1 otherwise)
	nodeManager.saveToMemory(_eeprom_address+EEPROM_CHILD_SIGN, _value >= 0 ? 0 : 1);
	// encode and save the integer value (e.g. 7240 -> int_1 = 72, int_2 = 40)
	nodeManager.saveToMemory(_eeprom_address+EEPROM_CHILD_INT_1,(int)(_value/100)%100);
	nodeManager.saveToMemory(_eeprom_address+EEPROM_CHILD_INT_2,(int)_value%100);
	// encode and save the first part of the decimal value (e.g. 7240.12 -> dec_1 = 12)
	if (_format == FLOAT || _format == DOUBLE) nodeManager.saveToMemory(_eeprom_address+EEPROM_CHILD_DEC_1,(int)(_value*100)%100);
	// encode and save the second part of the decimal value (e.g. 7240.1244 -> dec_2 = 44)
	if (_format == DOUBLE) nodeManager.saveToMemory(_eeprom_address+EEPROM_CHILD_DEC_1,(int)(_value*10000)%100);
}

// load value from EEPROM 
void Child::loadValue() { 
	if (_format == STRING) return;
	// ensure we are not going to read beyond the available EEPROM slots
	if (((_eeprom_address+EEPROM_CHILD_SIZE) > 255) ) return;
	// ensure the type is valid
	if (nodeManager.loadFromMemory(_eeprom_address+EEPROM_CHILD_TYPE) != _type) return;
	debug(PSTR(LOG_EEPROM "%s(%d):LOAD\n"),_description,_child_id);
	// decode the integer part
	double value = 0;
	value = nodeManager.loadFromMemory(_eeprom_address+EEPROM_CHILD_INT_1)*100 + nodeManager.loadFromMemory(_eeprom_address+EEPROM_CHILD_INT_2);
	if (value == 255) return;
	// decode the sign
	if (nodeManager.loadFromMemory(_eeprom_address+EEPROM_CHILD_SIGN) == 1) value = value * -1;
	// decode the first part of the decimal value
	if (_format == FLOAT || _format == DOUBLE) value += nodeManager.loadFromMemory(_eeprom_address+EEPROM_CHILD_DEC_1)/100.0;
	// decode the second part of the decimal value
	if (_format == DOUBLE) value += nodeManager.loadFromMemory(_eeprom_address+EEPROM_CHILD_DEC_2)/10000.0;
	setValue(value);
}
#endif
