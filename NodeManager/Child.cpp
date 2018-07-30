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

/*
Child class
*/

#include "Child.h"

Child::Child() {
}

// constructor
Child::Child(Sensor* sensor, int child_id, int presentation, int type, const char* description) {
	_child_id = child_id;
	_presentation = presentation;
	_type = type;
	_description = description;
	_sensor = sensor;
	// register the child with the sensor
	_sensor->registerChild(this);
#if NODEMANAGER_CONDITIONAL_REPORT == ON
	// initialize the timer for forcing updates to the gateway after a given timeframe
	_force_update_timer = new Timer(_sensor->_node);
#endif
}

// setter/getter
void Child::setChildId(int value) {
	_child_id = value;
}
int Child::getChildId() {
	return _child_id;
}
void Child::setPresentation(int value) {
	_presentation = value;
}
int Child::getPresentation() {
	return _presentation;
}
void Child::setType(int value) {
	_type = value;
}
int Child::getType() {
	return _type;
}
void Child::setFloatPrecision(int value) {
	_float_precision = value;
}
void Child::setDescription(const char* value) {
	_description = value;
}
const char* Child::getDescription() {
	return _description;
}
#if NODEMANAGER_CONDITIONAL_REPORT == ON
void Child::setForceUpdateTimerValue(int value) {
	_force_update_timer->setMode(TIME_INTERVAL);
	_force_update_timer->setValue(value*60);
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
#endif

// set a value, implemented by the subclasses
void Child::sendValue(bool force) {
}
// Print the child's value (variable type depending on the child class) to the given output
void Child::print(Print& device) {
}
// reset the counters
void Child::reset() {
}

/*
ChildInt class
*/

// ChildInt class
ChildInt::ChildInt(Sensor* sensor, int child_id, int presentation, int type, const char* description): Child(sensor, child_id, presentation, type, description)  {
}

// store a new value and update the total
void ChildInt::setValue(int value) {
	if (isnan(value)) return;
	_total = _total + value;
	_samples++;
	// averages the values
	_value = (int) (_total / _samples);
	// print out a debug message
	debug(PSTR(LOG_LOOP "%s(%d):SET t=%d v=%d\n"),_description,_child_id,_type,_value);
}

// return the value
int ChildInt::getValue() {
	return _value;
}

// send the value back to the controller
void ChildInt::sendValue(bool force) {
	if (_samples == 0) return;
#if NODEMANAGER_CONDITIONAL_REPORT == ON
	if (! force) {
		// if below or above the thresholds, do not send the value
		if (_value < _min_threshold || _value > _max_threshold) return;
		// if the force update timer is over, send the value regardless and restart it
		if (_force_update_timer->isOver()) _force_update_timer->start();
		else {
			// if the value does not differ enough from the previous one, do not send the value
			if (_value > (_last_value - _value_delta) && _value < (_last_value + _value_delta)) {
				// keep track of the previous value
				_last_value = _value;
				return;
			}
		}
	}
	// keep track of the previous value
	_last_value = _value;
#endif
	// send the value to the gateway
	_sensor->_node->sendMessage(_child_id,_type,_value);
	reset();
}

// Print the child's value (variable type depending on the child class) to the given output
void ChildInt::print(Print& device) {
	device.print(_value);
}

// reset the counters
void ChildInt::reset() {
	_total = 0;
	_samples = 0;
}

/*
ChildFloat class
*/

// ChildFloat class
ChildFloat::ChildFloat(Sensor* sensor, int child_id, int presentation, int type, const char* description): Child(sensor, child_id, presentation, type, description)  {
	_float_precision = 2;
}

// store a new value and update the total
void ChildFloat::setValue(float value) {
	if (isnan(value)) return;
	_total = _total + value;
	_samples++;
	// averages the values
	_value = _total / _samples;
	// round the value if float precision has been customized
	if (_float_precision != 2) {
		if (_float_precision == 0) _value = (int) _value;
		else _value = float((int) (_value * (_float_precision*10))) / (_float_precision*10);
	}
	// print out a debug message
	debug(PSTR(LOG_LOOP "%s(%d):SET t=%d v=%d.%02d\n"),_description,_child_id,_type,(int)_value, (int)(_value*100)%100);
}

// return the value
float ChildFloat::getValue() {
	return _value;
}

// send the value back to the controller
void ChildFloat::sendValue(bool force) {
	if (_samples == 0) return;
#if NODEMANAGER_CONDITIONAL_REPORT == ON
	if (! force) {
		// if below or above the thresholds, do not send the value
		if (_value < _min_threshold || _value > _max_threshold) return;
		//   if the force update timer is over, send the value regardless and restart it
		if (_force_update_timer->isOver()) _force_update_timer->start();
		else {
			// if the value does not differ enough from the previous one, do not send the value
			if (_value > (_last_value - _value_delta) && _value < (_last_value + _value_delta)) {
				// keep track of the previous value
				_last_value = _value;
				return;
			}
		}
	}
	// keep track of the previous value
	_last_value = _value;
#endif
	// send the value to the gateway
	_sensor->_node->sendMessage(_child_id,_type,_value,_float_precision);
}

// Print the child's value (variable type depending on the child class) to the given output
void ChildFloat::print(Print& device) {
	device.print(_value,_float_precision);
}

// reset the counters
void ChildFloat::reset() {
	_total = 0;
	_samples = 0;
}


/*
ChildDouble class
*/

// ChildDouble class
ChildDouble::ChildDouble(Sensor* sensor, int child_id, int presentation, int type, const char* description): Child(sensor, child_id, presentation, type, description)  {
	_float_precision = 4;
}

// store a new value and update the total
void ChildDouble::setValue(double value) {
	if (isnan(value)) return;
	_total = _total + value;
	_samples++;
	// averages the values
	_value = _total / _samples;
	// round the value if float precision has been customized
	if (_float_precision != 4) {
		if (_float_precision == 0) _value = (int) _value;
		else _value = double((int) (_value * (_float_precision*10))) / (_float_precision*10);
	}
	// print out a debug message
	debug(PSTR(LOG_LOOP "%s(%d):SET t=%d v=%d.%04d\n"),_description,_child_id,_type,(int)_value, (int)(_value*10000)%10000);
}

// return the value
double ChildDouble::getValue() {
	return _value;
}

// send the value back to the controller
void ChildDouble::sendValue(bool force) {
	if (_samples == 0) return;
#if NODEMANAGER_CONDITIONAL_REPORT == ON
	if (! force) {
		// if below or above the thresholds, do not send the value
		if (_value < _min_threshold || _value > _max_threshold) return;
		// if the force update timer is over, send the value regardless and restart it
		if (_force_update_timer->isOver()) _force_update_timer->start();
		else {
			// if the value does not differ enough from the previous one, do not send the value
			if (_value > (_last_value - _value_delta) && _value < (_last_value + _value_delta)) {
				// keep track of the previous value
				_last_value = _value;
				return;
			}
		}
	}
	// keep track of the previous value
	_last_value = _value;
#endif
	// send the value to the gateway
	_sensor->_node->sendMessage(_child_id,_type,_value,_float_precision);
}

// Print the child's value (variable type depending on the child class) to the given output
void ChildDouble::print(Print& device) {
	device.print(_value,_float_precision);
}

// reset the counters
void ChildDouble::reset() {
	_total = 0;
	_samples = 0;
}


/*
ChildString class
*/

// ChildString class
ChildString::ChildString(Sensor* sensor, int child_id, int presentation, int type, const char* description): Child(sensor, child_id, presentation, type, description)  {
}

// store a new value and update the total
void ChildString::setValue(const char* value) {
	_value = value;
	// print out a debug message
	debug(PSTR(LOG_LOOP "%s(%d):SET t=%d v=%s\n"),_description,_child_id,_type,_value);
}

// return the value
const char* ChildString::getValue() {
	return _value;
}

// send the value back to the controller
void ChildString::sendValue(bool force) {
#if NODEMANAGER_CONDITIONAL_REPORT == ON
	if (! force) {
		// if a delta is configured, do not report if the string is the same as the previous one
		if (_value_delta > 0 && strcmp(_value, _last_value) == 0) {
			// keep track of the previous value
			_last_value = _value;
			return;
		}
	}
	// keep track of the previous value
	_last_value = _value;
#endif
	// send the value to the gateway
	_sensor->_node->sendMessage(_child_id,_type,_value);
}

// Print the child's value (variable type depending on the child class) to the given output
void ChildString::print(Print& device) {
	device.print(_value);
}

// reset the counters
void ChildString::reset() {
	_value = "";
}