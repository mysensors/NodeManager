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
ConfigurationRequest: data structure for OTA configuration requests
*/

#include "ConfigurationRequest.h"

// contructor, tokenize a configuration request in the format "child_id,function,value"
ConfigurationRequest::ConfigurationRequest(uint8_t recipient_child_id, const char* string) {
	_recipient_child_id = recipient_child_id;
	char* ptr;
	// tokenize the string and get child id
	_child_id = atoi(strtok_r(const_cast<char*>(string), ",", &ptr));
	// tokenize the string and get function id
	_function = atoi(strtok_r(NULL, ",", &ptr));
	// copy user data to local area
	strcpy(_string_data, ptr);
	// tokenize the string and get the value
	_value = atof(strtok_r(NULL, ",", &ptr));
	debug(PSTR(LOG_OTA "REQ f=%du v=s%\n"),_function,_string_data);
}

// return the child id
uint8_t ConfigurationRequest::getRecipientChildId() {
	return _recipient_child_id;
}

// return the child id
uint8_t ConfigurationRequest::getChildId() {
	return _child_id;
}

// return the parsed function
uint8_t ConfigurationRequest::getFunction() {
	return _function;
}

// return the value as an int
int ConfigurationRequest::getValueInt() {
	return (int)_value;

}

// return the value as an unsigned int
unsigned int ConfigurationRequest::getValueUnsignedInt() {
	return (unsigned int)_value;

}

// return the value as an long
int ConfigurationRequest::getValueLong() {
	return (long)_value;

}

// return the value as an unsigned long
unsigned long ConfigurationRequest::getValueUnsignedLong() {
	return (unsigned long)_value;

}

// return the value as a float
float ConfigurationRequest::getValueFloat() {
	return _value;
}

// return the value as a string
char* ConfigurationRequest::getValueString() {
  return _string_data;
}
