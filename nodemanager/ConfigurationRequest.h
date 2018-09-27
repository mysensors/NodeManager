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
#ifndef ConfigurationRequest_h
#define ConfigurationRequest_h

/******************************************
ConfigurationRequest: data structure for OTA configuration requests
*/

class ConfigurationRequest {
public:
	ConfigurationRequest(uint8_t child_id, const char* string);
	// return the child id the message has been requested to
	uint8_t getRecipientChildId();
	// return the child id the request is for
	uint8_t getChildId();
	// return the parsed function
	uint8_t getFunction();
	// return the value as an int
	int getValueInt();
	// return the value as an unsigned int
	unsigned int getValueUnsignedInt();
	// return the value as an long
	int getValueLong();
	// return the value as an unsigned long
	unsigned long getValueUnsignedLong();
	// return the value as a float
	float getValueFloat();
    // return the value as a string
    char* getValueString();
private:
	uint8_t _function = 0;
	uint8_t _child_id = 255;
	uint8_t _recipient_child_id = 255;
	float _value;
	// Size of buffer to prevent overrun 
    char _string_data[MAX_PAYLOAD+1];
};

#endif