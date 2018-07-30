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

/*
ConfigurationRequest
*/

class ConfigurationRequest {
public:
	ConfigurationRequest(int child_id, const char* string);
	// return the child id the message has been requested to
	int getRecipientChildId();
	// return the child id the request is for
	int getChildId();
	// return the parsed function
	int getFunction();
	// return the value as an int
	int getValueInt();
	// return the value as a float
	float getValueFloat();
private:
	int _function = -1;
	int _child_id = -1;
	int _recipient_child_id = -1;
	float _value;
};

#endif