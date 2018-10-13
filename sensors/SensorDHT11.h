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
#ifndef SensorDHT11_h
#define SensorDHT11_h

/*
SensorDHT11
*/

#include <sensors/SensorDHT.h>

class SensorDHT11: public SensorDHT {
public:
	SensorDHT11(int8_t pin, uint8_t child_id = 0): SensorDHT(pin, child_id) {
		_name = "DHT11";
		_dht_type = DHT::DHT11;
		children.get(1)->setDescription(_name);
		children.get(2)->setDescription(_name);
	};
};
#endif