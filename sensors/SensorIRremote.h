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
#ifndef SensorIRremote_h
#define SensorIRremote_h

/*
SensorIRRemote
*/
#if defined(CHIP_ESP8266)
#include <IRremoteESP8266.h>
#include <IRsend.h>
#else
#include <IRremote.h>
#endif

class SensorIRremote: public Sensor {
protected:
	// IR remote object
	IRsend* _ir_remote;
	// signal to send. Use IRrecvDumpV2 from the IRremote examples to capture the code. Copy and past the "unsigned int irSignal[]" into the main sketch and pass it to the constructor. ESP8266 uses a different library expecting a const uint16_t array
#if defined(CHIP_ESP8266)
	const uint16_t* _signal;
#include <IRsend.h>
#else
	unsigned int* _signal;
#endif
	// frequency
#if defined(CHIP_ESP8266)
	uint16_t _khz = 38;
	uint16_t _len = 0;
#include <IRsend.h>
#else
	int _khz = 38;
	int _len = 0;
#endif

	
	
public:
#if defined(CHIP_ESP8266)
	SensorIRremote(const uint16_t signal[], uint16_t len, uint8_t child_id = 0): Sensor(-1) {
#include <IRsend.h>
#else
	SensorIRremote(unsigned int signal[], int len, uint8_t child_id = 0): Sensor(-1) {
#endif
		// attach IR led to pin 3 (D2 for ESP8266)
		_name = "IRREMOTE";
		_signal = signal;
		_len = len;
		children.allocateBlocks(1);
		new Child(this,INT,nodeManager.getAvailableChildId(child_id),S_CUSTOM,V_CUSTOM,_name);
	};

	// what to do during setup
	void onSetup() {
#if defined(CHIP_ESP8266)
		_ir_remote = new IRsend(4);
		_ir_remote->begin();
#else
		_ir_remote = new IRsend();
#endif
	};

	// what to do during loop
	void onLoop(Child* child) {
	};

	// what to do when receiving a message
	void onReceive(MyMessage* message) {
		Child* child = getChild(message->sensor);
		if (child == nullptr) return;
		 debug(PSTR(LOG_SENSOR "%s:ON\n"),_name);
		_ir_remote->sendRaw(_signal, _len, _khz);
	};
};
#endif