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
#ifndef SensorGSM_h
#define SensorGSM_h
 
/*
SensorGSM: send SMS through a serial GSM modem
*/

#include <SoftwareSerial.h>

class SensorGSM: public Sensor {
	
protected:
	SoftwareSerial* _serial;
	int8_t _rx_pin;
	int8_t _tx_pin;
	uint32_t _baud_rate = 115200;
	const char* _recipient = "";
	
public:
	SensorGSM(int8_t rxpin, int8_t txpin, uint8_t child_id = 0): Sensor(rxpin) {
		_name = "GSM";
		_rx_pin = rxpin;
		_tx_pin = txpin;
		children.allocateBlocks(1);
		new Child(this,STRING,nodeManager.getAvailableChildId(child_id), S_CUSTOM, V_CUSTOM, _name);
	};
	
	// set the baud rate of the serial port for connecting to the sensor (default: 115200)
	void setBaudRate(uint32_t value) {
		_baud_rate = value;
	};
	
	// [101] set the recipient phone number
	void setRecipient(const char* value) {
		_recipient = value;
	};
	
	// send the provided text via SMS to the configured recipient
	void sendSMS(const char* text) {
		_serial->println("AT+CMGF=1");    
		_read();
		_serial->print("AT+CMGS=\"");
		_serial->print(_recipient);
		_serial->println("\"\r");
		_read();
		_serial->print(text);
		_serial->println (char(26));
		_read();
		children.get()->setValue("OK");
	}
	
	// define what to do during setup
	void onSetup() {
		// report immediately
		setReportTimerMode(IMMEDIATELY);
		// setup software serial
		_serial = new SoftwareSerial(_rx_pin,_tx_pin);
		// connect to the sensor
		_serial->begin(_baud_rate);
		// wait for the SIM card to connect to the carrier
		wait(3000);
		_read();
		// AT handshake 
		_serial->println("AT");
		_read();
		// signal quality test
		_serial->println("AT+CSQ");
		_read();
		// read SIM information
		_serial->println("AT+CCID");
		_read();
		// check whether it has registered in the network
		_serial->println("AT+CREG?");
		_read();
	};
	
	// what to do when receiving a message
	void onReceive(MyMessage* message) {
		Child* child = getChild(message->sensor);
		if (child == nullptr) return;
		// send the SMS using the text provided in the message payload
		if (message->getCommand() == C_SET) sendSMS(message->getString());
	};

#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 101: setRecipient(request->getValueString()); break;
		default: return;
		}
	};
#endif

protected:
	// Forward what Software Serial received to Serial Port
	void _read() {
		wait(500);
		String input = _serial->readString();
		if (input.length() != 0) debug_verbose(PSTR(LOG_SENSOR "READ %s\n"),const_cast<char*>(input.c_str()));
	};
};
#endif