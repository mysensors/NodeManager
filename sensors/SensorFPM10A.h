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
#ifndef SensorFPM10A_h
#define SensorFPM10A_h

/*
SensorFPM10A
*/

#include <Adafruit_Fingerprint.h>
#include <SoftwareSerial.h>

class SensorFPM10A: public Sensor {
protected:
	Adafruit_Fingerprint* _finger;
	SoftwareSerial* _serial;
	int8_t _rx_pin;
	int8_t _tx_pin;
	int _wait_finger_for_seconds = 0;
	uint32_t _baud_rate = 57600;
	uint32_t _password = 0;
	uint16_t _min_confidence = 0;
	bool _fingerprint_is_valid = false;
	
public:
	SensorFPM10A(int8_t rxpin, int8_t txpin, uint8_t child_id = 255): Sensor(rxpin) {
		_name = "FPM10A";
		_rx_pin = rxpin;
		_tx_pin = txpin;
		children.allocateBlocks(1);
		new Child(this,INT,nodeManager.getAvailableChildId(child_id), S_CUSTOM, V_CUSTOM, _name);
	};
	
	// set the baud rate of the serial port for connecting to the sensor (default: 57600)
	void setBaudRate(uint32_t value) {
		_baud_rate = value;
	};
	// set the password for connecting to the sensor (default: 0)
	void setPassword(uint32_t value) {
		_password = value;
	};
	// [101] set the minimum confidence below which the match is not considered valid (default: 0)
	void setMinConfidence(uint16_t value) {
		_min_confidence = value;
	};
	// [102] wait for a valid fingerprint for the given amount of seconds. Useful when battery powered (default: 0)
	void setWaitFingerForSeconds(int value) {
		_wait_finger_for_seconds = value;
	};
	// return true if the fingerprint was recognized successfully, false otherwise. Useful when a hook function needs to act upon the result
	bool getFingerprintIsValid() {
		return _fingerprint_is_valid;
	};

	// define what to do during setup
	void onSetup() {
		// do not average the value
		children.get()->setValueProcessing(NONE);
		// setup software serial
		_serial = new SoftwareSerial(_rx_pin,_tx_pin);
		// setup fingerprint sensor
		_finger = new Adafruit_Fingerprint(_serial,_password);
		// connect to the sensor
		_finger->begin(_baud_rate);
		if (_finger->verifyPassword()) {
			_finger->getTemplateCount();
			debug(PSTR(LOG_SENSOR "%s:CONN OK t=%d\n"),_name,_finger->templateCount);
		}
		else debug(PSTR("!" LOG_SENSOR "%s:CONN KO\n"));
		// report immediately
		setReportTimerMode(IMMEDIATELY);
	};

	// define what to do during loop
	void onLoop(Child* child) {
		_fingerprint_is_valid = false;
		// start the timer
		long start_millis = millis();
		while(true) {
			// if a timer is set, leave the cycle if over
			if (_wait_finger_for_seconds > 0 && ((millis() - start_millis) > (unsigned long)_wait_finger_for_seconds*1000)) break;
			// read the fingerprint
			int finger = _readFingerprint();
			if (finger > 0) {
				_fingerprint_is_valid = true;
				// fingerprint match found, send the template ID back
				child->setValue(finger);
				// leave the loop so we can report back
				break;
			}
			//don't need to run this at full speed
			wait(50);
		}
	};

#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 101: setMinConfidence(request->getValueInt()); break;
		case 102: setWaitFingerForSeconds(request->getValueInt()); break;
		default: return;
		}
	};
#endif

protected:
	int _readFingerprint() {
		// take image
		uint8_t p = _finger->getImage();
		if (p != FINGERPRINT_OK) return -1;
		// convert image
		p = _finger->image2Tz();
		if (p != FINGERPRINT_OK) return -1;
		// search for a fingerprint
		p = _finger->fingerFastSearch();
		if (p != FINGERPRINT_OK) return -1;
		// fingerprint found
		debug(PSTR(LOG_SENSOR "%s:READ t=%d c=%d\n"),_name,_finger->fingerID,_finger->confidence);
		// ignore the match if not confident enough
		if (_finger->confidence < _min_confidence) return -1;
		return _finger->fingerID; 
	};
};
#endif