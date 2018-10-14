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
#ifndef SensorPN532_h
#define SensorPN532_h

/*
SensorPN532
*/

#include <Wire.h>
#include <PN532_I2C.h>
#include <PN532.h>
#include <NfcAdapter.h>
  
class SensorPN532: public Sensor {
protected:
	PN532_I2C* _pn532i2c = new PN532_I2C(Wire);
	PN532* _nfc = new PN532(*_pn532i2c);
	bool _card_is_valid = false;
	int _wait_card_for_seconds = 0;
	
public:
	SensorPN532(uint8_t child_id = 0): Sensor(-1) {
		_name = "PN532";
		children.allocateBlocks(1);
		new Child(this,STRING,nodeManager.getAvailableChildId(child_id), S_CUSTOM, V_CUSTOM, _name);
	};
	
	// [101] wait for a valid card for the given amount of seconds. Useful when battery powered (default: 0)
	void setWaitCardForSeconds(int value) {
		_wait_card_for_seconds = value;
	};

	// return true if the card was recognized successfully, false otherwise. Useful when a hook function needs to act upon the result
	bool getCardIsValid() {
		return _card_is_valid;
	};

	// define what to do during setup
	void onSetup() {
		// initialize the library
		_nfc->begin();
		uint32_t versiondata = _nfc->getFirmwareVersion();
		if (! versiondata) debug(PSTR(LOG_SENSOR "KO\n"));
		else debug(PSTR(LOG_LOOP "PN5%x v=%d.%d\n"),(versiondata>>24) & 0xFF,(versiondata>>16) & 0xFF,(versiondata>>8) & 0xFF);
		// Set the max number of retry attempts to read from a card. This prevents us from waiting forever for a card, which is the default behaviour of the PN532.
		_nfc->setPassiveActivationRetries(0xFF);		
		// configure board to read RFID tags
		_nfc->SAMConfig();
		// report immediately
		setReportTimerMode(IMMEDIATELY);
	};

	// define what to do during loop
	void onLoop(Child* child) {
		// Buffer to store the returned UID
		uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  
		// Length of the UID (4 or 7 bytes depending on ISO14443A card type)
		uint8_t uid_length;
		_card_is_valid = false;
		// start the timer
		long start_millis = millis();
		while(true) {
			// if a timer is set, leave the cycle if over
			if (_wait_card_for_seconds > 0 && ((millis() - start_millis) > (unsigned long)_wait_card_for_seconds*1000)) break;
			// read the card
			if (_nfc->readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uid_length)) {
				// card read successfully, send the UID back
				_card_is_valid = true;
				String uid_string = String(uid[0], HEX);
				for (uint8_t i = 1; i < uid_length; i++) uid_string = uid_string + String(uid[i], HEX);
				char* uid_value = new char[9];
				for (int j = 0; j < 8; j++) uid_value[j] = uid_string[j];
				uid_value[8] = 0;
				child->setValue(uid_value);
				// leave the loop so we can report back
				break;
			}
			wait(500);
		}
	};

#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 101: setWaitCardForSeconds(request->getValueInt()); break;
		default: return;
		}
	};
#endif
};
#endif