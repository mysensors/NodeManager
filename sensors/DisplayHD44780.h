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
#ifndef DisplayHD44780_h
#define DisplayHD44780_h

/*
* Hitachi HD44780 display
*/

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#include "sensors/Display.h"

class DisplayHD44780: public Display {
protected:
	LiquidCrystal_I2C* _lcd;
	uint8_t _i2caddress = 0x38;
	int _column = 0;
public:
	DisplayHD44780(NodeManager& node_manager, int child_id = -255): Display(node_manager, child_id) {
		_name = "HD44780";
		children.get(1)->setDescription(_name);
	};

	// set i2c address (default: 0x38)
	void setI2CAddress(uint8_t i2caddress) {
		_i2caddress = i2caddress;
	};
	// set the backlight (default: HIGH)
	void setBacklight(uint8_t value) {
		_lcd->setBacklight(value);
	};

	// display specific functions
	void printCaption(const char* value) {
		if (strlen(value) > 0) println(value);
	};
	void print(const char* value) {
		// print the string
		_lcd->print(value);
	};
	void println(const char* value) {
		if (value != nullptr) print(value);
		_column = _column + 1;
		setCursor(0,_column);
	};
	void printChild(Child* child) {
		child->print(*_lcd);
	};
	void clear() {
		_column = 0;
		_lcd->clear();
	};
	void setCursor(int col,int row) {
		_lcd->setCursor(col,row);
	};
	
	// define what to do during setup
	void onSetup() {
		_lcd = new LiquidCrystal_I2C(_i2caddress, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
		_lcd->begin(16,2);
		_lcd->home();
		clear();
	};
};
#endif