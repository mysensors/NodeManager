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
#ifndef DisplaySSD1306_h
#define DisplaySSD1306_h

/*
* SSD1306 OLED display
*/

#include <SSD1306Ascii.h>
#include <SSD1306AsciiAvrI2c.h>

#include "Display.h"

class DisplaySSD1306: public Display {
protected:
	SSD1306AsciiAvrI2c* _oled;
	const DevType* _dev = &Adafruit128x64;
	uint8_t _i2caddress = 0x3c;
	int _fontsize = 1;
	int _caption_fontsize = 2;
	const uint8_t* _font = Adafruit5x7;
	uint8_t _contrast = -1;
	
public:
	DisplaySSD1306(NodeManager& node_manager, int child_id = -255): Display(node_manager, child_id) {
		_name = "SSD1306";
		children.get(1)->setDescription(_name);
	};

	// set device
	void setDev(const DevType* dev) {
		_dev = dev;
	};
	// set i2c address
	void setI2CAddress(uint8_t i2caddress) {
		_i2caddress = i2caddress;
	};
	// set text font (default: &Adafruit5x7)
	void setFont(const uint8_t* font) {
		_font = font;
	};
	// [102] set the contrast of the display (0-255)
	void setContrast(uint8_t value) {
		_contrast = value;
	};
	// [104] Rotate the display 180 degree (use rotate=false to revert)
	void rotateDisplay(bool rotate = true) {
		if (rotate) {
			_oled->ssd1306WriteCmd(SSD1306_SEGREMAP);
			_oled->ssd1306WriteCmd(SSD1306_COMSCANINC);
		} else {
			_oled->ssd1306WriteCmd(SSD1306_SEGREMAP | 0x01);
			_oled->ssd1306WriteCmd(SSD1306_COMSCANDEC);
		}
	};
	// [105] Text font size (possible are 1 and 2; default is 1)
	void setFontSize(int fontsize) {
		_fontsize = (fontsize>=2) ? 2 : 1;
	};
	// [106] Text caption font size (possible are 1 and 2; default is 2)
	void setCaptionFontSize(int fontsize) {
		_caption_fontsize = (fontsize>=2) ? 2 : 1;
	};
	// [107] Invert display (black text on color background; use invert=false to revert)
	void invertDisplay(bool invert = true) {
		if (invert) {
			_oled->ssd1306WriteCmd(SSD1306_INVERTDISPLAY);
		} else {
			_oled->ssd1306WriteCmd(SSD1306_NORMALDISPLAY);
		}
	};
	
	// display specific functions
	void printCaption(const char* value) {
		// set caption font size
		if (_caption_fontsize >= 2) _oled->set2X();
		// print caption
		print(value);
		_oled->println();
		// if using a small font add an empty line
		if (_caption_fontsize == 1) _oled->println();
		// restore small font
		_oled->set1X();
	};
	void print(const char* value) {
		// set the font size
		if (_fontsize >= 2 && _oled->magFactor() != 2) _oled->set2X();
		// print the string
		_oled->print(value);
		_oled->clearToEOL();
	};
	void println(const char* value) {
		if (value != nullptr) print(value);
		_oled->println();
	};
	void printChild(Child* child) {
		child->print(*_oled);
	};
	void clear() {
		_oled->clear();
	};
	void setCursor(int col,int row) {
		_oled->setCursor(col,row);
	};
	SSD1306AsciiAvrI2c* getDisplay() {
		return _oled;
	};
	
	// define what to do during setup
	void onSetup() {
		_oled = new SSD1306AsciiAvrI2c();
		_oled->begin(_dev, _i2caddress);
		_oled->setFont(_font);
		if (_contrast > -1) _oled->setContrast(_contrast);
		_oled->clear();
	};
	
#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 102: setContrast((uint8_t)request->getValueInt()); break;
		case 104: rotateDisplay((bool)request->getValueInt()); break;
		case 105: setFontSize(request->getValueInt()); break;
		case 106: setCaptionFontSize(request->getValueInt()); break;
		case 107: invertDisplay((bool)request->getValueInt()); break;
		default: return;
		}
	};
#endif
};
#endif