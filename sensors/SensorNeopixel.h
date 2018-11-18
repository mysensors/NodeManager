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
#ifndef SensorNeopixel_h
#define SensorNeopixel_h

/*
* Neopixel LED Sensor
*/

#if defined(ARDUINO_ARCH_STM32F0) || defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F3) || defined(ARDUINO_ARCH_STM32F4) || defined(ARDUINO_ARCH_STM32L4)
#include <NeoMaple.h>
#else
#include <Adafruit_NeoPixel.h>
#endif

class SensorNeopixel: public Sensor {
protected:
#if defined(ARDUINO_ARCH_STM32F0) || defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F3) || defined(ARDUINO_ARCH_STM32F4) || defined(ARDUINO_ARCH_STM32L4)
	NeoMaple* _pixels;
#else  
	Adafruit_NeoPixel* _pixels;
#endif
	int _num_pixels = 16;
	int _default_brightness = 255;
	
public:
	SensorNeopixel(int8_t pin, uint8_t child_id = 0): Sensor(pin) {
		_name = "NEOPIXEL";
		children.allocateBlocks(1);
		// child for controlling the color
		new Child(this,STRING,nodeManager.getAvailableChildId(child_id), S_COLOR_SENSOR, V_RGB ,_name);
		// child for controlling the brightness
		new Child(this,INT,child_id > 0 ? nodeManager.getAvailableChildId(child_id+1) : nodeManager.getAvailableChildId(child_id),S_LIGHT_LEVEL,V_LEVEL,_name);
	};
	
	// set how many NeoPixels are attached
	void setNumPixels(int value) {
		_num_pixels = value;
	};
	
	// set default brightness
	void setDefaultBrightness(int value) {
		_default_brightness = value;
	};
	
	// format expeted is "<pixel_number>,<RGB color in a packed 32 bit format>"
	//string format:
	//RRGGBB                color for all LEDs
	//LED,RRGGBB            color for specific LED
	//LEDfrom-LEDto,RRGGBB  color for LED range from LEDfrom to LEDto
	void setColor(char* string) {
		Child* child = children.get(1);
		long color = 0;
		//find separator
		char * p = strstr(string, ",");
		if (p){ 
			//extract LED or LED range part
			char pixelnum[10];
			int pos = (int) (p - string);
			if (pos >= 10)
			return;
			strncpy(pixelnum, string, pos);
			pixelnum[pos] = 0;

			int pixel_num = 0;
			int pixel_end = 0;
			//may be range, try find range separator -
			char * r = strstr(pixelnum, "-");
			if (r){ 
				pixel_end = atoi(r+1);
				*r = 0; //null terminating instead of delimeter
				pixel_num = atoi(pixelnum);
			}
			else{
				pixel_num = atoi(pixelnum);
				pixel_end = pixel_num;
			}
			color = strtol(string + pos + 1, NULL, 16);
			debug(PSTR(LOG_SENSOR "%s(%d):PIX n=%d\n"),_name,child->getChildId(),pixel_num);
			if (pixel_num != pixel_end) debug(PSTR(LOG_SENSOR "%s(%d):PIX e=%d"),_name,child->getChildId(),pixel_end);
			debug(PSTR(LOG_SENSOR "%s(%d):PIX c=%d"),_name,child->getChildId(),color);
			//set LED to color
			for(int i=pixel_num;i<=pixel_end;i++)
			_pixels->setPixelColor(i,color);
		}
		else //set All pixels to single color
		{
			color = strtol(string, NULL, 16);
			for(int i=0;i<_num_pixels;i++)
			_pixels->setPixelColor(i,color);
		}
		_pixels->show();
		//send value back
		child->setValue(string);
	};
	
	//int format:
	//0-255                brighntess for all LEDs
	void setBrightness(int value) {
		Child* child = children.get(2);
		_pixels->setBrightness(value);
		_pixels->show();
		//send value back
		child->setValue(value);
	};
	
	// define what to do during setup
	void onSetup() {
#if defined(CHIP_STM32)
		_pixels = new NeoMaple(_num_pixels, NEO_GRB + NEO_KHZ800);
#else  
		_pixels = new Adafruit_NeoPixel(_num_pixels, _pin, NEO_GRB + NEO_KHZ800);
#endif
		_pixels->begin();
		_pixels->show();
		_pixels->setBrightness(_default_brightness);
	};

	// what to do as the main task when receiving a message
	void onReceive(MyMessage* message) {
		Child* child = getChild(message->sensor);
		if (child == nullptr) return;
		if (message->getCommand() == C_SET && message->type == child->getType()) {
			if (message->type == V_LEVEL) {
				int value = (int)message->getInt();
				setBrightness(value);
			}
		} 
		else {
			char* string = (char*)message->getString();
			setColor(string);
		}
    };
};
#endif