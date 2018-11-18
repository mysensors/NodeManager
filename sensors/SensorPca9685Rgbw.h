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
#ifndef SensorPca9685Rgbw_h
#define SensorPca9685Rgbw_h

/*
SensorPca9685Rgbw
*/

#include "SensorPca9685Led.h"

class SensorPca9685Rgbw: public Sensor {
protected:
    SensorPca9685Led* _pca9685r;
    SensorPca9685Led* _pca9685g;
    SensorPca9685Led* _pca9685b;
    SensorPca9685Led* _pca9685w;
    int _ch_r = -1;
    int _ch_g = -1;
    int _ch_b = -1;
    int _ch_w = -1;
    int _i2c_addr = 0xFF;
    Adafruit_PWMServoDriver* _pca9685 = NULL;
    int _status = OFF;
    int _easing = SensorPca9685Led::EASE_LINEAR;
    int _duration = 1000;
    int _step_duration = 100;
public:

	SensorPca9685Rgbw(uint8_t child_id = 0, int ch_r = 0,int ch_g = 0,int ch_b = 0,int ch_w = 0, uint8_t i2c_addr = 0x40, Adafruit_PWMServoDriver* pca9685 = NULL): Sensor(-1) {
	  _name = "Pca9685Rgbw";
	  //present as RGB
	  children.allocateBlocks(2);
	  new Child(this,STRING,nodeManager.getAvailableChildId(child_id),S_RGB_LIGHT,V_RGB,_name); 
	  new Child(this,STRING,child_id > 0 ? child_id > 0 ? nodeManager.getAvailableChildId(child_id+1) : nodeManager.getAvailableChildId(child_id) : nodeManager.getAvailableChildId(child_id),S_RGB_LIGHT,V_WATT,_name);
	  //store values for PWMServoDriver
	  _ch_r = ch_r;
	  _ch_g = ch_g;
	  _ch_b = ch_b;
	  _ch_w = ch_w;
	  _i2c_addr = i2c_addr;
	  _pca9685 = pca9685;
	};

	// [101] set the effect to use for a smooth transition, can be one of SensorPca9685Rgbw::EASE_LINEAR (0), SensorPca9685Rgbw::EASE_INSINE (1), SensorPca9685Rgbw::EASE_OUTSINE (2), SensorPca9685Rgbw::EASE_INOUTSINE (3) (default: EASE_LINEAR)
	void setEasing(int value) {
	  _pca9685r->setEasing(value);
	  _pca9685g->setEasing(value);
	  _pca9685b->setEasing(value);
	  _pca9685w->setEasing(value);
	};
	// [102] the duration of entire the transition in milliseconds (default: 1000)
	void setDuration(int value) {
	  _pca9685r->setDuration(value);
	  _pca9685g->setDuration(value);
	  _pca9685b->setDuration(value);
	  _pca9685w->setDuration(value);
	};
	// [103] the duration of a single step of the transition in milliseconds (default: 100)
	void setStepDuration(int value) {
	  _pca9685r->setStepDuration(value);
	  _pca9685g->setStepDuration(value);
	  _pca9685b->setStepDuration(value);
	  _pca9685w->setDuration(value);
	};
	//set instance of PCA9685-board, if using more than one pca9685-dimmer sensor on the same pca9685-board
	void setPWMServoDriver(Adafruit_PWMServoDriver* servoDriver) {
	  _pca9685 = servoDriver;
	};
	//get instance of PCA9685-board
	Adafruit_PWMServoDriver* getPWMServoDriver() {
	  return _pca9685;
	};

	// what to do during setup
	void onSetup() {
	    //init pca9685
	    _pca9685r = new SensorPca9685Led (_ch_r, _i2c_addr, _pca9685); 
	    _pca9685g = new SensorPca9685Led (_ch_g, _i2c_addr, _pca9685); 
	    _pca9685b = new SensorPca9685Led (_ch_b, _i2c_addr, _pca9685); 
		_pca9685w = new SensorPca9685Led (_ch_w, _i2c_addr, _pca9685); 
	    _pca9685r->onSetup();
	    _pca9685g->onSetup();
		_pca9685b->onSetup();
		_pca9685w->onSetup();
		//get current value from host
		request( children.get(1)->getChildId(), V_RGB  );
		// report immediately
		setReportTimerMode(IMMEDIATELY);
		// do not average the value
		children.get(1)->setValueProcessing(NONE);
		children.get(2)->setValueProcessing(NONE);
	};

	// what to do during loop
	void onLoop(Child* child) {
	  _pca9685r->faderInc();
	  _pca9685g->faderInc();
	  _pca9685b->faderInc();
	  _pca9685w->faderInc();
	};

	// what to do as the main task when receiving a message
	void onReceive(MyMessage* message) {
	  Child* child = getChild(message->sensor);
	  if (child == nullptr) return;
	  // heandle a SET command
	  if (message->getCommand() == C_SET && message->type == child->getType()) {
		// if changing the rgb value
		if (child->getType() == V_RGBW) setRgbwVal(String(message->getString()));
	  }
	  // handle REQ command
	  else if (message->getCommand() == C_REQ && message->type == child->getType()) {
		// return the current status
		if (child->getType() == V_RGBW) child->setValue(getRgbwVal().c_str());
	  }
	};

	// set the RGB value
	void setRgbwVal(String hexstring) {
	  //set value for hw
	  _pca9685r->setValHex(hexstring.substring(0,2));
	  _pca9685g->setValHex(hexstring.substring(2,4));
	  _pca9685b->setValHex(hexstring.substring(4,6));
	  _pca9685w->setValHex(hexstring.substring(6,8));
	  //also write to children
	  children.get(1)->setValue(getRgbwVal().c_str()); 
	};

	// get the RGB value
	String getRgbwVal() {
	  String return_value = "";
	  return_value += _pca9685r->getValHex();
	  return_value += _pca9685g->getValHex();
	  return_value += _pca9685b->getValHex();
	  return_value += _pca9685w->getValHex();
	  return return_value;
	};
	
#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 101: setEasing(request->getValueInt()); break;
		case 102: setDuration(request->getValueInt()); break;
		case 103: setStepDuration(request->getValueInt()); break;
		default: return;
		}
	};
#endif
};
#endif