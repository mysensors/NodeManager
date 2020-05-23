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
#ifndef SensorPca9685W_h
#define SensorPca9685W_h

/*
SensorPca9685W
*/

#include "SensorPca9685Led.h"

class SensorPca9685W: public Sensor {
protected:
    SensorPca9685Led* _pca9685w;
    int _channel = -1;
    int _i2c_addr = 0xFF;
    Adafruit_PWMServoDriver* _pca9685 = NULL;
    int _status = OFF;
    int _easing = SensorPca9685Led::EASE_LINEAR;
    int _duration = 1000;
    int _step_duration = 100;
public:

	SensorPca9685W(uint8_t child_id = 0, int channel = 0, uint8_t i2c_addr = 0x40, Adafruit_PWMServoDriver* pca9685 = NULL): Sensor(-1) {
	  _name = "Pca9685W";
	  //present as Dimmer
	  children.allocateBlocks(2);
	  new Child(this,INT,nodeManager.getAvailableChildId(child_id),S_DIMMER,V_PERCENTAGE,_name, NULL, true);
	  new Child(this,INT,child_id > 0 ? nodeManager.getAvailableChildId(child_id+1) : nodeManager.getAvailableChildId(child_id),S_DIMMER,V_STATUS,_name, NULL, true);
	  //store values for PWMServoDriver
	  _channel = channel;
	  _i2c_addr = i2c_addr;
	  _pca9685 = pca9685;
	};

	// [101] set the effect to use for a smooth transition, can be one of SensorPca9685Rgbw::EASE_LINEAR (0), SensorPca9685Rgbw::EASE_INSINE (1), SensorPca9685Rgbw::EASE_OUTSINE (2), SensorPca9685Rgbw::EASE_INOUTSINE (3) (default: EASE_LINEAR)
	void setEasing(int value) {
	  _pca9685w->setEasing(value);
	};
	// [102] the duration of entire the transition in milliseconds (default: 1000)
    void setDuration(int value) {
	  _pca9685w->setDuration(value);
	};
	// [103] the duration of a single step of the transition in milliseconds (default: 100)
	void setStepDuration(int value) {
	  _pca9685w->setStepDuration(value);
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
		_pca9685w = new SensorPca9685Led (_channel, _i2c_addr, _pca9685); 
		_pca9685w->onSetup();
		// report immediately
		setReportTimerMode(IMMEDIATELY);
		// do not average the value
		children.get(1)->setValueProcessing(NONE);
		children.get(2)->setValueProcessing(NONE);
	};

	
	// what to do during loop
	void onLoop(Child* child) {
	  _pca9685w->faderInc();
	};

	// what to do as the main task when receiving a message
	void onReceive(MyMessage* message) {
	  Child* child = getChild(message->sensor);
	  if (child == nullptr) return;
	  
	  // handle a SET command
	  if (message->getCommand() == C_SET && (message->type == V_PERCENTAGE || message->type == V_STATUS)) {
		// if changing the w value
		if (message->type == V_PERCENTAGE)  setWVal(message->getInt());
		else if (message->type == V_STATUS) setStatus(message->getInt());
	  }
	  // handle REQ command
	  else if (message->getCommand() == C_REQ && (message->type == V_PERCENTAGE || message->type == V_STATUS)) {
		// return the current percentage
		if (message->type == V_PERCENTAGE)  children.get(1)->setValue(getWVal());
		else if (message->type == V_STATUS) children.get(2)->setValue(getStatus());
	  }
	};

	// set the W value as status -- STATUS: 0=0%; 1=100%
	void setStatus(bool mystatus) {
	 //write to hw
	  _pca9685w->setValPercentage((int)mystatus * 100);
	 //also write to children
	 children.get(1)->setValue(getWVal());
	 children.get(2)->setValue(getStatus());   
	};

	// get the W value as status -- STATUS: 0=0%; 1 .. if>0%
	bool getStatus() {
	  return (_pca9685w->getValPercentage()>0 ? 1 : 0);  
	};

	// set the W value in percentage
	void setWVal(int percentage) {
	  //write to hw
	  _pca9685w->setValPercentage(percentage);  
	  //also write to children
	  children.get(1)->setValue(getWVal());
	  children.get(2)->setValue(getStatus());   
	};

	// get the W value in percentage
	int getWVal() {
	  return _pca9685w->getValPercentage();
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