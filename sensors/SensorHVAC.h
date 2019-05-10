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

Based on https://www.home-assistant.io/components/climate.mysensors/ sample sketch

*/
#ifndef SensorHVAC_h
#define SensorHVAC_h

// Uncomment your heatpump model
#include <FujitsuHeatpumpIR.h>
#include <PanasonicCKPHeatpumpIR.h>
#include <PanasonicHeatpumpIR.h>
#include <CarrierHeatpumpIR.h>
#include <MideaHeatpumpIR.h>
#include <MitsubishiHeatpumpIR.h>
#include <SamsungHeatpumpIR.h>
#include <SharpHeatpumpIR.h>
#include <DaikinHeatpumpIR.h>


/*
SensorHVAC: control an Air condntioner 
*/
class SensorHVAC: public Sensor {
protected:

	//variables to hold the states
	int _power_state;
	int _temp_state;
	int _fan_state;
	int _mode_state;
	int _vdir_state;
	int _hdir_state;

	// IR led on Arduino digital pin 3, using Arduino PWM
	IRSenderPWM* _irSender = NULL;       
	//Change to your Heatpump
	HeatpumpIR *_heatpumpIR = NULL;


	
public:
	SensorHVAC(int8_t pin, HeatpumpIR *heatpumpIR, uint8_t child_id = 0): Sensor(pin) {
		_name = "HVAC";

		_irSender = new IRSenderPWM(pin);
		_heatpumpIR = heatpumpIR;

		children.allocateBlocks(3);
		int cid = child_id;
		if (cid == 0)
 	           cid = nodeManager.getAvailableChildId(child_id); 

		new Child(this, FLOAT, cid, S_HVAC, V_HVAC_SETPOINT_COOL, _name);
		new Child(this, STRING, cid, S_HVAC, V_HVAC_SPEED, _name);
		new Child(this, STRING, cid, S_HVAC, V_HVAC_FLOW_STATE, _name);
	};
	
	// what to do during setup
	void onSetup() {

		pinMode(_pin, OUTPUT);

		getChild(1, V_HVAC_SETPOINT_COOL)->setValue(20.0f);
		getChild(1, V_HVAC_SPEED)->setValue("Auto");
		getChild(1, V_HVAC_FLOW_STATE)->setValue("Off");
	};

	// what to do when receiving a message
	void onReceive(MyMessage* message) {

		if (message->getCommand() != C_SET) 
			return;

		Child* child = getChild(message->sensor, message->type);
		if (child == nullptr) 
			return;

		switch (message->type) {
			case V_HVAC_SPEED:
			{
				String recvData = message->getString();

				//debug(PSTR(LOG_SENSOR "%s(%d): %s=%s\n"),_name,child->getChildId(),"V_HVAC_SPEED", recvData);
				//Serial.println("V_HVAC_SPEED");

				if(recvData.equalsIgnoreCase("auto")) _fan_state = 0;
				else if(recvData.equalsIgnoreCase("min")) _fan_state = 1;
				else if(recvData.equalsIgnoreCase("normal")) _fan_state = 2;
				else if(recvData.equalsIgnoreCase("max")) _fan_state = 3;
				
				char data[MAX_PAYLOAD + 1];
				recvData.toCharArray(data, sizeof(data));
				child->setValue(data);

				break;
			}
			case V_HVAC_SETPOINT_COOL:
			{
				_temp_state = message->getFloat();

				//debug(PSTR(LOG_SENSOR "%s(%d): %s = %f\n"),
				//	_name,child->getChildId(),"V_HVAC_SETPOINT_COOL", _temp_state);
				//Serial.println("V_HVAC_SETPOINT_COOL");
				//Serial.println(_temp_state);
				
				child->setValue(_temp_state);
			}
			break;

			case V_HVAC_FLOW_STATE:
			{
				String recvData = message->getString();

				//debug(PSTR(LOG_SENSOR "%s(%d): %s = %s\n"),
				//	_name,child->getChildId(),"V_HVAC_FLOW_STATE", recvData);
				//Serial.println("V_HVAC_FLOW_STATE");
				if (recvData.equalsIgnoreCase("coolon")) {
					_power_state = 1;
					_mode_state = MODE_COOL;
				}
				else if (recvData.equalsIgnoreCase("heaton")) {
					_power_state = 1;
					_mode_state = MODE_HEAT;
				}
				else if (recvData.equalsIgnoreCase("autochangeover")) {
					_power_state = 1;
					_mode_state = MODE_AUTO;
				}
				else if (recvData.equalsIgnoreCase("off")){
					_power_state = 0;
				}
				//send value back

				char data[MAX_PAYLOAD + 1];
				recvData.toCharArray(data, sizeof(data));
				child->setValue(data);
			break;
			}
		}
		sendHeatpumpCommand();
	};

protected:

	void sendHeatpumpCommand() {
		//Serial.println("Power = " + (String)POWER_STATE);
		//Serial.println("Mode = " + (String)MODE_STATE);
		//Serial.println("Fan = " + (String)FAN_STATE);
		//Serial.println("Temp = " + (String)TEMP_STATE);

		_heatpumpIR->send(*_irSender, _power_state, _mode_state, _fan_state, _temp_state, VDIR_AUTO, HDIR_AUTO);
	}
};
#endif
