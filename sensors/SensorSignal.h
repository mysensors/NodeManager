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
#ifndef SensorSignal_h
#define SensorSignal_h

/*
SensorSignal: report RSSI signal strength from the radio
*/

#define MY_SIGNAL_REPORT_ENABLED

class SensorSignal: public Sensor {
protected:
	int _signal_command = SR_RX_RSSI;
	
public:
	SensorSignal(NodeManager& nodeManager, int child_id = SIGNAL_CHILD_ID) {
		_name = "SIGNAL";
		children.allocateBlocks(1);
		new ChildInt(this,child_id,S_SOUND,V_LEVEL,_name);
		// report signal level every 60 minutes by default
		setReportIntervalMinutes(60);
	};
	
	// [101] define which signal report to send. Possible values are SR_UPLINK_QUALITY, SR_TX_POWER_LEVEL, SR_TX_POWER_PERCENT, SR_TX_RSSI, SR_RX_RSSI, SR_TX_SNR, SR_RX_SNR (default: SR_RX_RSSI)
	void setSignalCommand(int value) {
		_signal_command = value;
	};
	
	// define what to do during loop
	void onLoop(Child* child) {
		int16_t value = transportGetSignalReport((signalReport_t)_signal_command);
		((ChildInt*)child)->setValue(value);
	};
	
#ifdef USE_CONFIGURATION
	// define what to do when receiving an OTA configuration request
	void onConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 101: setSignalCommand(request->getValueInt()); break;
		default: return;
		}
	};
#endif
};
#endif