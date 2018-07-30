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
#ifndef SensorConfiguration_h
#define SensorConfiguration_h

/*
SensorConfiguration: allow remote configuration of the board and any configured sensor
*/

#define CONFIGURATION_CHILD_ID 200

class SensorConfiguration: public Sensor {
public:
	SensorConfiguration(NodeManager& node_manager, int child_id = CONFIGURATION_CHILD_ID): Sensor(node_manager) {
		_name = "CONFIG";
		children.allocateBlocks(1);
		new ChildInt(this,child_id,S_CUSTOM,V_CUSTOM,_name);
	};

	// what to do as the main task when receiving a message
	void onReceive(MyMessage* message) {
		// expect a REQ, V_CUSTOM message
		if (message->getCommand() != C_REQ && message->type != V_CUSTOM) return;
		// parse the request
		ConfigurationRequest request = ConfigurationRequest(message->sensor,message->getString());
		int function = request.getFunction();
		int child_id = request.getChildId();
		// if the message is for the node itself
		if (child_id == 0) {
			switch(function) {
			case 1: _node->hello(); break;
#if NODEMANAGER_SLEEP == ON
			case 3: _node->setSleepSeconds(request.getValueInt()); break;
			case 4: _node->setSleepMinutes(request.getValueInt()); break;
			case 5: _node->setSleepHours(request.getValueInt()); break;
			case 29: _node->setSleepDays(request.getValueInt()); break;
			case 20: _node->setSleepBetweenSend(request.getValueInt()); break;
			case 9: _node->wakeup(); break;
#endif
#ifdef CHIP_AVR
			case 6: _node->reboot(); return;
#endif
#if NODEMANAGER_EEPROM == ON
			case 7: _node->clearEeprom(); break;
			case 27: _node->saveToMemory(0,request.getValueInt()); break;
			case 40: _node->setSaveSleepSettings(request.getValueInt()); break;
#endif
			case 8: _node->sendMessage(children.get(1)->getChildId(),V_CUSTOM,VERSION); return;
			case 10: _node->setRetries(request.getValueInt()); break;
#if NODEMANAGER_INTERRUPTS == ON
			case 19: _node->setSleepInterruptPin(request.getValueInt()); break;
			case 28: _node->setInterruptDebounce(request.getValueInt()); break;
#endif
			case 21: _node->setAck(request.getValueInt()); break;
			case 22: _node->setIsMetric(request.getValueInt()); break;
#if NODEMANAGER_POWER_MANAGER == ON
			case 24: _node->powerOn(); break;
			case 25: _node->powerOff(); break;
#endif
			case 30: _node->setSleepOrWait(request.getValueInt()); break;
			case 31: _node->setRebootPin(request.getValueInt()); break;
			case 32: _node->setADCOff(); break;
			case 36: _node->setReportIntervalSeconds(request.getValueInt()); break;
			case 37: _node->setReportIntervalMinutes(request.getValueInt()); break;
			case 38: _node->setReportIntervalHours(request.getValueInt()); break;
			case 39: _node->setReportIntervalDays(request.getValueInt()); break;
#if NODEMANAGER_TIME == ON
			case 41: _node->syncTime(); break;
			case 42: _node->sendMessage(children.get(1)->getChildId(),V_CUSTOM,(int)_node->getTime()); return;
#endif
			default: return; 
			}
		} else {
			// the request is for a sensor, retrieve the sensor the child is belonging to
			Sensor* sensor = _node->getSensorWithChild(child_id);
			if (sensor == nullptr) return;
			// if the message is for a function common to all the sensors
			if (request.getFunction() < 100) {
				switch(function) {
				case 1: sensor->setPin(request.getValueInt()); break;
				case 5: sensor->setSamples(request.getValueInt()); break;
				case 6: sensor->setSamplesInterval(request.getValueInt()); break;
#if NODEMANAGER_POWER_MANAGER == ON
				case 13: sensor->powerOn(); break;
				case 14: sensor->powerOff(); break;
#endif
				case 24: sensor->setReportTimerMode((timer_mode)request.getValueInt()); break;
				case 25: sensor->setReportTimerValue(request.getValueInt()); break;
				case 26: sensor->setMeasureTimerMode((timer_mode)request.getValueInt()); break;
				case 27: sensor->setMeasureTimerValue(request.getValueInt()); break;
				case 16: sensor->setReportIntervalMinutes(request.getValueInt()); break;
				case 17: sensor->setReportIntervalSeconds(request.getValueInt()); break;
				case 19: sensor->setReportIntervalHours(request.getValueInt()); break;
				case 20: sensor->setReportIntervalDays(request.getValueInt()); break;
#if NODEMANAGER_INTERRUPTS == ON			
				case 22: sensor->setInterruptMode(request.getValueInt()); break;
				case 23: sensor->setWaitAfterInterrupt(request.getValueInt()); break;
#endif
				default: return;
				}
			} else {
				// dispatch the message to onOTAConfiguration() implemented by each sensor
				sensor->onOTAConfiguration(&request);
			}
		}
		// reply echoing back the request
		_node->sendMessage(children.get(1)->getChildId(),V_CUSTOM,function);
	};
};
#endif