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
#ifndef Sensor_h
#define Sensor_h

/*
Sensor
*/

#include "NodeManager.h"
#include "Timer.h"
#include "Child.h"
#if FEATURE_POWER_MANAGER == ON
#include "PowerManager.h"
#endif

class Sensor {
public:
	Sensor();
	Sensor(NodeManager& node_manager, int pin = -1);
	// return the name of the sensor
	const char* getName();
	// [1] where the sensor is attached to (default: not set)
	void setPin(int value);
	int getPin();
	// [5] For some sensors, the measurement can be queried multiple times and an average is returned (default: 1)
	void setSamples(int value);
	// [6] If more then one sample has to be taken, set the interval in milliseconds between measurements (default: 0)
	void setSamplesInterval(int value);
#if FEATURE_POWER_MANAGER == ON
	// to save battery the sensor can be optionally connected to two pins which will act as vcc and ground and activated on demand
	void setPowerPins(int ground_pin, int vcc_pin, int wait_time = 50);
	// [13] manually turn the power on
	void powerOn();
	// [14] manually turn the power off
	void powerOff();
#endif
	// [24] Set the way the timer used for reporting to the gateway should operate. It can be either TIME_INTERVAL (e.g. report every X seconds with the amount of time set with setReportTimerValue()), IMMEDIATELY (e.g. report at every cycle, useful for sensors like actuators which should report as soon as the value has changed), DO_NOT_REPORT (e.g. never report, useful for when there is no need to report, like a Display) and when FEATURE_TIME is ON, EVERY_MINUTE/EVERY_HOUR/EVERY_DAY (e.g. to report the value set in the previous timeframe, useful for sensors reporting an accumulated value linked to a timeframe at regular intervals), AT_MINUTE/AT_HOUR/AT_DAY (e.g. report at a given minute/hour/day, useful if the measure is expected at a specified time, set with setReportTimerValue())
	void setReportTimerMode(timer_mode value);
	// [25] Set the value for the reporting timer's mode which has been set with setReportTimerMode()
	void setReportTimerValue(int value);
	// [26] Set the way the timer used for taking measures should operate. Takes the same parameters as setReportTimerMode(). If not set explicitly, will be set as the reporting timer
	void setMeasureTimerMode(timer_mode value);
	// [27] Set the value for the reporting timer's mode which has been set with setReportTimerMode() If not set explicitely, will be set with the same value as the reporting timer
	void setMeasureTimerValue(int value);
    // [17] After how many seconds the sensor will report back its measure (default: 10 minutes)
    void setReportIntervalSeconds(int value);
    // [16] After how many minutes the sensor will report back its measure (default: 10 minutes)
    void setReportIntervalMinutes(int value);
    // [19] After how many hours the sensor will report back its measure (default: 10 minutes)
    void setReportIntervalHours(int value);
    // [20] After how many days the sensor will report back its measure (default: 10 minutes)
    void setReportIntervalDays(int value);
#if FEATURE_INTERRUPTS == ON
	// return the pin the interrupt is attached to
	int getInterruptPin();
	// set initial value of the configured pin. Can be used for internal pull up
	void setPinInitialValue(int value);
	// for interrupt-based sensor, set the interrupt mode. Can be CHANGE, RISING, FALLING (default: CHANGE)
	void setInterruptMode(int value);
	// [22] for interrupt-based sensor, milliseconds to wait/sleep after the interrupt before reporting (default: 0)
	void setWaitAfterInterrupt(int value);
	// [23] for interrupt-based sensor, the value of the pin is checked and the interrupt ignored if RISING and not HIGH or FALLING and not LOW (default: true)
	void setInterruptStrict(bool value);
#endif
#if FEATURE_POWER_MANAGER == ON
	// set a previously configured PowerManager to the sensor so to powering it up with custom pins
	void setPowerManager(PowerManager& powerManager);
#endif
#if FEATURE_HOOKING == ON
	// set a custom hook function to be called when the sensor executes its setup() function
	void setSetupHook(void (*function)(Sensor* sensor));
	// set a custom hook function to be called just before the sensor executes its loop() function
	void setPreLoopHook(void (*function)(Sensor* sensor));
	// set a custom hook function to be called just after the sensor executes its loop() function
	void setPostLoopHook(void (*function)(Sensor* sensor));
	// set a custom hook function to be called when the sensor executes its interrupt() function
	void setInterruptHook(void (*function)(Sensor* sensor));
	// set a custom hook function to be called when the sensor executes its receive() function
	void setReceiveHook(void (*function)(Sensor* sensor, MyMessage* message));
#endif
	// list of configured child
	List<Child*> children;
#if FEATURE_INTERRUPTS == ON
	bool interrupt();
#endif
	Child* getChild(int child_id);
	// register a child
	void registerChild(Child* child);
	NodeManager* _node;
	// define what to do at each stage of the sketch
	void presentation();
	void setup();
	void loop(MyMessage* message);
#if FEATURE_RECEIVE == ON
	void receive(MyMessage* message);
#endif
	// abstract functions, subclasses need to implement
	virtual void onSetup();
	virtual void onLoop(Child* child);
	virtual void onReceive(MyMessage* message);
	virtual void onInterrupt();
#if FEATURE_OTA_CONFIGURATION == ON
	virtual void onConfiguration(ConfigurationRequest* request);
#endif
protected:
	const char* _name = "";
	int _pin = -1;
	int _samples = 1;
	int _samples_interval = 0;
	bool _first_run = true;
#if FEATURE_INTERRUPTS == ON
	int _interrupt_pin = -1;
	int _interrupt_mode = -1;
	int _wait_after_interrupt = 0;
	int _initial_value = -1;
	bool _interrupt_strict = true;
#endif
#if FEATURE_POWER_MANAGER == ON
	PowerManager* _powerManager = nullptr;
#endif
	Timer* _report_timer;
	Timer* _measure_timer;
#if FEATURE_HOOKING == ON
	void (*_setup_hook)(Sensor* sensor);
	void (*_pre_loop_hook)(Sensor* sensor);
	void (*_post_loop_hook)(Sensor* sensor);
	void (*_interrupt_hook)(Sensor* sensor);
	void (*_receive_hook)(Sensor* sensor, MyMessage* message);
#endif
};

#endif