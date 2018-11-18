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

/******************************************
Sensor: provide functionalities common to all the sensors
*/

#include "Sensor.h"

// constructor
Sensor::Sensor() {  
}
Sensor::Sensor(int8_t pin) {
	_pin = pin;
	// initialize the timers
	_report_timer = new Timer();
	_measure_timer = new Timer();
	// register the sensor with the node
	nodeManager.registerSensor(this);
}

// return the name of the sensor
const char* Sensor::getName() {
	return _name;
}

// setter/getter
void Sensor::setPin(int8_t value) {
	_pin = value;
}
void Sensor::setSamples(unsigned int value) {
	_samples = value;
}
void Sensor::setSamplesInterval(unsigned long value) {
	_samples_interval = value;
}
#if NODEMANAGER_POWER_MANAGER == ON
void Sensor::setPowerPins(int8_t ground_pin, int8_t vcc_pin, unsigned long wait_time) {
	if (_powerManager == nullptr) return;
	_powerManager->setPowerPins(ground_pin, vcc_pin, wait_time);
}
void Sensor::powerOn() {
	if (_powerManager == nullptr) return;
	_powerManager->powerOn();
}
void Sensor::powerOff() {
	if (_powerManager == nullptr) return;
	_powerManager->powerOff();
}
#endif
#if NODEMANAGER_INTERRUPTS == ON
int8_t Sensor::getInterruptPin() {
	return _interrupt_pin;
}
void Sensor::setInterruptMode(uint8_t value) {
	_interrupt_mode = value;
}
void Sensor::setWaitAfterInterrupt(unsigned long value) {
	_wait_after_interrupt = value;
}
void Sensor::setPinInitialValue(int8_t value) {
	_initial_value = value;
}
void Sensor::setInterruptStrict(bool value) {
	_interrupt_strict = value;
}
#endif

void Sensor::setReportTimerMode(timer_mode value) {
	_report_timer->setMode(value);
}
void Sensor::setReportTimerValue(unsigned long value) {
	_report_timer->setValue(value);
}
void Sensor::setMeasureTimerMode(timer_mode value) {
	_measure_timer->setMode(value);
}
void Sensor::setMeasureTimerValue(unsigned long value) {
	_measure_timer->setValue(value);
}
void Sensor::setReportIntervalSeconds(unsigned long value) {
	_report_timer->setMode(TIME_INTERVAL);
	_report_timer->setValue(value);
}
void Sensor::setReportIntervalMinutes(unsigned long value) {
	setReportIntervalSeconds(value*60UL);
}
void Sensor::setReportIntervalHours(unsigned int value) {
	setReportIntervalSeconds(value*3600UL);
}
void Sensor::setReportIntervalDays(uint8_t value) {
	setReportIntervalSeconds(value*86400UL);
}

// return true if it is the first execution of loop on this sensor
bool Sensor::getFirstRun() {
	return _first_run;
}

// register a child
void Sensor::registerChild(Child* child) {
	children.push(child);
}

// return the requested child 
Child* Sensor::getChild(uint8_t child_id) {
	for (List<Child*>::iterator itr = children.begin(); itr != children.end(); ++itr) {
		Child* child = *itr;
		if (child->getChildId() == child_id) return child;
	}
	return nullptr;
}

// present the sensor to the gateway and controller
void Sensor::presentation() {
	for (List<Child*>::iterator itr = children.begin(); itr != children.end(); ++itr) {
		Child* child = *itr;
		debug(PSTR(LOG_PRESENTATION "%s(%d) p=%d t=%d\n"),child->getDescription(),child->getChildId(),child->getPresentation(),child->getType());
		present(child->getChildId(), child->getPresentation(), child->getDescription(), nodeManager.getAck());
	}
}

// call the sensor-specific implementation of setup
void Sensor::setup() {
#if NODEMANAGER_POWER_MANAGER == ON
	// turn the sensor on
	powerOn();
#endif
	// configure default reporting interval if not already set by the user
	if (_report_timer->getMode() == NOT_CONFIGURED) {
		_report_timer->setMode(TIME_INTERVAL);
		_report_timer->setValue(nodeManager.getReportIntervalSeconds());
	}
	// if the user has not set any custom measurement timer, measure and reporting timer will be the same
	if (_measure_timer->getMode() == NOT_CONFIGURED) {
		_measure_timer->setMode(_report_timer->getMode());
		_measure_timer->setValue(_report_timer->getValue());
	}
	// call onSetup(), the sensor implementation of setup()
	onSetup();
	// start the timers
	_report_timer->start();
	_measure_timer->start();
#if NODEMANAGER_INTERRUPTS == ON
	// for interrupt based sensors, register a callback for the interrupt
	if (_interrupt_mode != MODE_NOT_DEFINED) {
		_interrupt_pin = _pin;
		nodeManager.setInterrupt(_pin,_interrupt_mode,_initial_value);
	}
#endif
#if NODEMANAGER_EEPROM == ON
	// if there is any child which is supposed to persist its value in EEPROM, load the last saved value
	for (List<Child*>::iterator itr = children.begin(); itr != children.end(); ++itr) {
		Child* child = *itr;
		if (child->getPersistValue()) child->loadValue();
	}
#endif
#if NODEMANAGER_HOOKING == ON
	// if a hook function is defined, call it
	if (_setup_hook != 0) _setup_hook(this); 
#endif
#if NODEMANAGER_POWER_MANAGER == ON
	// turn the sensor off
	powerOff();
#endif
}

// call the sensor-specific implementation of loop
void Sensor::loop(MyMessage* message) {
	// run the sensor's loop() function if the timer is over OR we've been called from receive()
	if (_evaluateTimer(_measure_timer) || message != nullptr) {
#if NODEMANAGER_POWER_MANAGER == ON
		// turn the sensor on
		powerOn();
#endif
#if NODEMANAGER_HOOKING == ON
		// if a hook function is defined, call it
		if (_pre_loop_hook != 0) _pre_loop_hook(this); 
#endif
		// iterates over all the children
		for (List<Child*>::iterator itr = children.begin(); itr != children.end(); ++itr) {
			Child* child = *itr;
			// if a specific child is requested from receive(), skip all the others
			if (message != nullptr && message->sensor != child->getChildId()) continue;
			// collect multiple samples if needed
			for (unsigned int i = 0; i < _samples; i++) {
				// we've been called from receive(), pass the message along
				if (message != nullptr) onReceive(message);
				// we've been called from loop()
				else onLoop(child);
				// wait between samples
				if (_samples_interval > 0) nodeManager.sleepOrWait(_samples_interval);
			}
		}
#if NODEMANAGER_HOOKING == ON
		// if a hook function is defined, call it
		if (_post_loop_hook != 0) _post_loop_hook(this); 
#endif
#if NODEMANAGER_POWER_MANAGER == ON
		// turn the sensor off
		powerOff();
#endif
		// restart the timer if over
		if (_measure_timer->isOver()) _measure_timer->start();
	}
	// send the latest measure back to the network if the timer is over OR we've been called from receive()
	if (_evaluateTimer(_report_timer) || message != nullptr) {
		// iterates over all the children
		for (List<Child*>::iterator itr = children.begin(); itr != children.end(); ++itr) {
			Child* child = *itr;
			// if a specific child is requested from receive(), skip all the others
			if (message != nullptr && message->sensor != child->getChildId()) continue;
			// send the value back to the controller
			child->sendValue(message != nullptr);
		}
		// restart the timer if over
		if (_report_timer->isOver()) _report_timer->start();
	}
	// unset first run if set
	if (_first_run) _first_run = false;
}

#if NODEMANAGER_INTERRUPTS == ON
// receive and handle an interrupt
bool Sensor::interrupt() {
	// ignore the interrupt if the value is not matching the one expected
	if (_interrupt_strict && ((_interrupt_mode == RISING && nodeManager.getLastInterruptValue() != HIGH ) || (_interrupt_mode == FALLING && nodeManager.getLastInterruptValue() != LOW))) return false;
	// call the sensor's implementation of onInterrupt()
	onInterrupt();
	// wait after interrupt if needed
	if (_wait_after_interrupt > 0) nodeManager.sleepOrWait(_wait_after_interrupt);
#if NODEMANAGER_HOOKING == ON
	// if a hook function is defined, call it
	if (_interrupt_hook != 0) _interrupt_hook(this); 
#endif
	return true;
}
#endif

#if NODEMANAGER_RECEIVE == ON
// receive a message from the radio network
void Sensor::receive(MyMessage* message) {
	// a request would make the sensor executing its main task passing along the message
	loop(message);
#if NODEMANAGER_HOOKING == ON
	// if a hook function is defined, call it
	if (_receive_hook != 0) _receive_hook(this,message); 
#endif
}
#endif

#if NODEMANAGER_POWER_MANAGER == ON
void Sensor::setPowerManager(PowerManager& powerManager) {
	_powerManager = &powerManager;
}
#endif

#if NODEMANAGER_HOOKING == ON
void Sensor::setSetupHook(void (*function)(Sensor* sensor)) {
	_setup_hook = function;
}
void Sensor::setPreLoopHook(void (*function)(Sensor* sensor)) {
	_pre_loop_hook = function;
}
void Sensor::setPostLoopHook(void (*function)(Sensor* sensor)) {
	_post_loop_hook = function;
}
void Sensor::setInterruptHook(void (*function)(Sensor* sensor)) {
	_interrupt_hook = function;
}
void Sensor::setReceiveHook(void (*function)(Sensor* sensor, MyMessage* message)) {
	_receive_hook = function;
}
#endif

// virtual functions
void Sensor::onSetup(){
}
void Sensor::onLoop(Child* child){}

// by default when a child receive a REQ message and the type matches the type of the request, executes its onLoop function
void Sensor::onReceive(MyMessage* message){
	Child* child = getChild(message->sensor);
	if (child == nullptr) return;
	if (message->getCommand() == C_REQ && message->type == child->getType()) onLoop(child);
}
void Sensor::onInterrupt(){}
#if NODEMANAGER_OTA_CONFIGURATION == ON
void Sensor::onOTAConfiguration(ConfigurationRequest* request) {}
#endif

// evaluate the timer and return true if can be considered over
bool Sensor::_evaluateTimer(Timer* timer) {
	// timer is over
	if (timer->isOver()) return true;
	if (_first_run) {
#if NODEMANAGER_TIME == ON || NODEMANAGER_RTC == ON
		// if it is the first run but we need to expect a specific time, return false
		if (timer->getMode() == EVERY_MINUTE || 
			timer->getMode() == EVERY_HOUR || 
			timer->getMode() == EVERY_DAY || 
			timer->getMode() == AT_MINUTE || 
			timer->getMode() == AT_HOUR ||
			timer->getMode() == AT_DAY) return false;
#endif
		// this is the first run, always return tru
		return true;
	}
	return false;
}
