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

/**************************
Analog Light and Temperature Sensor

The following sketch can be used to report the temperature and the light level based on a thermistor and LDR sensors 
attached to two analog pins of the arduino board (A1 and A2). Both the thermistor and the LDR are connected to ground 
on one side and to vcc via a resistor on the other so to measure the voltage drop across each of them through the 
analog pins.
The sensor will be put to sleep after startup and will report both the measures every 10 minutes. NodeManager 
will take care of presenting the sensors, managing the sleep cycle, reporting the battery level every hour and 
report the measures in the appropriate format. 
Even if the sensor is sleeping most of the time, it can be potentially woke up by sending a V_CUSTOM message 
to NodeManager service child id (200 by default) just after having reported its heartbeat. At this point the node 
will report awake and the user can interact with it by e.g. sending REQ messages to its child IDs, changing the 
duration of a sleep cycle, etc.
*/


/**********************************
* MySensors node configuration
*/

// General settings
#define SKETCH_NAME "LightTemperatureSensor"
#define SKETCH_VERSION "1.0"
#define MY_BAUD_RATE 9600
#define MY_NODE_ID 99

// NRF24 radio settings
#define MY_RADIO_NRF24

/***********************************
* NodeManager configuration
*/

#define NODEMANAGER_SLEEP ON
#define NODEMANAGER_OTA_CONFIGURATION ON

// import NodeManager library (a nodeManager object will be then made available)
#include <MySensors_NodeManager.h>

/***********************************
* Add your sensors
*/

// Add a battery sensor
#include <sensors/SensorBattery.h>
SensorBattery battery;

// Add a LDR sensor
#include <sensors/SensorLDR.h>
SensorLDR ldr(A1);

// Add a thermistor sensor
#include <sensors/SensorThermistor.h>
SensorThermistor thermistor(A0);

/***********************************
* Main Sketch
*/

// before
void before() {

/***********************************
* Configure your sensors
*/

	// set reporting interval for all the sensors to 10 minutes
	nodeManager.setReportIntervalMinutes(10);
	// set sleep interval to 10 minutes
	nodeManager.setSleepMinutes(10);

	// call NodeManager before routine
	nodeManager.before();
}

// presentation
void presentation() {
	// call NodeManager presentation routine
	nodeManager.presentation();
}

// setup
void setup() {
	// call NodeManager setup routine
	nodeManager.setup();
}

// loop
void loop() {
	// call NodeManager loop routine
	nodeManager.loop();
}

#if NODEMANAGER_RECEIVE == ON
// receive
void receive(const MyMessage &message) {
	// call NodeManager receive routine
	nodeManager.receive(message);
}
#endif

#if NODEMANAGER_TIME == ON
// receiveTime
void receiveTime(unsigned long ts) {
	// call NodeManager receiveTime routine
	nodeManager.receiveTime(ts);
}
#endif