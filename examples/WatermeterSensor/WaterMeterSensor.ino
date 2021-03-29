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
Motion Sensor

The following sketch can be used to report back to the controller when a motion sensor attached to the board's pin 3 
triggers. In this example, the board will be put to sleep just after startup and will report a heartbeat every hour. 
NodeManager will take care of configuring an interrupt associated to the provided pin so automatically wake up when a 
motion is detected and report a V_TRIPPED message back.
*/


/**********************************
* MySensors node configuration
*/

// General settings
#define SKETCH_NAME "WaterMeterSensor"
#define SKETCH_VERSION "1.0"
#define MY_BAUD_RATE 9600
#define MY_NODE_ID 99

// NRF24 radio settings
#define MY_RADIO_RF24

/***********************************
 * NodeManager configuration
 */

#define NODEMANAGER_SLEEP OFF

// import NodeManager library (a nodeManager object will be then made available)
#include <MySensors_NodeManager.h>

/***********************************
 * Add your sensors
 */

#include <sensors/SensorWaterMeter.h>
SensorWaterMeter waterMeter(3);

/***********************************
* Main Sketch
*/

// before
void before() {

/***********************************
* Configure your sensors
*/

	// set reporting interval to 30 seconds.
	waterMeter.setReportIntervalSeconds(30);
    // set pulse factor to 1000 pulses per m3
    waterMeter.setPulseFactor(1000);

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