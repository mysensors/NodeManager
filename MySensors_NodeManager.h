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
#ifndef MySensors_NodeManager_h
#define MySensors_NodeManager_h

// include Arduino header file
#include <Arduino.h>

// include NodeManager's constants
#include "nodemanager/Constants.h"

/***********************************
Include required third-party libraries
*/

// include UDP library
#ifdef MY_USE_UDP
#include <WiFiUdp.h>
#endif

// include ESP8266 library
#ifdef CHIP_ESP8266
#include <ESP8266WiFi.h>
#endif

// include MySensors library
#include <MySensors.h>

// include additional libraries based on the requested configuration
#if NODEMANAGER_TIME == ON
#include <TimeLib.h>
#endif
#if NODEMANAGER_RTC == ON
#undef NODEMANAGER_TIME
#define NODEMANAGER_TIME ON
#include <DS3232RTC.h>
#endif
#if NODEMANAGER_CONDITIONAL_REPORT == ON
#include <float.h>
#endif
#if NODEMANAGER_SD == ON
#include <SD.h>
#endif
#if NODEMANAGER_SERIAL_INPUT == ON
#include "core/MyProtocolMySensors.cpp"
#endif

/***********************************
Include NodeManager core code
*/

// List class
#include "nodemanager/List.h"

// PowerManager class
#if NODEMANAGER_POWER_MANAGER == ON
#include "nodemanager/PowerManager.cpp"
#endif

// ConfigurationRequest class for OTA configuration
#if NODEMANAGER_OTA_CONFIGURATION == ON
#include "nodemanager/ConfigurationRequest.cpp"
#endif

// NodeManager class
#include "nodemanager/Node.cpp"
// create the global variable nodeManager that can be called from within the sketch
extern NodeManager nodeManager;
NodeManager nodeManager;
// Sensor class
#include "nodemanager/Sensor.cpp"
// Child class
#include "nodemanager/Child.cpp"
// InternalTimer class
#include "nodemanager/InternalTimer.cpp"

#if NODEMANAGER_OTA_CONFIGURATION == ON
// include SensorConfiguration if needed
#include <sensors/SensorConfiguration.h>
SensorConfiguration configuration;
#endif

#endif
