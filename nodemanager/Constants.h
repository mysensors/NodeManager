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
#ifndef Constants_h
#define Constants_h

/******************************************
Constants: define all the constants used by NodeManager
*/

// define NodeManager version
#define VERSION "1.9-dev"

// define on/off
#define OFF 0
#define ON 1

// define board sleep status
enum status {
	AWAKE,
	SLEEP
};

//define Timer mode
enum timer_mode {
	NOT_CONFIGURED,
	TIME_INTERVAL,
	IMMEDIATELY,
	DO_NOT_REPORT,
#if NODEMANAGER_TIME == ON || NODEMANAGER_RTC == ON
	EVERY_MINUTE,
	EVERY_HOUR,
	EVERY_DAY,
	AT_MINUTE,
	AT_HOUR,
	AT_DAY
#endif
};

// define Child processing
enum child_processing {
	NONE,
	AVG,
	SUM
};

// define value types
enum value_format {
	INT,
	FLOAT,
	DOUBLE,
	STRING
};

#if NODEMANAGER_CONDITIONAL_REPORT == ON
// define update last value modes
enum last_value_mode {
	UPDATE_ALWAYS,
	UPDATE_ON_SEND
};
#endif

/***********************************
Chip type
*/
// 168 and 328 Arduinos
#if defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
#define CHIP_TINYX4
#endif
#if defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#define CHIP_TINYX5
#endif
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
#define CHIP_MEGA
#endif
#if defined(ARDUINO_ARCH_STM32F0) || defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F3) || defined(ARDUINO_ARCH_STM32F4) || defined(ARDUINO_ARCH_STM32L4)
#define CHIP_STM32
#endif
#if defined(ESP8266) || defined(MY_GATEWAY_ESP8266)
#define CHIP_ESP8266
#endif
#if defined(ESP32) || defined(MY_GATEWAY_ESP32)
#define CHIP_ESP32
#endif
#if defined (MYBOARDNRF5) || defined(NRF51) || defined(NRF52)
#define CHIP_NRF5
#endif
#if defined (SAMD_SERIES)
#define CHIP_SAMD
#endif
#if !defined(CHIP_ESP8266) && !defined(CHIP_ESP32) && !defined(CHIP_STM32) && !defined(CHIP_NRF5) && !defined(CHIP_SAMD)
#define CHIP_AVR
#endif

// define interrupt pins
#ifndef INTERRUPT_PIN_1
#define INTERRUPT_PIN_1 3
#endif
#ifndef INTERRUPT_PIN_2
#define INTERRUPT_PIN_2 2
#endif

// define eeprom addresses
#define EEPROM_SLEEP_SAVED 0
#define EEPROM_SLEEP_1 5
#define EEPROM_SLEEP_2 6
#define EEPROM_SLEEP_3 7
#define EEPROM_USER_START 20

/***********************************
Default configuration settings
*/

#ifndef NODEMANAGER_DEBUG
#define NODEMANAGER_DEBUG ON
#endif
#ifndef NODEMANAGER_DEBUG_VERBOSE
#define NODEMANAGER_DEBUG_VERBOSE OFF
#endif
#ifndef NODEMANAGER_POWER_MANAGER
#define NODEMANAGER_POWER_MANAGER OFF
#endif
#ifndef NODEMANAGER_INTERRUPTS
#define NODEMANAGER_INTERRUPTS ON
#endif
#ifndef NODEMANAGER_CONDITIONAL_REPORT
#define NODEMANAGER_CONDITIONAL_REPORT OFF
#endif
#ifndef NODEMANAGER_EEPROM
#define NODEMANAGER_EEPROM OFF
#endif
#ifndef NODEMANAGER_SLEEP
#define NODEMANAGER_SLEEP ON
#endif
#ifndef NODEMANAGER_RECEIVE
#define NODEMANAGER_RECEIVE ON
#endif
#ifndef NODEMANAGER_TIME
#define NODEMANAGER_TIME OFF
#endif
#ifndef NODEMANAGER_RTC
#define NODEMANAGER_RTC OFF
#endif
#ifndef NODEMANAGER_SD
#define NODEMANAGER_SD OFF
#endif
#ifndef NODEMANAGER_HOOKING
#define NODEMANAGER_HOOKING OFF
#endif
#ifndef NODEMANAGER_OTA_CONFIGURATION
#define NODEMANAGER_OTA_CONFIGURATION OFF
#endif
#ifndef NODEMANAGER_SERIAL_INPUT
#define NODEMANAGER_SERIAL_INPUT OFF
#endif

// define debug output macro and log prefix
#define LOG_PREFIX 			"NM:"
#define LOG_INIT			LOG_PREFIX "INIT:"
#define LOG_BEFORE			LOG_PREFIX "BFR:"
#define LOG_PRESENTATION	LOG_PREFIX "PRES:"
#define LOG_SETUP			LOG_PREFIX "STP:"
#define LOG_LOOP			LOG_PREFIX "LOOP:"
#define LOG_POWER			LOG_PREFIX "PWR:"
#define LOG_SLEEP			LOG_PREFIX "SLP:"
#define LOG_MSG				LOG_PREFIX "MSG:"
#define LOG_TIME			LOG_PREFIX "TIME:"
#define LOG_EEPROM			LOG_PREFIX "EEPR:"
#define LOG_OTA				LOG_PREFIX "OTA:"
#define LOG_SENSOR			LOG_PREFIX "SENS:"

#if NODEMANAGER_DEBUG == ON
#define debug(x,...)		hwDebugPrint(x, ##__VA_ARGS__)
#else
#define debug(x,...)
#endif

#if NODEMANAGER_DEBUG == ON && NODEMANAGER_DEBUG_VERBOSE == ON
#define debug_verbose(x,...)		hwDebugPrint(x, ##__VA_ARGS__)
#else
#define debug_verbose(x,...)
#endif

#endif
