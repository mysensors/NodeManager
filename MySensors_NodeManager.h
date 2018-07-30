/*
* NodeManager Library
*/

#ifndef MySensors_NodeManager_h
#define MySensors_NodeManager_h

#include <Arduino.h>

// define NodeManager version
#define VERSION "1.8-dev"

/***********************************
Constants
*/
// define on/off
#define OFF 0
#define ON 1

// define board sleep status
enum status {
	AWAKE,
	SLEEP	
};

//define Timer type
enum timer_mode {
	NOT_CONFIGURED,
	TIME_INTERVAL,
	IMMEDIATELY,
	DO_NOT_REPORT,
#if FEATURE_TIME == ON
	EVERY_MINUTE,
	EVERY_HOUR,
	EVERY_DAY,
	AT_MINUTE,
	AT_HOUR,
	AT_DAY
#endif
} ;

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
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define CHIP_MEGA
#endif
#if defined(ARDUINO_ARCH_STM32F0) || defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F3) || defined(ARDUINO_ARCH_STM32F4) || defined(ARDUINO_ARCH_STM32L4)
#define CHIP_STM32
#endif
#if defined(ESP8266) || defined(MY_GATEWAY_ESP8266)
#define CHIP_ESP8266
#endif
#if defined (MYBOARDNRF5)
#define CHIP_NRF5
#endif
#if !defined(CHIP_ESP8266) && !defined(CHIP_STM32) && !defined(CHIP_NRF5)
#define CHIP_AVR
#endif

// define interrupt pins

#if defined(CHIP_STM32)
#define INTERRUPT_PIN_1 PB8
#define INTERRUPT_PIN_2 2
#else
#define INTERRUPT_PIN_1 3
#define INTERRUPT_PIN_2 2
#endif

// define eeprom addresses
#define EEPROM_SLEEP_SAVED 0
#define EEPROM_SLEEP_1 5
#define EEPROM_SLEEP_2 6
#define EEPROM_SLEEP_3 7
#define EEPROM_USER_START 100


/***********************************
Default configuration settings
*/
// define default sketch name and version
#ifndef SKETCH_NAME
#define SKETCH_NAME "NodeManager"
#endif
#ifndef SKETCH_VERSION
#define SKETCH_VERSION "1.0"
#endif

// SensorConfiguration default child_id
#ifndef CONFIGURATION_CHILD_ID
#define CONFIGURATION_CHILD_ID 200
#endif
// SensorBattery default child_id
#ifndef BATTERY_CHILD_ID
#define BATTERY_CHILD_ID 201
#endif
// SensorSignal default child_id
#ifndef SIGNAL_CHILD_ID
#define SIGNAL_CHILD_ID 202
#endif

// default built-in features if not defined
#ifndef FEATURE_DEBUG
#define FEATURE_DEBUG ON
#endif
#ifndef FEATURE_POWER_MANAGER
#define FEATURE_POWER_MANAGER OFF
#endif
#ifndef FEATURE_INTERRUPTS
#define FEATURE_INTERRUPTS ON
#endif
#ifndef FEATURE_CONDITIONAL_REPORT
#define FEATURE_CONDITIONAL_REPORT OFF
#endif
#ifndef FEATURE_EEPROM
#define FEATURE_EEPROM OFF
#endif
#ifndef FEATURE_SLEEP
#define FEATURE_SLEEP ON
#endif
#ifndef FEATURE_RECEIVE
#define FEATURE_RECEIVE ON
#endif
#ifndef FEATURE_TIME
#define FEATURE_TIME OFF
#endif
#ifndef FEATURE_RTC
#define FEATURE_RTC OFF
#endif
#ifndef FEATURE_SD
#define FEATURE_SD OFF
#endif
#ifndef FEATURE_HOOKING
#define FEATURE_HOOKING OFF
#endif

/***********************************
Libraries
*/

// include supporting libraries for enabled sensors
#ifdef MY_USE_UDP
#include <WiFiUdp.h>
#endif
#ifdef CHIP_ESP8266
#include <ESP8266WiFi.h>
#endif
// load MySensors library
#include <MySensors.h>

// define debug output macro and log prefix
#if FEATURE_DEBUG == ON
#define debug(x,...)		hwDebugPrint(x, ##__VA_ARGS__)
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
#else
#define debug(x,...)
#endif

// include third party libraries






// include third party libraries for enabled features
#if FEATURE_TIME == ON
#include <TimeLib.h>
#endif
#if FEATURE_RTC == ON
#define FEATURE_TIME ON
#include <DS3232RTC.h>
#endif
#if FEATURE_CONDITIONAL_REPORT == ON
#include <float.h>
#endif
#if FEATURE_SD == ON
#include <SD.h>
#endif

/*******************************************************************
Classes
*/

#ifdef USE_CONFIGURATION
/*
ConfigurationRequest
*/

class ConfigurationRequest {
public:
	ConfigurationRequest(int child_id, const char* string);
	// return the child id the message has been requested to
	int getRecipientChildId();
	// return the child id the request is for
	int getChildId();
	// return the parsed function
	int getFunction();
	// return the value as an int
	int getValueInt();
	// return the value as a float
	float getValueFloat();
private:
	int _function = -1;
	int _child_id = -1;
	int _recipient_child_id = -1;
	float _value;
};
#endif

#include "NodeManager/List.h"


#if FEATURE_POWER_MANAGER == ON
#include "NodeManager/PowerManager.cpp"
#endif

#include "NodeManager/NodeManager.cpp"
#include "NodeManager/Sensor.cpp"
#include "NodeManager/Child.cpp"
#include "NodeManager/Timer.cpp"








/***************************************
Sensor: generic sensor class
*/





#ifdef USE_CONFIGURATION
/*
SensorConfiguration: allow remote configuration of the board and any configured sensor
*/

class SensorConfiguration: public Sensor {
public:
	SensorConfiguration(NodeManager& nodeManager);
	// define what to do at each stage of the sketch
	void onSetup();
	void onLoop(Child* child);
	void onReceive(MyMessage* message);
protected:
};
#endif



//XXXX











// include NodeManager's library file
#include "NodeManager/MySensors_NodeManager.cpp"

#ifdef USE_BATTERY
#include "sensors/SensorBattery.h"
#endif
#ifdef USE_SIGNAL
#include "sensors/SensorSignal.h"
#endif
#ifdef USE_ANALOG_INPUT
#include "sensors/SensorLDR.h"
#include "sensors/SensorRain.h"
#include "sensors/SensorSoilMoisture.h"
#endif
#ifdef USE_THERMISTOR
#include "sensors/SensorThermistor.h"
#endif
#ifdef USE_ML8511
#include "sensors/SensorML8511.h"
#endif
#ifdef USE_ACS712
#include "sensors/SensorACS712.h"
#endif
#ifdef USE_DIGITAL_INPUT
#include "sensors/SensorDigitalInput.h"
#endif
#ifdef USE_DIGITAL_OUTPUT
#include "sensors/SensorDigitalOutput.h"
#include "sensors/SensorRelay.h"
#include "sensors/SensorLatchingRelay1Pin.h"
#include "sensors/SensorLatchingRelay2Pins.h"
#endif
#ifdef USE_DHT
#include "sensors/SensorDHT11.h"
#include "sensors/SensorDHT22.h"
#endif
#ifdef USE_SHT21
#include "sensors/SensorSHT21.h"
#endif
#ifdef USE_INTERRUPT
#include "sensors/SensorInterrupt.h"
#include "sensors/SensorDoor.h"
#include "sensors/SensorMotion.h"
#endif
#ifdef USE_DS18B20
#include "sensors/SensorDs18b20.h"
#endif
#ifdef USE_BH1750
#include "sensors/SensorBH1750.h"
#endif
#ifdef USE_MLX90614
#include "sensors/SensorMLX90614.h"
#endif
#ifdef USE_BME280
#include "sensors/SensorBME280.h"
#endif
#ifdef USE_BMP085_180
#include "sensors/SensorBMP085.h"
#include "sensors/SensorBMP180.h"
#endif
#ifdef USE_BMP280
#include "sensors/SensorBMP280.h"
#endif
#ifdef USE_SONOFF
#include "sensors/SensorSonoff.h"
#endif
#ifdef USE_HCSR04
#include "sensors/SensorHCSR04.h"
#endif
#ifdef USE_MCP9808
#include "sensors/SensorMCP9808.h"
#endif
#ifdef USE_MQ
#include "sensors/SensorMQ.h"
#endif
#ifdef USE_MHZ19
#include "sensors/SensorMHZ19.h"
#endif
#ifdef USE_AM2320
#include "sensors/SensorAM2320.h"
#endif
#ifdef USE_TSL2561
#include "sensors/SensorTSL2561.h"
#endif
#ifdef USE_PT100
#include "sensors/SensorPT100.h"
#endif
#ifdef USE_DIMMER
#include "sensors/SensorDimmer.h"
#endif
#ifdef USE_PULSE_METER
#include "sensors/SensorRainGauge.h"
#include "sensors/SensorPowerMeter.h"
#include "sensors/SensorWaterMeter.h"
#endif
#ifdef USE_PMS
#include "sensors/SensorPlantowerPMS.h"
#endif
#ifdef USE_VL53L0X
#include "sensors/SensorVL53L0X.h"
#endif
#ifdef USE_SSD1306
#include "sensors/DisplaySSD1306.h"
#endif
#ifdef USE_SHT31
#include "sensors/SensorSHT31.h"
#endif
#ifdef USE_SI7021
#include "sensors/SensorSI7021.h"
#endif
#ifdef USE_CHIRP
#include "sensors/SensorChirp.h"
#endif
#ifdef USE_HD44780
#include "sensors/DisplayHD44780.h"
#endif
#ifdef USE_TTP
#include "sensors/SensorTTP.h"
#endif
#ifdef USE_SERVO
#include "sensors/SensorServo.h"
#endif
#ifdef USE_APDS9960
#include "sensors/SensorAPDS9960.h"
#endif
#ifdef USE_NEOPIXEL
#include "sensors/SensorNeopixel.h"
#endif
#ifdef USE_SDS011
#include "sensors/SensorSDS011.h"
#endif
#ifdef USE_FPM10A
#include "sensors/SensorFPM10A.h"
#endif




#endif
