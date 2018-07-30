/*
* NodeManager Library
*/

#ifndef NodeManager_h
#define NodeManager_h

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
class NodeManager;
class Sensor;

/*
* List
*/
template<typename T> class List {
public:
	typedef T* iterator;
	List() {
		_internalArray = NULL;
		_endPosition = 0;
		_allocBlocks = 0;
	}
	~List() {
		delete[] _internalArray;
		_internalArray = NULL;
		_endPosition = 0;
		_allocBlocks = 0;
	}
	void push(T item) {
		if (_endPosition == _allocBlocks) _AllocOneBlock(false);
		_internalArray[_endPosition] = item;
		++_endPosition;
	}
	void pop() {
		if (_endPosition == 0) return;
		--_endPosition;
		_DeAllocOneBlock(false);
	}
	T get(int position) {
		position = position -1;
		if (position > _endPosition) position = _endPosition;
		return _internalArray[position];
	}
	void clear() {
		T* newArray = NULL;
		if (_allocBlocks > 0) newArray = new T[_allocBlocks];
		delete[] _internalArray;
		_internalArray = newArray;
		_endPosition = 0;
	}
	inline iterator begin() { return _internalArray; }
	inline iterator end() { return _internalArray + _endPosition; }
	inline bool empty() { return (_endPosition == 0); }
	inline int size() { return _endPosition; }
	void allocateBlocks(int alloc) {
		_allocBlocks = alloc;
		T* newArray = new T[_allocBlocks];
		for (int i = 0; i < _endPosition; ++i) newArray[i] = _internalArray[i];
		delete[] _internalArray;
		_internalArray = newArray;
	}

private:
	T* _internalArray;
	int _endPosition;
	int _allocBlocks;
	void _AllocOneBlock(bool shiftItems) {
		++_allocBlocks;
		T* newArray = new T[_allocBlocks];
		for (int i = 0; i < _endPosition; ++i) newArray[shiftItems ? (i + 1) : i] = _internalArray[i];
		delete[] _internalArray;
		_internalArray = newArray;
	}
	void _DeAllocOneBlock(bool shiftItems) {
		--_allocBlocks;
		if (_allocBlocks == 0) {
			delete[] _internalArray;
			_internalArray = NULL;
			return;
		}
		T* newArray = new T[_allocBlocks];
		for (int i = 0; i < _endPosition; ++i) newArray[i] = _internalArray[shiftItems ? (i + 1) : i];
		delete[] _internalArray;
		_internalArray = newArray;
	}
};

/*
PowerManager
*/

#if FEATURE_POWER_MANAGER == ON
class PowerManager {
public:
	PowerManager(int ground_pin, int vcc_pin, int wait_time = 50);
	// to save battery the sensor can be optionally connected to two pins which will act as vcc and ground and activated on demand
	void setPowerPins(int ground_pin, int vcc_pin, int wait_time = 50);
	// if enabled the pins will be automatically powered on while awake and off during sleeping
	// turns the power pins on
	void powerOn();
	// turns the power pins on
	void powerOff();
private:
	int _vcc_pin = -1;
	int _ground_pin = -1;
	long _wait = 0;
};
#endif

/*
Timer
*/

class Timer {
public:
	Timer(NodeManager* node_manager);
	void setMode(timer_mode mode);
	timer_mode getMode();
	void setValue(int value);
	int getValue();
	// start the timer
	void start();
	// stop the timer
	void stop();
	// return true if the time is over
	bool isOver();
	// return elapsed time in seconds
	long getElapsed();
private:
	NodeManager* _node;
	timer_mode _mode = NOT_CONFIGURED;
	int _value = 0;
	bool _is_running = false;
	long _last = 0;
#if FEATURE_TIME == ON
	bool _already_reported = false;
#endif
};

/***************************************
Child: child class
*/

class Child {
public:
	Child();
	Child(Sensor* sensor, int child_id, int presentation, int type, const char* description = "");
	// set child id used to communicate with the gateway/controller
	void setChildId(int value);
	int getChildId();
	// set sensor presentation (default: S_CUSTOM)
	void setPresentation(int value);
	int getPresentation();
	// set sensor type (default: V_CUSTOM)
	void setType(int value);
	int getType();
	// set how many decimal digits to use (default: 2 for ChildFloat, 4 for ChildDouble)
	void setFloatPrecision(int value);
	// set sensor description
	void setDescription(const char* value);
	const char* getDescription();
#if FEATURE_CONDITIONAL_REPORT == ON
	// force to send an update after the configured number of minutes
	void setForceUpdateTimerValue(int value);
	// never report values below this threshold (default: FLT_MIN)
	void setMinThreshold(float value);
	// never report values above this threshold (default: FLT_MAX)
	void setMaxThreshold(float value);
	// do not report values if too close to the previous one (default: 0)
	void setValueDelta(float value);
#endif
	// send the current value to the gateway
	virtual void sendValue(bool force);
	// print the current value on a LCD display
	virtual void print(Print& device);
	// reset all the counters
	virtual void reset();
protected:
	int _samples = 0;
	Sensor* _sensor;
	int _child_id;
	int _presentation = S_CUSTOM;
	int _type = V_CUSTOM;
	int _float_precision;
	const char* _description = "";
#if FEATURE_CONDITIONAL_REPORT == ON
	Timer* _force_update_timer;
	float _min_threshold = FLT_MIN;
	float _max_threshold = FLT_MAX;
	float _value_delta = 0;
#endif
};

class ChildInt: public Child {
public:
	ChildInt(Sensor* sensor, int child_id, int presentation, int type, const char* description = "");
	void setValue(int value);
	int getValue();
	void sendValue(bool force);
	void print(Print& device);
	void reset();
private:
	int _value;
#if FEATURE_CONDITIONAL_REPORT == ON
	int _last_value = -256;
#endif
	int _total = 0;
};

class ChildFloat: public Child {
public:
	ChildFloat(Sensor* sensor, int child_id, int presentation, int type, const char* description = "");
	void setValue(float value);
	float getValue();
	void sendValue(bool force);
	void print(Print& device);
	void reset();
private:
	float _value;
#if FEATURE_CONDITIONAL_REPORT == ON
	float _last_value = -256;
#endif
	float _total = 0;
};

class ChildDouble: public Child {
public:
	ChildDouble(Sensor* sensor, int child_id, int presentation, int type, const char* description = "");
	void setValue(double value);
	double getValue();
	void sendValue(bool force);
	void print(Print& device);
	void reset();
private:
	double _value;
#if FEATURE_CONDITIONAL_REPORT == ON
	double _last_value = -256;
#endif
	double _total = 0;
};

class ChildString: public Child {
public:
	ChildString(Sensor* sensor, int child_id, int presentation, int type, const char* description = "");
	void setValue(const char* value);
	const char* getValue();
	void sendValue(bool force);
	void print(Print& device);
	void reset();
private:
	const char* _value = "";
#if FEATURE_CONDITIONAL_REPORT == ON
	const char* _last_value = "";
#endif
};

/***************************************
Sensor: generic sensor class
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
#ifdef USE_CONFIGURATION
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










/***************************************
NodeManager: manages all the aspects of the node
*/
class NodeManager {
public:
	NodeManager(int sensorcount = 0);
	// [10] send the same message multiple times (default: 1)
	void setRetries(int value);
	int getRetries();
#if FEATURE_SLEEP == ON
	// [3] set the duration (in seconds) of a sleep cycle
	void setSleepSeconds(int value);
	long getSleepSeconds();
	// [4] set the duration (in minutes) of a sleep cycle
	void setSleepMinutes(int value);
	// [5] set the duration (in hours) of a sleep cycle
	void setSleepHours(int value);
	// [29] set the duration (in days) of a sleep cycle
	void setSleepDays(int value);
	// [20] optionally sleep interval in milliseconds before sending each message to the radio network (default: 0)
	void setSleepBetweenSend(int value);
	// [9] wake up the board
	void wakeup();
	// use smart sleep for sleeping boards (default: true)
	void setSmartSleep(bool value);
#endif
#if FEATURE_INTERRUPTS == ON
	// [19] if enabled, when waking up from the interrupt, the board stops sleeping. Disable it when attaching e.g. a motion sensor (default: true)
	void setSleepInterruptPin(int value);
	// configure the interrupt pin and mode. Mode can be CHANGE, RISING, FALLING (default: MODE_NOT_DEFINED)
	void setInterrupt(int pin, int mode, int initial = -1);
	// [28] ignore two consecutive interrupts if happening within this timeframe in milliseconds (default: 100)
	void setInterruptDebounce(long value);
#endif
	// register a sensor
	void registerSensor(Sensor* sensor);
	// to save battery the sensor can be optionally connected to two pins which will act as vcc and ground and activated on demand
#if FEATURE_POWER_MANAGER == ON
	void setPowerPins(int ground_pin, int vcc_pin, int wait_time = 50);
	// [24] manually turn the power on
	void powerOn();
	// [25] manually turn the power off
	void powerOff();
#endif
	// [21] set this to true if you want destination node to send ack back to this node (default: false)
	void setAck(bool value);
	bool getAck();
	// Request the controller's configuration on startup (default: true)
	void setGetControllerConfig(bool value);
	// [22] Manually set isMetric setting
	void setIsMetric(bool value);
	bool getIsMetric();
	// Convert a temperature from celsius to fahrenheit depending on how isMetric is set
	float celsiusToFahrenheit(float temperature);
	// return true if sleep or wait is configured and hence this is a sleeping node
	bool isSleepingNode();
	// [1] Send a hello message back to the controller
	void hello();
	// [6] reboot the board
	void reboot();
#if FEATURE_EEPROM == ON
	// [7] clear the EEPROM
	void clearEeprom();
	// return the value stored at the requested index from the EEPROM
	int loadFromMemory(int index);
	// [27] save the given index of the EEPROM the provided value
	void saveToMemory(int index, int value);
	// [40] if set save the sleep settings in memory, also when changed remotely (default: false)
	void setSaveSleepSettings(bool value);
#endif
	// return vcc in V
	float getVcc();
#if FEATURE_INTERRUPTS == ON
	// setup the configured interrupt pins
	void setupInterrupts();
	// return the pin from which the last interrupt came
	int getLastInterruptPin();
	// return the value of the pin from which the last interrupt came
	int getLastInterruptValue();
#endif
    // [36] set the default interval in minutes all the sensors will report their measures. If the same function is called on a specific sensor, this will not change the previously set value. On sleeping sensors, the elapsed time can be evaluated only upon wake up (default: 10 minutes)
    void setReportIntervalSeconds(int value);
	int getReportIntervalSeconds();
    // [37] set the default interval in minutes all the sensors will report their measures. If the same function is called on a specific sensor, this will not change the previously set value. On sleeping sensors, the elapsed time can be evaluated only upon wake up (default: 10 minutes)
    void setReportIntervalMinutes(int value);
    // [38] set the default interval in minutes all the sensors will report their measures. If the same function is called on a specific sensor, this will not change the previously set value. On sleeping sensors, the elapsed time can be evaluated only upon wake up (default: 10 minutes)
    void setReportIntervalHours(int value);
    // [39] set the default interval in minutes all the sensors will report their measures. If the same function is called on a specific sensor, this will not change the previously set value. On sleeping sensors, the elapsed time can be evaluated only upon wake up (default: 10 minutes)
    void setReportIntervalDays(int value);
	// [30] if set and when the board is battery powered, sleep() is always called instead of wait() (default: true)
	void setSleepOrWait(bool value);
	// sleep if the node is a battery powered or wait if it is not for the given number of milliseconds 
	void sleepOrWait(long value);
	// [31] set which pin is connected to RST of the board to reboot the board when requested. If not set the software reboot is used instead (default: -1)
	void setRebootPin(int value);
	// [32] turn the ADC off so to save 0.2 mA
	void setADCOff();
#if FEATURE_TIME == ON
	// [41] synchronize the local time with the controller
	void syncTime();
	// [42] returns the current system time
	long getTime();
	void receiveTime(unsigned long ts);
#endif
#if FEATURE_INTERRUPTS == ON
	// handle interrupts
	static void _onInterrupt_1();
	static void _onInterrupt_2();
	static void _saveInterrupt(int pin);
#endif
	// send a message by providing the source child, type of the message and value
	void sendMessage(int child_id, int type, int value);
	void sendMessage(int child_id, int type, float value, int precision);
	void sendMessage(int child_id, int type, double value, int precision);
	void sendMessage(int child_id, int type, const char* value);
#if FEATURE_POWER_MANAGER == ON
	void setPowerManager(PowerManager& powerManager);
#endif
	int getAvailableChildId(int child_id = -255);
	List<Sensor*> sensors;
	Child* getChild(int child_id);
	Sensor* getSensorWithChild(int child_id);
#if FEATURE_SD == ON
	Sd2Card sd_card;
	SdVolume sd_volume;
	SdFile sd_root;
	SdFile sd_file;
#endif
	// hook into the main sketch functions
	void before();
	void presentation();
	void setup();
	void loop();
#if FEATURE_RECEIVE == ON
	void receive(const MyMessage & msg);
#endif
private:
#if FEATURE_POWER_MANAGER == ON
	PowerManager* _powerManager = nullptr;
#endif
	MyMessage _message;
	void _sendMessage(int child_id, int type);
	status _status = AWAKE;
	long _sleep_time = 0;
	int _sleep_interrupt_pin = -1;
	int _retries = 1;
	int _sleep_between_send = 0;
#if FEATURE_INTERRUPTS == ON
	int _interrupt_1_mode = MODE_NOT_DEFINED;
	int _interrupt_2_mode = MODE_NOT_DEFINED;
	int _interrupt_1_initial = -1;
	int _interrupt_2_initial = -1;
	static int _last_interrupt_pin;
	static int _last_interrupt_value;
	static long unsigned _interrupt_debounce;
	static long unsigned _last_interrupt_millis;
#endif
	bool _ack = false;
#if FEATURE_SLEEP == ON
	void _sleep();
	bool _smart_sleep = true;
#endif
	void _present(int child_id, int type);
	bool _get_controller_config = true;
	int _is_metric = 1;
	int _report_interval_seconds = 10*60;
	bool _sleep_or_wait = true;
	int _reboot_pin = -1;
#if FEATURE_EEPROM == ON
	bool _save_sleep_settings = false;
	void _loadSleepSettings();
	void _saveSleepSettings();
#endif
	void _sleepBetweenSend();
#if FEATURE_TIME == ON
	bool _time_is_valid = false;
	long _remainder_sleep_time = -1;
	long _time_last_sync;
#endif
};

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
