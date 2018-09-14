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
#ifndef NodeManager_h
#define NodeManager_h

/******************************************
NodeManager: provide the most common functionalities a user would need when leveraging the MySensors library
*/

#include "Sensor.h"

class NodeManager {
public:
	// instantiate a NodeManager object. An optional fixed number of sensors can be passed as an argument
	NodeManager(int sensorcount = 0);
	// [10] send the same message multiple times (default: 1)
	void setRetries(int value);
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
	// return vcc in V
	float getVcc();
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
	// send a message by providing the source child, type of the message and value
	void sendMessage(int child_id, int type, int value);
	void sendMessage(int child_id, int type, float value, int precision);
	void sendMessage(int child_id, int type, double value, int precision);
	void sendMessage(int child_id, int type, const char* value);
	// register a sensor
	void registerSensor(Sensor* sensor);
	// register a timer
	void registerTimer(Timer* timer);
	// return the next-available child id
	int getAvailableChildId(int child_id = -255);
	// list containing all the registered sensors
	List<Sensor*> sensors;
	// return the Child object of the given child_id
	Child* getChild(int child_id);
	// return the sensor object of the given child_id
	Sensor* getSensorWithChild(int child_id);
#if NODEMANAGER_SLEEP == ON
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
#if NODEMANAGER_INTERRUPTS == ON
	// [19] if enabled, when waking up from the interrupt, the board stops sleeping. Disable it when attaching e.g. a motion sensor (default: true)
	void setSleepInterruptPin(int value);
	// configure the interrupt pin and mode. Mode can be CHANGE, RISING, FALLING (default: MODE_NOT_DEFINED)
	void setInterrupt(int pin, int mode, int initial = -1);
	// [28] ignore two consecutive interrupts if happening within this timeframe in milliseconds (default: 100)
	void setInterruptDebounce(long value);
	// return the pin from which the last interrupt came
	int getLastInterruptPin();
	// return the value of the pin from which the last interrupt came
	int getLastInterruptValue();
#endif
#if NODEMANAGER_POWER_MANAGER == ON
	// configure a PowerManager common to all the sensors
	void setPowerManager(PowerManager& powerManager);
	// to save battery the sensor can be optionally connected to two pins which will act as vcc and ground and activated on demand
	void setPowerPins(int ground_pin, int vcc_pin, int wait_time = 50);
	// [24] manually turn the power on
	void powerOn();
	// [25] manually turn the power off
	void powerOff();
#endif
#if NODEMANAGER_EEPROM == ON
	// [7] clear the EEPROM
	void clearEeprom();
	// return the value stored at the requested index from the EEPROM
	int loadFromMemory(int index);
	// [27] save the given index of the EEPROM the provided value
	void saveToMemory(int index, int value);
	// [40] if set save the sleep settings in memory, also when changed remotely (default: false)
	void setSaveSleepSettings(bool value);
#endif
#if NODEMANAGER_TIME == ON
	// [41] synchronize the local time with the controller
	void syncTime();
	// [42] returns the current system time
	long getTime();
	// [43] set the hour offset for when syncronizing the time (default: 0)
	void setTimezone(int value);
	// request the current time to the controller during setup(). Time with a RTC if configured is always synchronized (default: true)
	void setSyncTimeOnSetup(bool value);
	// request the current time to the controller just after a sleep cycle. Time with a RTC if configured is always synchronized (default: true)
	void setSyncTimeAfterSleep(bool value);
	// request the current time to the controller after the configured number of minutes (default: 0)
	void setSyncTimeAfterInterval(int value);
	// receiveTime() callback
	void receiveTime(unsigned long ts);
#endif
#if NODEMANAGER_SD == ON
	// SD card variables
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
#if NODEMANAGER_RECEIVE == ON
	void receive(const MyMessage & msg);
#endif
private:
	bool _get_controller_config = true;
	int _is_metric = 1;
	status _status = AWAKE;
	bool _ack = false;
	int _retries = 1;
	MyMessage _message;
	void _sendMessage(int child_id, int type);
	long _sleep_time = 0;
	bool _sleep_or_wait = true;
	int _sleep_interrupt_pin = -1;
	int _sleep_between_send = 0;
	void _sleepBetweenSend();
	int _report_interval_seconds = 10*60;
	int _reboot_pin = -1;
	void _present(int child_id, int type);
	List<Timer*> _timers;
#if NODEMANAGER_INTERRUPTS == ON
	int _interrupt_1_mode = MODE_NOT_DEFINED;
	int _interrupt_2_mode = MODE_NOT_DEFINED;
	int _interrupt_1_initial = -1;
	int _interrupt_2_initial = -1;
	static int _last_interrupt_pin;
	static int _last_interrupt_value;
	static long unsigned _interrupt_debounce;
	static long unsigned _last_interrupt_millis;
	void _setupInterrupts();
	static void _onInterrupt_1();
	static void _onInterrupt_2();
	static void _saveInterrupt(int pin);
#endif
#if NODEMANAGER_SLEEP == ON
	void _sleep();
	bool _smart_sleep = true;
#endif
#if NODEMANAGER_POWER_MANAGER == ON
	PowerManager* _powerManager = nullptr;
#endif
#if NODEMANAGER_EEPROM == ON
	bool _save_sleep_settings = false;
	void _loadSleepSettings();
	void _saveSleepSettings();
#endif
#if NODEMANAGER_TIME == ON
	bool _time_is_valid = false;
	long _remainder_sleep_time = -1;
	long _time_last_sync;
	int _timezone = 0;
	bool _sync_time_on_setup = true;
	bool _sync_time_after_sleep = true;
	unsigned int _sync_time_after_interval = 0;
#endif
};

#endif