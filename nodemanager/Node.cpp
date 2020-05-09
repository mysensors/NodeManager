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
NodeManager: provide the most common functionalities a user would need when leveraging the MySensors library
*/

#include "Node.h"

// initialize node manager
NodeManager::NodeManager(uint8_t sensor_count) {
	// setup the message container
	_message = MyMessage();
	// allocate block for all the sensors if sensor_count is provided
	if (sensor_count > 0) sensors.allocateBlocks(sensor_count);
}

#if NODEMANAGER_INTERRUPTS == ON
int8_t NodeManager::_last_interrupt_pin = -1;
int8_t NodeManager::_last_interrupt_value = LOW;
long unsigned NodeManager::_last_interrupt_millis = millis();
long unsigned NodeManager::_interrupt_debounce = 0;
#endif

// setter/getter
void NodeManager::setRetries(uint8_t value) {
	_retries = value;
}
#if NODEMANAGER_SLEEP == ON
void NodeManager::setSleepSeconds(unsigned long value) {
	// set the status to AWAKE if the time provided is 0, SLEEP otherwise
	if (value == 0) _status = AWAKE;
	else _status = SLEEP;
	// store the time
	_sleep_time = value;
#if NODEMANAGER_EEPROM == ON
	// save sleep settings to eeprom
	if (_save_sleep_settings) _saveSleepSettings();
#endif
}
void NodeManager::setSleepMinutes(unsigned long value) {
	setSleepSeconds(value*60);
}
void NodeManager::setSleepHours(unsigned int value) {
	setSleepMinutes(value*60);
}
void NodeManager::setSleepDays(uint8_t value) {
	setSleepHours(value*24);
}
unsigned long NodeManager::getSleepSeconds() {
	return _sleep_time;
}
void NodeManager::setSleepBetweenSend(unsigned int value) {
	_sleep_between_send = value;
}
#endif
#if NODEMANAGER_INTERRUPTS == ON
void NodeManager::setSleepInterruptPin(int8_t value) {
	_sleep_interrupt_pin = value;
}
void NodeManager::setInterrupt(int8_t pin, uint8_t mode, int8_t initial) {
	if (pin == INTERRUPT_PIN_1) {
		_interrupt_1_mode = mode;
		_interrupt_1_initial = initial;
	}
	if (pin == INTERRUPT_PIN_2) { 
		_interrupt_2_mode = mode;
		_interrupt_2_initial = initial;
	}
}
void NodeManager::setInterruptDebounce(unsigned long value) {
	_interrupt_debounce = value;
}
#endif
#if NODEMANAGER_POWER_MANAGER == ON
void NodeManager::setPowerPins(int8_t ground_pin, int8_t vcc_pin, unsigned long wait_time) {
	if (_powerManager == nullptr) return;
	_powerManager->setPowerPins(ground_pin, vcc_pin, wait_time);
}
void NodeManager::powerOn() {
	if (_powerManager == nullptr) return;
	_powerManager->powerOn();
}
void NodeManager::powerOff() {
	if (_powerManager == nullptr) return;
	_powerManager->powerOff();
}
#endif
void NodeManager::setAck(bool value) {
	_ack = value;
}
bool NodeManager::getAck() {
	return _ack;
}
void NodeManager::setGetControllerConfig(bool value) {
	_get_controller_config = value;
}
void NodeManager::setIsMetric(bool value) {
	_is_metric = value;
}
bool NodeManager::getIsMetric() {
	return _is_metric;
}
#if NODEMANAGER_EEPROM == ON
void NodeManager::setSaveSleepSettings(bool value) {
	_save_sleep_settings = value;
}
#endif

// Convert a temperature from celsius to fahrenheit depending on how isMetric is set
float NodeManager::celsiusToFahrenheit(float temperature) {
	if (_is_metric) return temperature;
	// convert the temperature from C to F
	return temperature * 1.8 + 32;
}

// return true if sleep or wait is configured and hence this is a sleeping node
bool NodeManager::isSleepingNode() {
	if (_status == SLEEP) return true;
	return false;
}

// register a sensor against NodeManager
void NodeManager::registerSensor(Sensor* sensor) {
	sensors.push(sensor);
}

// register a timer
void NodeManager::registerTimer(InternalTimer* timer) {
	_timers.push(timer);
}


// setup NodeManager
void NodeManager::before() {
	// print out the version
	debug(PSTR(LOG_INIT "VER=" VERSION "\n"));
	// print out sketch name
	debug(PSTR(LOG_INIT "INO=" SKETCH_NAME " v" SKETCH_VERSION "\n"));
	// print out MySensors' library capabilities
	debug(PSTR(LOG_INIT "LIB VER=" MYSENSORS_LIBRARY_VERSION " CP=" MY_CAPABILITIES " \n"));
	// setup the reboot pin if needed
	if (_reboot_pin > 0) {
		debug(PSTR(LOG_INIT "RBT p=%d\n"),_reboot_pin);
		pinMode(_reboot_pin, OUTPUT);
		digitalWrite(_reboot_pin, HIGH);
	}
#if NODEMANAGER_EEPROM == ON
	// restore the sleep settings saved in the eeprom
	if (_save_sleep_settings) _loadSleepSettings();
#endif
	debug(PSTR(LOG_BEFORE "INIT\n"));
}

// present NodeManager and its sensors
void NodeManager::presentation() {
	debug(PSTR(LOG_BEFORE "OK\n"));
	// Send the sketch version information to the gateway and Controller
	sleepBetweenSend();
#ifdef SKETCH_NAME
	sendSketchInfo(SKETCH_NAME,SKETCH_VERSION);
#endif
	sleepBetweenSend();
	// present each sensor
	for (List<Sensor*>::iterator itr = sensors.begin(); itr != sensors.end(); ++itr) {
		Sensor* sensor = *itr;
		// call each sensor's presentation()
		sensor->presentation();
		sleepBetweenSend();
	}
	// wait a bit before leaving this function
	sleepBetweenSend();
	sleepBetweenSend();
	sleepBetweenSend();
}


// setup NodeManager
void NodeManager::setup() {
#if NODEMANAGER_POWER_MANAGER == ON
	// turn the sensor on
	powerOn();
#endif
	// retrieve and store isMetric from the controller
	if (_get_controller_config) _is_metric = getControllerConfig().isMetric;
	debug(PSTR(LOG_SETUP "ID=%d M=%d\n"),getNodeId(),_is_metric);
#if NODEMANAGER_RTC == ON
	// sync the time with the RTC
	setSyncProvider(RTC.get);
#endif	
#if NODEMANAGER_TIME == ON
	// sync the time with the controller
	if (_sync_time_on_setup) syncTime();
#endif
#if NODEMANAGER_SD == ON
	// initialize connection to the SD card
	if (sd_card.init(SPI_HALF_SPEED)) {
		debug(PSTR(LOG_SETUP "SD T=%d\n"),sd_card.type());
		// initialize the volume
		sd_volume.init(sd_card);
		// open up the volume
		sd_root.openRoot(sd_volume);
	}
#endif
	// run setup for all the registered sensors
	for (List<Sensor*>::iterator itr = sensors.begin(); itr != sensors.end(); ++itr) {
		Sensor* sensor = *itr;
		// call each sensor's setup()
		sensor->setup();
	}
#if NODEMANAGER_INTERRUPTS == ON
	// setup the interrupt pins
	_setupInterrupts();
#endif
#if NODEMANAGER_POWER_MANAGER == ON
	// turn the sensor off
	powerOff();
#endif
	// print out information regarding CPU voltage, frequency and free memory
	debug(PSTR(LOG_SETUP "HW V=%d F=%d M=%d\n"),hwCPUVoltage(),hwCPUFrequency()/10,hwFreeMem());
}

// run the main function for all the register sensors
void NodeManager::loop() {
#if NODEMANAGER_TIME == ON
	// if the time was last updated more than the given number of minutes ago, update it
	if (_time_is_valid && _sync_time_after_interval > 0 && (now() - _time_last_sync) > (unsigned long)_sync_time_after_interval*60) syncTime();
#endif
#if NODEMANAGER_POWER_MANAGER == ON
	// turn on the pin powering all the sensors
	powerOn();
#endif
	// update all the registered timers
	for (List<InternalTimer*>::iterator itr = _timers.begin(); itr != _timers.end(); ++itr) {
		InternalTimer* timer = *itr;
		timer->update();
	}
	// run loop for all the registered sensors
	for (List<Sensor*>::iterator itr = sensors.begin(); itr != sensors.end(); ++itr) {
		Sensor* sensor = *itr;
		// clear the MyMessage so will be ready to be used for the sensor
		_message.clear();
#if NODEMANAGER_INTERRUPTS == ON
		// if there was an interrupt for this sensor, call ONLY this sensor's functions
		if (_last_interrupt_pin != -1 && sensor->getInterruptPin() == _last_interrupt_pin) {
			// call the sensor interrupt() and then loop(), provided the interrupt has been "accepted" by interrupt()
			if (sensor->interrupt()) sensor->loop(nullptr);
		}
		else if (_last_interrupt_pin == -1) {
#endif
			// if just at the end of a cycle, call all the sensor's loop() 
			sensor->loop(nullptr);
#if NODEMANAGER_INTERRUPTS == ON
		}
#endif
	}
	// reset the last interrupt pin
	_last_interrupt_pin = -1;
#if NODEMANAGER_POWER_MANAGER == ON
	// turn off the pin powering all the sensors
	powerOff();
#endif
#if NODEMANAGER_SERIAL_INPUT == ON
	// read a string from the serial input. Timeout is 1000 millis and can be customized with Serial.setTimeout()
	debug_verbose(PSTR(LOG_LOOP "INPUT...\n"));
	String input = MY_SERIALDEVICE.readString();
	if (input.length() != 0) {
		debug_verbose(PSTR(LOG_LOOP "INPUT v=%s\n"),const_cast<char*>(input.c_str()));
		// parse the message
		_message.clear();
		bool ok = protocolParse(_message,const_cast<char*>(input.c_str()));
		if (ok) receive(_message);
	}
#endif
#if NODEMANAGER_SLEEP == ON
	// continue/start sleeping as requested
	if (isSleepingNode()) _sleep();
#endif
}

#if NODEMANAGER_RECEIVE == ON
	// dispacth inbound messages
	void NodeManager::receive(const MyMessage &message) {
		debug_verbose(PSTR(LOG_MSG "RECV(%d) c=%d t=%d p=%s\n"),message.sensor,message.getCommand(),message.type,message.getString());
		// dispatch the message to the registered sensor
		Sensor* sensor = getSensorWithChild(message.sensor);
		if (sensor != nullptr) {
			// turn on the pin powering all the sensors
			#if NODEMANAGER_POWER_MANAGER == ON
			powerOn();
			#endif
			// call the sensor's receive()
			sensor->receive((MyMessage*) &message);
			// turn off the pin powering all the sensors
			#if NODEMANAGER_POWER_MANAGER == ON
			powerOff();
			#endif
		}
	}
#endif

#if NODEMANAGER_TIME == ON
	// receive the time from the controller and save it
	void NodeManager::receiveTime(unsigned long ts) {
		// apply time adjustment for the configured timezone
		ts = ts + (long)_timezone*60*60;
		debug(PSTR(LOG_TIME "OK ts=%lu\n"),ts);
		// time is now valid
		_time_is_valid = true;
#if NODEMANAGER_RTC == ON
		// set the RTC time to the time received from the controller
		RTC.set(ts);
		// sync the system time with the RTC
		setSyncProvider(RTC.get);
#else
		// set the current system time to the received time
		setTime(ts);
#endif
		_time_last_sync = now();
	}
#endif

	// Send a hello message back to the controller
	void NodeManager::hello() {
		// do nothing, the request will be echoed back
	}

	// reboot the board
	void NodeManager::reboot() {
		debug(PSTR(LOG_POWER "RBT\n"));
		if (_reboot_pin > 0) {
			// reboot the board through the reboot pin which is connected to RST by setting it to low
			digitalWrite(_reboot_pin, LOW);
		}
#ifdef CHIP_AVR
		else {
			// Software reboot with watchdog timer. Enter Watchdog Configuration mode:
			WDTCSR |= (1<<WDCE) | (1<<WDE);
			// Reset enable
			WDTCSR= (1<<WDE);
			// Infinite loop until watchdog reset after 16 ms
			while(true){}
		}
#endif
	}

#if NODEMANAGER_EEPROM == ON
	// clear the EEPROM
	void NodeManager::clearEeprom() {
		debug(PSTR(LOG_EEPROM "CLR\n"));
		for (uint16_t i=0; i<EEPROM_LOCAL_CONFIG_ADDRESS; i++) saveState(i, 0xFF);
	}

	// return the value stored at the requested index from the EEPROM
	int NodeManager::loadFromMemory(int index) {
		int value = loadState(index+EEPROM_USER_START);
		debug_verbose(PSTR(LOG_EEPROM "LOAD i=%d v=%d\n"),index,value);
		return value;
	}

	// save the given index of the EEPROM the provided value
	void NodeManager::saveToMemory(int index, int value) {
		saveState(index+EEPROM_USER_START, value);
		debug_verbose(PSTR(LOG_EEPROM "SAVE i=%d v=%d\n"),index,value);
	}
#endif

#if NODEMANAGER_SLEEP == ON
	// wake up the board
	void NodeManager::wakeup() {
		debug(PSTR(LOG_SLEEP "WKP\n"));
		_status = AWAKE;
	}

	// use smart sleep for sleeping boards
	void NodeManager::setSmartSleep(bool value) {
		_smart_sleep = value;
	}

#endif

#if NODEMANAGER_INTERRUPTS == ON
	// return the pin from which the last interrupt came
	int8_t NodeManager::getLastInterruptPin() {
		return _last_interrupt_pin;
	}

	// return the value of the pin from which the last interrupt came
	int8_t NodeManager::getLastInterruptValue() {
		return _last_interrupt_value;
	}
#endif

	// set the default interval in seconds all the sensors will report their measures
	void NodeManager::setReportIntervalSeconds(unsigned long value) {
		_report_interval_seconds = value;
	}
	unsigned long NodeManager::getReportIntervalSeconds() {
		return _report_interval_seconds;
	}
	
	// set the default interval in minutes all the sensors will report their measures
	void NodeManager::setReportIntervalMinutes(unsigned long value) {
		_report_interval_seconds = value*60UL;
	}

	// set the default interval in hours all the sensors will report their measures
	void NodeManager::setReportIntervalHours(unsigned int value) {
		_report_interval_seconds = (unsigned long)value*60UL *60UL;
	}

	// set the default interval in days all the sensors will report their measures
	void NodeManager::setReportIntervalDays(uint8_t value) {
		_report_interval_seconds = (unsigned long)value*60UL *60UL *24UL;
	}

	// if set and when the board is battery powered, sleep() is always called instead of wait()
	void NodeManager::setSleepOrWait(bool value) {
		_sleep_or_wait = value;
	}

	// set which pin is connected to RST of the board to reboot the board when requested. If not set the software reboot is used instead (default: -1)
	void NodeManager::setRebootPin(int8_t value) {
		_reboot_pin = value;
	}

	// turn the ADC off so to save 0.2 mA
	void NodeManager::setADCOff() {
#ifdef CHIP_AVR
		// Disable the ADC by setting the ADEN bit (bit 7) to zero
		ADCSRA = ADCSRA & B01111111;
		// Disable the analog comparator by setting the ACD bit (bit 7) to one
		ACSR = B10000000;
#endif
	}

	// sleep if the node is a battery powered or wait if it is not for the given number of milliseconds 
	void NodeManager::sleepOrWait(unsigned long value) {
		// if the node is sleeping, sleep-or-wait is enabled and we need to sleep for a decent amount of time, call sleep() otherwise wait()
		if (isSleepingNode() && _sleep_or_wait && value > 200) sleep(value,false);
		else wait(value);
	}

	// return the next available child_id
	uint8_t NodeManager::getAvailableChildId(uint8_t child_id) {
		if (child_id != 0) {
			// requested a specific child_id, check if it is available
			Child* child = getChild(child_id);
			if (child == nullptr) return child_id;
		}
		// automatic child_id assignment or child already in use
		for (int i = 1; i < 255; i++) {
			// look for the first available child_id
			Child* child = getChild(i);
			if (child == nullptr) return i;
		}
		return 254;
	}

#if NODEMANAGER_INTERRUPTS == ON
	// setup the interrupt pins
	void NodeManager::_setupInterrupts() {
		// configure wakeup pin if needed
		if (_sleep_interrupt_pin > 0) {
			// set the interrupt when the pin is connected to ground
			setInterrupt(_sleep_interrupt_pin,FALLING,HIGH);
		}
		// setup the interrupt pins
		if (_interrupt_1_mode != MODE_NOT_DEFINED) {
			pinMode(INTERRUPT_PIN_1,INPUT);
			if (_interrupt_1_initial > -1) digitalWrite(INTERRUPT_PIN_1,_interrupt_1_initial);
			// for non sleeping nodes, we need to handle the interrupt by ourselves  
#if defined(CHIP_STM32)
			if (_status != SLEEP) attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_1), _onInterrupt_1, (ExtIntTriggerMode)_interrupt_1_mode);
#else
			if (_status != SLEEP) attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_1), _onInterrupt_1, _interrupt_1_mode);
#endif
			debug(PSTR(LOG_BEFORE "INT p=%d m=%d\n"),INTERRUPT_PIN_1,_interrupt_1_mode);
		}
		if (_interrupt_2_mode != MODE_NOT_DEFINED) {
			pinMode(INTERRUPT_PIN_2, INPUT);
			if (_interrupt_2_initial > -1) digitalWrite(INTERRUPT_PIN_2,_interrupt_2_initial);
			// for non sleeping nodes, we need to handle the interrupt by ourselves  
#if defined(CHIP_STM32)
			if (_status != SLEEP) attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_2), _onInterrupt_2, (ExtIntTriggerMode)_interrupt_2_mode);
#else
			if (_status != SLEEP) attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_2), _onInterrupt_2, _interrupt_2_mode);
#endif
			debug(PSTR(LOG_BEFORE "INT p=%d m=%d\n"),INTERRUPT_PIN_2,_interrupt_2_mode);
		}
	}
	
	// handle an interrupt
	void NodeManager::_onInterrupt_1() {
		_saveInterrupt(INTERRUPT_PIN_1);
	}
	
	void NodeManager::_onInterrupt_2() {
		_saveInterrupt(INTERRUPT_PIN_2);
	}
	
	// keep track of the last interrupt pin and value
	void NodeManager::_saveInterrupt(int8_t pin) {
		// ingore the interrupt if triggering too close to the previous one
		if (_interrupt_debounce > 0 &&  (millis() - _last_interrupt_millis < _interrupt_debounce) && pin == _last_interrupt_pin && millis() > _last_interrupt_millis) return;
		// save pin and value
		_last_interrupt_pin = pin;
		_last_interrupt_value = digitalRead(pin);
		debug(PSTR(LOG_LOOP "INT p=%d v=%d\n"),_last_interrupt_pin,_last_interrupt_value);
	}
#endif

	// send a message by providing the source child, type of the message and value
	void NodeManager::sendMessage(uint8_t child_id, uint8_t type, int value) {
		_message.clear();
		_message.set((int32_t)value);
		_sendMessage(child_id,type);
	}
	void NodeManager::sendMessage(uint8_t child_id, uint8_t type, float value, uint8_t precision) {
		_message.clear();
		_message.set(value,precision);
		_sendMessage(child_id,type);
	}
	void NodeManager::sendMessage(uint8_t child_id, uint8_t type, double value, uint8_t precision) {
		_message.clear();
		_message.set(value,precision);
		_sendMessage(child_id,type);
	}
	void NodeManager::sendMessage(uint8_t child_id, uint8_t type, const char* value) {
		_message.clear();
		_message.set(value);
		_sendMessage(child_id,type);
	}

	// send a message to the network
	void NodeManager::_sendMessage(uint8_t child_id, uint8_t type) {
		// prepare the message
		_message.setSensor(child_id);
		_message.setType(type);
		// send the message, multiple times if requested
		for (int i = 0; i < _retries; i++) {
			if (mGetPayloadType(_message) == P_INT16) { debug_verbose(PSTR(LOG_MSG "SEND(%d) t=%d p=%d\n"),_message.sensor,_message.type,_message.getInt()); }
			if (mGetPayloadType(_message) == P_LONG32) { debug_verbose(PSTR(LOG_MSG "SEND(%d) t=%d p=%ld\n"),_message.sensor,_message.type,_message.getLong()); }
			if (mGetPayloadType(_message) == P_FLOAT32) { debug_verbose(PSTR(LOG_MSG "SEND(%d) t=%d p=%d.%02d\n"),_message.sensor,_message.type,(unsigned int)_message.getFloat(), (unsigned int)(_message.getFloat()*100)%100); }
			if (mGetPayloadType(_message) == P_STRING) { debug_verbose(PSTR(LOG_MSG "SEND(%d) t=%d p=%s\n"),_message.sensor,_message.type,_message.getString()); }
			send(_message, _ack);
			// if configured, sleep between each send
			sleepBetweenSend();
		}
	}

#if NODEMANAGER_POWER_MANAGER == ON
	void NodeManager::setPowerManager(PowerManager& powerManager) {
		_powerManager = &powerManager;
	}
#endif

	// return the requested child 
	Child* NodeManager::getChild(uint8_t child_id) {
		Sensor* sensor = getSensorWithChild(child_id);
		if (sensor == nullptr) return nullptr;
		return sensor->getChild(child_id);
	}

	// return the sensor with the requested child 
	Sensor* NodeManager::getSensorWithChild(uint8_t child_id) {
		for (List<Sensor*>::iterator itr = sensors.begin(); itr != sensors.end(); ++itr) {
			Sensor* sensor = *itr;
			Child* child = sensor->getChild(child_id);
			if (child != nullptr) return sensor;
		}
		return nullptr;  
	}

#if NODEMANAGER_TIME == ON
	// sync the time with the controller
	void NodeManager::syncTime() {
		_time_is_valid = false;
		int8_t retries = 10;
		// ask the controller for the time up to 10 times until received
		while ( ! _time_is_valid && retries >= 0) {
			debug(PSTR(LOG_TIME "REQ\n"));
			requestTime();
			wait(1000);
			retries = retries - 1;
		}
	}

	// returns the current system time
	unsigned long NodeManager::getTime() {
		return now();
	}
	
	// set the hour offset for when syncronizing the time
	void NodeManager::setTimezone(int8_t value) {
		_timezone = value;
	}
	
	// request the current time to the controller during setup()
	void NodeManager::setSyncTimeOnSetup(bool value) {
		_sync_time_on_setup = value;
	}
	
	// request the current time to the controller just after a sleep cycle
	void NodeManager::setSyncTimeAfterSleep(bool value) {
		_sync_time_after_sleep = value;
	}
	
	// request the current time to the controller after the configured number of minutes
	void NodeManager::setSyncTimeAfterInterval(unsigned long value) {
		_sync_time_after_interval = value;
	}
#endif

#if NODEMANAGER_SLEEP == ON
	// wrapper of smart sleep
	void NodeManager::_sleep() {
		unsigned long sleep_time = _sleep_time;
#if NODEMANAGER_TIME == ON
		// if there is time still to sleep, sleep for that timeframe only
		if (_remainder_sleep_time > 0) sleep_time = _remainder_sleep_time;
#endif
		debug(PSTR(LOG_SLEEP "SLEEP s=%d\n\n"),sleep_time);
		// go to sleep
		int interrupt = -1;
#if NODEMANAGER_INTERRUPTS == ON
		// setup interrupt pins
		int interrupt_1_pin = _interrupt_1_mode == MODE_NOT_DEFINED ? INTERRUPT_NOT_DEFINED  : digitalPinToInterrupt(INTERRUPT_PIN_1);
		int interrupt_2_pin = _interrupt_2_mode == MODE_NOT_DEFINED ? INTERRUPT_NOT_DEFINED  : digitalPinToInterrupt(INTERRUPT_PIN_2);
		// enter sleep cycle for the requested sleep interval and with the configured interrupts
		if (interrupt_1_pin != INTERRUPT_NOT_DEFINED && interrupt_2_pin != INTERRUPT_NOT_DEFINED) {
			interrupt = sleep(interrupt_1_pin,_interrupt_1_mode,interrupt_2_pin,_interrupt_2_mode,sleep_time*1000,_smart_sleep);
		} else if (interrupt_1_pin != INTERRUPT_NOT_DEFINED) {
			interrupt = sleep(interrupt_1_pin,_interrupt_1_mode,sleep_time*1000,_smart_sleep);
		} else if (interrupt_2_pin != INTERRUPT_NOT_DEFINED) {
			interrupt = sleep(interrupt_2_pin,_interrupt_2_mode,sleep_time*1000,_smart_sleep);
		} else {
			sleep(INTERRUPT_NOT_DEFINED,MODE_NOT_DEFINED,INTERRUPT_NOT_DEFINED,MODE_NOT_DEFINED,sleep_time*1000,_smart_sleep);
		}
		// woke up by an interrupt
		if (interrupt > -1) {
			if (digitalPinToInterrupt(INTERRUPT_PIN_1) == interrupt) _saveInterrupt(INTERRUPT_PIN_1);
			if (digitalPinToInterrupt(INTERRUPT_PIN_2) == interrupt) _saveInterrupt(INTERRUPT_PIN_2);
			// when waking up from an interrupt on the wakup pin, stop sleeping
			if (_sleep_interrupt_pin == _last_interrupt_pin) _status = AWAKE;
		}
#else
		sleep(INTERRUPT_NOT_DEFINED,MODE_NOT_DEFINED,INTERRUPT_NOT_DEFINED,MODE_NOT_DEFINED,sleep_time*1000,_smart_sleep);
#endif
		// coming out of sleep
		debug(PSTR(LOG_SLEEP "AWAKE\n"));
#if NODEMANAGER_TIME == ON
		// keep track of the old time so to calculate the amount of time slept
		unsigned long old_time = now();
#if NODEMANAGER_RTC == ON
		// sync the time with the RTC
		setSyncProvider(RTC.get);
#else
		// sync the time with the controller
		if (_sync_time_after_sleep) syncTime();
#endif
		// calculate the remainder time to sleep if woken up by an interrupt
		if (interrupt > -1) {
			if (_remainder_sleep_time == -1) _remainder_sleep_time = _sleep_time;
			_remainder_sleep_time = _remainder_sleep_time - (now() - old_time);
		}
		else _remainder_sleep_time = -1;
#endif
	}
#endif

#if NODEMANAGER_EEPROM == ON
	// load the configuration stored in the eeprom
	void NodeManager::_loadSleepSettings() {
		if (loadState(EEPROM_SLEEP_SAVED) == 1) {
			// load sleep settings
			int bit_1 = loadState(EEPROM_SLEEP_1);
			int bit_2 = loadState(EEPROM_SLEEP_2);
			int bit_3 = loadState(EEPROM_SLEEP_3);
			_sleep_time = bit_3*255*255 + bit_2*255 + bit_1;
			debug(PSTR(LOG_SLEEP "LOAD s=%d\n"),_sleep_time);
		}
	}

	// save the configuration in the eeprom
	void NodeManager::_saveSleepSettings() {
		if (_sleep_time == 0) return;
		// encode the sleep time in 3 bits
		int bit_1, bit_2, bit_3 = 0;
		bit_1 = _sleep_time;
		if (bit_1 >= 255) {
			bit_2 = (int)bit_1/255;
			bit_1 = bit_1 - bit_2*255;
		}
		if (bit_2 >= 255) {
			bit_3 = (int)bit_2/255;
			bit_2 = bit_2 - bit_3*255;
		}
		// save the 3 bits
		saveState(EEPROM_SLEEP_SAVED,1);
		saveState(EEPROM_SLEEP_1,bit_1);
		saveState(EEPROM_SLEEP_2,bit_2);
		saveState(EEPROM_SLEEP_3,bit_3);
	}
#endif

// sleep between send()
void NodeManager::sleepBetweenSend() {
	// if we sleep here ack management will not work
	if (_sleep_between_send > 0) wait(_sleep_between_send);
}

// set the analog reference 
void NodeManager::setAnalogReference(uint8_t value, uint8_t pin) {
#ifdef CHIP_AVR
	// nothing to do
	if (value == _analog_reference) return;
	// change the analog reference
	analogReference(value);
	// update _analog_reference
	_analog_reference = value;
	// wait a bit 
	wait(200);
	// perform some reading before actually reading the value
	if (pin > 0) {
		for (int i = 0; i < 5; i++) {
			analogRead(pin);
			wait(50);
		}
	}
#endif
}

