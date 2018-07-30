/*
* NodeManager Library
*/

// include dependencies
#include <Arduino.h>
#include <MySensors_NodeManager.h>

/***************************************
PowerManager
*/

#if FEATURE_POWER_MANAGER == ON
PowerManager::PowerManager(int ground_pin, int vcc_pin, int wait_time) {
	setPowerPins(ground_pin, vcc_pin, wait_time);
}

// set the vcc and ground pin the sensor is connected to
void PowerManager::setPowerPins(int ground_pin, int vcc_pin, int wait_time) {
	_ground_pin = ground_pin;
	_vcc_pin = vcc_pin;
	if (_ground_pin > 0) {
		// configure the ground pin as output and initialize to low
		pinMode(_ground_pin, OUTPUT);
		digitalWrite(_ground_pin, LOW);
	}
	if (_vcc_pin > 0) {
		// configure the vcc pin as output and initialize to high (power on)
		pinMode(_vcc_pin, OUTPUT);
		digitalWrite(_vcc_pin, HIGH);
	}
	// save wait time
	_wait = wait_time;
}

// turn on the sensor by activating its power pins
void PowerManager::powerOn() {
	if (_vcc_pin == -1) return;
	debug(PSTR(LOG_POWER "ON p=%d\n"),_vcc_pin);
	// power on the sensor by turning high the vcc pin
	digitalWrite(_vcc_pin, HIGH);
	// wait a bit for the device to settle down
	if (_wait > 0) wait(_wait);
}

// turn off the sensor
void PowerManager::powerOff() {
	if (_vcc_pin == -1) return;
	debug(PSTR(LOG_POWER "OFF p=%d\n"),_ground_pin);
	// power off the sensor by turning low the vcc pin
	digitalWrite(_vcc_pin, LOW);
}
#endif

/******************************************
	Timer
*/

Timer::Timer(NodeManager* node_manager) {
	_node = node_manager;
}

void Timer::setMode(timer_mode mode) {
	_mode = mode;
}
timer_mode Timer::getMode() {
	return _mode;
}

void Timer::setValue(int value) {
	_value = value;
}

int Timer::getValue() {
	return _value;
}

// start the timer
void Timer::start() {
	// reset the timer
	_last = 0;
	_is_running = true;
#if FEATURE_TIME == ON
	// save the current timestamp (which is sync'ed when sleeping or not sleeping)
	if (_mode == TIME_INTERVAL) _last = now();
	// keep track of the current minute/hour/day
	if (_mode == EVERY_MINUTE) _last = minute();
	if (_mode == EVERY_HOUR) _last = hour();
	if (_mode == EVERY_DAY) _last = day();
#else
	// keep track of millis() for calculating the difference
	_last = millis();
#endif
}

// stop the timer
void Timer::stop() {
	_is_running = false;
}

// return true if the time is over
bool Timer::isOver() {
	if (_mode == DO_NOT_REPORT || _mode == NOT_CONFIGURED) return false;
	if (_mode == IMMEDIATELY) return true;
	if (_mode == TIME_INTERVAL) {
		if (! _is_running) return false;
		long elapsed = getElapsed();
		// check if time has elapsed or millis has started over
		if (elapsed >= _value || elapsed < 0) return true;
		return false;
	}
#if FEATURE_TIME == ON
	// if the minute/hour/day has changed, the timer is over
	if (_mode == EVERY_HOUR && hour() != _last) return true;
	if (_mode == EVERY_HOUR && hour() != _last) return true;
	if (_mode == EVERY_DAY && day() != _last) return true;
	// if we are in the requested minute/hour/day and not already reported, timer is over
	if (_mode == AT_MINUTE && minute() >= _value && ! _already_reported) {
		_already_reported = true;
		return true;
	}
	if (_mode == AT_HOUR && hour() >= _value && ! _already_reported) {
		_already_reported = true;
		return true;
	}
	if (_mode == AT_DAY && day() >= _value && ! _already_reported) { 
		_already_reported = true;
		return true;
	}
#endif
	return false;
}

// return elapsed seconds so far
long Timer::getElapsed() {
	// calculate the elapsed time
	long elapsed = 0;
#if FEATURE_TIME == ON
	// system time is available so use now() to calculated the elapsed time
	elapsed = (long)(now() - _last);
#else
	// system time is not available
#if FEATURE_SLEEP == ON
	// millis() is not reliable while sleeping so calculate how long a sleep cycle would last in seconds and update the elapsed time
	if (_node->isSleepingNode()) elapsed += _node->getSleepSeconds();
#endif
	// use millis() to calculate the elapsed time in seconds
	if (! _node->isSleepingNode()) elapsed = (long)((millis() - _last)/1000);
#endif
	return elapsed;
}


/******************************************
	Sensors
*/

/*
Child class
*/

Child::Child() {

}

// constructor
Child::Child(Sensor* sensor, int child_id, int presentation, int type, const char* description) {
	_child_id = child_id;
	_presentation = presentation;
	_type = type;
	_description = description;
	_sensor = sensor;
	// register the child with the sensor
	_sensor->registerChild(this);
#if FEATURE_CONDITIONAL_REPORT == ON
	// initialize the timer for forcing updates to the gateway after a given timeframe
	_force_update_timer = new Timer(_sensor->_node);
#endif
}

// setter/getter
void Child::setChildId(int value) {
	_child_id = value;
}
int Child::getChildId() {
	return _child_id;
}
void Child::setPresentation(int value) {
	_presentation = value;
}
int Child::getPresentation() {
	return _presentation;
}
void Child::setType(int value) {
	_type = value;
}
int Child::getType() {
	return _type;
}
void Child::setFloatPrecision(int value) {
	_float_precision = value;
}
void Child::setDescription(const char* value) {
	_description = value;
}
const char* Child::getDescription() {
	return _description;
}
#if FEATURE_CONDITIONAL_REPORT == ON
void Child::setForceUpdateTimerValue(int value) {
	_force_update_timer->setMode(TIME_INTERVAL);
	_force_update_timer->setValue(value*60);
}
void Child::setMinThreshold(float value) {
	_min_threshold = value;
}
void Child::setMaxThreshold(float value) {
	_max_threshold = value;
}
void Child::setValueDelta(float value) {
	_value_delta = value;
}
#endif

// set a value, implemented by the subclasses
void Child::sendValue(bool force) {
}
// Print the child's value (variable type depending on the child class) to the given output
void Child::print(Print& device) {
}
// reset the counters
void Child::reset() {
}

/*
ChildInt class
*/

// ChildInt class
ChildInt::ChildInt(Sensor* sensor, int child_id, int presentation, int type, const char* description): Child(sensor, child_id, presentation, type, description)  {
}

// store a new value and update the total
void ChildInt::setValue(int value) {
	if (isnan(value)) return;
	_total = _total + value;
	_samples++;
	// averages the values
	_value = (int) (_total / _samples);
	// print out a debug message
	debug(PSTR(LOG_LOOP "%s(%d):SET t=%d v=%d\n"),_description,_child_id,_type,_value);
}

// return the value
int ChildInt::getValue() {
	return _value;
}

// send the value back to the controller
void ChildInt::sendValue(bool force) {
	if (_samples == 0) return;
#if FEATURE_CONDITIONAL_REPORT == ON
	if (! force) {
		// if below or above the thresholds, do not send the value
		if (_value < _min_threshold || _value > _max_threshold) return;
		// if the force update timer is over, send the value regardless and restart it
		if (_force_update_timer->isOver()) _force_update_timer->start();
		else {
			// if the value does not differ enough from the previous one, do not send the value
			if (_value > (_last_value - _value_delta) && _value < (_last_value + _value_delta)) {
				// keep track of the previous value
				_last_value = _value;
				return;
			}
		}
	}
	// keep track of the previous value
	_last_value = _value;
#endif
	// send the value to the gateway
	_sensor->_node->sendMessage(_child_id,_type,_value);
	reset();
}

// Print the child's value (variable type depending on the child class) to the given output
void ChildInt::print(Print& device) {
	device.print(_value);
}

// reset the counters
void ChildInt::reset() {
	_total = 0;
	_samples = 0;
}

/*
ChildFloat class
*/

// ChildFloat class
ChildFloat::ChildFloat(Sensor* sensor, int child_id, int presentation, int type, const char* description): Child(sensor, child_id, presentation, type, description)  {
	_float_precision = 2;
}

// store a new value and update the total
void ChildFloat::setValue(float value) {
	if (isnan(value)) return;
	_total = _total + value;
	_samples++;
	// averages the values
	_value = _total / _samples;
	// round the value if float precision has been customized
	if (_float_precision != 2) {
		if (_float_precision == 0) _value = (int) _value;
		else _value = float((int) (_value * (_float_precision*10))) / (_float_precision*10);
	}
	// print out a debug message
	debug(PSTR(LOG_LOOP "%s(%d):SET t=%d v=%d.%02d\n"),_description,_child_id,_type,(int)_value, (int)(_value*100)%100);
}

// return the value
float ChildFloat::getValue() {
	return _value;
}

// send the value back to the controller
void ChildFloat::sendValue(bool force) {
	if (_samples == 0) return;
#if FEATURE_CONDITIONAL_REPORT == ON
	if (! force) {
		// if below or above the thresholds, do not send the value
		if (_value < _min_threshold || _value > _max_threshold) return;
		//   if the force update timer is over, send the value regardless and restart it
		if (_force_update_timer->isOver()) _force_update_timer->start();
		else {
			// if the value does not differ enough from the previous one, do not send the value
			if (_value > (_last_value - _value_delta) && _value < (_last_value + _value_delta)) {
				// keep track of the previous value
				_last_value = _value;
				return;
			}
		}
	}
	// keep track of the previous value
	_last_value = _value;
#endif
	// send the value to the gateway
	_sensor->_node->sendMessage(_child_id,_type,_value,_float_precision);
}

// Print the child's value (variable type depending on the child class) to the given output
void ChildFloat::print(Print& device) {
	device.print(_value,_float_precision);
}

// reset the counters
void ChildFloat::reset() {
	_total = 0;
	_samples = 0;
}


/*
ChildDouble class
*/

// ChildDouble class
ChildDouble::ChildDouble(Sensor* sensor, int child_id, int presentation, int type, const char* description): Child(sensor, child_id, presentation, type, description)  {
	_float_precision = 4;
}

// store a new value and update the total
void ChildDouble::setValue(double value) {
	if (isnan(value)) return;
	_total = _total + value;
	_samples++;
	// averages the values
	_value = _total / _samples;
	// round the value if float precision has been customized
	if (_float_precision != 4) {
		if (_float_precision == 0) _value = (int) _value;
		else _value = double((int) (_value * (_float_precision*10))) / (_float_precision*10);
	}
	// print out a debug message
	debug(PSTR(LOG_LOOP "%s(%d):SET t=%d v=%d.%04d\n"),_description,_child_id,_type,(int)_value, (int)(_value*10000)%10000);
}

// return the value
double ChildDouble::getValue() {
	return _value;
}

// send the value back to the controller
void ChildDouble::sendValue(bool force) {
	if (_samples == 0) return;
#if FEATURE_CONDITIONAL_REPORT == ON
	if (! force) {
		// if below or above the thresholds, do not send the value
		if (_value < _min_threshold || _value > _max_threshold) return;
		// if the force update timer is over, send the value regardless and restart it
		if (_force_update_timer->isOver()) _force_update_timer->start();
		else {
			// if the value does not differ enough from the previous one, do not send the value
			if (_value > (_last_value - _value_delta) && _value < (_last_value + _value_delta)) {
				// keep track of the previous value
				_last_value = _value;
				return;
			}
		}
	}
	// keep track of the previous value
	_last_value = _value;
#endif
	// send the value to the gateway
	_sensor->_node->sendMessage(_child_id,_type,_value,_float_precision);
}

// Print the child's value (variable type depending on the child class) to the given output
void ChildDouble::print(Print& device) {
	device.print(_value,_float_precision);
}

// reset the counters
void ChildDouble::reset() {
	_total = 0;
	_samples = 0;
}


/*
ChildString class
*/

// ChildString class
ChildString::ChildString(Sensor* sensor, int child_id, int presentation, int type, const char* description): Child(sensor, child_id, presentation, type, description)  {
}

// store a new value and update the total
void ChildString::setValue(const char* value) {
	_value = value;
	// print out a debug message
	debug(PSTR(LOG_LOOP "%s(%d):SET t=%d v=%s\n"),_description,_child_id,_type,_value);
}

// return the value
const char* ChildString::getValue() {
	return _value;
}

// send the value back to the controller
void ChildString::sendValue(bool force) {
#if FEATURE_CONDITIONAL_REPORT == ON
	if (! force) {
		// if a delta is configured, do not report if the string is the same as the previous one
		if (_value_delta > 0 && strcmp(_value, _last_value) == 0) {
			// keep track of the previous value
			_last_value = _value;
			return;
		}
	}
	// keep track of the previous value
	_last_value = _value;
#endif
	// send the value to the gateway
	_sensor->_node->sendMessage(_child_id,_type,_value);
}

// Print the child's value (variable type depending on the child class) to the given output
void ChildString::print(Print& device) {
	device.print(_value);
}

// reset the counters
void ChildString::reset() {
	_value = "";
}

/*
Sensor class
*/
// constructor
Sensor::Sensor() {  
}
Sensor::Sensor(NodeManager& node_manager, int pin) {
	_node = &node_manager;
	_pin = pin;
	// initialize the timers
	_report_timer = new Timer(_node);
	_measure_timer = new Timer(_node);
	// register the sensor with the node
	_node->registerSensor(this);
}

// return the name of the sensor
const char* Sensor::getName() {
	return _name;
}

// setter/getter
void Sensor::setPin(int value) {
	_pin = value;
}
int Sensor::getPin() {
	return _pin;
}
void Sensor::setSamples(int value) {
	_samples = value;
}
void Sensor::setSamplesInterval(int value) {
	_samples_interval = value;
}
#if FEATURE_POWER_MANAGER == ON
void Sensor::setPowerPins(int ground_pin, int vcc_pin, int wait_time) {
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
#if FEATURE_INTERRUPTS == ON
int Sensor::getInterruptPin() {
	return _interrupt_pin;
}
void Sensor::setInterruptMode(int value) {
	_interrupt_mode = value;
}
void Sensor::setWaitAfterInterrupt(int value) {
	_wait_after_interrupt = value;
}
void Sensor::setPinInitialValue(int value) {
	_initial_value = value;
}
void Sensor::setInterruptStrict(bool value) {
	_interrupt_strict = value;
}
#endif

void Sensor::setReportTimerMode(timer_mode value) {
	_report_timer->setMode(value);
}

// After how many seconds the sensor will report back its measure
void Sensor::setReportTimerValue(int value) {
	_report_timer->setValue(value);
}

void Sensor::setMeasureTimerMode(timer_mode value) {
	_measure_timer->setMode(value);
}

void Sensor::setMeasureTimerValue(int value) {
	_measure_timer->setValue(value);
}

// After how many seconds the sensor will report back its measure
void Sensor::setReportIntervalSeconds(int value) {
	_report_timer->setMode(TIME_INTERVAL);
	_report_timer->setValue(value);
}

// After how many minutes the sensor will report back its measure 
void Sensor::setReportIntervalMinutes(int value) {
  setReportIntervalSeconds(value*60);
}

// After how many hours the sensor will report back its measure 
void Sensor::setReportIntervalHours(int value) {
  setReportIntervalSeconds(value*60*60);
}

// After how many days the sensor will report back its measure 
void Sensor::setReportIntervalDays(int value) {
  setReportIntervalSeconds(value*60*60*24);
}


// register a child
void Sensor::registerChild(Child* child) {
	children.push(child);
}

// present the sensor to the gateway and controller
void Sensor::presentation() {
	for (List<Child*>::iterator itr = children.begin(); itr != children.end(); ++itr) {
		Child* child = *itr;
		debug(PSTR(LOG_PRESENTATION "%s(%d) p=%d t=%d\n"),child->getDescription(),child->getChildId(),child->getPresentation(),child->getType());
		present(child->getChildId(), child->getPresentation(), child->getDescription(), _node->getAck());
	}

}

// call the sensor-specific implementation of setup
void Sensor::setup() {
	// configure default reporting interval if not already set by the user
	if (_report_timer->getMode() == NOT_CONFIGURED) {
		_report_timer->setMode(TIME_INTERVAL);
		_report_timer->setValue(_node->getReportIntervalSeconds());
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
#if FEATURE_INTERRUPTS == ON
	// for interrupt based sensors, register a callback for the interrupt
	_interrupt_pin = _pin;
	_node->setInterrupt(_pin,_interrupt_mode,_initial_value);
#endif
#if FEATURE_HOOKING == ON
	// if a hook function is defined, call it
	if (_setup_hook != 0) _setup_hook(this); 
#endif
}

// call the sensor-specific implementation of loop
void Sensor::loop(MyMessage* message) {
	// run the sensor's loop() function if the timer is over OR it is the first run OR we've been called from receive()
	if (_measure_timer->isOver() || _first_run || message != nullptr) {
#if FEATURE_POWER_MANAGER == ON
		// turn the sensor on
		powerOn();
#endif
#if FEATURE_HOOKING == ON
		// if a hook function is defined, call it
		if (_pre_loop_hook != 0) _pre_loop_hook(this); 
#endif
		// iterates over all the children
		for (List<Child*>::iterator itr = children.begin(); itr != children.end(); ++itr) {
			Child* child = *itr;
			// if a specific child is requested from receive(), skip all the others
			if (message != nullptr && message->sensor != child->getChildId()) continue;
			// collect multiple samples if needed
			for (int i = 0; i < _samples; i++) {
				// we've been called from receive(), pass the message along
				if (message != nullptr) onReceive(message);
				// we'be been called from loop()
				else onLoop(child);
				// wait between samples
				if (_samples_interval > 0) _node->sleepOrWait(_samples_interval);
			}
		}
#if FEATURE_HOOKING == ON
		// if a hook function is defined, call it
		if (_post_loop_hook != 0) _post_loop_hook(this); 
#endif
#if FEATURE_POWER_MANAGER == ON
		// turn the sensor off
		powerOff();
#endif
		// restart the timer if over
		if (_measure_timer->isOver()) _measure_timer->start();
	}
	// send the latest measure back to the network if the timer is over OR it is the first run OR we've been called from receive()
	if (_report_timer->isOver() || _first_run || message != nullptr) {
		// iterates over all the children
		for (List<Child*>::iterator itr = children.begin(); itr != children.end(); ++itr) {
			Child* child = *itr;
			// if a specific child is requested from receive(), skip all the others
			if (message != nullptr && message->sensor != child->getChildId()) continue;
			// send the value back to the controller
			child->sendValue(message != nullptr);
			// reset the counters
			child->reset();
		}
		// restart the timer if over
		if (_report_timer->isOver()) _report_timer->start();
	}
	// unset first run if set
	if (_first_run) _first_run = false;
}

#if FEATURE_INTERRUPTS == ON
// receive and handle an interrupt
bool Sensor::interrupt() {
	// ignore the interrupt if the value is not matching the one expected
	if (_interrupt_strict && ((_interrupt_mode == RISING && _node->getLastInterruptValue() != HIGH ) || (_interrupt_mode == FALLING && _node->getLastInterruptValue() != LOW))) return false;
	// call the sensor's implementation of onInterrupt()
	onInterrupt();
	// wait after interrupt if needed
	if (_wait_after_interrupt > 0) _node->sleepOrWait(_wait_after_interrupt);
#if FEATURE_HOOKING == ON
	// if a hook function is defined, call it
	if (_interrupt_hook != 0) _interrupt_hook(this); 
#endif
	return true;
}
#endif

#if FEATURE_RECEIVE == ON
// receive a message from the radio network
void Sensor::receive(MyMessage* message) {
	// a request would make the sensor executing its main task passing along the message
	loop(message);
#if FEATURE_HOOKING == ON
	// if a hook function is defined, call it
	if (_receive_hook != 0) _receive_hook(this,message); 
#endif
}
#endif

// return the requested child 
Child* Sensor::getChild(int child_id) {
	for (List<Child*>::iterator itr = children.begin(); itr != children.end(); ++itr) {
		Child* child = *itr;
		if (child->getChildId() == child_id) return child;
	}
	return nullptr;
}

#if FEATURE_POWER_MANAGER == ON
void Sensor::setPowerManager(PowerManager& powerManager) {
	_powerManager = &powerManager;
}
#endif
#if FEATURE_HOOKING == ON
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
#ifdef USE_CONFIGURATION
void Sensor::onConfiguration(ConfigurationRequest* request) {}
#endif




#ifdef USE_CONFIGURATION


// contructor
SensorConfiguration::SensorConfiguration(NodeManager& node_manager): Sensor(node_manager) {
	_name = "CONFIG";
	children.allocateBlocks(1);
	new ChildInt(this,CONFIGURATION_CHILD_ID,S_CUSTOM,V_CUSTOM,_name);
}

// what to do during setup
void SensorConfiguration::onSetup() {
}

// what to do during loop
void SensorConfiguration::onLoop(Child* child) {
}

// what to do as the main task when receiving a message
void SensorConfiguration::onReceive(MyMessage* message) {
	// expect a REQ, V_CUSTOM message
	if (message->getCommand() != C_REQ && message->type != V_CUSTOM) return;
	// parse the request
	ConfigurationRequest request = ConfigurationRequest(message->sensor,message->getString());
	int function = request.getFunction();
	int child_id = request.getChildId();
	// if the message is for the board itself
	if (child_id == 0) {
		switch(function) {
		case 1: _node->hello(); break;
#if FEATURE_SLEEP == ON
		case 3: _node->setSleepSeconds(request.getValueInt()); break;
		case 4: _node->setSleepMinutes(request.getValueInt()); break;
		case 5: _node->setSleepHours(request.getValueInt()); break;
		case 29: _node->setSleepDays(request.getValueInt()); break;
		case 20: _node->setSleepBetweenSend(request.getValueInt()); break;
		case 9: _node->wakeup(); break;
#endif
#ifdef CHIP_AVR
		case 6: _node->reboot(); return;
#endif
#if FEATURE_EEPROM == ON
		case 7: _node->clearEeprom(); break;
		case 27: _node->saveToMemory(0,request.getValueInt()); break;
		case 40: _node->setSaveSleepSettings(request.getValueInt()); break;
#endif
		case 8: _node->sendMessage(CONFIGURATION_CHILD_ID,V_CUSTOM,VERSION); return;
		case 10: _node->setRetries(request.getValueInt()); break;
#if FEATURE_INTERRUPTS == ON
		case 19: _node->setSleepInterruptPin(request.getValueInt()); break;
		case 28: _node->setInterruptDebounce(request.getValueInt()); break;
#endif
		case 21: _node->setAck(request.getValueInt()); break;
		case 22: _node->setIsMetric(request.getValueInt()); break;
#if FEATURE_POWER_MANAGER == ON
		case 24: _node->powerOn(); break;
		case 25: _node->powerOff(); break;
#endif
		case 30: _node->setSleepOrWait(request.getValueInt()); break;
		case 31: _node->setRebootPin(request.getValueInt()); break;
		case 32: _node->setADCOff(); break;
		case 36: _node->setReportIntervalSeconds(request.getValueInt()); break;
		case 37: _node->setReportIntervalMinutes(request.getValueInt()); break;
		case 38: _node->setReportIntervalHours(request.getValueInt()); break;
		case 39: _node->setReportIntervalDays(request.getValueInt()); break;
#if FEATURE_TIME == ON
		case 41: _node->syncTime(); break;
		case 42: _node->sendMessage(CONFIGURATION_CHILD_ID,V_CUSTOM,(int)_node->getTime()); return;
#endif
		default: return; 
		}
		// the request is for a sensor
	} else {
		// retrieve the sensor the child is belonging to
		Sensor* sensor = _node->getSensorWithChild(child_id);
		if (sensor == nullptr) return;
		// if the message is for a function common to all the sensors
		if (request.getFunction() < 100) {
			switch(function) {
			case 1: sensor->setPin(request.getValueInt()); break;
			case 5: sensor->setSamples(request.getValueInt()); break;
			case 6: sensor->setSamplesInterval(request.getValueInt()); break;
#if FEATURE_POWER_MANAGER == ON
			case 13: sensor->powerOn(); break;
			case 14: sensor->powerOff(); break;
#endif
			case 24: sensor->setReportTimerMode((timer_mode)request.getValueInt()); break;
			case 25: sensor->setReportTimerValue(request.getValueInt()); break;
			case 26: sensor->setMeasureTimerMode((timer_mode)request.getValueInt()); break;
			case 27: sensor->setMeasureTimerValue(request.getValueInt()); break;
			case 16: sensor->setReportIntervalMinutes(request.getValueInt()); break;
			case 17: sensor->setReportIntervalSeconds(request.getValueInt()); break;
			case 19: sensor->setReportIntervalHours(request.getValueInt()); break;
			case 20: sensor->setReportIntervalDays(request.getValueInt()); break;
#if FEATURE_INTERRUPTS == ON			
			case 22: sensor->setInterruptMode(request.getValueInt()); break;
			case 23: sensor->setWaitAfterInterrupt(request.getValueInt()); break;
#endif
			default: return;
			}
		} else {
			sensor->onConfiguration(&request);
		}
	}
	_node->sendMessage(CONFIGURATION_CHILD_ID,V_CUSTOM,function);
}
			/*
			// the message is for a function specific to a sensor
			#ifdef USE_BATTERY
			if (strcmp(sensor->getName(),"BATTERY") == 0) {
				SensorBattery* custom_sensor = (SensorBattery*)sensor;
				switch(function) {
				case 102: custom_sensor->setMinVoltage(request.getValueFloat()); break;
				case 103: custom_sensor->setMaxVoltage(request.getValueFloat()); break;
				case 104: custom_sensor->setBatteryInternalVcc(request.getValueInt()); break;
				case 105: custom_sensor->setBatteryPin(request.getValueInt()); break;
				case 106: custom_sensor->setBatteryVoltsPerBit(request.getValueFloat()); break;
				case 107: custom_sensor->setBatteryCalibrationFactor(request.getValueFloat()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_SIGNAL
			if (strcmp(sensor->getName(),"SIGNAL") == 0) {
				SensorSignal* custom_sensor = (SensorSignal*)sensor;
				switch(function) {
				case 101: custom_sensor->setSignalCommand(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_ANALOG_INPUT
			if (strcmp(sensor->getName(),"ANALOG_I") == 0 || strcmp(sensor->getName(),"LDR") == 0 || strcmp(sensor->getName(),"RAIN") == 0 || strcmp(sensor->getName(),"SOIL") == 0) {
				SensorAnalogInput* custom_sensor = (SensorAnalogInput*)sensor;
				switch(function) {
				case 101: custom_sensor->setReference(request.getValueInt()); break;
				case 102: custom_sensor->setReverse(request.getValueInt()); break;
				case 103: custom_sensor->setOutputPercentage(request.getValueInt()); break;
				case 104: custom_sensor->setRangeMin(request.getValueInt()); break;
				case 105: custom_sensor->setRangeMax(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_THERMISTOR
			if (strcmp(sensor->getName(),"THERMISTOR") == 0) {
				SensorThermistor* custom_sensor = (SensorThermistor*)sensor;
				switch(function) {
				case 101: custom_sensor->setNominalResistor((long)request.getValueInt()); break;
				case 102: custom_sensor->setNominalTemperature(request.getValueInt()); break;
				case 103: custom_sensor->setBCoefficient(request.getValueInt()); break;
				case 104: custom_sensor->setSeriesResistor((long)request.getValueInt()); break;
				case 105: custom_sensor->setOffset(request.getValueFloat()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_ACS712
			if (strcmp(sensor->getName(),"ACS712") == 0) {
				SensorACS712* custom_sensor = (SensorACS712*)sensor;
				switch(function) {
				case 100: custom_sensor->setmVPerAmp(request.getValueInt()); break;
				case 102: custom_sensor->setOffset(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_DIGITAL_OUTPUT
			if (strcmp(sensor->getName(),"DIGITAL_O") == 0 || strcmp(sensor->getName(),"RELAY") == 0 || strcmp(sensor->getName(),"LATCHING1PIN") == 0 || strcmp(sensor->getName(),"LATCHING2PINS") == 0) {
				SensorDigitalOutput* custom_sensor = (SensorDigitalOutput*)sensor;
				switch(function) {
				case 104: custom_sensor->setLegacyMode(request.getValueInt()); break;
				case 105: custom_sensor->setSafeguard(request.getValueInt()); break;
				case 106: custom_sensor->setInputIsElapsed(request.getValueInt()); break;
				case 107: custom_sensor->setWaitAfterSet(request.getValueInt()); break;
				case 108: custom_sensor->setPulseWidth(request.getValueInt()); break;
				case 109: custom_sensor->setInvertValueToWrite(request.getValueInt()); break;
				case 110: custom_sensor->setPinOff(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_INTERRUPT
			if (strcmp(sensor->getName(),"INTERRUPT") == 0 || strcmp(sensor->getName(),"DOOR") == 0 || strcmp(sensor->getName(),"MOTION") == 0) {
				SensorInterrupt* custom_sensor = (SensorInterrupt*)sensor;
				switch(function) {
				case 105: custom_sensor->setInvertValueToReport(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_DS18B20
			if (strcmp(sensor->getName(),"DS18B20") == 0) {
				SensorDs18b20* custom_sensor = (SensorDs18b20*)sensor;
				switch(function) {
				case 101: custom_sensor->setResolution(request.getValueInt()); break;
				case 102: custom_sensor->setSleepDuringConversion(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_BH1750
			if (strcmp(sensor->getName(),"BH1750") == 0) {
				SensorBH1750* custom_sensor = (SensorBH1750*)sensor;
				switch(function) {
				case 101: custom_sensor->setMode(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#if defined(USE_BME280) || defined(USE_BMP085_180) || defined(USE_BMP280)
			if (strcmp(sensor->getName(),"BMP085") == 0 || strcmp(sensor->getName(),"BMP180") == 0 || strcmp(sensor->getName(),"BME280") == 0 || strcmp(sensor->getName(),"BMP280") == 0) {
				SensorBosch* custom_sensor = (SensorBosch*)sensor;
				switch(function) {
				case 101: custom_sensor->setForecastSamplesCount(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_SONOFF
			if (strcmp(sensor->getName(),"SONOFF") == 0) {
				SensorSonoff* custom_sensor = (SensorSonoff*)sensor;
				switch(function) {
				case 101: custom_sensor->setButtonPin(request.getValueInt()); break;
				case 102: custom_sensor->setRelayPin(request.getValueInt()); break;
				case 103: custom_sensor->setLedPin(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_HCSR04
			if (strcmp(sensor->getName(),"HCSR04") == 0) {
				SensorHCSR04* custom_sensor = (SensorHCSR04*)sensor;
				switch(function) {
				case 103: custom_sensor->setMaxDistance(request.getValueInt()); break;
				case 104: custom_sensor->setReportIfInvalid(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_MQ
			if (strcmp(sensor->getName(),"MQ") == 0) {
				SensorMQ* custom_sensor = (SensorMQ*)sensor;
				switch(function) {
				case 102: custom_sensor->setRlValue(request.getValueFloat()); break;
				case 103: custom_sensor->setRoValue(request.getValueFloat()); break;
				case 104: custom_sensor->setKnownPpm(request.getValueInt()); break;
				case 105: custom_sensor->setCalibrationSamples(request.getValueInt()); break;
				case 106: custom_sensor->setCalibrationSampleInterval(request.getValueInt()); break;
				case 107: custom_sensor->setSamples(request.getValueInt()); break;
				case 108: custom_sensor->setSampleInterval(request.getValueInt()); break;
				case 109: custom_sensor->setPoint1Ppm(request.getValueFloat()); break;
				case 110: custom_sensor->setPoint1Ratio(request.getValueFloat()); break;
				case 111: custom_sensor->setPoint2Ppm(request.getValueFloat()); break;
				case 112: custom_sensor->setPoint2Ratio(request.getValueFloat()); break;
				case 113: custom_sensor->setCurveScalingFactor(request.getValueFloat()); break;
				case 114: custom_sensor->setCurveExponent(request.getValueFloat()); break; 
				default: return;
				}
			}
			#endif
			#ifdef USE_TSL2561
			if (strcmp(sensor->getName(),"TSL2561") == 0) {
				SensorTSL2561* custom_sensor = (SensorTSL2561*)sensor;
				switch(function) {
				case 101: custom_sensor->setGain(request.getValueInt()); break;
				case 102: custom_sensor->setTiming(request.getValueInt()); break;
				case 103: custom_sensor->setSpectrum(request.getValueInt()); break;
				case 104: custom_sensor->setAddress(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_PT100
			if (strcmp(sensor->getName(),"PT100") == 0) {
				SensorPT100* custom_sensor = (SensorPT100*)sensor;
				switch(function) {
				case 101: custom_sensor->setVoltageRef(request.getValueFloat()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_DIMMER
			if (strcmp(sensor->getName(),"DIMMER") == 0) {
				SensorDimmer* custom_sensor = (SensorDimmer*)sensor;
				switch(function) {
				case 101: custom_sensor->setEasing(request.getValueInt()); break;
				case 102: custom_sensor->setDuration(request.getValueInt()); break;
				case 103: custom_sensor->setStepDuration(request.getValueInt()); break;
				case 104: custom_sensor->setReverse(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_PULSE_METER
			if (strcmp(sensor->getName(),"RAIN_GAUGE") == 0 || strcmp(sensor->getName(),"POWER") == 0 || strcmp(sensor->getName(),"WATER") == 0) {
				SensorPulseMeter* custom_sensor = (SensorPulseMeter*)sensor;
				switch(function) {
				case 102: custom_sensor->setPulseFactor(request.getValueFloat()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_SSD1306
			if (strcmp(sensor->getName(),"SSD1306") == 0) {
				DisplaySSD1306* display_SSD1306 = (DisplaySSD1306*)sensor;
				switch(function) {
				case 102: display_SSD1306->setContrast((uint8_t)request.getValueInt()); break;
				case 104: display_SSD1306->rotateDisplay((bool)request.getValueInt()); break;
				case 105: display_SSD1306->setFontSize(request.getValueInt()); break;
				case 106: display_SSD1306->setCaptionFontSize(request.getValueInt()); break;
				case 107: display_SSD1306->invertDisplay((bool)request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_CHIRP
			if (strcmp(sensor->getName(),"CHIRP") == 0) {
				SensorChirp* custom_sensor = (SensorChirp*)sensor;
				switch(function) {
				case 101: custom_sensor->setMoistureOffset(request.getValueInt()); break;
				case 102: custom_sensor->setMoistureRange(request.getValueInt()); break;
				case 103: custom_sensor->setReturnMoistureNormalized(request.getValueInt()); break;
				case 104: custom_sensor->setReturnLightReversed(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_FPM10A
			if (strcmp(sensor->getName(),"FPM10A") == 0) {
				SensorFPM10A* custom_sensor = (SensorFPM10A*)sensor;
				switch(function) {
				case 101: custom_sensor->setMinConfidence(request.getValueInt()); break;
				case 102: custom_sensor->setWaitFingerForSeconds(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
		}
	}
	_node->sendMessage(CONFIGURATION_CHILD_ID,V_CUSTOM,function);
}
*/

/******************************************
	ConfigurationRequest
*/

// contructor, tokenize a configuration request in the format "child_id,function,value"
ConfigurationRequest::ConfigurationRequest(int recipient_child_id, const char* string) {
	_recipient_child_id = recipient_child_id;
	char* ptr;
	// tokenize the string and get child id
	_child_id = atoi(strtok_r(const_cast<char*>(string), ",", &ptr));
	// tokenize the string and get function id
	_function = atoi(strtok_r(NULL, ",", &ptr));
	// tokenize the string and get the value
	_value = atof(strtok_r(NULL, ",", &ptr));
	debug(PSTR(LOG_OTA "REQ f=%d v=%d\n"),_function,_value);
}

// return the child id
int ConfigurationRequest::getRecipientChildId() {
	return _recipient_child_id;
}

// return the child id
int ConfigurationRequest::getChildId() {
	return _child_id;
}

// return the parsed function
int ConfigurationRequest::getFunction() {
	return _function;
}

// return the value as an int
int ConfigurationRequest::getValueInt() {
	return (int)_value;

}

// return the value as a float
float ConfigurationRequest::getValueFloat() {
	return _value;
}
#endif

/*******************************************
NodeManager
*/

// initialize the node manager
NodeManager::NodeManager(int sensor_count) {
	// setup the message container
	_message = MyMessage();
	// allocate block for all the sensors if sensor_count is provided
	if (sensor_count > 0) sensors.allocateBlocks(sensor_count);
	// setup serial port baud rate
	Serial.begin(MY_BAUD_RATE);
	// print out the version
	debug(PSTR(LOG_INIT "VER=" VERSION "\n"));
	// print out sketch name
	debug(PSTR(LOG_INIT "INO=" SKETCH_NAME " v" SKETCH_VERSION "\n"));
	// print out MySensors' library capabilities
	debug(PSTR(LOG_INIT "LIB VER=" MYSENSORS_LIBRARY_VERSION " CP=" MY_CAPABILITIES " \n"));
}

#if FEATURE_INTERRUPTS == ON
int NodeManager::_last_interrupt_pin = -1;
int NodeManager::_last_interrupt_value = LOW;
long unsigned NodeManager::_last_interrupt_millis = millis();
long unsigned NodeManager::_interrupt_debounce = 0;
#endif

// setter/getter
void NodeManager::setRetries(int value) {
	_retries = value;
}
int NodeManager::getRetries() {
	return _retries;
}
#if FEATURE_SLEEP == ON
void NodeManager::setSleepSeconds(int value) {
	// set the status to AWAKE if the time provided is 0, SLEEP otherwise
	if (value == 0) _status = AWAKE;
	else _status = SLEEP;
	// store the time
	_sleep_time = value;
#if FEATURE_EEPROM == ON
	// save sleep settings to eeprom
	if (_save_sleep_settings) _saveSleepSettings();
#endif
}
void NodeManager::setSleepMinutes(int value) {
	setSleepSeconds(value*60);
}
void NodeManager::setSleepHours(int value) {
	setSleepMinutes(value*60);
}
void NodeManager::setSleepDays(int value) {
	setSleepHours(value*24);
}
long NodeManager::getSleepSeconds() {
	return _sleep_time;
}
void NodeManager::setSleepBetweenSend(int value) {
	_sleep_between_send = value;
}
#endif
#if FEATURE_INTERRUPTS == ON
void NodeManager::setSleepInterruptPin(int value) {
	_sleep_interrupt_pin = value;
}
void NodeManager::setInterrupt(int pin, int mode, int initial) {
	if (pin == INTERRUPT_PIN_1) {
		_interrupt_1_mode = mode;
		_interrupt_1_initial = initial;
	}
	if (pin == INTERRUPT_PIN_2) { 
		_interrupt_2_mode = mode;
		_interrupt_2_initial = initial;
	}
}
void NodeManager::setInterruptDebounce(long value) {
	_interrupt_debounce = value;
}
#endif
#if FEATURE_POWER_MANAGER == ON
void NodeManager::setPowerPins(int ground_pin, int vcc_pin, int wait_time) {
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
#if FEATURE_EEPROM == ON
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

// setup NodeManager
void NodeManager::before() {
	// setup the reboot pin if needed
	if (_reboot_pin > -1) {
		debug(PSTR(LOG_INIT "RBT p=%d\n"),_reboot_pin);
		pinMode(_reboot_pin, OUTPUT);
		digitalWrite(_reboot_pin, HIGH);
	}
#if FEATURE_EEPROM == ON
	// restore the sleep settings saved in the eeprom
	if (_save_sleep_settings) _loadSleepSettings();
#endif
	debug(PSTR(LOG_BEFORE "INIT\n"));
}

// present NodeManager and its sensors
void NodeManager::presentation() {
	debug(PSTR(LOG_BEFORE "OK\n"));
	// Send the sketch version information to the gateway and Controller
	_sleepBetweenSend();
	sendSketchInfo(SKETCH_NAME,SKETCH_VERSION);
	_sleepBetweenSend();
	// present each sensor
	for (List<Sensor*>::iterator itr = sensors.begin(); itr != sensors.end(); ++itr) {
		Sensor* sensor = *itr;
		// call each sensor's presentation()
		sensor->presentation();
		_sleepBetweenSend();
	}
	// wait a bit before leaving this function
	_sleepBetweenSend();
	_sleepBetweenSend();
	_sleepBetweenSend();
}


// setup NodeManager
void NodeManager::setup() {
	// retrieve and store isMetric from the controller
	if (_get_controller_config) _is_metric = getControllerConfig().isMetric;
	debug(PSTR(LOG_SETUP "ID=%d M=%d\n"),getNodeId(),_is_metric);
#if FEATURE_TIME == ON
	// sync the time with the controller
	syncTime();
#endif
#if FEATURE_SD == ON
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
#if FEATURE_INTERRUPTS == ON
	// setup the interrupt pins
	setupInterrupts();
#endif
}

// run the main function for all the register sensors
void NodeManager::loop() {
#if FEATURE_TIME == ON
	// if the time was last updated more than 60 minutes ago, update it
	if (_time_is_valid && (now() - _time_last_sync) > 60*60) syncTime();
#endif
	// turn on the pin powering all the sensors
#if FEATURE_POWER_MANAGER == ON
	powerOn();
#endif
	// run loop for all the registered sensors
	for (List<Sensor*>::iterator itr = sensors.begin(); itr != sensors.end(); ++itr) {
		Sensor* sensor = *itr;
#if FEATURE_INTERRUPTS == ON
		if (_last_interrupt_pin != -1 && sensor->getInterruptPin() == _last_interrupt_pin) {
			// if there was an interrupt for this sensor, call the sensor's interrupt()
			_message.clear();
			// call the sensor loop, provided the interrupt has been "accepted" by interrupt()
			if (sensor->interrupt()) sensor->loop(nullptr);
			// reset the last interrupt pin
			_last_interrupt_pin = -1;
		}
		else if (_last_interrupt_pin == -1) {
#else
			if (true) {
#endif
				// if just at the end of a cycle, call the sensor's loop() 
				_message.clear();
				sensor->loop(nullptr);
			}
		}
		// turn off the pin powering all the sensors
#if FEATURE_POWER_MANAGER == ON
		powerOff();
#endif
#if FEATURE_SLEEP == ON
		// continue/start sleeping as requested
		if (isSleepingNode()) _sleep();
#endif
	}

#if FEATURE_RECEIVE == ON
	// dispacth inbound messages
	void NodeManager::receive(const MyMessage &message) {
		debug(PSTR(LOG_MSG "RECV(%d) c=%d t=%d p=%s\n"),message.sensor,message.getCommand(),message.type,message.getString());
		// dispatch the message to the registered sensor
		Sensor* sensor = getSensorWithChild(message.sensor);
		if (sensor != nullptr) {
			// turn on the pin powering all the sensors
			#if FEATURE_POWER_MANAGER == ON
			powerOn();
			#endif
			// call the sensor's receive()
			sensor->receive((MyMessage*) &message);
			// turn off the pin powering all the sensors
			#if FEATURE_POWER_MANAGER == ON
			powerOff();
			#endif
		}
	}
#endif

#if FEATURE_TIME == ON
	// receive the time from the controller and save it
	void NodeManager::receiveTime(unsigned long ts) {
		debug(PSTR(LOG_TIME "OK ts=%d\n"),ts);
		// time is now valid
		_time_is_valid = true;
#if FEATURE_RTC == ON
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
		if (_reboot_pin > -1) {
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

#if FEATURE_EEPROM == ON
	// clear the EEPROM
	void NodeManager::clearEeprom() {
		debug(PSTR(LOG_EEPROM "CLR\n"));
		for (uint16_t i=0; i<EEPROM_LOCAL_CONFIG_ADDRESS; i++) saveState(i, 0xFF);
	}

	// return the value stored at the requested index from the EEPROM
	int NodeManager::loadFromMemory(int index) {
		return loadState(index+EEPROM_USER_START);
	}

	// save the given index of the EEPROM the provided value
	void NodeManager::saveToMemory(int index, int value) {
		saveState(index+EEPROM_USER_START, value);
	}
#endif

#if FEATURE_SLEEP == ON
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

	// return vcc in V
	float NodeManager::getVcc() {
#ifdef CHIP_AVR
		// Measure Vcc against 1.1V Vref
		#if defined(CHIP_MEGA)
		ADMUX = (_BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1));
		#elif defined (CHIP_TINYX4)
		ADMUX = (_BV(MUX5) | _BV(MUX0));
		#elif defined (CHIP_TINYX5)
		ADMUX = (_BV(MUX3) | _BV(MUX2));
		#else
		ADMUX = (_BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1));
		#endif
		// Vref settle
		wait(70);
		// Do conversion
		ADCSRA |= _BV(ADSC);
		while (bit_is_set(ADCSRA, ADSC)) {};
		// return Vcc in mV
		return (float)((1125300UL) / ADC) / 1000;
#else
		return (float)0;
#endif
	}

#if FEATURE_INTERRUPTS == ON
	// setup the interrupt pins
	void NodeManager::setupInterrupts() {
		// configure wakeup pin if needed
		if (_sleep_interrupt_pin > -1) {
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

	// return the pin from which the last interrupt came
	int NodeManager::getLastInterruptPin() {
		return _last_interrupt_pin;
	}

	// return the value of the pin from which the last interrupt came
	int NodeManager::getLastInterruptValue() {
		return _last_interrupt_value;
	}
#endif

	// set the default interval in seconds all the sensors will report their measures
	void NodeManager::setReportIntervalSeconds(int value) {
		_report_interval_seconds = value;
	}
	int NodeManager::getReportIntervalSeconds() {
		return _report_interval_seconds;
	}
	
	// set the default interval in minutes all the sensors will report their measures
	void NodeManager::setReportIntervalMinutes(int value) {
		_report_interval_seconds = value*60;
	}

	// set the default interval in hours all the sensors will report their measures
	void NodeManager::setReportIntervalHours(int value) {
		_report_interval_seconds = value*60*60;
	}

	// set the default interval in days all the sensors will report their measures
	void NodeManager::setReportIntervalDays(int value) {
		_report_interval_seconds = value*60*60*24;
	}

	// if set and when the board is battery powered, sleep() is always called instead of wait()
	void NodeManager::setSleepOrWait(bool value) {
		_sleep_or_wait = value;
	}

	// set which pin is connected to RST of the board to reboot the board when requested. If not set the software reboot is used instead (default: -1)
	void NodeManager::setRebootPin(int value) {
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
	void NodeManager::sleepOrWait(long value) {
		// if the node is sleeping, sleep-or-wait is enabled and we need to sleep for a decent amount of time, call sleep() otherwise wait()
		if (isSleepingNode() && _sleep_or_wait && value > 200) sleep(value);
		else wait(value);
	}

	// return the next available child_id
	int NodeManager::getAvailableChildId(int child_id) {
		if (child_id > -1) return child_id;
		for (int i = 1; i < 255; i++) {
			if (i == CONFIGURATION_CHILD_ID || i == BATTERY_CHILD_ID || i == SIGNAL_CHILD_ID) continue;
			Child* child = getChild(i);
			if (child == nullptr) return i;
		}
		return 254;
	}

#if FEATURE_INTERRUPTS == ON
	// handle an interrupt
	void NodeManager::_onInterrupt_1() {
		_saveInterrupt(INTERRUPT_PIN_1);
	}
	
	void NodeManager::_onInterrupt_2() {
		_saveInterrupt(INTERRUPT_PIN_2);
	}
	
	// keep track of the last interrupt pin and value
	void NodeManager::_saveInterrupt(int pin) {
		// ingore the interrupt if triggering too close to the previous one
		if (_interrupt_debounce > 0 &&  (millis() - _last_interrupt_millis < _interrupt_debounce) && pin == _last_interrupt_pin && millis() > _last_interrupt_millis) return;
		// save pin and value
		_last_interrupt_pin = pin;
		_last_interrupt_value = digitalRead(pin);
		debug(PSTR(LOG_LOOP "INT p=%d v=%d\n"),_last_interrupt_pin,_last_interrupt_value);
	}
#endif

	// send a message by providing the source child, type of the message and value
	void NodeManager::sendMessage(int child_id, int type, int value) {
		_message.clear();
		_message.set(value);
		_sendMessage(child_id,type);
	}
	void NodeManager::sendMessage(int child_id, int type, float value, int precision) {
		_message.clear();
		_message.set(value,precision);
		_sendMessage(child_id,type);
	}
	void NodeManager::sendMessage(int child_id, int type, double value, int precision) {
		_message.clear();
		_message.set(value,precision);
		_sendMessage(child_id,type);
	}
	void NodeManager::sendMessage(int child_id, int type, const char* value) {
		_message.clear();
		_message.set(value);
		_sendMessage(child_id,type);
	}

	// send a message to the network
	void NodeManager::_sendMessage(int child_id, int type) {
		// prepare the message
		_message.setSensor(child_id);
		_message.setType(type);
		// send the message, multiple times if requested
		for (int i = 0; i < _retries; i++) {
			if (mGetPayloadType(_message) == P_INT16) debug(PSTR(LOG_MSG "SEND(%d) t=%d p=%d\n"),_message.sensor,_message.type,_message.getInt());
			if (mGetPayloadType(_message) == P_FLOAT32) debug(PSTR(LOG_MSG "SEND(%d) t=%d p=%d.%02d\n"),_message.sensor,_message.type,(int)_message.getFloat(), (int)(_message.getFloat()*100)%100);
			if (mGetPayloadType(_message) == P_STRING) debug(PSTR(LOG_MSG "SEND(%d) t=%d p=%s\n"),_message.sensor,_message.type,_message.getString());
			send(_message, _ack);
			// if configured, sleep beetween each send
			_sleepBetweenSend();
		}
	}

#if FEATURE_POWER_MANAGER == ON
	void NodeManager::setPowerManager(PowerManager& powerManager) {
		_powerManager = &powerManager;
	}
#endif

	// return the requested child 
	Child* NodeManager::getChild(int child_id) {
		Sensor* sensor = getSensorWithChild(child_id);
		if (sensor == nullptr) return nullptr;
		return sensor->getChild(child_id);
	}

	// return the sensor with the requested child 
	Sensor* NodeManager::getSensorWithChild(int child_id) {
		for (List<Sensor*>::iterator itr = sensors.begin(); itr != sensors.end(); ++itr) {
			Sensor* sensor = *itr;
			Child* child = sensor->getChild(child_id);
			if (child != nullptr) return sensor;
		}
		return nullptr;  
	}

#if FEATURE_TIME == ON
	// sync the time with the controller
	void NodeManager::syncTime() {
		_time_is_valid = false;
		int retries = 10;
		// ask the controller for the time up to 10 times until received
		while ( ! _time_is_valid && retries >= 0) {
			debug(PSTR(LOG_TIME "REQ\n"));
			requestTime();
			wait(1000);
			retries = retries - 1;
		}
	}

	// returns the current system time
	long NodeManager::getTime() {
		return now();
	}
#endif

#if FEATURE_SLEEP == ON
	// wrapper of smart sleep
	void NodeManager::_sleep() {
		long sleep_time = _sleep_time;
#if FEATURE_TIME == ON
		// if there is time still to sleep, sleep for that timeframe only
		if (_remainder_sleep_time > 0) sleep_time = _remainder_sleep_time;
#endif
		debug(PSTR(LOG_SLEEP "SLEEP s=%d\n\n"),sleep_time);
		// go to sleep
		int interrupt = -1;
#if FEATURE_INTERRUPTS == ON
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
#if FEATURE_TIME == ON
		// keep track of the old time so to calculate the amount of time slept
		long old_time = now();
#if FEATURE_RTC == ON
		// sync the time with the RTC
		setSyncProvider(RTC.get);
#else
		// sync the time with the controller
		syncTime();
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

#if FEATURE_EEPROM == ON
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
	void NodeManager::_sleepBetweenSend() {
		if (_sleep_between_send > 0) sleep(_sleep_between_send);
	}
