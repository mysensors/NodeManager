/*
 * NodeManager
 */

#include "NodeManager.h"

/***************************************
   PowerManager
*/

// set the vcc and ground pin the sensor is connected to
void PowerManager::setPowerPins(int ground_pin, int vcc_pin, int wait_time) {
  _ground_pin = ground_pin;
  _vcc_pin = vcc_pin;
  #if DEBUG == 1
    Serial.print(F("PWR G="));
    Serial.print(_ground_pin);
    Serial.print(F(" V="));
    Serial.println(_vcc_pin);
  #endif
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
  #if DEBUG == 1
    Serial.print(F("ON P="));
    Serial.println(_vcc_pin);
  #endif
  // power on the sensor by turning high the vcc pin
  digitalWrite(_vcc_pin, HIGH);
  // wait a bit for the device to settle down
  if (_wait > 0) wait(_wait);
}

// turn off the sensor
void PowerManager::powerOff() {
  if (_vcc_pin == -1) return;
  #if DEBUG == 1
    Serial.print(F("OFF P="));
    Serial.println(_vcc_pin);
  #endif
  // power off the sensor by turning low the vcc pin
  digitalWrite(_vcc_pin, LOW);
}

/******************************************
    Timer
*/

Timer::Timer(NodeManager* node_manager) {
  _node_manager = node_manager;
}

// start the timer
void Timer::start(int target, int unit) {
  set(target,unit);
  start();
}
void Timer::start() {
  if (_is_configured) _is_running = true;
}

// stop the timer
void Timer::stop() {
  _is_running = false;
}

// reset the timer
void Timer::reset() {
  // reset the timer
  _elapsed = 0;
  _last_millis = 0;
}

// restart the timer
void Timer::restart() {
  if (! isRunning()) return;
  stop();
  reset();
  // if using millis(), keep track of the current timestamp for calculating the difference
  if (! _node_manager->isSleepingNode()) _last_millis = millis();
  start();
}

// setup the timer
void Timer::set(int target, int unit) {
  reset();
  // save the settings
  _target = target;
  if (unit == MINUTES) _target = _target * 60;
  else if (unit == HOURS) _target = _target * 60 *60;
  else if (unit == DAYS) _target = _target * 60 * 60 *24;
  _is_running = false;
  _is_configured = true;
}

// unset the timer
void Timer::unset() {
  stop();
  _is_configured = true;
}

// update the timer at every cycle
void Timer::update() {
  if (! isRunning()) return;
  if (_node_manager->isSleepingNode()) {
    // millis() is not reliable while sleeping so calculate how long a sleep cycle would last in seconds and update the elapsed time
    _elapsed += _node_manager->getSleepSeconds();
  } else {
    // use millis() to calculate the elapsed time in seconds
    _elapsed = (long)((millis() - _last_millis)/1000);
  }
  _first_run = false;
}

// return true if the time is over
bool Timer::isOver() {
  if (! isRunning()) return false;
  // time has elapsed
  if (_elapsed >= _target) return true;
  // millis has started over
  if (_elapsed < 0 ) return true;
  return false;
}

// return true if the timer is running
bool Timer::isRunning() {
  if (! isConfigured()) return false;
  return _is_running;
}

// return true if the time is configured
bool Timer::isConfigured() {
  return _is_configured;
}

// return true if this is the first time the timer runs
bool Timer::isFirstRun() {
  return _first_run;
}

// return elapsed seconds so far
float Timer::getElapsed() {
  return _elapsed;
}


/******************************************
    Request
*/

Request::Request(const char* string) {
  char str[10];
  char* ptr;
  strcpy(str,string);
  // tokenize the string and split function from value
  strtok_r(str,",",&ptr);
  _function = atoi(str);
  strcpy(_value,ptr);
  #if DEBUG == 1
    Serial.print(F("REQ F="));
    Serial.print(getFunction());
    Serial.print(F(" I="));
    Serial.print(getValueInt());
    Serial.print(F(" F="));
    Serial.print(getValueFloat());
    Serial.print(F(" S="));
    Serial.println(getValueString());
  #endif
}

// return the parsed function
int Request::getFunction() {
  return _function;
}

// return the value as an int
int Request::getValueInt() {
  return atoi(_value);
  
}

// return the value as a float
float Request::getValueFloat() {
  return atof(_value);
}

// return the value as a string
char* Request::getValueString() {
  return _value;
}


/******************************************
    Sensors
*/

/*
   Sensor class
*/
// constructor
Sensor::Sensor(NodeManager* node_manager, int child_id, int pin) {
  _node_manager = node_manager;
  _child_id = child_id;
  _pin = pin;
  _msg = MyMessage(_child_id, _type);
  _msg_service = MyMessage(_child_id, V_CUSTOM);
  _report_timer = new Timer(_node_manager);
  _force_update_timer = new Timer(_node_manager);
}

// setter/getter
void Sensor::setPin(int value) {
  _pin = value;
}
int Sensor::getPin() {
  return _pin;
}
void Sensor::setChildId(int value) {
  _child_id = value;
}
int Sensor::getChildId() {
  return _child_id;
}
void Sensor::setPresentation(int value) {
  _presentation = value;
}
int Sensor::getPresentation() {
  return _presentation;
}
void Sensor::setType(int value) {
  _type = value;
  _msg.setType(_type);
}
int Sensor::getType() {
  return _type;
}
void Sensor::setDescription(char* value) {
  _description = value;
}
void Sensor::setSamples(int value) {
  _samples = value;
}
void Sensor::setSamplesInterval(int value) {
  _samples_interval = value;
}
void Sensor::setTrackLastValue(bool value) {
  _track_last_value = value;
}
void Sensor::setForceUpdateMinutes(int value) {
  _force_update_timer->start(value,MINUTES);
}
void Sensor::setForceUpdateHours(int value) {
  _force_update_timer->start(value,HOURS);
}
void Sensor::setValueType(int value) {
  _value_type = value;
}
int Sensor::getValueType() {
  return _value_type;
}
void Sensor::setFloatPrecision(int value) {
  _float_precision = value;
}
#if POWER_MANAGER == 1
    void Sensor::setPowerPins(int ground_pin, int vcc_pin, int wait_time) {
      _powerManager.setPowerPins(ground_pin, vcc_pin, wait_time);
    }
    void Sensor::setAutoPowerPins(bool value) {
      _auto_power_pins = value;
    }
    void Sensor::powerOn() {
      _powerManager.powerOn();
    }
    void Sensor::powerOff() {
      _powerManager.powerOff();
    }
#endif
int Sensor::getInterruptPin() {
  return _interrupt_pin;
}
int Sensor::getValueInt() {
  return _last_value_int;
}
float Sensor::getValueFloat() {
  return _last_value_float;
}
char* Sensor::getValueString() {
  return _last_value_string;
}

// After how many seconds the sensor will report back its measure
void Sensor::setReportIntervalSeconds(int value) {
  _report_timer->start(value,SECONDS);
}

// After how many minutes the sensor will report back its measure 
void Sensor::setReportIntervalMinutes(int value) {
  _report_timer->start(value,MINUTES);
}

// After how many minutes the sensor will report back its measure 
void Sensor::setReportIntervalHours(int value) {
  _report_timer->start(value,HOURS);
}

// After how many minutes the sensor will report back its measure 
void Sensor::setReportIntervalDays(int value) {
  _report_timer->start(value,DAYS);
}


// return true if the report interval has been already configured
bool Sensor::isReportIntervalConfigured() {
  return _report_timer->isConfigured();
}

// listen for interrupts on the given pin so interrupt() will be called when occurring
void Sensor::setInterrupt(int pin, int mode, int initial) {
  _interrupt_pin = pin;
  _node_manager->setInterrupt(pin,mode,initial);
}

// present the sensor to the gateway and controller
void Sensor::presentation() {
  #if DEBUG == 1
    Serial.print(F("PRES I="));
    Serial.print(_child_id);
    Serial.print(F(" T="));
    Serial.println(_presentation);
  #endif
  present(_child_id, _presentation,_description,_node_manager->getAck());
}

// call the sensor-specific implementation of before
void Sensor::before() {
  if (_pin == -1) return;
  onBefore();
}

// call the sensor-specific implementation of setup
void Sensor::setup() {
  if (_pin == -1) return;
  onSetup();
}

// call the sensor-specific implementation of loop
void Sensor::loop(const MyMessage & message) {
  if (_pin == -1) return;
  // update the timers if within a loop cycle
  if (! _isReceive(message)) {
    if (_report_timer->isRunning()) {
      // store the elapsed time before updating it
      bool first_run = _report_timer->isFirstRun();
      // update the timer
      _report_timer->update();
      // if it is not the time yet to report a new measure, just return (unless the first time)
      if (! _report_timer->isOver() && ! first_run) return;
    }
    if (_force_update_timer->isRunning()) _force_update_timer->update();
  }
  #if POWER_MANAGER == 1
    // turn the sensor on
    if (_auto_power_pins) powerOn();
  #endif
  // for numeric sensor requiring multiple samples, keep track of the total
  float total = 0;
  // collect multiple samples if needed
  for (int i = 0; i < _samples; i++) {
    // call the sensor-specific implementation of the main task which will store the result in the _value variable
    if (_isReceive(message)) {
      // we've been called from receive(), pass the message along
      onReceive(message);
    }
    else {
      // we'be been called from loop()
      onLoop();
    }
    // for integers and floats, keep track of the total
    if (_value_type == TYPE_INTEGER) total += (float)_value_int;
    else if (_value_type == TYPE_FLOAT) total += _value_float;
    // wait between samples
    if (_samples_interval > 0) _node_manager->sleepOrWait(_samples_interval);
  }
  // process the result and send a response back
  if (_value_type == TYPE_INTEGER && total > -1) {
    // if the value is an integer, calculate the average value of the samples
    int avg = (int) (total / _samples);
    // if track last value is disabled or if enabled and the current value is different then the old value, send it back
    if (_isReceive(message) || _isWorthSending(avg != _last_value_int))  {
      _last_value_int = avg;
      _send(_msg.set(avg));
      _value_int = -1;
    }
  }
  // process a float value
  else if (_value_type == TYPE_FLOAT && total > -1) {
    // calculate the average value of the samples
    float avg = total / _samples;
    // report the value back
    if (_isReceive(message) || _isWorthSending(avg != _last_value_float))  {
      _last_value_float = avg;
      _send(_msg.set(avg, _float_precision));
      _value_float = -1;
    }
  }
  // process a string value
  else if (_value_type == TYPE_STRING) {
    // if track last value is disabled or if enabled and the current value is different then the old value, send it back
    if (_isReceive(message) || _isWorthSending(strcmp(_value_string, _last_value_string) != 0))  {
      _last_value_string = _value_string;
      _send(_msg.set(_value_string));
      _value_string = "";
    }
  }
  // turn the sensor off
  #if POWER_MANAGER == 1
    if (_auto_power_pins) powerOff();
  #endif
  // restart the report timer if over
  if (! _isReceive(message) && _report_timer->isRunning() && _report_timer->isOver()) _report_timer->restart();
}

// receive and handle an interrupt
void Sensor::interrupt() {
  // call the implementation of onInterrupt()
  onInterrupt();
}

// receive a message from the radio network
void Sensor::receive(const MyMessage &message) {
  // return if not for this sensor
  if (message.sensor != _child_id) return;
  // check if it is a request for the API
  if (message.getCommand() == C_REQ && message.type == V_CUSTOM) {
    #if REMOTE_CONFIGURATION == 1
      // parse the request
      Request request = Request(message.getString());
      // if it is for a sensor-generic function, call process(), otherwise the sensor-specific onProcess();
      if (request.getFunction() < 100) process(request);
      else onProcess(request);
    #endif
  }
  // return if the type is not correct
  if (message.type != _type) return;
  // a request would make the sensor executing its main task passing along the message
  loop(message);
}

// process a remote configuration request message
void Sensor::process(Request & request) {
  int function = request.getFunction();
  switch(function) {
    case 1: setPin(request.getValueInt()); break;
    case 2: setChildId(request.getValueInt()); break;
    case 3: setType(request.getValueInt()); break;
    case 4: setDescription(request.getValueString()); break;
    case 5: setSamples(request.getValueInt()); break;
    case 6: setSamplesInterval(request.getValueInt()); break;
    case 7: setTrackLastValue(request.getValueInt()); break;
    case 9: setForceUpdateMinutes(request.getValueInt()); break;
    case 10: setValueType(request.getValueInt()); break;
    case 11: setFloatPrecision(request.getValueInt()); break;
    #if POWER_MANAGER == 1
      case 12: setAutoPowerPins(request.getValueInt()); break;
      case 13: powerOn(); break;
      case 14: powerOff(); break;
    #endif
    case 16: setReportIntervalMinutes(request.getValueInt()); break;
    case 17: setReportIntervalSeconds(request.getValueInt()); break;
    case 19: setReportIntervalHours(request.getValueInt()); break;
    case 20: setReportIntervalDays(request.getValueInt()); break;
    case 18: setForceUpdateHours(request.getValueInt()); break;
    default: return;
  }
  _send(_msg_service.set(function));
}

// send a message to the network
void Sensor::_send(MyMessage & message) {
  // send the message, multiple times if requested
  for (int i = 0; i < _node_manager->getRetries(); i++) {
    // if configured, sleep beetween each send
    if (_node_manager->getSleepBetweenSend() > 0) sleep(_node_manager->getSleepBetweenSend());
    #if DEBUG == 1
      Serial.print(F("SEND D="));
      Serial.print(message.destination);
      Serial.print(F(" I="));
      Serial.print(message.sensor);
      Serial.print(F(" C="));
      Serial.print(message.getCommand());
      Serial.print(F(" T="));
      Serial.print(message.type);
      Serial.print(F(" S="));
      Serial.print(message.getString());
      Serial.print(F(" I="));
      Serial.print(message.getInt());
      Serial.print(F(" F="));
      Serial.println(message.getFloat());
    #endif
    send(message,_node_manager->getAck());
  }
}

// return true if the message is coming from the radio network
bool Sensor::_isReceive(const MyMessage & message) {
  if (message.sender == 0 && message.sensor == 0 && message.getCommand() == 0 && message.type == 0) return false;
  return true;
}

// determine if a value is worth sending back to the controller
bool Sensor::_isWorthSending(bool comparison) {
  // track last value is disabled
  if (! _track_last_value) return true;
  // track value is enabled and the current value is different then the old value
  if (_track_last_value && comparison) return true;
  // track value is enabled and the timer is over
  if (_track_last_value && _force_update_timer->isRunning() && _force_update_timer->isOver()) {
    // restart the timer
    _force_update_timer->restart();
    return true;
  }
  return false;
}

#if MODULE_ANALOG_INPUT == 1
/*
   SensorAnalogInput
*/

// contructor
SensorAnalogInput::SensorAnalogInput(NodeManager* node_manager, int child_id, int pin): Sensor(node_manager, child_id, pin) {
}

// setter/getter
void SensorAnalogInput::setReference(int value) {
  _reference = value;
}
void SensorAnalogInput::setReverse(bool value) {
  _reverse = value;
}
void SensorAnalogInput::setOutputPercentage(bool value) {
  _output_percentage = value;
}
void SensorAnalogInput::setRangeMin(int value) {
  _range_min = value;
}
void SensorAnalogInput::setRangeMax(int value) {
  _range_max = value;
}

// what to do during before
void SensorAnalogInput::onBefore() {
  // prepare the pin for input
  pinMode(_pin, INPUT);
}

// what to do during setup
void SensorAnalogInput::onSetup() {
}

// what to do during loop
void SensorAnalogInput::onLoop() {
  // read the input
  int adc = _getAnalogRead();
  // calculate the percentage
  int percentage = 0;
  if (_output_percentage) percentage = _getPercentage(adc);
  #if DEBUG == 1
    Serial.print(F("A-IN I="));
    Serial.print(_child_id);
    Serial.print(F(" V="));
    Serial.print(adc);
    Serial.print(F(" %="));
    Serial.println(percentage);
  #endif
  // store the result
  _value_int = _output_percentage ? percentage : adc;
}

// what to do during loop
void SensorAnalogInput::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_REQ) onLoop();
}

// what to do when receiving a remote message
void SensorAnalogInput::onProcess(Request & request) {
  int function = request.getFunction();
  switch(function) {
    case 101: setReference(request.getValueInt()); break;
    case 102: setReverse(request.getValueInt()); break;
    case 103: setOutputPercentage(request.getValueInt()); break;
    case 104: setRangeMin(request.getValueInt()); break;
    case 105: setRangeMax(request.getValueInt()); break;
    default: return;
  }
  _send(_msg_service.set(function));
}

// what to do when receiving an interrupt
void SensorAnalogInput::onInterrupt() {
}

// read the analog input
int SensorAnalogInput::_getAnalogRead() {
  #ifndef MY_GATEWAY_ESP8266
    // set the reference
    if (_reference != -1) {
      analogReference(_reference);
      wait(100);
    }
  #endif
  // read and return the value
  int value = analogRead(_pin);
  if (_reverse) value = _range_max - value;
  return value;
}

// return a percentage from an analog value
int SensorAnalogInput::_getPercentage(int adc) {
  float value = (float)adc;
  // restore the original value
  if (_reverse) value = 1024 - value;
  // scale the percentage based on the range provided
  float percentage = ((value - _range_min) / (_range_max - _range_min)) * 100;
  if (_reverse) percentage = 100 - percentage;
  if (percentage > 100) percentage = 100;
  if (percentage < 0) percentage = 0;
  return (int)percentage;
}

/*
   SensorLDR
*/

// contructor
SensorLDR::SensorLDR(NodeManager* node_manager, int child_id, int pin): SensorAnalogInput(node_manager, child_id, pin) {
  // set presentation and type and reverse (0: no light, 100: max light)
  setPresentation(S_LIGHT_LEVEL);
  setType(V_LIGHT_LEVEL);
  setReverse(true);
}

/*
   SensorThermistor
*/

// contructor
SensorThermistor::SensorThermistor(NodeManager* node_manager, int child_id, int pin): Sensor(node_manager, child_id, pin) {
  // set presentation, type and value type
  setPresentation(S_TEMP);
  setType(V_TEMP);
  setValueType(TYPE_FLOAT);
}

// setter/getter
void SensorThermistor::setNominalResistor(long value) {
  _nominal_resistor = value;
}
void SensorThermistor::setNominalTemperature(int value) {
  _nominal_temperature = value;
}
void SensorThermistor::setBCoefficient(int value) {
  _b_coefficient = value;
}
void SensorThermistor::setSeriesResistor(long value) {
  _series_resistor = value;
}
void SensorThermistor::setOffset(float value) {
  _offset = value;
}

// what to do during before
void SensorThermistor::onBefore() {
  // set the pin as input
  pinMode(_pin, INPUT);
}

// what to do during setup
void SensorThermistor::onSetup() {
}

// what to do during loop
void SensorThermistor::onLoop() {
  // read the voltage across the thermistor
  float adc = analogRead(_pin);
  // calculate the temperature
  float reading = (1023 / adc)  - 1;
  reading = _series_resistor / reading;
  float temperature;
  temperature = reading / _nominal_resistor;     // (R/Ro)
  temperature = log(temperature);                  // ln(R/Ro)
  temperature /= _b_coefficient;                   // 1/B * ln(R/Ro)
  temperature += 1.0 / (_nominal_temperature + 273.15); // + (1/To)
  temperature = 1.0 / temperature;                 // Invert
  temperature -= 273.15;                         // convert to C
  temperature = _node_manager->celsiusToFahrenheit(temperature);
  #if DEBUG == 1
    Serial.print(F("THER I="));
    Serial.print(_child_id);
    Serial.print(F(" V="));
    Serial.print(adc);
    Serial.print(F(" T="));
    Serial.println(temperature);
  #endif
  // store the value
  _value_float = temperature;
}

// what to do as the main task when receiving a message
void SensorThermistor::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_REQ) onLoop();
}

// what to do when receiving a remote message
void SensorThermistor::onProcess(Request & request) {
  int function = request.getFunction();
  switch(function) {
    case 101: setNominalResistor((long)request.getValueInt()); break;
    case 102: setNominalTemperature(request.getValueInt()); break;
    case 103: setBCoefficient(request.getValueInt()); break;
    case 104: setSeriesResistor((long)request.getValueString()); break;
    case 105: setOffset(request.getValueFloat()); break;
    default: return;
  }
  _send(_msg_service.set(function));
}

// what to do when receiving an interrupt
void SensorThermistor::onInterrupt() {
}

/*
   SensorML8511
*/

// contructor
SensorML8511::SensorML8511(NodeManager* node_manager, int child_id, int pin): Sensor(node_manager, child_id, pin) {
  // set presentation, type and value type
  setPresentation(S_UV);
  setType(V_UV);
  setValueType(TYPE_FLOAT);
}

// what to do during before
void SensorML8511::onBefore() {
  // set the pin as input
  pinMode(_pin, INPUT);
}

// what to do during setup
void SensorML8511::onSetup() {
}

// what to do during loop
void SensorML8511::onLoop() {
  // read the voltage 
  int uvLevel = analogRead(_pin);
  int refLevel = _node_manager->getVcc()*1024/3.3;
  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  //Convert the voltage to a UV intensity level
  float uvIntensity = _mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); 
  #if DEBUG == 1
    Serial.print(F("UV I="));
    Serial.print(_child_id);
    Serial.print(F(" V="));
    Serial.print(outputVoltage);
    Serial.print(F(" I="));
    Serial.println(uvIntensity);
  #endif
  // store the value
  _value_float = uvIntensity;
}

// what to do as the main task when receiving a message
void SensorML8511::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_REQ) onLoop();
}

// what to do when receiving a remote message
void SensorML8511::onProcess(Request & request) {
}

// what to do when receiving an interrupt
void SensorML8511::onInterrupt() {
}

// The Arduino Map function but for floats
float SensorML8511::_mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
   SensorACS712
*/

// contructor
SensorACS712::SensorACS712(NodeManager* node_manager, int child_id, int pin): Sensor(node_manager, child_id, pin) {
  // set presentation, type and value type
  setPresentation(S_MULTIMETER);
  setType(V_CURRENT);
  setValueType(TYPE_FLOAT);
}

// setter/getter
void SensorACS712::setmVPerAmp(int value) {
  _mv_per_amp = value;
}
void SensorACS712::setOffset(int value) {
  _ACS_offset = value;
}

// what to do during before
void SensorACS712::onBefore() {
  // set the pin as input
  pinMode(_pin, INPUT);
}

// what to do during setup
void SensorACS712::onSetup() {
}

// what to do during loop
void SensorACS712::onLoop() {
  int value = analogRead(_pin);
  // convert the analog read in mV
  double voltage = (value / 1024.0) * 5000; 
  // convert voltage in amps
  _value_float = ((voltage - _ACS_offset) / _mv_per_amp);
  #if DEBUG == 1
    Serial.print(F("ACS I="));
    Serial.print(_child_id);
    Serial.print(F(" A="));
    Serial.println(_value_float);
  #endif
}

// what to do as the main task when receiving a message
void SensorACS712::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_REQ) onLoop();
}

// what to do when receiving a remote message
void SensorACS712::onProcess(Request & request) {
  int function = request.getFunction();
  switch(function) {
    case 100: setmVPerAmp(request.getValueInt()); break;
    case 102: setOffset(request.getValueInt()); break;
    default: return;
  }
  _send(_msg_service.set(function));
}

// what to do when receiving an interrupt
void SensorACS712::onInterrupt() {
}

/*
   SensorRainGauge
*/

// contructor
SensorRainGauge::SensorRainGauge(NodeManager* node_manager, int child_id, int pin): Sensor(node_manager,child_id, pin) {
  // set presentation, type and value type
  setPresentation(S_RAIN);
  setType(V_RAIN);
  setValueType(TYPE_FLOAT);
}

// setter/getter
void SensorRainGauge::setSingleTip(float value) {
  _single_tip = value;
}
void SensorRainGauge::setInitialValue(int value) {
  _initial_value = value;
}

// what to do during before
void SensorRainGauge::onBefore() {
  // configure the interrupt pin so onInterrupt() will be called on tip
  setInterrupt(_pin,FALLING,_initial_value);
}

// what to do during setup
void SensorRainGauge::onSetup() {
}

// what to do during loop
void SensorRainGauge::onLoop() {
  // do not execute loop if called by an interrupt
  if (_node_manager->getLastInterruptPin() == _interrupt_pin) return;
  // time to report the rain so far
  _value_float = _count * _single_tip;
  #if DEBUG == 1
    Serial.print(F("RAIN I="));
    Serial.print(_child_id);
    Serial.print(F(" T="));
    Serial.println(_value_float);
  #endif
  // reset the counter
  _count = 0;
}

// what to do as the main task when receiving a message
void SensorRainGauge::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_REQ) {
    // report the total amount of rain for the last period
    _value_float = _count * _single_tip;
  }
}

// what to do when receiving a remote message
void SensorRainGauge::onProcess(Request & request) {
  int function = request.getFunction();
  switch(function) {
    case 102: setSingleTip(request.getValueFloat()); break;
    default: return;
  }
  _send(_msg_service.set(function));
}

// what to do when receiving an interrupt
void SensorRainGauge::onInterrupt() {
  // increase the counter
  _count++;
  #if DEBUG == 1
    Serial.println(F("RAIN+"));
  #endif
}

/*
   SensorRain
*/

// contructor
SensorRain::SensorRain(NodeManager* node_manager, int child_id, int pin): SensorAnalogInput(node_manager,child_id, pin) {
  // set presentation and type and reverse
  setPresentation(S_RAIN);
  setType(V_RAINRATE);
  setReference(DEFAULT);
  setOutputPercentage(true);
  setReverse(true);
  setRangeMin(100);
}

/*
   SensorSoilMoisture
*/

// contructor
SensorSoilMoisture::SensorSoilMoisture(NodeManager* node_manager, int child_id, int pin): SensorAnalogInput(node_manager, child_id, pin) {
  // set presentation and type and reverse
  setPresentation(S_MOISTURE);
  setType(V_LEVEL);
  setReference(DEFAULT);
  setOutputPercentage(true);
  setReverse(true);
  setRangeMin(100);
}

#endif

#if MODULE_DIGITAL_INPUT == 1
/*
   SensorDigitalInput
*/

// contructor
SensorDigitalInput::SensorDigitalInput(NodeManager* node_manager, int child_id, int pin): Sensor(node_manager,child_id, pin) {
}

// what to do during before
void SensorDigitalInput::onBefore() {
  // set the pin for input
  pinMode(_pin, INPUT);
}

// what to do during setup
void SensorDigitalInput::onSetup() {
}

// what to do during loop
void SensorDigitalInput::onLoop() {
  // read the value
  int value = digitalRead(_pin);
  #if DEBUG == 1
    Serial.print(F("D-IN I="));
    Serial.print(_child_id);
    Serial.print(F(" P="));
    Serial.print(_pin);
    Serial.print(F(" V="));
    Serial.println(value);
  #endif
  // store the value
  _value_int = value;
}

// what to do as the main task when receiving a message
void SensorDigitalInput::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_REQ) onLoop();
}

// what to do when receiving a remote message
void SensorDigitalInput::onProcess(Request & request) {
}

// what to do when receiving an interrupt
void SensorDigitalInput::onInterrupt() {
}
#endif


#if MODULE_DIGITAL_OUTPUT == 1
/*
   SensorDigitalOutput
*/

SensorDigitalOutput::SensorDigitalOutput(NodeManager* node_manager, int child_id, int pin): Sensor(node_manager,child_id, pin) {
  _safeguard_timer = new Timer(node_manager);
}

// what to do during before
void SensorDigitalOutput::onBefore() {
  _setupPin(_pin);

}

// what to do during setup
void SensorDigitalOutput::onSetup() {
}

// setter/getter
void SensorDigitalOutput::setOnValue(int value) {
  _on_value = value;
}
void SensorDigitalOutput::setLegacyMode(bool value) {
  _legacy_mode = value;
}
void SensorDigitalOutput::setSafeguard(int value) {
  _safeguard_timer->set(value,MINUTES);
}
int SensorDigitalOutput::getStatus() {
  return _status;
}
void SensorDigitalOutput::setInputIsElapsed(bool value) {
  _input_is_elapsed = value;
}
void SensorDigitalOutput::setWaitAfterSet(int value) {
  _wait_after_set = value;
}

// main task
void SensorDigitalOutput::onLoop() {
  // set the value to -1 so to avoid reporting to the gateway during loop
  _value_int = -1;
  _last_value_int = -1;
  // if a safeguard is set, check if it is time for it
  if (_safeguard_timer->isRunning()) {
    // update the timer
    _safeguard_timer->update();
    // if the time is over, turn the output off
    if (_safeguard_timer->isOver()) setStatus(OFF);
  }
}

// what to do as the main task when receiving a message
void SensorDigitalOutput::onReceive(const MyMessage & message) {
  // by default handle a SET message but when legacy mode is set when a REQ message is expected instead
  if ( (message.getCommand() == C_SET && ! _legacy_mode) || (message.getCommand() == C_REQ && _legacy_mode)) {
    // switch the output
    setStatus(message.getInt());
  }
  if (message.getCommand() == C_REQ && ! _legacy_mode) {
    // return the current status
    _value_int = _status;
  }
}

// what to do when receiving a remote message
void SensorDigitalOutput::onProcess(Request & request) {
  int function = request.getFunction();
  switch(function) {
    case 103: setOnValue(request.getValueInt()); break;
    case 104: setLegacyMode(request.getValueInt()); break;
    case 105: setSafeguard(request.getValueInt()); break;
    case 106: setInputIsElapsed(request.getValueInt()); break;
    case 107: setWaitAfterSet(request.getValueInt()); break;
    default: return;
  }
  _send(_msg_service.set(function));
}

// what to do when receiving an interrupt
void SensorDigitalOutput::onInterrupt() {
}

// write the value to the output
void SensorDigitalOutput::setStatus(int value) {
  // pre-process the input value
  if (_input_is_elapsed) {
    // the input provided is an elapsed time
    if (value == OFF) {
      // turning it off, no need for a safeguard anymore, stop the timer
      _safeguard_timer->stop();
    } 
    else if (value == ON) {
      // configure and start the timer
      _safeguard_timer->start(value,MINUTES);
      // if the input is an elapsed time, unless the value is OFF, the output will be always ON
      value = ON;
    }
  } else {
    // if turning the output on and a safeguard timer is configured, start it
    if (value == ON && _safeguard_timer->isConfigured() && ! _safeguard_timer->isRunning()) _safeguard_timer->start();
  }
  _setStatus(value);
  // wait if needed for relay drawing a lot of current
  if (_wait_after_set > 0) _node_manager->sleepOrWait(_wait_after_set);
  // store the new status so it will be sent to the controller
  _status = value;
  _value_int = value;
}

// setup the provided pin for output
void SensorDigitalOutput::_setupPin(int pin) {
  // set the pin as output and initialize it accordingly
  pinMode(pin, OUTPUT);
  // setup the pin in a off status
  _status = ! _on_value;
  digitalWrite(pin, _status);
  // the initial value is now the current value
  _value_int = _status;
}

// switch to the requested status
void SensorDigitalOutput::_setStatus(int value) {
  int value_to_write = _getValueToWrite(value);
  // set the value to the pin
  digitalWrite(_pin, value_to_write);
  #if DEBUG == 1
    Serial.print(F("DOUT I="));
    Serial.print(_child_id);
    Serial.print(F(" P="));
    Serial.print(_pin);
    Serial.print(F(" V="));
    Serial.println(value_to_write);
  #endif
}

// reverse the value if needed based on the _on_value
int SensorDigitalOutput::_getValueToWrite(int value) {
  int value_to_write = value;
  if (_on_value == LOW) {
    // if the "on" value is LOW, reverse the value
    if (value == ON) value_to_write = LOW;
    if (value == OFF) value_to_write = HIGH;
  }
  return value_to_write;
}


/*
   SensorRelay
*/

// contructor
SensorRelay::SensorRelay(NodeManager* node_manager, int child_id, int pin): SensorDigitalOutput(node_manager, child_id, pin) {
  // set presentation and type
  setPresentation(S_BINARY);
  setType(V_STATUS);
}

/*
   SensorLatchingRelay
*/

// contructor
SensorLatchingRelay::SensorLatchingRelay(NodeManager* node_manager, int child_id, int pin): SensorRelay(node_manager, child_id, pin) {
  // set the "off" pin to the provided pin
  setPinOff(pin);
  // set the "on" pin to the provided pin + 1
  setPinOn(pin + 1);
}

// setter/getter
void SensorLatchingRelay::setPulseWidth(int value) {
  _pulse_width = value;
}
void SensorLatchingRelay::setPinOn(int value) {
  _pin_on = value;
}
void SensorLatchingRelay::setPinOff(int value) {
  _pin_off = value;
}

// what to do during before
void SensorLatchingRelay::onBefore() {
  _setupPin(_pin_on);
  _setupPin(_pin_off);
}

// what to do when receiving a remote message
void SensorLatchingRelay::onProcess(Request & request) {
  int function = request.getFunction();
  if (function < 200) {
    // if this is for SensorDigitalOutput call its onProcess()
    SensorDigitalOutput::onProcess(request);
    return;
  }
  switch(function) {
    case 201: setPulseWidth(request.getValueInt()); break;
    case 202: setPinOff(request.getValueInt()); break;
    case 203: setPinOn(request.getValueInt()); break;
    default: return;
  }
  _send(_msg_service.set(function));
}

// switch to the requested status
void SensorLatchingRelay::_setStatus(int value) {
  // select the right pin to send the pulse to
  int pin = value == OFF ? _pin_off : _pin_on;
  // set the value
  digitalWrite(pin, _on_value);
  // wait for the given time before restoring the value to the original value after the pulse
  _node_manager->sleepOrWait(_pulse_width);
  digitalWrite(pin, ! _on_value);
  #if DEBUG == 1
    Serial.print(F("LAT I="));
    Serial.print(_child_id);
    Serial.print(F(" P="));
    Serial.print(pin);
    Serial.print(F(" S="));
    Serial.print(value);
    Serial.print(F(" V="));
    Serial.print(_on_value);
    Serial.print(F(" P="));
    Serial.println(_pulse_width);
  #endif
}

#endif
/*
   SensorDHT
*/
#if MODULE_DHT == 1
// contructor
SensorDHT::SensorDHT(NodeManager* node_manager, int child_id, int pin, DHT* dht, int sensor_type, int dht_type): Sensor(node_manager, child_id, pin) {
  // store the dht object
  _dht = dht;
  _sensor_type = sensor_type;
  _dht_type = dht_type;
  if (_sensor_type == SensorDHT::TEMPERATURE) {
    // temperature sensor
    setPresentation(S_TEMP);
    setType(V_TEMP);
    setValueType(TYPE_FLOAT);
  }
  else if (_sensor_type == SensorDHT::HUMIDITY) {
    // humidity sensor
    setPresentation(S_HUM);
    setType(V_HUM);
    setValueType(TYPE_FLOAT);
  }
}

// what to do during before
void SensorDHT::onBefore() {
}

// what to do during setup
void SensorDHT::onSetup() {
  // initialize the dht library
  _dht->setup(_pin,_dht_type);
}

// what to do during loop
void SensorDHT::onLoop() {
  _node_manager->sleepOrWait(_dht->getMinimumSamplingPeriod());
  _dht->readSensor(true);
  // temperature sensor
  if (_sensor_type == SensorDHT::TEMPERATURE) {
    // read the temperature
    float temperature = _dht->getTemperature();
    if (! _node_manager->getIsMetric()) temperature = _dht->toFahrenheit(temperature);
    #if DEBUG == 1
      Serial.print(F("DHT I="));
      Serial.print(_child_id);
      Serial.print(F(" T="));
      Serial.println(temperature);
    #endif
    // store the value
    if (! isnan(temperature)) _value_float = temperature;
  }
  // humidity sensor
  else if (_sensor_type == SensorDHT::HUMIDITY) {
    // read humidity
    float humidity = _dht->getHumidity();
    #if DEBUG == 1
      Serial.print(F("DHT I="));
      Serial.print(_child_id);
      Serial.print(F(" H="));
      Serial.println(humidity);
    #endif
    // store the value
    if (! isnan(humidity)) _value_float = humidity;
  }
}

// what to do as the main task when receiving a message
void SensorDHT::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_REQ) onLoop();
}

// what to do when receiving a remote message
void SensorDHT::onProcess(Request & request) {
}

// what to do when receiving an interrupt
void SensorDHT::onInterrupt() {
}
#endif

/*
   SensorSHT21
*/
#if MODULE_SHT21 == 1
// contructor
SensorSHT21::SensorSHT21(NodeManager* node_manager, int child_id, int sensor_type): Sensor(node_manager,child_id,A2) {
  // store the sensor type (0: temperature, 1: humidity)
  _sensor_type = sensor_type;
  if (_sensor_type == SensorSHT21::TEMPERATURE) {
    // temperature sensor
    setPresentation(S_TEMP);
    setType(V_TEMP);
    setValueType(TYPE_FLOAT);
  }
  else if (_sensor_type == SensorSHT21::HUMIDITY) {
    // humidity sensor
    setPresentation(S_HUM);
    setType(V_HUM);
    setValueType(TYPE_FLOAT);
  }
}

// what to do during before
void SensorSHT21::onBefore() {
  // initialize the library
  Wire.begin();
}

// what to do during setup
void SensorSHT21::onSetup() {
}

// what to do during loop
void SensorSHT21::onLoop() {
  // temperature sensor
  if (_sensor_type == SensorSHT21::TEMPERATURE) {
    // read the temperature
    float temperature = SHT2x.GetTemperature();
    // convert it
    temperature = _node_manager->celsiusToFahrenheit(temperature);
    #if DEBUG == 1
      Serial.print(F("SHT I="));
      Serial.print(_child_id);
      Serial.print(F(" T="));
      Serial.println(temperature);
    #endif
    // store the value
    if (! isnan(temperature)) _value_float = temperature;
  }
  // Humidity Sensor
  else if (_sensor_type == SensorSHT21::HUMIDITY) {
    // read humidity
    float humidity = SHT2x.GetHumidity();
    if (isnan(humidity)) return;
    #if DEBUG == 1
      Serial.print(F("SHT I="));
      Serial.print(_child_id);
      Serial.print(F(" H="));
      Serial.println(humidity);
    #endif
    // store the value
    if (! isnan(humidity)) _value_float = humidity;
  }
}

// what to do as the main task when receiving a message
void SensorSHT21::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_REQ) onLoop();
}

// what to do when receiving a remote message
void SensorSHT21::onProcess(Request & request) {
}

// what to do when receiving an interrupt
void SensorSHT21::onInterrupt() {
}
#endif

/*
 * SensorHTU21D
 */
 #if MODULE_SHT21 == 1
// constructor
SensorHTU21D::SensorHTU21D(NodeManager* node_manager, int child_id, int pin): SensorSHT21(node_manager, child_id, pin) {
}
#endif 

#if MODULE_SWITCH == 1
/*
 * SensorSwitch
 */
SensorSwitch::SensorSwitch(NodeManager* node_manager, int child_id, int pin): Sensor(node_manager,child_id,pin) {
  setType(V_TRIPPED);
}

// setter/getter
void SensorSwitch::setMode(int value) {
  _mode = value;
}
void SensorSwitch::setDebounce(int value) {
  _debounce = value;
}
void SensorSwitch::setTriggerTime(int value) {
  _trigger_time = value;
}
void SensorSwitch::setInitial(int value) {
  _initial = value;
}

// what to do during before
void SensorSwitch::onBefore() {
  // set the interrupt pin so it will be called only when waking up from that interrupt
  setInterrupt(_pin,_mode,_initial);
}

// what to do during setup
void SensorSwitch::onSetup() {
  // report immediately
  _report_timer->unset();
}

// what to do during loop
void SensorSwitch::onLoop() {
}

// what to do as the main task when receiving a message
void SensorSwitch::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_REQ) {
    _value_int = digitalRead(_pin);
  }
}

// what to do when receiving a remote message
void SensorSwitch::onProcess(Request & request) {
  int function = request.getFunction();
  switch(function) {
    case 101: setMode(request.getValueInt()); break;
    case 102: setDebounce(request.getValueInt()); break;
    case 103: setTriggerTime(request.getValueInt()); break;
    case 104: setInitial(request.getValueInt()); break;
    default: return;
  }
  _send(_msg_service.set(function));
}

// what to do when receiving an interrupt
void SensorSwitch::onInterrupt() {
  // wait to ensure the the input is not floating
  if (_debounce > 0) _node_manager->sleepOrWait(_debounce);
  // read the value of the pin
  int value = digitalRead(_pin);
  // process the value
  if ( (_mode == RISING && value == HIGH ) || (_mode == FALLING && value == LOW) || (_mode == CHANGE) )  {
    #if DEBUG == 1
      Serial.print(F("SWITCH I="));
      Serial.print(_child_id);
      Serial.print(F(" P="));
      Serial.print(_pin);
      Serial.print(F(" V="));
      Serial.println(value);
    #endif
    _value_int = value;
    // allow the signal to be restored to its normal value
    if (_trigger_time > 0) _node_manager->sleepOrWait(_trigger_time);
  } else {
    // invalid
    _value_int = -1;
  }
}

/*
 * SensorDoor
 */
SensorDoor::SensorDoor(NodeManager* node_manager, int child_id, int pin): SensorSwitch(node_manager,child_id,pin) {
  setPresentation(S_DOOR);
}

/*
 * SensorMotion
 */
SensorMotion::SensorMotion(NodeManager* node_manager, int child_id, int pin): SensorSwitch(node_manager, child_id,pin) {
  setPresentation(S_MOTION);
  // set initial value to LOW
  setInitial(LOW);
}
#endif

/*
   SensorDs18b20
*/
#if MODULE_DS18B20 == 1
// contructor
SensorDs18b20::SensorDs18b20(NodeManager* node_manager, int child_id, int pin, DallasTemperature* sensors, int index): Sensor(node_manager,child_id, pin) {
  setPresentation(S_TEMP);
  setType(V_TEMP);
  setValueType(TYPE_FLOAT);
  _index = index;
  _sensors = sensors;
  // retrieve and store the address from the index
  _sensors->getAddress(_device_address, index);
}

// what to do during before
void SensorDs18b20::onBefore() {
}

// what to do during setup
void SensorDs18b20::onSetup() {
}

// what to do during loop
void SensorDs18b20::onLoop() {
  // do not wait for conversion, will sleep manually during it
  if (_sleep_during_conversion) _sensors->setWaitForConversion(false);
  // request the temperature
  _sensors->requestTemperatures();
  if (_sleep_during_conversion) {
    // calculate conversion time and sleep
    int16_t conversion_time = _sensors->millisToWaitForConversion(_sensors->getResolution());
    sleep(conversion_time);
  }
  // read the temperature
  float temperature = _sensors->getTempCByIndex(_index);
  // convert it
  temperature = _node_manager->celsiusToFahrenheit(temperature);
  #if DEBUG == 1
    Serial.print(F("DS18B20 I="));
    Serial.print(_child_id);
    Serial.print(F(" T="));
    Serial.println(temperature);
  #endif
  // store the value
  _value_float = temperature;
}

// what to do as the main task when receiving a message
void SensorDs18b20::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_REQ) onLoop();
}

// what to do when receiving a remote message
void SensorDs18b20::onProcess(Request & request) {
  int function = request.getFunction();
  switch(function) {
    case 101: setResolution(request.getValueInt()); break;
    case 102: setSleepDuringConversion(request.getValueInt()); break;
    default: return;
  }
  _send(_msg_service.set(function));
}

// what to do when receiving an interrupt
void SensorDs18b20::onInterrupt() {
}

// function to print a device address
DeviceAddress* SensorDs18b20::getDeviceAddress() {
  return &_device_address;
}

// returns the sensor's resolution in bits
int SensorDs18b20::getResolution() {
  return _sensors->getResolution(_device_address);
}

// set the sensor's resolution in bits
void SensorDs18b20::setResolution(int value) {
   _sensors->setResolution(_device_address, value);
}

// sleep while DS18B20 calculates temperature
void SensorDs18b20::setSleepDuringConversion(bool value) {
   _sleep_during_conversion = value;
}

#endif

/*
   SensorBH1750
*/
#if MODULE_BH1750 == 1
// contructor
SensorBH1750::SensorBH1750(NodeManager* node_manager, int child_id): Sensor(node_manager,child_id,A4) {
  setPresentation(S_LIGHT_LEVEL);
  setType(V_LEVEL);
  _lightSensor = new BH1750();
}

// what to do during before
void SensorBH1750::onBefore() {
  _lightSensor->begin();
}

// what to do during setup
void SensorBH1750::onSetup() {
}

// what to do during loop
void SensorBH1750::onLoop() {
  // request the light level
  _value_int = _lightSensor->readLightLevel();
  #if DEBUG == 1
    Serial.print(F("BH1 I="));
    Serial.print(_child_id);
    Serial.print(F(" L="));
    Serial.println(_value_int);
  #endif
}

// what to do as the main task when receiving a message
void SensorBH1750::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_REQ) onLoop();
}

// what to do when receiving a remote message
void SensorBH1750::onProcess(Request & request) {
}


// what to do when receiving an interrupt
void SensorBH1750::onInterrupt() {
}
#endif

/*
   SensorMLX90614
*/
#if MODULE_MLX90614 == 1
// contructor
SensorMLX90614::SensorMLX90614(NodeManager* node_manager, int child_id, Adafruit_MLX90614* mlx, int sensor_type): Sensor(node_manager,child_id,A4) {
  _sensor_type = sensor_type;
  _mlx = mlx;
  // set presentation and type
  setPresentation(S_TEMP);
  setType(V_TEMP);
  setValueType(TYPE_FLOAT);
}

// what to do during before
void SensorMLX90614::onBefore() {
  // initialize the library
  _mlx->begin();
}

// what to do during setup
void SensorMLX90614::onSetup() {
}

// what to do during loop
void SensorMLX90614::onLoop() {
  float temperature = _sensor_type == SensorMLX90614::TEMPERATURE_OBJECT ? _mlx->readAmbientTempC() : _mlx->readObjectTempC();
  // convert it
  temperature = _node_manager->celsiusToFahrenheit(temperature);
  #if DEBUG == 1
    Serial.print(F("MLX I="));
    Serial.print(_child_id);
    Serial.print(F(" T="));
    Serial.println(temperature);
  #endif
  if (! isnan(temperature)) _value_float = temperature;
}

// what to do as the main task when receiving a message
void SensorMLX90614::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_REQ) onLoop();
}

// what to do when receiving a remote message
void SensorMLX90614::onProcess(Request & request) {
}

// what to do when receiving an interrupt
void SensorMLX90614::onInterrupt() {
}
#endif


/*
   SensorBosch
*/
#if MODULE_BME280 == 1 || MODULE_BMP085 == 1 || MODULE_BMP280 == 1
// contructor
SensorBosch::SensorBosch(NodeManager* node_manager, int child_id, int sensor_type): Sensor(node_manager, child_id,A4) {
  _sensor_type = sensor_type;
  if (_sensor_type == SensorBosch::TEMPERATURE) {
    // temperature sensor
    setPresentation(S_TEMP);
    setType(V_TEMP);
    setValueType(TYPE_FLOAT);
  }
  else if (_sensor_type == SensorBosch::HUMIDITY) {
    // humidity sensor
    setPresentation(S_HUM);
    setType(V_HUM);
    setValueType(TYPE_FLOAT);
  }
  else if (_sensor_type == SensorBosch::PRESSURE) {
    // pressure sensor
    setPresentation(S_BARO);
    setType(V_PRESSURE);
    setValueType(TYPE_FLOAT);
  }
  else if (_sensor_type == SensorBosch::FORECAST) {
    // pressure sensor
    setPresentation(S_BARO);
    setType(V_FORECAST);
    setValueType(TYPE_STRING);
  }
}

// setter/getter
void SensorBosch::setForecastSamplesCount(int value) {
  _forecast_samples_count = value;
}

// what to do during before
void SensorBosch::onBefore() {
  // initialize the forecast samples array
  _forecast_samples = new float[_forecast_samples_count];
}

// what to do during setup
void SensorBosch::onSetup() {
}

// what to do during loop
void SensorBosch::onLoop() {
}

// what to do as the main task when receiving a message
void SensorBosch::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_REQ) onLoop();
}

// what to do when receiving a remote message
void SensorBosch::onProcess(Request & request) {
  int function = request.getFunction();
  switch(function) {
    case 101: setForecastSamplesCount(request.getValueInt()); break;
    default: return;
  }
  _send(_msg_service.set(function));
}

// what to do when receiving an interrupt
void SensorBosch::onInterrupt() {
}

// calculate and send the forecast back
void SensorBosch::_forecast(float pressure) {
  if (isnan(pressure)) return;
  // Calculate the average of the last n minutes.
  int index = _minute_count % _forecast_samples_count;
  _forecast_samples[index] = pressure;
  _minute_count++;
  if (_minute_count > 185) _minute_count = 6;
  if (_minute_count == 5) _pressure_avg = _getLastPressureSamplesAverage();
  else if (_minute_count == 35) {
    float last_pressure_avg = _getLastPressureSamplesAverage();
    float change = (last_pressure_avg - _pressure_avg) * 0.1;
    // first time initial 3 hour
    if (_first_round) _dP_dt = change * 2; // note this is for t = 0.5hour
    else _dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
  }
  else if (_minute_count == 65) {
    float last_pressure_avg = _getLastPressureSamplesAverage();
    float change = (last_pressure_avg - _pressure_avg) * 0.1;
    //first time initial 3 hour
    if (_first_round) _dP_dt = change; //note this is for t = 1 hour
    else _dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
  }
  else if (_minute_count == 95) {
    float last_pressure_avg = _getLastPressureSamplesAverage();
    float change = (last_pressure_avg - _pressure_avg) * 0.1;
    // first time initial 3 hour
    if (_first_round)_dP_dt = change / 1.5; // note this is for t = 1.5 hour
    else _dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
  }
  else if (_minute_count == 125) {
    float last_pressure_avg = _getLastPressureSamplesAverage();
    // store for later use.
    _pressure_avg2 = last_pressure_avg; 
    float change = (last_pressure_avg - _pressure_avg) * 0.1;
    if (_first_round) _dP_dt = change / 2; // note this is for t = 2 hour
    else _dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
  }
  else if (_minute_count == 155) {
    float last_pressure_avg = _getLastPressureSamplesAverage();
    float change = (last_pressure_avg - _pressure_avg) * 0.1;
    if (_first_round) _dP_dt = change / 2.5; // note this is for t = 2.5 hour
    else _dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
  }
  else if (_minute_count == 185) {
    float last_pressure_avg = _getLastPressureSamplesAverage();
    float change = (last_pressure_avg - _pressure_avg) * 0.1;
    if (_first_round) _dP_dt = change / 3; // note this is for t = 3 hour
    else _dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
  }
  // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
  _pressure_avg = _pressure_avg2; 
  // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
  _first_round = false; 
  // calculate the forecast (STABLE = 0, SUNNY = 1, CLOUDY = 2, UNSTABLE = 3, THUNDERSTORM = 4, UNKNOWN = 5)
  int forecast = 5;
  //if time is less than 35 min on the first 3 hour interval.
  if (_minute_count < 35 && _first_round) forecast = 5;
  else if (_dP_dt < (-0.25)) forecast = 5;
  else if (_dP_dt > 0.25) forecast = 4;
  else if ((_dP_dt > (-0.25)) && (_dP_dt < (-0.05))) forecast = 2;
  else if ((_dP_dt > 0.05) && (_dP_dt < 0.25)) forecast = 1;
  else if ((_dP_dt >(-0.05)) && (_dP_dt < 0.05)) forecast = 0;
  else forecast = 5;
  _value_string = _weather[forecast];
  #if DEBUG == 1
    Serial.print(F("BMP I="));
    Serial.print(_child_id);
    Serial.print(F(" M="));
    Serial.print(_minute_count);
    Serial.print(F(" dP="));
    Serial.print(_dP_dt);
    Serial.print(F(" F="));
    Serial.println(_value_string);
  #endif
}

// returns the average of the latest pressure samples
float SensorBosch::_getLastPressureSamplesAverage() {
  float avg = 0;
  for (int i = 0; i < _forecast_samples_count; i++) avg += _forecast_samples[i];
  avg /= _forecast_samples_count;
  return avg;
}

// search for a given chip on i2c bus
uint8_t SensorBosch::GetI2CAddress(uint8_t chip_id) {
  uint8_t addresses[] = {0x77, 0x76};
  uint8_t register_address = 0xD0;
  for (int i = 0; i <= sizeof(addresses); i++) { 
    uint8_t i2c_address = addresses[i];
    uint8_t value;
    Wire.beginTransmission((uint8_t)i2c_address);
    Wire.write((uint8_t)register_address);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)i2c_address, (byte)1);
    value = Wire.read();
    if (value == chip_id) {
      #if DEBUG == 1
        Serial.print(F("I2C=")); 
        Serial.println(i2c_address);
      #endif
      return i2c_address;
    }
  }
  return addresses[0]; 
}
#endif

/*
 * SensorBME280
 */
#if MODULE_BME280 == 1
SensorBME280::SensorBME280(NodeManager* node_manager, int child_id, Adafruit_BME280* bme, int sensor_type): SensorBosch(node_manager, child_id,sensor_type) {
  _bme = bme;
}

void SensorBME280::onLoop() {
  // temperature sensor
  if (_sensor_type == SensorBME280::TEMPERATURE) {
    // read the temperature
    float temperature = _bme->readTemperature();
    // convert it
    temperature = _node_manager->celsiusToFahrenheit(temperature);
    #if DEBUG == 1
      Serial.print(F("BME I="));
      Serial.print(_child_id);
      Serial.print(F(" T="));
      Serial.println(temperature);
    #endif
    if (isnan(temperature)) return;
    // store the value
    _value_float = temperature;
  }
  // Humidity Sensor
  else if (_sensor_type == SensorBME280::HUMIDITY) {
    // read humidity
    float humidity = _bme->readHumidity();
    #if DEBUG == 1
      Serial.print(F("BME I="));
      Serial.print(_child_id);
      Serial.print(F(" H="));
      Serial.println(humidity);
    #endif
    if (isnan(humidity)) return;
    // store the value
    _value_float = humidity;
  }
  // Pressure Sensor
  else if (_sensor_type == SensorBME280::PRESSURE) {
    // read pressure
    float pressure = _bme->readPressure() / 100.0F;
    if (isnan(pressure)) return;
    #if DEBUG == 1
      Serial.print(F("BME I="));
      Serial.print(_child_id);
      Serial.print(F(" P="));
      Serial.println(pressure);
    #endif
    if (isnan(pressure)) return;
    // store the value
    _value_float = pressure;
  }
  // Forecast Sensor
  else if (_sensor_type == SensorBME280::FORECAST) {
    float pressure = _bme->readPressure() / 100.0F;
    _forecast(pressure);
  }
}
#endif

/*
   SensorBMP085
*/
#if MODULE_BMP085 == 1
// contructor
SensorBMP085::SensorBMP085(NodeManager* node_manager, int child_id, Adafruit_BMP085* bmp, int sensor_type): SensorBosch(node_manager, child_id,sensor_type) {
  _bmp = bmp;
}

// what to do during loop
void SensorBMP085::onLoop() {
  // temperature sensor
  if (_sensor_type == SensorBMP085::TEMPERATURE) {
    // read the temperature
    float temperature = _bmp->readTemperature();
    // convert it
    temperature = _node_manager->celsiusToFahrenheit(temperature);
    #if DEBUG == 1
      Serial.print(F("BMP I="));
      Serial.print(_child_id);
      Serial.print(F(" T="));
      Serial.println(temperature);
    #endif
    if (isnan(temperature)) return;
    // store the value
    _value_float = temperature;
  }
  // Pressure Sensor
  else if (_sensor_type == SensorBMP085::PRESSURE) {
    // read pressure
    float pressure = _bmp->readPressure() / 100.0F;
    #if DEBUG == 1
      Serial.print(F("BMP I="));
      Serial.print(_child_id);
      Serial.print(F(" P="));
      Serial.println(pressure);
    #endif
    if (isnan(pressure)) return;
    // store the value
    _value_float = pressure;
  }
  // Forecast Sensor
  else if (_sensor_type == SensorBMP085::FORECAST) {
    float pressure = _bmp->readPressure() / 100.0F;
    _forecast(pressure);    
  }
}
#endif

/*
 * SensorBMP280
 */
#if MODULE_BMP280 == 1
SensorBMP280::SensorBMP280(NodeManager* node_manager, int child_id, Adafruit_BMP280* bmp, int sensor_type): SensorBosch(node_manager, child_id,sensor_type) {
  _bmp = bmp;
}

void SensorBMP280::onLoop() {
  // temperature sensor
  if (_sensor_type == SensorBMP280::TEMPERATURE) {
    // read the temperature
    float temperature = _bmp->readTemperature();
    // convert it
    temperature = _node_manager->celsiusToFahrenheit(temperature);
    #if DEBUG == 1
      Serial.print(F("BMP I="));
      Serial.print(_child_id);
      Serial.print(F(" T="));
      Serial.println(temperature);
    #endif
    if (isnan(temperature)) return;
    // store the value
    _value_float = temperature;
  }
  // Pressure Sensor
  else if (_sensor_type == SensorBMP280::PRESSURE) {
    // read pressure
    float pressure = _bmp->readPressure() / 100.0F;
    if (isnan(pressure)) return;
    #if DEBUG == 1
      Serial.print(F("BMP I="));
      Serial.print(_child_id);
      Serial.print(F(" P="));
      Serial.println(pressure);
    #endif
    if (isnan(pressure)) return;
    // store the value
    _value_float = pressure;
  }
  // Forecast Sensor
  else if (_sensor_type == SensorBMP280::FORECAST) {
    float pressure = _bmp->readPressure() / 100.0F;
    _forecast(pressure);
  }
}
#endif

/*
   SensorHCSR04
*/
#if MODULE_HCSR04 == 1
// contructor
SensorHCSR04::SensorHCSR04(NodeManager* node_manager, int child_id, int pin): Sensor(node_manager, child_id, pin) {
  // set presentation and type
  setPresentation(S_DISTANCE);
  setType(V_DISTANCE);
  _trigger_pin = pin;
  _echo_pin = pin;
}

// what to do during before
void SensorHCSR04::onBefore() {
  // initialize the library
  _sonar = new NewPing(_trigger_pin,_echo_pin,_max_distance);
}

// setter/getter
void SensorHCSR04::setTriggerPin(int value) {
  _trigger_pin = value;
}
void SensorHCSR04::setEchoPin(int value) {
  _echo_pin = value;
}
void SensorHCSR04::setMaxDistance(int value) {
  _max_distance = value;
}

// what to do during setup
void SensorHCSR04::onSetup() {
}

// what to do during loop
void SensorHCSR04::onLoop() {
  int distance = _node_manager->getIsMetric() ? _sonar->ping_cm() : _sonar->ping_in();
  #if DEBUG == 1
    Serial.print(F("HC I="));
    Serial.print(_child_id);
    Serial.print(F(" D="));
    Serial.println(distance);
  #endif
  _value_int = distance;
}

// what to do as the main task when receiving a message
void SensorHCSR04::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_REQ) onLoop();
}

// what to do when receiving a remote message
void SensorHCSR04::onProcess(Request & request) {
  int function = request.getFunction();
  switch(function) {
    case 101: setTriggerPin(request.getValueInt()); break;
    case 102: setEchoPin(request.getValueInt()); break;
    case 103: setMaxDistance(request.getValueInt()); break;
    default: return;
  }
  _send(_msg_service.set(function));
}

// what to do when receiving an interrupt
void SensorHCSR04::onInterrupt() {
}
#endif

/*
   SensorSonoff
*/
#if MODULE_SONOFF == 1
// contructor
SensorSonoff::SensorSonoff(NodeManager* node_manager, int child_id): Sensor(node_manager, child_id,1) {
  setPresentation(S_BINARY);
  setType(V_STATUS);
}

// setter/getter
void SensorSonoff::setButtonPin(int value) {
    _button_pin = value;
}
void SensorSonoff::setRelayPin(int value) {
    _relay_pin = value;
}
void SensorSonoff::setLedPin(int value) {
    _led_pin = value;
}

// what to do during before
void SensorSonoff::onBefore() {
}

// what to do during setup
void SensorSonoff::onSetup() {
  // Setup the button
  pinMode(_button_pin, INPUT_PULLUP);
  // After setting up the button, setup debouncer
  _debouncer.attach(_button_pin);
  _debouncer.interval(5);
  // Make sure relays and LED are off when starting up
  digitalWrite(_relay_pin, _relay_off);
  digitalWrite(_led_pin, _led_off);
  // Then set relay pins in output mode
  pinMode(_relay_pin, OUTPUT);
  pinMode(_led_pin, OUTPUT);
  _blink();
}

// what to do during loop
void SensorSonoff::onLoop() {
  _debouncer.update();
  // Get the update value from the button
  int value = _debouncer.read();
  if (value != _old_value && value == 0) {
    // button pressed, toggle the state
    _toggle();
  }
  _old_value = value;
}

// what to do as the main task when receiving a message
void SensorSonoff::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_SET) {
    // retrieve from the message the value to set
    int value = message.getInt();
    if (value != 0 && value != 1 || value == _state) return;
    // toggle the state
    _toggle();
  }
  if (message.getCommand() == C_REQ) {
    // return the current state
    _value_int = _state;
  }
}

// what to do when receiving a remote message
void SensorSonoff::onProcess(Request & request) {
  int function = request.getFunction();
  switch(function) {
    case 101: setButtonPin(request.getValueInt()); break;
    case 102: setRelayPin(request.getValueInt()); break;
    case 103: setLedPin(request.getValueInt()); break;
    default: return;
  }
  _send(_msg_service.set(function));
}

// what to do when receiving an interrupt
void SensorSonoff::onInterrupt() {
}

// toggle the state
void SensorSonoff::_toggle() {
  // toggle the state
  _state = _state ? false : true;
  // Change relay state
  digitalWrite(_relay_pin, _state? _relay_on: _relay_off);
  // Change LED state
  digitalWrite(_led_pin, _state? _led_on: _led_off);
  #if DEBUG == 1
    Serial.print(F("SONOFF I="));
    Serial.print(_child_id);
    Serial.print(F(" V="));
    Serial.println(_state);
  #endif
  _value_int = _state;
}

// blink the led
void SensorSonoff::_blink() {
  digitalWrite(_led_pin, digitalRead(_led_pin) ? _led_on : _led_off);
  wait(200);
  digitalWrite(_led_pin, digitalRead(_led_pin) ? _led_on : _led_off);
  wait(200);
  digitalWrite(_led_pin, digitalRead(_led_pin) ? _led_on : _led_off);
  wait(200);
  digitalWrite(_led_pin, digitalRead(_led_pin) ? _led_on : _led_off);
}
#endif


/*
   SensorMCP9808
*/
#if MODULE_MCP9808 == 1
// contructor
SensorMCP9808::SensorMCP9808(NodeManager* node_manager, int child_id, Adafruit_MCP9808* mcp): Sensor(node_manager, child_id,A2) {
  _mcp = mcp;
  setPresentation(S_TEMP);
  setType(V_TEMP);
  setValueType(TYPE_FLOAT);
}

// what to do during before
void SensorMCP9808::onBefore() {
}

// what to do during setup
void SensorMCP9808::onSetup() {
}

// what to do during loop
void SensorMCP9808::onLoop() {
  float temperature = _mcp->readTempC();
  // convert it
  temperature = _node_manager->celsiusToFahrenheit(temperature);
  #if DEBUG == 1
    Serial.print(F("MCP I="));
    Serial.print(_child_id);
    Serial.print(F(" T="));
    Serial.println(temperature);
  #endif
  // store the value
  if (! isnan(temperature)) _value_float = temperature;
}

// what to do as the main task when receiving a message
void SensorMCP9808::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_REQ) onLoop();
}

// what to do when receiving a remote message
void SensorMCP9808::onProcess(Request & request) {
}

// what to do when receiving an interrupt
void SensorMCP9808::onInterrupt() {
}
#endif


/*
 * SensorMQ
 */
#if MODULE_MQ == 1

static float SensorMQ::_default_LPGCurve[3] = {2.3,0.21,-0.47};
static float SensorMQ::_default_COCurve[3] = {2.3,0.72,-0.34};
static float SensorMQ::_default_SmokeCurve[3] = {2.3,0.53,-0.44};

SensorMQ::SensorMQ(NodeManager* node_manager, int child_id, int pin): Sensor(node_manager,child_id,pin) {
  setPresentation(S_AIR_QUALITY);
  setType(V_LEVEL);
  _LPGCurve = SensorMQ::_default_LPGCurve;
  _COCurve = SensorMQ::_default_COCurve;
  _SmokeCurve = SensorMQ::_default_SmokeCurve;
}

//setter/getter
void SensorMQ::setTargetGas(int value) {
  _target_gas = value;
}
void SensorMQ::setRlValue(float value) {
  _rl_value = value;
}
void SensorMQ::setRoValue(float value) {
  _ro = value;
}
void SensorMQ::setCleanAirFactor(float value) {
  _ro_clean_air_factor = value;
}
void SensorMQ::setCalibrationSampleTimes(int value) {
  _calibration_sample_times = value;
}
void SensorMQ::setCalibrationSampleInterval(int value){
  _calibration_sample_interval = value;
}
void SensorMQ::setReadSampleTimes(int value) {
  _read_sample_times = value;
}
void SensorMQ::setReadSampleInterval(int value) {
  _read_sample_interval = value;
}
void SensorMQ::setLPGCurve(float *value) {
  _LPGCurve = value;
}
void SensorMQ::setCOCurve(float *value) {
  _COCurve = value;
}
void SensorMQ::setSmokeCurve(float *value) {
  _SmokeCurve = value;
}

// what to do during before
void SensorMQ::onBefore() {
  // prepare the pin for input
  pinMode(_pin, INPUT);
}

// what to do during setup
void SensorMQ::onSetup() {
  _ro = _MQCalibration();
}

// what to do during loop
void SensorMQ::onLoop() {
  if (_pin == -1) return;
  // calculate rs/ro
  float mq = _MQRead()/_ro;
  // calculate the ppm
  float lpg = _MQGetGasPercentage(mq,_gas_lpg);
  float co = _MQGetGasPercentage(mq,_gas_co);
  float smoke = _MQGetGasPercentage(mq,_gas_smoke);
  // assign to the value the requested gas
  uint16_t value;
  if (_target_gas == _gas_lpg) value = lpg;
  if (_target_gas == _gas_co) value = co;
  if (_target_gas == _gas_smoke) value = smoke;
  #if DEBUG == 1
    Serial.print(F("MQ I="));
    Serial.print(_child_id);
    Serial.print(F(" V="));
    Serial.print(value);
    Serial.print(F(" LPG="));
    Serial.print(lpg);
    Serial.print(F(" CO="));
    Serial.print(co);
    Serial.print(F(" SMOKE="));
    Serial.println(smoke);
  #endif
  // store the value
  _value_int = (int16_t)ceil(value);
}

// what to do as the main task when receiving a message
void SensorMQ::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_REQ) onLoop();
}

// what to do when receiving a remote message
void SensorMQ::onProcess(Request & request) {
  int function = request.getFunction();
  switch(function) {
    case 1: setTargetGas(request.getValueInt()); break;
    case 2: setRlValue(request.getValueFloat()); break;
    case 3: setRoValue(request.getValueFloat()); break;
    case 4: setCleanAirFactor(request.getValueFloat()); break;
    case 5: setCalibrationSampleTimes(request.getValueInt()); break;
    case 6: setCalibrationSampleInterval(request.getValueInt()); break;
    case 7: setReadSampleTimes(request.getValueInt()); break;
    case 8: setReadSampleInterval(request.getValueInt()); break;
    default: return;
  }
  _send(_msg_service.set(function));
}

// what to do when receiving an interrupt
void SensorMQ::onInterrupt() {
}

// returns the calculated sensor resistance
float SensorMQ::_MQResistanceCalculation(int raw_adc) {
  return ( ((float)_rl_value*(1023-raw_adc)/raw_adc));
}

//  This function assumes that the sensor is in clean air
float SensorMQ::_MQCalibration() {
  int i;
  float val=0;
  //take multiple samples
  for (i=0; i< _calibration_sample_times; i++) {  
    val += _MQResistanceCalculation(analogRead(_pin));
    wait(_calibration_sample_interval);
  }
  //calculate the average value
  val = val/_calibration_sample_times;                   
  //divided by RO_CLEAN_AIR_FACTOR yields the Ro
  val = val/_ro_clean_air_factor;
  //according to the chart in the datasheet
  return val;
}

// This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
float SensorMQ::_MQRead() {
  int i;
  float rs=0;
  for (i=0; i<_read_sample_times; i++) {
    rs += _MQResistanceCalculation(analogRead(_pin));
    wait(_read_sample_interval);
  }
  rs = rs/_read_sample_times;
  return rs;
}

// This function passes different curves to the MQGetPercentage function which calculates the ppm (parts per million) of the target gas.
int SensorMQ::_MQGetGasPercentage(float rs_ro_ratio, int gas_id) {
  if ( gas_id == _gas_lpg ) {
    return _MQGetPercentage(rs_ro_ratio,_LPGCurve);
  } else if ( gas_id == _gas_co) {
    return _MQGetPercentage(rs_ro_ratio,_COCurve);
  } else if ( gas_id == _gas_smoke) {
    return _MQGetPercentage(rs_ro_ratio,_SmokeCurve);
  }
  return 0;
}

// returns ppm of the target gas
int SensorMQ::_MQGetPercentage(float rs_ro_ratio, float *pcurve) {
  return (pow(10,( ((log10(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
#endif



/*
   SensorMHZ19
*/
#if MODULE_MHZ19 == 1
// contructor
SensorMHZ19::SensorMHZ19(NodeManager* node_manager, int child_id, int pin): Sensor(node_manager, child_id, pin) {
  // set presentation, type and value type
  setPresentation(S_AIR_QUALITY);
  setType(V_LEVEL);
  setRxTx(pin, pin+1);
}

void SensorMHZ19::setRxTx(int rxpin, int txpin) {
  _rx_pin = rxpin;
  _tx_pin = txpin;
}


// what to do during before
void SensorMHZ19::onBefore() {
}

// what to do during setup
void SensorMHZ19::onSetup() {
  _ser = new SoftwareSerial(_rx_pin, _tx_pin);
  _ser->begin(9600);
  delay(2000);
  while (_ser->read()!=-1) {};  // clear CO2 buffer.
}

// what to do during loop
void SensorMHZ19::onLoop() {
  // Read the ppm value
  int co2ppm = readCO2(); // This is there the function gets called that talks to the Co2 sensor.
  #if DEBUG == 1
    Serial.print(F("CO2 I="));
    Serial.print(_child_id);
    Serial.print(F(" ppm="));
    Serial.println(co2ppm);
  #endif
  // store the value
  _value_int = co2ppm;
}

// Read out the CO2 data
int SensorMHZ19::readCO2() {
  while (_ser->read() != -1) {};  //clear serial buffer

  unsigned char response[9]; // for answer
  byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};

  // Command to ask for data.
  _ser->write(cmd, 9); //request PPM CO2

  // Then for 1 second listen for 9 bytes of data.
  _ser->readBytes(response, 9);

  #if DEBUG == 1
  for (int i=0; i<9; i++) {
    Serial.print(response[i], HEX);
    Serial.print(F("-"));
  }
  Serial.println(F("END"));
  #endif

  if (response[0] != 0xFF) {
    Serial.println(F("Wrong starting byte from co2 sensor! (should be FF)"));
    return -1;
  }

  if (response[1] != 0x86) {
    Serial.println(F("Wrong command from co2 sensor! (should be 86)"));
    return -1;
  }

  int responseHigh = (int) response[2];
  int responseLow = (int) response[3];
  int ppm = (256 * responseHigh) + responseLow;
  
  return ppm;
}


// what to do as the main task when receiving a message
void SensorMHZ19::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_REQ) onLoop();
}

// what to do when receiving a remote message
void SensorMHZ19::onProcess(Request & request) {
  int function = request.getFunction();
  switch(function) {
    default: return;
  }
  _send(_msg_service.set(function));
}

// what to do when receiving an interrupt
void SensorMHZ19::onInterrupt() {
}

#endif

/*
   SensorAM2320
*/
#if MODULE_AM2320 == 1
// constructor
SensorAM2320::SensorAM2320(NodeManager* node_manager, int child_id, AM2320* th, int sensor_type): Sensor(node_manager, child_id,A2) {
  _th = th;
  _sensor_type = sensor_type;
  if (_sensor_type == SensorAM2320::TEMPERATURE) {
    // temperature sensor
    setPresentation(S_TEMP);
    setType(V_TEMP);
    setValueType(TYPE_FLOAT);
  }
  else if (_sensor_type == SensorAM2320::HUMIDITY) {
    // humidity sensor
    setPresentation(S_HUM);
    setType(V_HUM);
    setValueType(TYPE_FLOAT);
  }
}

// what do to during before
void SensorAM2320::onBefore() {

}

// what do to during setup
void SensorAM2320::onSetup() {
}

// what do to during loop
void SensorAM2320::onLoop() {
  switch(_th->Read()) {
    case 0:
      // temperature sensor
      if (_sensor_type == SensorAM2320::TEMPERATURE) {
        // read the temperature
        float temperature = _th->t;
        #if DEBUG == 1
          Serial.print(F("AM2320 I="));
          Serial.print(_child_id);
          Serial.print(F(" T="));
          Serial.println(temperature);
        #endif
        // store the value
        _value_float = temperature;
      }
      // humidity sensor
      else if (_sensor_type == SensorAM2320::HUMIDITY) {
        // read humidity
        float humidity = _th->h;
        if (isnan(humidity)) return;
          #if DEBUG == 1
            Serial.print(F("AM2320 I="));
            Serial.print(_child_id);
            Serial.print(F(" H="));
            Serial.println(humidity);
          #endif
          // store the value
          _value_float = humidity;
        }
        break;
      case 1: Serial.println(F("AM2320 offline")); break;
      case 2: Serial.println(F("AM2320 CRC failed")); break;
    }
}

// what do to as the main task when receiving a message
void SensorAM2320::onReceive(const MyMessage & message) {
  onLoop();
}

// what to do when receiving a remote message
void SensorAM2320::onProcess(Request & request) {
}

// what to do when receiving an interrupt
void SensorAM2320::onInterrupt() {
}
#endif

/*
   SensorTSL2561
*/
#if MODULE_TSL2561 == 1
// contructor
SensorTSL2561::SensorTSL2561(NodeManager* node_manager, int child_id): Sensor(node_manager, child_id,A2) {
  setPresentation(S_LIGHT_LEVEL);
  setType(V_LEVEL);
}

// setter/getter
void SensorTSL2561::setGain(int value) {
  _tsl_gain = value;
}
void SensorTSL2561::setTiming(int value) {
  _tsl_timing = value;
}
void SensorTSL2561::setSpectrum(int value) {
  _tsl_spectrum = value;
}
void SensorTSL2561::setAddress(int value) {
  _tsl_address = value;
}

// what do to during before
void SensorTSL2561::onBefore() {
   switch (_tsl_address) {
    case SensorTSL2561::ADDR_FLOAT:
      _tsl = new TSL2561(TSL2561_ADDR_FLOAT);
      break;
    case SensorTSL2561::ADDR_LOW:
      _tsl = new TSL2561(TSL2561_ADDR_LOW);
      break;   
    case SensorTSL2561::ADDR_HIGH:
      _tsl = new TSL2561(TSL2561_ADDR_HIGH);
      break;   
  }
}

// what do to during setup
void SensorTSL2561::onSetup() {
   if (_tsl->begin()) {
    switch (_tsl_gain) {
      case SensorTSL2561::GAIN_0X:
        _tsl->setGain(TSL2561_GAIN_0X);
        break; 
      case SensorTSL2561::GAIN_16X:
        _tsl->setGain(TSL2561_GAIN_16X);
        break;      
    }
    switch (_tsl_timing) {
      case SensorTSL2561::INTEGRATIONTIME_13MS:
        _tsl->setTiming(TSL2561_INTEGRATIONTIME_13MS);
        break; 
      case SensorTSL2561::INTEGRATIONTIME_101MS:
        _tsl->setTiming(TSL2561_INTEGRATIONTIME_101MS); 
        break; 
      case SensorTSL2561::INTEGRATIONTIME_402MS:
        _tsl->setTiming(TSL2561_INTEGRATIONTIME_402MS); 
        break;
    }
  }
  else {
    Serial.println(F("TSL2561 offline"));
  } 
}

// what do to during loop
void SensorTSL2561::onLoop() {
  // request the light level
   switch (_tsl_spectrum) {
    case SensorTSL2561::VISIBLE:
      _value_int = _tsl->getLuminosity(TSL2561_VISIBLE); 
      break; 
    case SensorTSL2561::FULLSPECTRUM:
      _value_int = _tsl->getLuminosity(TSL2561_FULLSPECTRUM); 
      break; 
    case SensorTSL2561::INFRARED:
      _value_int = _tsl->getLuminosity(TSL2561_INFRARED); 
      break; 
    case SensorTSL2561::FULL:
      // request the full light level
      uint32_t lum = _tsl->getFullLuminosity(); 
      uint16_t ir, full;
      ir = lum >> 16;
      full = lum & 0xFFFF;
      _value_int = _tsl->calculateLux(full, ir);
  #if DEBUG == 1
      Serial.print(F("TSL I="));
      Serial.print(_child_id);
      Serial.print(F(" LUX="));
      Serial.print(_value_int);
      Serial.print(F(" IR="));
      Serial.print(ir);
      Serial.print(F(" FULL="));
      Serial.print(full);
      Serial.print(F(" VIS="));
      Serial.println(full-ir);
   #endif
      break; 
  }
  #if DEBUG == 1
    if (_tsl_spectrum < 3) {
      Serial.print(F("TSL I="));
      Serial.print(_child_id);
      Serial.print(F(" L="));
      Serial.println(_value_int);
    }
  #endif
}

// what do to as the main task when receiving a message
void SensorTSL2561::onReceive(const MyMessage & message) {
  onLoop();
}

// what to do when receiving a remote message
void SensorTSL2561::onProcess(Request & request) {
  int function = request.getFunction();
  switch(function) {
    case 101: setGain(request.getValueInt()); break;
    case 102: setTiming(request.getValueInt()); break;
    case 103: setSpectrum(request.getValueInt()); break;
    case 104: setAddress(request.getValueInt()); break;
    default: return;
  }
  _send(_msg_service.set(function));
}

// what to do when receiving an interrupt
void SensorTSL2561::onInterrupt() {
}
#endif

/*
   SensorPT100
*/
#if MODULE_PT100 == 1
// contructor
SensorPT100::SensorPT100(NodeManager* node_manager, int child_id, int pin): Sensor(node_manager, child_id, pin) {
  // set presentation, type and value type
  setPresentation(S_TEMP);
  setType(V_TEMP);
  setValueType(TYPE_FLOAT);
}

// setter/getter
void SensorPT100::setVoltageRef(float value) {
   _voltageRef = value;
}

// what to do during before
void SensorPT100::onBefore() {
  _PT100 = new DFRobotHighTemperature(_voltageRef); 
  // set the pin as input
  pinMode(_pin, INPUT);
}

// what to do during setup
void SensorPT100::onSetup() {
}

// what to do during loop
void SensorPT100::onLoop() {
  // read the PT100 sensor
  int temperature = _PT100->readTemperature(_pin);  
  #if DEBUG == 1
    Serial.print(F("PT100 I="));
    Serial.print(_child_id);
    Serial.print(F(" T="));
    Serial.println(temperature);
  #endif
  // store the value
  _value_float = temperature;
}

// what to do as the main task when receiving a message
void SensorPT100::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_REQ) onLoop();
}

// what to do when receiving a remote message
void SensorPT100::onProcess(Request & request) {
   int function = request.getFunction();
  switch(function) {
    case 101: setVoltageRef(request.getValueFloat()); break;
    default: return;
  }
  _send(_msg_service.set(function));
}

// what to do when receiving an interrupt
void SensorPT100::onInterrupt() {
}
#endif

/*
   SensorDimmer
*/

#if MODULE_DIMMER == 1
// contructor
SensorDimmer::SensorDimmer(NodeManager* node_manager, int child_id, int pin): Sensor(node_manager, child_id, pin) {
  // set presentation, type and value type
  setPresentation(S_DIMMER);
  setType(V_PERCENTAGE);
}

// setter/getter
void SensorDimmer::setEasing(int value) {
  _easing = value;
}
void SensorDimmer::setDuration(int value) {
  _duration = value*1000;
}
void SensorDimmer::setStepDuration(int value) {
  _duration = value;
}

// what to do during before
void SensorDimmer::onBefore() {
  pinMode(_pin, OUTPUT);
}

// what to do during setup
void SensorDimmer::onSetup() {
}

// what to do during loop
void SensorDimmer::onLoop() {
}

// what to do as the main task when receiving a message
void SensorDimmer::onReceive(const MyMessage & message) {
  if (message.getCommand() == C_SET) {
    int percentage = message.getInt();
    // normalize the provided percentage
    if (percentage < 0) percentage = 0;
    if (percentage > 100) percentage = 100;
    fadeTo(percentage);
    _value_int = percentage;
  }
  if (message.getCommand() == C_REQ) {
    // return the current status
    _value_int = _percentage;
  }
}

// what to do when receiving a remote message
void SensorDimmer::onProcess(Request & request) {
   int function = request.getFunction();
  switch(function) {
    case 101: setEasing(request.getValueInt()); break;
    case 102: setDuration(request.getValueInt()); break;
    case 103: setStepDuration(request.getValueInt()); break;
    default: return;
  }
  _send(_msg_service.set(function));
}

// what to do when receiving an interrupt
void SensorDimmer::onInterrupt() {
}

// fade to the provided value
void SensorDimmer::fadeTo(int target_percentage) {
  #if DEBUG == 1
    Serial.print(F("DIM I="));
    Serial.print(_child_id);
    Serial.print(F(" V="));
    Serial.println(target_percentage);
  #endif
  // count how many steps we need to do
  int steps = _duration / _step_duration;
  // for each step
  for (int current_step = 1; current_step <= steps; current_step++) {
    // calculate the delta between the target value and the current
    int delta = target_percentage - _percentage;
    // calculate the smooth transition and adjust it in the 0-255 range
    int value_to_write = (int)(_getEasing(current_step,_percentage,delta,steps) / 100. * 255);
    // write to the PWM output
    analogWrite(_pin,value_to_write);
    // wait at the end of this step
    wait(_step_duration);
  }
  _percentage = target_percentage;
}

// for smooth transitions. t: current time, b: beginning value, c: change in value, d: duration
float SensorDimmer::_getEasing(float t, float b, float c, float d) {
  if (_easing == EASE_INSINE) return -c * cos(t/d * (M_PI/2)) + c + b;
  else if (_easing == EASE_OUTSINE) return c * sin(t/d * (M_PI/2)) + b;
  else if (_easing == EASE_INOUTSINE) return -c/2 * (cos(M_PI*t/d) - 1) + b;
  else return c*t/d + b;
}

#endif

/*******************************************
   NodeManager
*/

// initialize the node manager
NodeManager::NodeManager() {
  // setup the service message container
  _msg = MyMessage(CONFIGURATION_CHILD_ID, V_CUSTOM);
}

int NodeManager::_last_interrupt_pin = -1;
long NodeManager::_last_interrupt_1 = millis();
long NodeManager::_last_interrupt_2 = millis();
long NodeManager::_interrupt_min_delta = 100;

// setter/getter
void NodeManager::setRetries(int value) {
  _retries = value;
}
int NodeManager::getRetries() {
  return _retries;
}
#if BATTERY_MANAGER == 1
  void NodeManager::setBatteryMin(float value) {
    _battery_min = value;
  }
  void NodeManager::setBatteryMax(float value) {
    _battery_max = value;
  }
  void NodeManager::setBatteryReportSeconds(int value) {
    _battery_report_timer.set(value,SECONDS);
  }
  void NodeManager::setBatteryReportMinutes(int value) {
    _battery_report_timer.set(value,MINUTES);
  }
  void NodeManager::setBatteryReportHours(int value) {
    _battery_report_timer.set(value,HOURS);
  }
  void NodeManager::setBatteryReportDays(int value) {
    _battery_report_timer.set(value,DAYS);
  }
  void NodeManager::setBatteryInternalVcc(bool value) {
    _battery_internal_vcc = value;
  }
  void NodeManager::setBatteryPin(int value) {
    _battery_pin = value;
  }
  void NodeManager::setBatteryVoltsPerBit(float value) {
    _battery_volts_per_bit = value;
  }
  void NodeManager::setBatteryReportWithInterrupt(bool value) {
    _battery_report_with_interrupt = value;
  }
#endif

void NodeManager::setSleepSeconds(int value) {
  // set the status to AWAKE if the time provided is 0, SLEEP otherwise
  if (value == 0) _status = AWAKE;
  else _status = SLEEP;
  // store the time
  _sleep_time = value;
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
void NodeManager::setInterruptMinDelta(long value) {
  _interrupt_min_delta = value;
}
#if POWER_MANAGER == 1
  void NodeManager::setPowerPins(int ground_pin, int vcc_pin, int wait_time) {
    _powerManager.setPowerPins(ground_pin, vcc_pin, wait_time);
  }
  void NodeManager::setAutoPowerPins(bool value) {
    _auto_power_pins = value;  
  }
  void NodeManager::powerOn() {
    _powerManager.powerOn();
  }
  void NodeManager::powerOff() {
    _powerManager.powerOff();
  }
#endif
void NodeManager::setSleepBetweenSend(int value) {
  _sleep_between_send = value;
}
int NodeManager::getSleepBetweenSend() {
  return _sleep_between_send;
}
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

// register a sensor to this manager
int NodeManager::registerSensor(int sensor_type, int pin, int child_id) {
  // get a child_id if not provided by the user
  if (child_id < 0) child_id = _getAvailableChildId();
  // based on the given sensor type instantiate the appropriate class
  if (sensor_type < 0) return -1;
  #if MODULE_ANALOG_INPUT == 1
    else if (sensor_type == SENSOR_ANALOG_INPUT) return registerSensor(new SensorAnalogInput(this,child_id, pin));
    else if (sensor_type == SENSOR_LDR) return registerSensor(new SensorLDR(this,child_id, pin));
    else if (sensor_type == SENSOR_THERMISTOR) return registerSensor(new SensorThermistor(this,child_id, pin));
    else if (sensor_type == SENSOR_ML8511) return registerSensor(new SensorML8511(this,child_id, pin));
    else if (sensor_type == SENSOR_ACS712) return registerSensor(new SensorACS712(this,child_id, pin));
    else if (sensor_type == SENSOR_RAIN_GAUGE) return registerSensor(new SensorRainGauge(this,child_id, pin));
    else if (sensor_type == SENSOR_RAIN) return registerSensor(new SensorRain(this,child_id, pin));
    else if (sensor_type == SENSOR_SOIL_MOISTURE) return registerSensor(new SensorSoilMoisture(this,child_id, pin));
  #endif
  #if MODULE_DIGITAL_INPUT == 1
    else if (sensor_type == SENSOR_DIGITAL_INPUT) return registerSensor(new SensorDigitalInput(this,child_id, pin));
  #endif
  #if MODULE_DIGITAL_OUTPUT == 1
    else if (sensor_type == SENSOR_DIGITAL_OUTPUT) return registerSensor(new SensorDigitalOutput(this,child_id, pin));
    else if (sensor_type == SENSOR_RELAY) return registerSensor(new SensorRelay(this,child_id, pin));
    else if (sensor_type == SENSOR_LATCHING_RELAY) return registerSensor(new SensorLatchingRelay(this,child_id, pin));
  #endif
  #if MODULE_DHT == 1
    else if (sensor_type == SENSOR_DHT11 || sensor_type == SENSOR_DHT22) {
      int dht_type;
      if (sensor_type == SENSOR_DHT11) dht_type = DHT::DHT11;
      else if (sensor_type == SENSOR_DHT22) dht_type = DHT::DHT22;
      DHT* dht = new DHT();
      // register temperature sensor
      registerSensor(new SensorDHT(this,child_id,pin,dht,SensorDHT::TEMPERATURE,dht_type));
      // register humidity sensor
      child_id = _getAvailableChildId();
      return registerSensor(new SensorDHT(this,child_id,pin,dht,SensorDHT::HUMIDITY,dht_type));
    }
  #endif
  #if MODULE_SHT21 == 1
    else if (sensor_type == SENSOR_SHT21) {
      // register temperature sensor
      registerSensor(new SensorSHT21(this,child_id,SensorSHT21::TEMPERATURE));
      // register humidity sensor
      child_id = _getAvailableChildId();
      return registerSensor(new SensorSHT21(this,child_id,SensorSHT21::HUMIDITY));
    }
    else if (sensor_type == SENSOR_HTU21D) {
      // register temperature sensor
      registerSensor(new SensorHTU21D(this,child_id,SensorHTU21D::TEMPERATURE));
      // register humidity sensor
      child_id = _getAvailableChildId();
      return registerSensor(new SensorHTU21D(this,child_id,SensorHTU21D::HUMIDITY));
    }
  #endif
  #if MODULE_SWITCH == 1
    else if (sensor_type == SENSOR_SWITCH || sensor_type == SENSOR_DOOR || sensor_type == SENSOR_MOTION) {
      // ensure an interrupt pin is provided
      if (pin != INTERRUPT_PIN_1 && pin != INTERRUPT_PIN_2) return -1;
      // register the sensor
      if (sensor_type == SENSOR_SWITCH) return registerSensor(new SensorSwitch(this,child_id, pin));
      else if (sensor_type == SENSOR_DOOR) return registerSensor(new SensorDoor(this,child_id, pin));
      else if (sensor_type == SENSOR_MOTION) return registerSensor(new SensorMotion(this,child_id, pin));
    }
  #endif
  #if MODULE_DS18B20 == 1
    else if (sensor_type == SENSOR_DS18B20) {
      // initialize the library
      OneWire* oneWire = new OneWire(pin);
      DallasTemperature* sensors = new DallasTemperature(oneWire);
      // initialize the sensors
      sensors->begin();
      int index = 0;
      // register a new child for each sensor on the bus
      for(int i = 0; i < sensors->getDeviceCount(); i++) {
        if (i > 0) child_id = _getAvailableChildId();
        index = registerSensor(new SensorDs18b20(this,child_id,pin,sensors,i));
      }
      return index;
    }
  #endif
  #if MODULE_BH1750 == 1
    else if (sensor_type == SENSOR_BH1750) {
      return registerSensor(new SensorBH1750(this,child_id));
    }
  #endif
  #if MODULE_MLX90614 == 1
    else if (sensor_type == SENSOR_MLX90614) {
      Adafruit_MLX90614* mlx = new Adafruit_MLX90614();
      // register ambient temperature sensor
      registerSensor(new SensorMLX90614(this,child_id,mlx,SensorMLX90614::TEMPERATURE_AMBIENT));
      // register object temperature sensor
      child_id = _getAvailableChildId();
      return registerSensor(new SensorMLX90614(this,child_id,mlx,SensorMLX90614::TEMPERATURE_OBJECT));
    }
  #endif
  #if MODULE_BME280 == 1
    else if (sensor_type == SENSOR_BME280) {
      Adafruit_BME280* bme = new Adafruit_BME280();
      if (! bme->begin(SensorBosch::GetI2CAddress(0x60))) {
        #if DEBUG == 1
          Serial.println(F("NO BME"));
        #endif
        return -1;
      }
      // register temperature sensor
      registerSensor(new SensorBME280(this,child_id,bme,SensorBME280::TEMPERATURE));
      child_id = _getAvailableChildId();
      // register humidity sensor
      registerSensor(new SensorBME280(this,child_id,bme,SensorBME280::HUMIDITY));
      // register pressure sensor
      child_id = _getAvailableChildId();
      registerSensor(new SensorBME280(this,child_id,bme,SensorBME280::PRESSURE));
      // register forecast sensor
      child_id = _getAvailableChildId();
      return registerSensor(new SensorBME280(this,child_id,bme,SensorBME280::FORECAST));
    }
  #endif
  #if MODULE_BMP280 == 1
    else if (sensor_type == SENSOR_BMP280) {
      Adafruit_BMP280* bmp = new Adafruit_BMP280();
      if (! bmp->begin(SensorBosch::GetI2CAddress(0x58))) {
        #if DEBUG == 1
          Serial.println(F("NO BMP"));
        #endif
        return -1;
      }
      // register temperature sensor
      registerSensor(new SensorBMP280(this,child_id,bmp,SensorBMP280::TEMPERATURE));
      child_id = _getAvailableChildId();
      // register pressure sensor
      child_id = _getAvailableChildId();
      registerSensor(new SensorBMP280(this,child_id,bmp,SensorBMP280::PRESSURE));
      // register forecast sensor
      child_id = _getAvailableChildId();
      return registerSensor(new SensorBMP280(this,child_id,bmp,SensorBMP280::FORECAST));
    }
  #endif
  #if MODULE_SONOFF == 1
    else if (sensor_type == SENSOR_SONOFF) {
      return registerSensor(new SensorSonoff(this,child_id));
    }
  #endif
  #if MODULE_BMP085 == 1
    else if (sensor_type == SENSOR_BMP085) {
      Adafruit_BMP085* bmp = new Adafruit_BMP085();
      if (! bmp->begin(SensorBosch::GetI2CAddress(0x55))) {
        #if DEBUG == 1
          Serial.println(F("NO BMP"));
        #endif
        return -1;
      }
      // register temperature sensor
      registerSensor(new SensorBMP085(this,child_id,bmp,SensorBMP085::TEMPERATURE));
      // register pressure sensor
      child_id = _getAvailableChildId();
      registerSensor(new SensorBMP085(this,child_id,bmp,SensorBMP085::PRESSURE));
      // register forecast sensor
      child_id = _getAvailableChildId();
      return registerSensor(new SensorBMP085(this,child_id,bmp,SensorBMP085::FORECAST));
    }
  #endif
  #if MODULE_HCSR04 == 1
    else if (sensor_type == SENSOR_HCSR04) {
      return registerSensor(new SensorHCSR04(this,child_id, pin));
    }
  #endif
  #if MODULE_MCP9808 == 1
    else if (sensor_type == SENSOR_MCP9808) {
      Adafruit_MCP9808 * mcp = new Adafruit_MCP9808();
      if (! mcp->begin()) {
        #if DEBUG == 1
          Serial.println(F("NO MCP"));
        #endif
        return -1;
      }
      // register temperature sensor
      registerSensor(new SensorMCP9808(this,child_id,mcp));
    }
  #endif
  #if MODULE_MQ == 1
    else if (sensor_type == SENSOR_MQ) {
      return registerSensor(new SensorMQ(this,child_id, pin));
    }
  #endif
  #if MODULE_MHZ19 == 1
    else if (sensor_type == SENSOR_MHZ19) {
      return registerSensor(new SensorMHZ19(this, child_id, pin));
    }
  #endif
  #if MODULE_AM2320 == 1
    else if (sensor_type == SENSOR_AM2320) {
      AM2320* th = new AM2320();
      // register temperature sensor
      registerSensor(new SensorAM2320(this,child_id,th,SensorAM2320::TEMPERATURE));
      // register humidity sensor
      child_id = _getAvailableChildId();
      return registerSensor(new SensorAM2320(this,child_id,th,SensorAM2320::HUMIDITY));
    }
  #endif
  #if MODULE_TSL2561 == 1 
    else if (sensor_type == SENSOR_TSL2561) {    
      // register light sensor
      return registerSensor(new SensorTSL2561(this,child_id));
    }
  #endif
   #if MODULE_PT100 == 1 
    else if (sensor_type == SENSOR_PT100) {   
      // register temperature sensor
      return registerSensor(new SensorPT100(this,child_id,pin));
    }
  #endif
   #if MODULE_DIMMER == 1 
    else if (sensor_type == SENSOR_DIMMER) {
      // register the dimmer sensor
      return registerSensor(new SensorDimmer(this,child_id,pin));
    }
  #endif
  else {
    #if DEBUG == 1
      Serial.print(F("INVALID "));
      Serial.println(sensor_type);
    #endif
    return -1;
  };
}

// attach a built-in or custom sensor to this manager
int NodeManager::registerSensor(Sensor* sensor) {
  if (sensor->getChildId() > MAX_SENSORS) return;
  #if DEBUG == 1
    Serial.print(F("REG I="));
    Serial.print(sensor->getChildId());
    Serial.print(F(" P="));
    Serial.print(sensor->getPin());
    Serial.print(F(" P="));
    Serial.print(sensor->getPresentation());
    Serial.print(F(" T="));
    Serial.println(sensor->getType());
  #endif
  #if POWER_MANAGER == 1
    // set auto power pin
    sensor->setAutoPowerPins(_auto_power_pins);
  #endif
  // add the sensor to the array of registered sensors
  _sensors[sensor->getChildId()] = sensor;
  // return the child_id
  return sensor->getChildId();
}

// un-register a sensor to this manager
void NodeManager::unRegisterSensor(int sensor_index) {
  if (sensor_index > MAX_SENSORS) return;
  // unlink the pointer to this sensor
  _sensors[sensor_index] == 0;
}

// return a sensor given its index
Sensor* NodeManager::get(int child_id) {
  if (child_id > MAX_SENSORS) return 0;
  // return a pointer to the sensor from the given child_id
  return _sensors[child_id];
}
Sensor* NodeManager::getSensor(int child_id) {
  return get(child_id);
}

// assign a different child id to a sensor'
bool NodeManager::renameSensor(int old_child_id, int new_child_id) {
  if (old_child_id > MAX_SENSORS || new_child_id > MAX_SENSORS) return;
  // ensure the old id exists and the new is available
  if (_sensors[old_child_id] == 0 || _sensors[new_child_id] != 0) return false;
  // assign the sensor to new id
  _sensors[new_child_id] = _sensors[old_child_id];
  // set the new child id
  _sensors[new_child_id]->setChildId(new_child_id);
  // free up the old id
  _sensors[old_child_id] = 0;
  return true;
}

// setup NodeManager
void NodeManager::before() {
  // print out the version
  #if DEBUG == 1
    Serial.print(F("NodeManager v"));
    Serial.println(VERSION);
  #endif
  // setup the reboot pin if needed
  if (_reboot_pin > -1) {
    #if DEBUG == 1
      Serial.print("REB P=");
      Serial.println(_reboot_pin);
    #endif
    pinMode(_reboot_pin, OUTPUT);
    digitalWrite(_reboot_pin, HIGH);
  }
  // print out MySensors' library capabilities
  #if DEBUG == 1
    Serial.print(F("LIB V="));
    Serial.print(MYSENSORS_LIBRARY_VERSION);
    Serial.print(F(" R="));
    Serial.print(MY_CAP_RADIO);
    #ifdef MY_CAP_ENCR
      Serial.print(F(" E="));
      Serial.print(MY_CAP_ENCR);
    #endif
    Serial.print(F(" T="));
    Serial.print(MY_CAP_TYPE);
    Serial.print(F(" A="));
    Serial.print(MY_CAP_ARCH);
    Serial.print(F(" S="));
    Serial.print(MY_CAP_SIGN);
    Serial.print(F(" B="));
    Serial.println(MY_CAP_RXBUF);
  #endif
  #if PERSIST == 1
    // restore the configuration saved in the eeprom
    _loadConfig();
  #endif
  #if BATTERY_MANAGER == 1 && !defined(MY_GATEWAY_ESP8266)
    // set analogReference to internal if measuring the battery through a pin
    if (! _battery_internal_vcc && _battery_pin > -1) analogReference(INTERNAL);
    // if not already configured, report battery level every 60 minutes
    if (! _battery_report_timer.isConfigured()) _battery_report_timer.set(60,MINUTES);
    _battery_report_timer.start();
  #endif
  #if SIGNAL_SENSOR == 1 && defined(MY_SIGNAL_REPORT_ENABLED)
    // if not already configured, report signal level every 60 minutes
    if (! _signal_report_timer.isConfigured()) _signal_report_timer.set(60,MINUTES);
    _signal_report_timer.start();
  #endif
  // setup individual sensors
  for (int i = 1; i <= MAX_SENSORS; i++) {
    if (_sensors[i] == 0) continue;
    // configure reporting interval
    if (! _sensors[i]->isReportIntervalConfigured()) _sensors[i]->setReportIntervalSeconds(_report_interval_seconds);
    // call each sensor's before()
    _sensors[i]->before();
  }
  // setup the interrupt pins
  setupInterrupts();
}

// present NodeManager and its sensors
void NodeManager::presentation() {
  #if DEBUG == 1
    Serial.println(F("RADIO OK"));
  #endif
  // Send the sketch version information to the gateway and Controller
  if (_sleep_between_send > 0) sleep(_sleep_between_send);
  sendSketchInfo(SKETCH_NAME,SKETCH_VERSION);
  // present the service as a custom sensor to the controller
  _present(CONFIGURATION_CHILD_ID, S_CUSTOM);
  #if BATTERY_MANAGER == 1 && BATTERY_SENSOR == 1
    // present the battery service
    _present(BATTERY_CHILD_ID, S_MULTIMETER);
    // report battery level
    batteryReport();
  #endif
  #if SIGNAL_SENSOR == 1 && defined(MY_SIGNAL_REPORT_ENABLED)
    // present the signal service
    _present(SIGNAL_CHILD_ID, S_SOUND);
    // report battery level
    signalReport();
  #endif
  // present each sensor
  for (int i = 1; i <= MAX_SENSORS; i++) {
    if (_sensors[i] == 0) continue;
    // call each sensor's presentation()
    if (_sleep_between_send > 0) sleep(_sleep_between_send);
    _sensors[i]->presentation();
  }
  #if DEBUG == 1
    Serial.println(F("READY"));
    Serial.println("");
  #endif
}


// setup NodeManager
void NodeManager::setup() {
  // retrieve and store isMetric from the controller
  if (_get_controller_config) _is_metric = getControllerConfig().isMetric;
  #if DEBUG == 1
    Serial.print(F("MY I="));
    Serial.print(getNodeId());
    Serial.print(F(" M="));
    Serial.println(_is_metric);
  #endif
  #if SERVICE_MESSAGES == 1
    _send(_msg.set("STARTED"));
  #endif
  // run setup for all the registered sensors
  for (int i = 1; i <= MAX_SENSORS; i++) {
    if (_sensors[i] == 0) continue;
    // call each sensor's setup()
    _sensors[i]->setup();
  }
}

// run the main function for all the register sensors
void NodeManager::loop() {
  MyMessage empty;
  #if BATTERY_MANAGER == 1
    // update the timer for battery report when not waking up from an interrupt
    if (_battery_report_timer.isRunning() && _last_interrupt_pin == -1) _battery_report_timer.update();
    // if it is time to report the battery level
    if (_battery_report_timer.isOver()) {
      // time to report the battery level again
      batteryReport();
      // restart the timer
      _battery_report_timer.restart();
    }
  #endif
  #if SIGNAL_SENSOR == 1 && defined(MY_SIGNAL_REPORT_ENABLED)
    // update the timer for signal report when not waking up from an interrupt
    if (_signal_report_timer.isRunning() && _last_interrupt_pin == -1) _signal_report_timer.update();
    // if it is time to report the signal level
    if (_signal_report_timer.isOver()) {
      // time to report the signal level again
      signalReport();
      // restart the timer
      _signal_report_timer.restart();
    }
  #endif
  #if POWER_MANAGER == 1
    // turn on the pin powering all the sensors
    if (_auto_power_pins) powerOn();
  #endif
  // run loop for all the registered sensors
  for (int i = 1; i <= MAX_SENSORS; i++) {
    // skip unconfigured sensors
    if (_sensors[i] == 0) continue;
    if (_last_interrupt_pin != -1 && _sensors[i]->getInterruptPin() == _last_interrupt_pin) {
      // if there was an interrupt for this sensor, call the sensor's interrupt() and then loop()
      _sensors[i]->interrupt();
      _sensors[i]->loop(empty);
        // reset the last interrupt pin
      _last_interrupt_pin = -1;
    }
    else if (_last_interrupt_pin == -1) {
      // if just at the end of a cycle, call the sensor's loop() 
      _sensors[i]->loop(empty);
    }
  }
  #if POWER_MANAGER == 1
    // turn off the pin powering all the sensors
    if (_auto_power_pins) powerOff();
  #endif
  // continue/start sleeping as requested
  if (isSleepingNode()) _sleep();
}

// dispacth inbound messages
void NodeManager::receive(const MyMessage &message) {
  #if DEBUG == 1
    Serial.print(F("RECV S="));
    Serial.print(message.sender);
    Serial.print(F(" I="));
    Serial.print(message.sensor);
    Serial.print(F(" C="));
    Serial.print(message.getCommand());
    Serial.print(F(" T="));
    Serial.print(message.type);
    Serial.print(F(" P="));
    Serial.println(message.getString());
  #endif
  // process incoming configuration message
  if (message.sensor == CONFIGURATION_CHILD_ID && message.getCommand() == C_REQ && message.type == V_CUSTOM) {
    #if REMOTE_CONFIGURATION == 1
      // parse the request
      Request request = Request(message.getString());
      // process the request
      process(request);
    #endif
  }
  // dispatch the message to the registered sensor
  else if (message.sensor <= MAX_SENSORS && _sensors[message.sensor] != 0) {
    #if POWER_MANAGER == 1
      // turn on the pin powering all the sensors
      if (_auto_power_pins) powerOn();
    #endif
    // call the sensor's receive()
     _sensors[message.sensor]->receive(message);
    #if POWER_MANAGER == 1
      // turn off the pin powering all the sensors
      if (_auto_power_pins) powerOff();
    #endif
  }
}

// request and return the current timestamp from the controller
long NodeManager::getTimestamp() {
  int retries = 3;
  _timestamp = -1;
  while (_timestamp == -1 && retries > 0) {
    #if DEBUG == 1
      Serial.println(F("TIME"));
    #endif
    // request the time to the controller
    requestTime();
    // keep asking every 1 second
    sleepOrWait(1000);
    retries--;
  }  
  return _timestamp;
}

// receive the time from the controller and save it
void NodeManager::receiveTime(unsigned long ts) {
  _timestamp = ts;
  #if DEBUG == 1
    Serial.print(F("TIME T="));
    Serial.print(_timestamp);
  #endif
}

// process a request message
void NodeManager::process(Request & request) {
  int function = request.getFunction();
  switch(function) {
    case 1: hello(); break;
    #if BATTERY_MANAGER == 1
      case 2: batteryReport(); return;
      case 11: setBatteryMin(request.getValueFloat()); break;
      case 12: setBatteryMax(request.getValueFloat()); break;
      case 14: setBatteryReportMinutes(request.getValueInt()); break;
      case 15: setBatteryInternalVcc(request.getValueInt()); break;
      case 16: setBatteryPin(request.getValueInt()); break;
      case 17: setBatteryVoltsPerBit(request.getValueFloat()); break;
      case 18: setBatteryReportWithInterrupt(request.getValueInt()); break;
    #endif
    case 3:
      setSleepSeconds(request.getValueInt());
      #if PERSIST == 1
        _saveConfig();
      #endif
      break;
    case 4:
      setSleepMinutes(request.getValueInt());
      #if PERSIST == 1
        _saveConfig();
      #endif
      break;
    case 5:
      setSleepHours(request.getValueInt());
      #if PERSIST == 1
        _saveConfig();
      #endif
      break;
    case 29:
      setSleepDays(request.getValueInt());
      #if PERSIST == 1
        _saveConfig();
      #endif
      break;
    #ifndef MY_GATEWAY_ESP8266
      case 6: reboot(); return;
    #endif
    case 7: clearEeprom(); break;
    case 8: version(); return;
    case 9: wakeup(); break;
    case 10: setRetries(request.getValueInt()); break;
    case 19: setSleepInterruptPin(request.getValueInt()); break;
    case 20: setSleepBetweenSend(request.getValueInt()); break;
    case 21: setAck(request.getValueInt()); break;
    case 22: setIsMetric(request.getValueInt()); break;
    #if POWER_MANAGER == 1
      case 23: setAutoPowerPins(request.getValueInt()); break;
      case 24: powerOn(); break;
      case 25: powerOff(); break;
    #endif
    case 26: unRegisterSensor(request.getValueInt()); break;
    case 27: saveToMemory(0,request.getValueInt()); break;
    case 28: setInterruptMinDelta(request.getValueInt()); break;
    case 30: setSleepOrWait(request.getValueInt()); break;
    case 31: setRebootPin(request.getValueInt()); break;
    case 32: setADCOff(); break;
    #if SIGNAL_SENSOR == 1 && defined(MY_SIGNAL_REPORT_ENABLED)
      case 33: setSignalReportMinutes(request.getValueInt()); break;
      case 43: setSignalReportSeconds(request.getValueInt()); break;
      case 44: setSignalReportHours(request.getValueInt()); break;
      case 45: setSignalReportDays(request.getValueInt()); break;
      case 34: setSignalCommand(request.getValueInt()); break;
      case 35: signalReport(); break;
    #endif
    case 36: setReportIntervalSeconds(request.getValueInt()); break;
    case 37: setReportIntervalMinutes(request.getValueInt()); break;
    case 38: setReportIntervalHours(request.getValueInt()); break;
    case 39: setReportIntervalDays(request.getValueInt()); break;
    case 40: setBatteryReportSeconds(request.getValueInt()); break;
    case 41: setBatteryReportHours(request.getValueInt()); break;
    case 42: setBatteryReportDays(request.getValueInt()); break;
    default: return; 
  }
  _send(_msg.set(function));
}


// Send a hello message back to the controller
void NodeManager::hello() {
  // do nothing, the request will be echoed back
}

#if BATTERY_MANAGER == 1
// Send a battery level report to the controller
void NodeManager::batteryReport() {
  // measure the board vcc
  float volt = 0;
  if (_battery_internal_vcc || _battery_pin == -1) volt = getVcc();
  else volt = analogRead(_battery_pin) * _battery_volts_per_bit;
  // calculate the percentage
  int percentage = ((volt - _battery_min) / (_battery_max - _battery_min)) * 100;
  if (percentage > 100) percentage = 100;
  if (percentage < 0) percentage = 0;
  #if DEBUG == 1
    Serial.print(F("BATT V="));
    Serial.print(volt);
    Serial.print(F(" P="));
    Serial.println(percentage);
  #endif
  #if BATTERY_SENSOR == 1
    // report battery voltage
    MyMessage battery_msg(BATTERY_CHILD_ID, V_VOLTAGE);
    _send(battery_msg.set(volt, 2));
  #endif
  // report battery level percentage
  sendBatteryLevel(percentage,_ack);
}
#endif

// reboot the board
void NodeManager::reboot() {
  #if DEBUG == 1
    Serial.println(F("REBOOT"));
  #endif
  if (_reboot_pin > -1) {
    // reboot the board through the reboot pin which is connected to RST by setting it to low
    digitalWrite(_reboot_pin, LOW);
  } else {
    // Software reboot with watchdog timer. Enter Watchdog Configuration mode:
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    // Reset enable
    WDTCSR= (1<<WDE);
    // Infinite loop until watchdog reset after 16 ms
    while(true){}
  }
}

// send NodeManager's the version back to the controller
void NodeManager::version() {
  _send(_msg.set(VERSION));
}

// clear the EEPROM
void NodeManager::clearEeprom() {
  #if DEBUG == 1
    Serial.println(F("CLEAR"));
  #endif
  for (uint16_t i=0; i<EEPROM_LOCAL_CONFIG_ADDRESS; i++) saveState(i, 0xFF);
}

// wake up the board
void NodeManager::wakeup() {
  #if DEBUG == 1
    Serial.println(F("WAKEUP"));
  #endif
  _status = AWAKE;
}

// return the value stored at the requested index from the EEPROM
int NodeManager::loadFromMemory(int index) {
  return loadState(index+EEPROM_USER_START);
}

// save the given index of the EEPROM the provided value
void NodeManager::saveToMemory(int index, int value) {
  saveState(index+EEPROM_USER_START, value);
}

// return vcc in V
float NodeManager::getVcc() {
  #ifndef MY_GATEWAY_ESP8266
    // Measure Vcc against 1.1V Vref
    #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
      ADMUX = (_BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1));
    #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
      ADMUX = (_BV(MUX5) | _BV(MUX0));
    #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
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
    if (_status != SLEEP) attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_1), _onInterrupt_1, _interrupt_1_mode);
  }
  if (_interrupt_2_mode != MODE_NOT_DEFINED) {
    pinMode(INTERRUPT_PIN_2, INPUT);
    if (_interrupt_2_initial > -1) digitalWrite(INTERRUPT_PIN_2,_interrupt_2_initial);
    // for non sleeping nodes, we need to handle the interrupt by ourselves  
    if (_status != SLEEP) attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_2), _onInterrupt_2, _interrupt_2_mode);
  }
  #if DEBUG == 1
    Serial.print(F("INT P="));
    Serial.print(INTERRUPT_PIN_1);
    Serial.print(F(" M="));
    Serial.println(_interrupt_1_mode);
    Serial.print(F("INT P="));
    Serial.print(INTERRUPT_PIN_2);
    Serial.print(F(" M="));
    Serial.println(_interrupt_2_mode);
  #endif
}

// return the pin from which the last interrupt came
int NodeManager::getLastInterruptPin() {
  return _last_interrupt_pin;
}

// set the default interval in seconds all the sensors will report their measures
void NodeManager::setReportIntervalSeconds(int value) {
  _report_interval_seconds = value;
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
  // Disable the ADC by setting the ADEN bit (bit 7) to zero
  ADCSRA = ADCSRA & B01111111;
  // Disable the analog comparator by setting the ACD bit (bit 7) to one
  ACSR = B10000000;
}

// sleep if the node is a battery powered or wait if it is not for the given number of milliseconds 
void NodeManager::sleepOrWait(long value) {
  // if the node is sleeping, sleep-or-wait is enabled and we need to sleep for a decent amount of time, call sleep() otherwise wait()
  if (isSleepingNode() && _sleep_or_wait && value > 200) sleep(value);
  else wait(value);
}

#if SIGNAL_SENSOR == 1 && defined(MY_SIGNAL_REPORT_ENABLED)
  void NodeManager::setSignalReportSeconds(int value) {
    _signal_report_timer.set(value,SECONDS);
  }
  void NodeManager::setSignalReportMinutes(int value) {
    _signal_report_timer.set(value,MINUTES);
  }
  void NodeManager::setSignalReportHours(int value) {
    _signal_report_timer.set(value,HOURS);
  }
  void NodeManager::setSignalReportDays(int value) {
    _signal_report_timer.set(value,DAYS);
  }
  void NodeManager::setSignalCommand(int value) {
    _signal_command = value;
  }
  void NodeManager::signalReport() {
    int16_t value = transportGetSignalReport(_signal_command);
    #if DEBUG == 1
      Serial.print(F("SIG V="));
      Serial.println(value);
    #endif
    // report signal level
    MyMessage signal_msg(SIGNAL_CHILD_ID, V_LEVEL);
    _send(signal_msg.set(value));
  }
#endif

// handle an interrupt
void NodeManager::_onInterrupt_1() {
  long now = millis();
  if ( (now - _last_interrupt_1 > _interrupt_min_delta) || (now < _last_interrupt_1) ) {
    _last_interrupt_pin = INTERRUPT_PIN_1;
    #if DEBUG == 1
      Serial.print(F("INT P="));
      Serial.println(INTERRUPT_PIN_1);
    #endif
    _last_interrupt_1 = now;
  }
}
void NodeManager::_onInterrupt_2() {
  long now = millis();
  if ( (now - _last_interrupt_2 > _interrupt_min_delta) || (now < _last_interrupt_2) ) {
    _last_interrupt_pin = INTERRUPT_PIN_2;
    #if DEBUG == 1
      Serial.print(F("INT P="));
      Serial.println(INTERRUPT_PIN_2);
    #endif
    _last_interrupt_2 = now;
  }
}

// send a message to the network
void NodeManager::_send(MyMessage & message) {
  // send the message, multiple times if requested
  for (int i = 0; i < _retries; i++) {
    // if configured, sleep beetween each send
    if (_sleep_between_send > 0) sleep(_sleep_between_send);
    #if DEBUG == 1
      Serial.print(F("SEND D="));
      Serial.print(message.destination);
      Serial.print(F(" I="));
      Serial.print(message.sensor);
      Serial.print(F(" C="));
      Serial.print(message.getCommand());
      Serial.print(F(" T="));
      Serial.print(message.type);
      Serial.print(F(" S="));
      Serial.print(message.getString());
      Serial.print(F(" I="));
      Serial.print(message.getInt());
      Serial.print(F(" F="));
      Serial.println(message.getFloat());
    #endif
    send(message,_ack);
  }
}

// wrapper of smart sleep
void NodeManager::_sleep() {
  #if DEBUG == 1
    Serial.print(F("SLEEP "));
    Serial.print(_sleep_time);
    Serial.println(F("s"));
  #endif
  #if SERVICE_MESSAGES == 1
    // notify the controller I'm going to sleep
    _send(_msg.set("SLEEPING"));
  #endif
  #if DEBUG == 1
    // print a new line to separate the different cycles
    Serial.println("");
  #endif
  // go to sleep
  int interrupt = -1;
  // setup interrupt pins
  int interrupt_1_pin = _interrupt_1_mode == MODE_NOT_DEFINED ? INTERRUPT_NOT_DEFINED  : digitalPinToInterrupt(INTERRUPT_PIN_1);
  int interrupt_2_pin = _interrupt_2_mode == MODE_NOT_DEFINED ? INTERRUPT_NOT_DEFINED  : digitalPinToInterrupt(INTERRUPT_PIN_2);
  // enter smart sleep for the requested sleep interval and with the configured interrupts
  interrupt = sleep(interrupt_1_pin,_interrupt_1_mode,interrupt_2_pin,_interrupt_2_mode,_sleep_time*1000, true);
  if (interrupt > -1) {
    // woke up by an interrupt
    int pin_number = -1;
    int interrupt_mode = -1;
    // map the interrupt to the pin
    if (digitalPinToInterrupt(INTERRUPT_PIN_1) == interrupt) {
      pin_number = INTERRUPT_PIN_1;
      interrupt_mode = _interrupt_1_mode;
    }
    if (digitalPinToInterrupt(INTERRUPT_PIN_2) == interrupt) {
      pin_number = INTERRUPT_PIN_2;
      interrupt_mode = _interrupt_2_mode;
    }
    _last_interrupt_pin = pin_number;
    #if DEBUG == 1
      Serial.print(F("INT P="));
      Serial.print(pin_number);
      Serial.print(F(", M="));
      Serial.println(interrupt_mode);
    #endif
    // when waking up from an interrupt on the wakup pin, stop sleeping
    if (_sleep_interrupt_pin == pin_number) _status = AWAKE;
  }
  // coming out of sleep
  #if DEBUG == 1
    Serial.println(F("AWAKE"));
  #endif
  #if SERVICE_MESSAGES == 1
    // notify the controller I am awake
    _send(_msg.set("AWAKE"));
  #endif
}

// present the service
void NodeManager::_present(int child_id, int type) {
  #if DEBUG == 1
    Serial.print(F("PRES I="));
    Serial.print(child_id);
    Serial.print(F(", T="));
    Serial.println(type);
  #endif
  if (_sleep_between_send > 0) sleep(_sleep_between_send);
  present(child_id,type,"",_ack);
}

// return the next available child_id
int NodeManager::_getAvailableChildId() {
  for (int i = 1; i <= MAX_SENSORS; i++) {
    if (i == CONFIGURATION_CHILD_ID) continue;
    if (i == BATTERY_CHILD_ID) continue;
    // empty place, return it
    if (_sensors[i] == 0) return i;
  }
  return MAX_SENSORS;
}

// guess the initial value of a digital output based on the configured interrupt mode
int NodeManager::_getInterruptInitialValue(int mode) {
  if (mode == RISING) return LOW; 
  if (mode == FALLING) return HIGH; 
  return -1;
}

// load the configuration stored in the eeprom
void NodeManager::_loadConfig() {
  if (loadState(EEPROM_SLEEP_SAVED) == 1) {
    // load sleep settings
    int bit_1 = loadState(EEPROM_SLEEP_1);
    int bit_2 = loadState(EEPROM_SLEEP_2);
    int bit_3 = loadState(EEPROM_SLEEP_3);
    _sleep_time = bit_3*255*255 + bit_2*255 + bit_1;
    #if DEBUG == 1
      Serial.print(F("LOADSLP T="));
      Serial.println(_sleep_time);
    #endif
  }
}

// save the configuration in the eeprom
void NodeManager::_saveConfig() {
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
