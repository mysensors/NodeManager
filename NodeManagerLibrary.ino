/*
 * NodeManager Library
 */
 
/***************************************
   PowerManager
*/

PowerManager::PowerManager(int ground_pin, int vcc_pin, int wait_time) {
  setPowerPins(ground_pin, vcc_pin, wait_time);
}

// set the vcc and ground pin the sensor is connected to
void PowerManager::setPowerPins(int ground_pin, int vcc_pin, int wait_time) {
  _ground_pin = ground_pin;
  _vcc_pin = vcc_pin;
  #ifdef NODEMANAGER_DEBUG
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
  #ifdef NODEMANAGER_DEBUG
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
  #ifdef NODEMANAGER_DEBUG
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
  _node = node_manager;
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
  if (! _node->isSleepingNode()) _last_millis = millis();
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
  if (_node->isSleepingNode()) {
    // millis() is not reliable while sleeping so calculate how long a sleep cycle would last in seconds and update the elapsed time
    _elapsed += _node->getSleepSeconds();
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

// contructor, tokenize a request in the format "child_id,function,value"
Request::Request(int recipient_child_id, const char* string) {
  _recipient_child_id = recipient_child_id;
  char* ptr;
  // tokenize the string and get child id
  _child_id = atoi(strtok_r(const_cast<char*>(string), ",", &ptr));
  // tokenize the string and get function id
  _function = atoi(strtok_r(NULL, ",", &ptr));
  // tokenize the string and get the value
  _value = atof(strtok_r(NULL, ",", &ptr));
  #ifdef NODEMANAGER_DEBUG
    Serial.print(F("REQ C="));
    Serial.print(_child_id);
    Serial.print(F(" F="));
    Serial.print(_function);
    Serial.print(F(" V="));
    Serial.println(_value);
  #endif
}

// return the child id
int Request::getRecipientChildId() {
  return _recipient_child_id;
}

// return the child id
int Request::getChildId() {
  return _child_id;
}

// return the parsed function
int Request::getFunction() {
  return _function;
}

// return the value as an int
int Request::getValueInt() {
  return (int)_value;
  
}

// return the value as a float
float Request::getValueFloat() {
  return _value;
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
Child::Child(Sensor* __sensor, int _child_id, int _presentation, int _type, char* _description = "") {
  child_id = _child_id;
  presentation = _presentation;
  type = _type;
  description = _description;
  _sensor = __sensor;
  _sensor->registerChild(this);
  force_update_timer = new Timer(_sensor->_node);
}
// set a value, implemented by the subclasses
void Child::sendValue() {
}

// check if it is an updated value, implemented by the subclasses
bool Child::isNewValue() {
}

/*
 ChildInt class
*/

// ChildInt class
ChildInt::ChildInt(Sensor* sensor, int child_id, int presentation, int type, char* description = ""): Child(sensor, child_id, presentation, type, description)  {
}

// store a new value and update the total
void ChildInt::setValueInt(int value) {
  _total = _total + value;
  _samples++;
  _value = (int) (_total / _samples);
}

// return the value
int ChildInt::getValueInt() {
  return _value;
}

// send the value back to the controller
void ChildInt::sendValue() {
  if (_samples == 0) return;
  _sensor->_node->sendMessage(child_id,type,_value);
  _last_value = _value;
  _total = 0;
  _samples = 0;
}

// check if it is an updated value
bool ChildInt::isNewValue() {
  return _last_value != _value;
}

/*
 ChildFloat class
*/

// ChildFloat class
ChildFloat::ChildFloat(Sensor* sensor, int child_id, int presentation, int type, char* description = ""): Child(sensor, child_id, presentation, type, description)  {
}

// store a new value and update the total
void ChildFloat::setValueFloat(float value) {
  _total = _total + value;
  _samples++;
  _value = _total / _samples;
}

// return the value
float ChildFloat::getValueFloat() {
  return _value;
}

// send the value back to the controller
void ChildFloat::sendValue() {
  if (_samples == 0) return;
  _sensor->_node->sendMessage(child_id,type,_value);
  _last_value = _value;
  _total = 0;
  _samples = 0;
}

// check if it is an updated value
bool ChildFloat::isNewValue() {
  return _last_value != _value;
}

/*
 ChildDouble class
*/

// ChildDouble class
ChildDouble::ChildDouble(Sensor* sensor, int child_id, int presentation, int type, char* description = ""): Child(sensor, child_id, presentation, type, description)  {
}

// store a new value and update the total
void ChildDouble::setValueDouble(double value) {
  _total = _total + value;
  _samples++;
  _value = _total / _samples;
}

// return the value
double ChildDouble::getValueDouble() {
  return _value;
}

// send the value back to the controller
void ChildDouble::sendValue() {
  if (_samples == 0) return;
  _sensor->_node->sendMessage(child_id,type,_value);
  _last_value = _value;
  _total = 0;
  _samples = 0;
}

// check if it is an updated value
bool ChildDouble::isNewValue() {
  return _last_value != _value;
}

/*
 ChildString class
*/

// ChildString class
ChildString::ChildString(Sensor* sensor, int child_id, int presentation, int type, char* description = ""): Child(sensor, child_id, presentation, type, description)  {
}

// store a new value and update the total
void ChildString::setValueString(char* value) {
  _value = value;
}

// return the value
char* ChildString::getValueString() {
  return _value;
}

// send the value back to the controller
void ChildString::sendValue() {
  _sensor->_node->sendMessage(child_id,type,_value);
  _last_value = _value;
  _value = "";
}

// check if it is an updated value
bool ChildString::isNewValue() {
  return strcmp(_value, _last_value) != 0;
}

/*
   Sensor class
*/
// constructor
Sensor::Sensor() {  
}
Sensor::Sensor(NodeManager& node_manager, int pin = -1) {
  _node = &node_manager;
  _pin = pin;
  _report_timer = new Timer(_node);
  _node->registerSensor(this);
}

// return the name of the sensor
char* Sensor::getName() {
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
void Sensor::setTrackLastValue(bool value) {
  _track_last_value = value;
}
void Sensor::setForceUpdateMinutes(int value) {
  for (List<Child*>::iterator itr = children.begin(); itr != children.end(); ++itr) {
    Child* child = *itr;
    child->force_update_timer->start(value,MINUTES);
  }
}
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
int Sensor::getInterruptPin() {
  return _interrupt_pin;
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
  _node->setInterrupt(pin,mode,initial);
}

// register a child
void Sensor::registerChild(Child* child) {
  children.push(child);
}

// present the sensor to the gateway and controller
void Sensor::presentation() {
  for (List<Child*>::iterator itr = children.begin(); itr != children.end(); ++itr) {
    Child* child = *itr;
    #ifdef NODEMANAGER_DEBUG
      Serial.print(F("PRES I="));
      Serial.print(child->child_id);
      Serial.print(F(" T="));
      Serial.println(child->presentation);
    #endif
    present(child->child_id, child->presentation,child->description,_node->getAck());
  }

}

// call the sensor-specific implementation of before
void Sensor::before() {
  onBefore();
  for (List<Child*>::iterator itr = children.begin(); itr != children.end(); ++itr) {
    Child* child = *itr;
    #ifdef NODEMANAGER_DEBUG
      Serial.print(_name);
      Serial.print(F(" I="));
      Serial.print(child->child_id);
      Serial.print(F(" P="));
      Serial.print(child->presentation);
      Serial.print(F(" T="));
      Serial.println(child->type);
    #endif
  }
}

// call the sensor-specific implementation of setup
void Sensor::setup() {
  onSetup();
}

// call the sensor-specific implementation of loop
void Sensor::loop(MyMessage* message) {
  // update the timers if within a loop cycle
  if (message == nullptr) {
    if (_report_timer->isRunning()) {
      // keep track if it is the first time
      bool first_run = _report_timer->isFirstRun();
      // update the timer
      _report_timer->update();
      // if it is not the time yet to report a new measure, just return (unless it is the first time)
      if (! _report_timer->isOver() && ! first_run) return;
    }
  }
  // turn the sensor on
  powerOn();
  // iterates over all the children
  for (List<Child*>::iterator itr = children.begin(); itr != children.end(); ++itr) {
    Child* child = *itr;
    // update the force update timer if running
    if (child->force_update_timer->isRunning()) child->force_update_timer->update();
    // if a specific child is requested, skip all the others
    if (message != nullptr && message->sensor != child->child_id) continue;
    // collect multiple samples if needed
    for (int i = 0; i < _samples; i++) {
      // we've been called from receive(), pass the message along
      if (message != nullptr) onReceive(message);
      // we'be been called from loop()
      else onLoop(child);
      // wait between samples
      if (_samples_interval > 0) _node->sleepOrWait(_samples_interval);
    }
    // process the result and send a response back if 1) is not a loop 2) not tracking last value 3) tracking last value and there is a new value 4) tracking last value and timer is over
    if (
      message != nullptr || 
      ! _track_last_value || 
      _track_last_value && child->isNewValue() || 
      _track_last_value && child->force_update_timer->isRunning() && child->force_update_timer->isOver()) 
        child->sendValue();
  }
  // turn the sensor off
  powerOff();
  // if called from loop(), restart the report timer if over
  if (message == nullptr && _report_timer->isRunning() && _report_timer->isOver()) _report_timer->restart();
}

// receive and handle an interrupt
void Sensor::interrupt() {
  // call the implementation of onInterrupt()
  onInterrupt();
}

// receive a message from the radio network
void Sensor::receive(MyMessage &message) {
  // a request would make the sensor executing its main task passing along the message
  loop(&message);
}

// return the requested child 
Child* Sensor::getChild(int child_id) {
  for (List<Child*>::iterator itr = children.begin(); itr != children.end(); ++itr) {
    Child* child = *itr;
    if (child->child_id == child_id) return child;
  }
  return nullptr;
}

void Sensor::setPowerManager(PowerManager& powerManager) {
  _powerManager = &powerManager;
}


// virtual functions
void Sensor::onBefore() {}
void Sensor::onSetup(){}
void Sensor::onLoop(Child* child){}
void Sensor::onReceive(MyMessage* message){}
void Sensor::onInterrupt(){}

/*
   SensorBattery
*/
#ifndef MY_GATEWAY_ESP8266
// contructor
SensorBattery::SensorBattery(NodeManager& node_manager): Sensor(node_manager) {
  _name = "BATTERY";
  // report battery level every 60 minutes by default
  setReportIntervalMinutes(60);
}
void SensorBattery::setMinVoltage(float value) {
  _battery_min = value;
}
void SensorBattery::setMaxVoltage(float value) {
  _battery_max = value;
}
void SensorBattery::setBatteryInternalVcc(bool value) {
  _battery_internal_vcc = value;
}
void SensorBattery::setBatteryPin(int value) {
  _battery_pin = value;
}
void SensorBattery::setBatteryVoltsPerBit(float value) {
  _battery_volts_per_bit = value;
}

// what to do during before
void SensorBattery::onBefore() {
  new ChildFloat(this,BATTERY_CHILD_ID,S_MULTIMETER,V_VOLTAGE);
}

// what to do during setup
void SensorBattery::onSetup() {
  // when measuring the battery from a pin, analog reference must be internal
  if (! _battery_internal_vcc && _battery_pin > -1)
    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
      analogReference(INTERNAL1V1);
    #else
      analogReference(INTERNAL);
    #endif
}

// what to do during loop
void SensorBattery::onLoop(Child* child) {
  // measure the board vcc
  float volt = 0;
  if (_battery_internal_vcc || _battery_pin == -1) volt = _node->getVcc();
  else volt = analogRead(_battery_pin) * _battery_volts_per_bit;
  // calculate the percentage
  int percentage = ((volt - _battery_min) / (_battery_max - _battery_min)) * 100;
  if (percentage > 100) percentage = 100;
  if (percentage < 0) percentage = 0;
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.print(F(" V="));
    Serial.print(volt);
    Serial.print(F(" %="));
    Serial.println(percentage);
  #endif
  ((ChildFloat*)child)->setValueFloat(volt);
  // report battery level percentage
  sendBatteryLevel(percentage);
}

// what to do as the main task when receiving a message
void SensorBattery::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
}
#endif

/*
   SensorSignal
*/
#ifndef MY_GATEWAY_ESP8266
// contructor
SensorSignal::SensorSignal(NodeManager& node_manager): Sensor(node_manager) {
  _name = "SIGNAL";
  // report signal level every 60 minutes by default
  setReportIntervalMinutes(60);
}
// setter/getter
void SensorSignal::setSignalCommand(int value) {
  _signal_command = value;
}

// what to do during before
void SensorSignal::onBefore() {
  new ChildInt(this,SIGNAL_CHILD_ID,S_SOUND,V_LEVEL);
}

// what to do during setup
void SensorSignal::onSetup() {

}

// what to do during loop
void SensorSignal::onLoop(Child* child) {
  int16_t value = transportGetSignalReport((signalReport_t)_signal_command);
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.print(F(" V="));
    Serial.println(value);
  #endif
  ((ChildInt*)child)->setValueInt(value);
}

// what to do as the main task when receiving a message
void SensorSignal::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
}
#endif

#ifdef MODULE_ANALOG_INPUT
/*
   SensorAnalogInput
*/

// contructor
SensorAnalogInput::SensorAnalogInput(NodeManager& node_manager, int pin): Sensor(node_manager, pin) {
  _name = "ANALOG_I";
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
  new ChildInt(this,_node->getAvailableChildId(),S_CUSTOM,V_CUSTOM);
}

// what to do during setup
void SensorAnalogInput::onSetup() {
  // prepare the pin for input
  pinMode(_pin, INPUT);
}

// what to do during loop
void SensorAnalogInput::onLoop(Child* child) {
  // read the input
  int adc = _getAnalogRead();
  // calculate the percentage
  int percentage = 0;
  if (_output_percentage) percentage = _getPercentage(adc);
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.print(F(" I="));
    Serial.print(child->child_id);
    Serial.print(F(" V="));
    Serial.print(adc);
    Serial.print(F(" %="));
    Serial.println(percentage);
  #endif
  // store the result
  ((ChildInt*)child)->setValueInt(_output_percentage ? percentage : adc);
}

// what to do during loop
void SensorAnalogInput::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
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
  if (_reverse) value = _range_max - value;
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
SensorLDR::SensorLDR(NodeManager& node_manager, int pin): SensorAnalogInput(node_manager, pin) {
  _name = "LDR";
}

// what to do during before
void SensorLDR::onBefore() {
  new ChildInt(this,_node->getAvailableChildId(),S_LIGHT_LEVEL,V_LIGHT_LEVEL);
}

// what to do during setup
void SensorLDR::onSetup() {
  setReverse(true);
}

/*
   SensorRain
*/

// contructor
SensorRain::SensorRain(NodeManager& node_manager, int pin): SensorAnalogInput(node_manager, pin) {
  _name = "RAIN";
}

// what to do during before
void SensorRain::onBefore() {
  new ChildInt(this,_node->getAvailableChildId(),S_RAIN,V_RAINRATE);
}

// what to do during setup
void SensorRain::onSetup() {
  setReference(DEFAULT);
  setOutputPercentage(true);
  setReverse(true);
  setRangeMin(100);
}

/*
   SensorSoilMoisture
*/

// contructor
SensorSoilMoisture::SensorSoilMoisture(NodeManager& node_manager, int pin): SensorAnalogInput(node_manager, pin) {
  _name = "SOIL";
}

// what to do during before
void SensorSoilMoisture::onBefore() {
  new ChildInt(this,_node->getAvailableChildId(),S_MOISTURE,V_LEVEL);
}

// what to do during setup
void SensorSoilMoisture::onSetup() {
  setReverse(true);
  setReference(DEFAULT);
  setOutputPercentage(true);
  setReverse(true);
  setRangeMin(100);
}
#endif

#ifdef MODULE_THERMISTOR
/*
   SensorThermistor
*/

// contructor
SensorThermistor::SensorThermistor(NodeManager& node_manager, int pin): Sensor(node_manager, pin) {
  _name = "THERMISTOR";
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
  new ChildFloat(this,_node->getAvailableChildId(),S_TEMP,V_TEMP);
}

// what to do during setup
void SensorThermistor::onSetup() {
  // set the pin as input
  pinMode(_pin, INPUT);
}

// what to do during loop
void SensorThermistor::onLoop(Child* child) {
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
  temperature = _node->celsiusToFahrenheit(temperature);
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.print(F(" I="));
    Serial.print(child->child_id);
    Serial.print(F(" V="));
    Serial.print(adc);
    Serial.print(F(" T="));
    Serial.println(temperature);
  #endif
  // store the value
  if (! isnan(temperature)) ((ChildFloat*)child)->setValueFloat(temperature);
}

// what to do as the main task when receiving a message
void SensorThermistor::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
}
#endif

#ifdef MODULE_ML8511
/*
   SensorML8511
*/

// contructor
SensorML8511::SensorML8511(NodeManager& node_manager, int pin): Sensor(node_manager, pin) {
  _name = "ML8511";
}

// what to do during before
void SensorML8511::onBefore() {
  new ChildFloat(this,_node->getAvailableChildId(),S_UV,V_UV);
}

// what to do during setup
void SensorML8511::onSetup() {
  // set the pin as input
  pinMode(_pin, INPUT);
}

// what to do during loop
void SensorML8511::onLoop(Child* child) {
  // read the voltage 
  int uvLevel = analogRead(_pin);
  int refLevel = _node->getVcc()*1024/3.3;
  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  //Convert the voltage to a UV intensity level
  float uvIntensity = _mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); 
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.print(F(" I="));
    Serial.print(child->child_id);
    Serial.print(F(" V="));
    Serial.print(outputVoltage);
    Serial.print(F(" I="));
    Serial.println(uvIntensity);
  #endif
  // store the value
  ((ChildFloat*)child)->setValueFloat(uvIntensity);
}

// what to do as the main task when receiving a message
void SensorML8511::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
}

// The Arduino Map function but for floats
float SensorML8511::_mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#endif

#ifdef MODULE_ACS712
/*
   SensorACS712
*/

// contructor
SensorACS712::SensorACS712(NodeManager& node_manager, int pin): Sensor(node_manager, pin) {
  _name = "ACS712";
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
  new ChildFloat(this,_node->getAvailableChildId(),S_MULTIMETER,V_CURRENT);
}

// what to do during setup
void SensorACS712::onSetup() {
  // set the pin as input
  pinMode(_pin, INPUT);
}

// what to do during loop
void SensorACS712::onLoop(Child* child) {
  int value = analogRead(_pin);
  // convert the analog read in mV
  double voltage = (value / 1024.0) * 5000; 
  // convert voltage in amps
  float value_float = ((voltage - _ACS_offset) / _mv_per_amp);
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.print(F(" I="));
    Serial.print(child->child_id);
    Serial.print(F(" A="));
    Serial.println(value_float);
  #endif
  ((ChildFloat*)child)->setValueFloat(value_float);
}

// what to do as the main task when receiving a message
void SensorACS712::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
}

#endif

#ifdef MODULE_DIGITAL_INPUT
/*
   SensorDigitalInput
*/

// contructor
SensorDigitalInput::SensorDigitalInput(NodeManager& node_manager, int pin): Sensor(node_manager, pin) {
  _name = "DIGITAL_I";
}

// what to do during before
void SensorDigitalInput::onBefore() {
  // register the child
  new ChildInt(this,_node->getAvailableChildId(),S_CUSTOM,V_CUSTOM);
}

// what to do during setup
void SensorDigitalInput::onSetup() {
  // set the pin for input
  pinMode(_pin, INPUT);
}

// what to do during loop
void SensorDigitalInput::onLoop(Child* child) {
  // read the value
  int value = digitalRead(_pin);
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.print(F(" I="));
    Serial.print(child->child_id);
    Serial.print(F(" P="));
    Serial.print(_pin);
    Serial.print(F(" V="));
    Serial.println(value);
  #endif
  // store the value
  ((ChildInt*)child)->setValueInt(value);
}

// what to do as the main task when receiving a message
void SensorDigitalInput::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
}
#endif


#ifdef MODULE_DIGITAL_OUTPUT
/*
   SensorDigitalOutput
*/

SensorDigitalOutput::SensorDigitalOutput(NodeManager& node_manager, int pin): Sensor(node_manager, pin) {
  _name = "DIGITAL_O";
}

// what to do during before
void SensorDigitalOutput::onBefore() {
  new ChildInt(this,_node->getAvailableChildId(),S_CUSTOM,V_CUSTOM);
}

// what to do during setup
void SensorDigitalOutput::onSetup() {
  _setupPin(children.get(1), _pin);
  _safeguard_timer = new Timer(_node);
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
void SensorDigitalOutput::onLoop(Child* child) {
  // if a safeguard is set, check if it is time for it
  if (_safeguard_timer->isRunning()) {
    // update the timer
    _safeguard_timer->update();
    // if the time is over, turn the output off
    if (_safeguard_timer->isOver()) setStatus(child,OFF);
  }
}

// what to do as the main task when receiving a message
void SensorDigitalOutput::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  // by default handle a SET message but when legacy mode is set when a REQ message is expected instead
  if ( (message->getCommand() == C_SET && ! _legacy_mode) || (message->getCommand() == C_REQ && _legacy_mode)) {
    // switch the output
    setStatus(child, message->getInt());
  }
  if (message->getCommand() == C_REQ && ! _legacy_mode) {
    // just return the current status
    ((ChildInt*)child)->setValueInt(_status);
  }
}

// write the value to the output
void SensorDigitalOutput::setStatus(Child* child, int value) {
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
  _setStatus(child, value);
  // wait if needed for relay drawing a lot of current
  if (_wait_after_set > 0) _node->sleepOrWait(_wait_after_set);
  // store the new status so it will be sent to the controller
  _status = value;
  ((ChildInt*)child)->setValueInt(value);
}

// setup the provided pin for output
void SensorDigitalOutput::_setupPin(Child* child, int pin) {
  // set the pin as output and initialize it accordingly
  pinMode(pin, OUTPUT);
  // setup the pin in a off status
  _status = ! _on_value;
  digitalWrite(pin, _status);
  // the initial value is now the current value
  ((ChildInt*)child)->setValueInt(_status);
}

// switch to the requested status
void SensorDigitalOutput::_setStatus(Child* child, int value) {
  int value_to_write = _getValueToWrite(value);
  // set the value to the pin
  digitalWrite(_pin, value_to_write);
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.print(F(" I="));
    Serial.print(child->child_id);
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
SensorRelay::SensorRelay(NodeManager& node_manager, int pin): SensorDigitalOutput(node_manager, pin) {
  _name = "RELAY";
}

// what to do during before
void SensorRelay::onBefore() {
  new ChildInt(this,_node->getAvailableChildId(),S_BINARY,V_STATUS);
}

/*
   SensorLatchingRelay
*/

// contructor
SensorLatchingRelay::SensorLatchingRelay(NodeManager& node_manager, int pin): SensorRelay(node_manager, pin) {
  _name = "LATCHING";
  // set the "off" pin to the provided pin and the "on" pin to the provided pin + 1
  _pin_on = pin;
  _pin_off = pin + 1;
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
  new ChildInt(this,_node->getAvailableChildId(),S_BINARY,V_STATUS);
}

// what to do during setup
void SensorLatchingRelay::onSetup() {
  _setupPin(children.get(1),_pin_on);
  _setupPin(children.get(1),_pin_off);
}

// switch to the requested status
void SensorLatchingRelay::_setStatus(Child* child, int value) {
  // select the right pin to send the pulse to
  int pin = value == OFF ? _pin_off : _pin_on;
  // set the value
  digitalWrite(pin, _on_value);
  // wait for the given time before restoring the value to the original value after the pulse
  _node->sleepOrWait(_pulse_width);
  digitalWrite(pin, ! _on_value);
  #ifdef NODEMANAGER_DEBUG
    Serial.print(F("LAT I="));
    Serial.print(child->child_id);
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

#ifdef MODULE_DHT
/*
   SensorDHT
*/

// contructor
SensorDHT::SensorDHT(NodeManager& node_manager, int pin): Sensor(node_manager, pin) {
  _name = "DHT";
  _dht_type = DHT::DHT11;
}

// what to do during before
void SensorDHT::onBefore() {
  // register the child
  new ChildFloat(this,_node->getAvailableChildId(),S_TEMP,V_TEMP);
  new ChildFloat(this,_node->getAvailableChildId(),S_HUM,V_HUM);
}

// what to do during setup
void SensorDHT::onSetup() {
  // store the dht object
  _dht = new DHT();
  // initialize the dht library
  _dht->setup(_pin,(DHT::DHT_MODEL_t)_dht_type);
}

// what to do during loop
void SensorDHT::onLoop(Child* child) {
  _node->sleepOrWait(_dht->getMinimumSamplingPeriod());
  _dht->readSensor(true);
  // temperature sensor
  if (child->type == V_TEMP) {
    // read the temperature
    float temperature = _dht->getTemperature();
    if (! _node->getIsMetric()) temperature = _dht->toFahrenheit(temperature);
    #ifdef NODEMANAGER_DEBUG
      Serial.print(_name);
      Serial.print(F(" I="));
      Serial.print(child->child_id);
      Serial.print(F(" T="));
      Serial.println(temperature);
    #endif
    // store the value
    if (! isnan(temperature)) ((ChildFloat*)child)->setValueFloat(temperature);
  }
  // humidity sensor
  else if (child->type == V_HUM) {
    // read humidity
    float humidity = _dht->getHumidity();
    #ifdef NODEMANAGER_DEBUG
      Serial.print(_name);
      Serial.print(F(" I="));
      Serial.print(child->child_id);
      Serial.print(F(" H="));
      Serial.println(humidity);
    #endif
    // store the value
    if (! isnan(humidity)) ((ChildFloat*)child)->setValueFloat(humidity);
  }
}

// what to do as the main task when receiving a message
void SensorDHT::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
}

/*
   SensorDHT11
*/

// contructor
SensorDHT11::SensorDHT11(NodeManager& node_manager, int pin): SensorDHT(node_manager, pin) {
  _name = "DHT11";
  _dht_type = DHT::DHT11;
}

/*
   SensorDHT11
*/

// contructor
SensorDHT22::SensorDHT22(NodeManager& node_manager, int pin): SensorDHT(node_manager, pin) {
  _name = "DHT22";
  _dht_type = DHT::DHT22;
}
#endif

/*
   SensorSHT21
*/
#ifdef MODULE_SHT21
// contructor
SensorSHT21::SensorSHT21(NodeManager& node_manager): Sensor(node_manager) {
  _name = "SHT21";
}

// what to do during before
void SensorSHT21::onBefore() {
  // register the child
  new ChildFloat(this,_node->getAvailableChildId(),S_TEMP,V_TEMP);
  new ChildFloat(this,_node->getAvailableChildId(),S_HUM,V_HUM);
}

// what to do during setup
void SensorSHT21::onSetup() {
  // initialize the library
  Wire.begin();
}

// what to do during loop
void SensorSHT21::onLoop(Child* child) {
  // temperature sensor
  if (child->type == V_TEMP) {
    // read the temperature
    float temperature = SHT2x.GetTemperature();
    // convert it
    temperature = _node->celsiusToFahrenheit(temperature);
    #ifdef NODEMANAGER_DEBUG
      Serial.print(_name);
      Serial.print(F(" I="));
      Serial.print(child->child_id);
      Serial.print(F(" T="));
      Serial.println(temperature);
    #endif
    // store the value
    if (! isnan(temperature)) ((ChildFloat*)child)->setValueFloat(temperature);
  }
  // Humidity Sensor
  else if (child->type == V_HUM) {
    // read humidity
    float humidity = SHT2x.GetHumidity();
    if (isnan(humidity)) return;
    #ifdef NODEMANAGER_DEBUG
      Serial.print(_name);
      Serial.print(F(" I="));
      Serial.print(child->child_id);
      Serial.print(F(" H="));
      Serial.println(humidity);
    #endif
    // store the value
   if (! isnan(humidity)) ((ChildFloat*)child)->setValueFloat(humidity);
  }
}

// what to do as the main task when receiving a message
void SensorSHT21::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
}

/*
 * SensorHTU21D
 */
 // constructor
SensorHTU21D::SensorHTU21D(NodeManager& nodeManager): SensorSHT21(nodeManager) {
  _name = "HTU21";
}
#endif 

#ifdef MODULE_SWITCH
/*
 * SensorSwitch
 */
SensorSwitch::SensorSwitch(NodeManager& node_manager, int pin): Sensor(node_manager, pin) {
  _name = "SWITCH";
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
  new ChildInt(this,_node->getAvailableChildId(),S_CUSTOM,V_TRIPPED);
}

// what to do during setup
void SensorSwitch::onSetup() {
  // set the interrupt pin so it will be called only when waking up from that interrupt
  setInterrupt(_pin,_mode,_initial);
  // report immediately
  _report_timer->unset();
}

// what to do during loop
void SensorSwitch::onLoop(Child* child) {
}

// what to do as the main task when receiving a message
void SensorSwitch::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == V_STATUS) {
    // return current status
    ((ChildInt*)child)->setValueInt(digitalRead(_pin));
  }
}

// what to do when receiving an interrupt
void SensorSwitch::onInterrupt() {
  Child* child = children.get(1);
  // wait to ensure the the input is not floating
  if (_debounce > 0) _node->sleepOrWait(_debounce);
  // read the value of the pin
  int value = digitalRead(_pin);
  // process the value
  if ( (_mode == RISING && value == HIGH ) || (_mode == FALLING && value == LOW) || (_mode == CHANGE) )  {
    #ifdef NODEMANAGER_DEBUG
      Serial.print(_name);
      Serial.print(F(" I="));
      Serial.print(child->child_id);
      Serial.print(F(" P="));
      Serial.print(_pin);
      Serial.print(F(" V="));
      Serial.println(value);
    #endif
    ((ChildInt*)child)->setValueInt(value);
    // allow the signal to be restored to its normal value
    if (_trigger_time > 0) _node->sleepOrWait(_trigger_time);
  } else {
    // invalid
    ((ChildInt*)child)->setValueInt(-255);
  }
}

/*
 * SensorDoor
 */
SensorDoor::SensorDoor(NodeManager& node_manager, int pin): SensorSwitch(node_manager, pin) {
  _name = "DOOR";
}

// what to do during before
void SensorDoor::onBefore() {
  new ChildInt(this,_node->getAvailableChildId(),S_DOOR,V_TRIPPED);
}

/*
 * SensorMotion
 */
SensorMotion::SensorMotion(NodeManager& node_manager, int pin): SensorSwitch(node_manager, pin) {
  _name = "MOTION";
}

// what to do during before
void SensorMotion::onBefore() {
  new ChildInt(this,_node->getAvailableChildId(),S_MOTION,V_TRIPPED);
}

// what to do during setup
void SensorMotion::onSetup() {
  SensorSwitch::onSetup();
  // set initial value to LOW
  setInitial(LOW);
}
#endif

/*
   SensorDs18b20
*/
#ifdef MODULE_DS18B20
// contructor
SensorDs18b20::SensorDs18b20(NodeManager& node_manager, int pin): Sensor(node_manager, pin) {
  _name = "DS18B20";
}

// what to do during before
void SensorDs18b20::onBefore() {
  // initialize the library
  OneWire* oneWire = new OneWire(_pin);
  _sensors = new DallasTemperature(oneWire);
  // initialize the sensors
  _sensors->begin();
  // register a new child for each sensor on the bus
  for(int i = 0; i < _sensors->getDeviceCount(); i++) {
    new ChildFloat(this,_node->getAvailableChildId(),S_TEMP,V_TEMP);
  }
}

// what to do during setup
void SensorDs18b20::onSetup() {
}

// what to do during loop
void SensorDs18b20::onLoop(Child* child) {
  int index = -1;
  // get the index of the requested child
  for (int i = 1; i <= children.size(); i++) {
    if (children.get(i) == child) index = i-1;
  }
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
  float temperature = _sensors->getTempCByIndex(index);
  // convert it
  temperature = _node->celsiusToFahrenheit(temperature);
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.print(F(" I="));
    Serial.print(child->child_id);
    Serial.print(F(" T="));
    Serial.println(temperature);
  #endif
  // store the value
  if (! isnan(temperature)) ((ChildFloat*)child)->setValueFloat(temperature);
}

// what to do as the main task when receiving a message
void SensorDs18b20::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
}

// returns the sensor's resolution in bits
int SensorDs18b20::getResolution() {
  return _sensors->getResolution();
}

// set the sensor's resolution in bits
void SensorDs18b20::setResolution(int value) {
  _sensors->setResolution(value);
}

// sleep while DS18B20 calculates temperature
void SensorDs18b20::setSleepDuringConversion(bool value) {
   _sleep_during_conversion = value;
}

#endif

/*
   SensorBH1750
*/
#ifdef MODULE_BH1750
// contructor
SensorBH1750::SensorBH1750(NodeManager& node_manager): Sensor(node_manager) {
  _name = "BH1750";
}
// setter/getter
void SensorBH1750::setMode(uint8_t mode) {
  _lightSensor->configure(mode);
}

// what to do during before
void SensorBH1750::onBefore() {
  new ChildInt(this,_node->getAvailableChildId(),S_LIGHT_LEVEL,V_LEVEL);
}

// what to do during setup
void SensorBH1750::onSetup() {
  _lightSensor = new BH1750();
  _lightSensor->begin();
}

// what to do during loop
void SensorBH1750::onLoop(Child* child) {
  // request the light level
  int value = _lightSensor->readLightLevel();
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.print(F(" I="));
    Serial.print(child->child_id);
    Serial.print(F(" L="));
    Serial.println(value);
  #endif
  ((ChildInt*)child)->setValueInt(value);
}

// what to do as the main task when receiving a message
void SensorBH1750::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
}
#endif

/*
   SensorMLX90614
*/
#ifdef MODULE_MLX90614
// contructor
SensorMLX90614::SensorMLX90614(NodeManager& node_manager): Sensor(node_manager) {
  _name = "MLX90614";
}

// what to do during before
void SensorMLX90614::onBefore() {
  new ChildFloat(this,_node->getAvailableChildId(),S_TEMP,V_TEMP);
  new ChildFloat(this,_node->getAvailableChildId(),S_TEMP,V_TEMP);
}

// what to do during setup
void SensorMLX90614::onSetup() {
  // initialize the library
  _mlx = new Adafruit_MLX90614();
  _mlx->begin();
}

// what to do during loop
void SensorMLX90614::onLoop(Child* child) {
  float temperature;
  // the first child is the ambient temperature, the second the object temperature
  if (children.get(1) == child) temperature = _mlx->readAmbientTempC();
  else temperature = _mlx->readObjectTempC();
  // convert it
  temperature = _node->celsiusToFahrenheit(temperature);
  #ifdef NODEMANAGER_DEBUG
    Serial.print(F("MLX I="));
    Serial.print(child->child_id);
    Serial.print(F(" T="));
    Serial.println(temperature);
  #endif
  if (! isnan(temperature)) ((ChildFloat*)child)->setValueFloat(temperature);
}

// what to do as the main task when receiving a message
void SensorMLX90614::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
}
#endif


/*
   SensorBosch
*/
#if defined(MODULE_BME280) || defined(MODULE_BMP085) || defined(MODULE_BMP280)
// contructor
SensorBosch::SensorBosch(NodeManager& node_manager): Sensor(node_manager) {
  _name = "BOSH";
  // initialize the forecast samples array
  _forecast_samples = new float[_forecast_samples_count];
}

// setter/getter
void SensorBosch::setForecastSamplesCount(int value) {
  _forecast_samples_count = value;
}

// what to do during before
void SensorBosch::onBefore() {
}

// what to do during setup
void SensorBosch::onSetup() {
}

// what to do during loop
void SensorBosch::onLoop(Child* child) {
}

// what to do as the main task when receiving a message
void SensorBosch::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
}

// calculate and send the forecast back
char* SensorBosch::_forecast(float pressure) {
  if (isnan(pressure)) return "";
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
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.print(F(" M="));
    Serial.print(_minute_count);
    Serial.print(F(" dP="));
    Serial.print(_dP_dt);
    Serial.print(F(" F="));
    Serial.println(_weather[forecast]);
  #endif
  return _weather[forecast];
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
      #ifdef NODEMANAGER_DEBUG
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
#ifdef MODULE_BME280
SensorBME280::SensorBME280(NodeManager& node_manager): SensorBosch(node_manager) {
  _name = "BME280";
}

// what to do during before
void SensorBME280::onBefore() {
  // register the child
  new ChildFloat(this,_node->getAvailableChildId(),S_TEMP,V_TEMP);
  new ChildFloat(this,_node->getAvailableChildId(),S_HUM,V_HUM);
  new ChildFloat(this,_node->getAvailableChildId(),S_BARO,V_PRESSURE);
  new ChildString(this,_node->getAvailableChildId(),S_BARO,V_FORECAST);
}

// what to do during setup
void SensorBME280::onSetup() {
  _bm = new Adafruit_BME280();
  if (! _bm->begin(SensorBosch::GetI2CAddress(0x60))) {
    #ifdef NODEMANAGER_DEBUG
      Serial.println(F("ERR"));
    #endif
  }
}

void SensorBME280::onLoop(Child* child) {
  // temperature sensor
  if (child->type == V_TEMP) {
    // read the temperature
    float temperature = _bm->readTemperature();
    // convert it
    temperature = _node->celsiusToFahrenheit(temperature);
    #ifdef NODEMANAGER_DEBUG
      Serial.print(_name);
      Serial.print(F(" I="));
      Serial.print(child->child_id);
      Serial.print(F(" T="));
      Serial.println(temperature);
    #endif
    // store the value
    if (! isnan(temperature)) ((ChildFloat*)child)->setValueFloat(temperature);
  }
  // Humidity Sensor
  else if (child->type == V_HUM) {
    // read humidity
    float humidity = _bm->readHumidity();
    #ifdef NODEMANAGER_DEBUG
      Serial.print(_name);
      Serial.print(F(" I="));
      Serial.print(child->child_id);
      Serial.print(F(" H="));
      Serial.println(humidity);
    #endif
    // store the value
    if (! isnan(humidity)) ((ChildFloat*)child)->setValueFloat(humidity);
  }
  // Pressure Sensor
  else if (child->type == V_PRESSURE) {
    // read pressure
    float pressure = _bm->readPressure() / 100.0F;
    #ifdef NODEMANAGER_DEBUG
      Serial.print(_name);
      Serial.print(F(" I="));
      Serial.print(child->child_id);
      Serial.print(F(" P="));
      Serial.println(pressure);
    #endif
    // store the value
    if (! isnan(pressure)) ((ChildFloat*)child)->setValueFloat(pressure);
  }
  // Forecast Sensor
  else if (child->type == V_FORECAST) {
    float pressure = _bm->readPressure() / 100.0F;
    _forecast(pressure);
  }
}
#endif

/*
   SensorBMP085
*/
#ifdef MODULE_BMP085
// contructor
SensorBMP085::SensorBMP085(NodeManager& node_manager): SensorBosch(node_manager) {
  _name = "BMP085";
}

// what to do during before
void SensorBMP085::onBefore() {
  // register the child
  new ChildFloat(this,_node->getAvailableChildId(),S_TEMP,V_TEMP);
  new ChildFloat(this,_node->getAvailableChildId(),S_BARO,V_PRESSURE);
  new ChildString(this,_node->getAvailableChildId(),S_BARO,V_FORECAST);
}

// what to do during setup
void SensorBMP085::onSetup() {
  _bm = new Adafruit_BMP085();
  if (! _bm->begin(SensorBosch::GetI2CAddress(0x55))) {
    #ifdef NODEMANAGER_DEBUG
      Serial.println(F("ERR"));
    #endif
  }
}

// what to do during loop
void SensorBMP085::onLoop(Child* child) {
  // temperature sensor
  if (child->type == V_TEMP) {
    // read the temperature
    float temperature = _bm->readTemperature();
    // convert it
    temperature = _node->celsiusToFahrenheit(temperature);
    #ifdef NODEMANAGER_DEBUG
      Serial.print(_name);
      Serial.print(F(" I="));
      Serial.print(child->child_id);
      Serial.print(F(" T="));
      Serial.println(temperature);
    #endif
    // store the value
    if (! isnan(temperature)) ((ChildFloat*)child)->setValueFloat(temperature);
  }
  // Pressure Sensor
  else if (child->type == V_PRESSURE) {
    // read pressure
    float pressure = _bm->readPressure() / 100.0F;
    #ifdef NODEMANAGER_DEBUG
      Serial.print(_name);
      Serial.print(F(" I="));
      Serial.print(child->child_id);
      Serial.print(F(" P="));
      Serial.println(pressure);
    #endif
    // store the value
    if (! isnan(pressure)) ((ChildFloat*)child)->setValueFloat(pressure);
  }
  // Forecast Sensor
  else if (child->type == V_FORECAST) {
    float pressure = _bm->readPressure() / 100.0F;
    _forecast(pressure);
  }
}
#endif

/*
 * SensorBMP280
 */
#ifdef MODULE_BMP280
SensorBMP280::SensorBMP280(NodeManager& node_manager): SensorBosch(node_manager) {
  _name = "BMP280";
}

  // what to do during before
void SensorBMP280::onBefore() {
  // register the child
  new ChildFloat(this,_node->getAvailableChildId(),S_TEMP,V_TEMP);
  new ChildFloat(this,_node->getAvailableChildId(),S_BARO,V_PRESSURE);
  new ChildString(this,_node->getAvailableChildId(),S_BARO,V_FORECAST);
}

// what to do during setup
void SensorBMP280::onSetup() {
  _bm = new Adafruit_BMP280();
  if (! _bm->begin(SensorBosch::GetI2CAddress(0x58))) {
    #ifdef NODEMANAGER_DEBUG
      Serial.println(F("ERR"));
    #endif
  }
}

void SensorBMP280::onLoop(Child* child) {
  // temperature sensor
  if (child->type == V_TEMP) {
    // read the temperature
    float temperature = _bm->readTemperature();
    // convert it
    temperature = _node->celsiusToFahrenheit(temperature);
    #ifdef NODEMANAGER_DEBUG
      Serial.print(_name);
      Serial.print(F(" I="));
      Serial.print(child->child_id);
      Serial.print(F(" T="));
      Serial.println(temperature);
    #endif
    // store the value
    if (! isnan(temperature)) ((ChildFloat*)child)->setValueFloat(temperature);
  }
  // Pressure Sensor
  else if (child->type == V_PRESSURE) {
    // read pressure
    float pressure = _bm->readPressure() / 100.0F;
    #ifdef NODEMANAGER_DEBUG
      Serial.print(_name);
      Serial.print(F(" I="));
      Serial.print(child->child_id);
      Serial.print(F(" P="));
      Serial.println(pressure);
    #endif
    // store the value
    if (! isnan(pressure)) ((ChildFloat*)child)->setValueFloat(pressure);
  }
  // Forecast Sensor
  else if (child->type == V_FORECAST) {
    float pressure = _bm->readPressure() / 100.0F;
    _forecast(pressure);
  }
}
#endif

/*
   SensorSonoff
*/
#ifdef MODULE_SONOFF
// contructor
SensorSonoff::SensorSonoff(NodeManager& node_manager): Sensor(node_manager) {
  _name = "SONOFF";
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
  // register the child
  new ChildInt(this,_node->getAvailableChildId(),S_BINARY,V_STATUS);
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
void SensorSonoff::onLoop(Child* child) {
  _debouncer.update();
  // Get the update value from the button
  int value = _debouncer.read();
  if (value != _old_value && value == 0) {
    // button pressed, toggle the state
    _toggle(child);
  }
  _old_value = value;
}

// what to do as the main task when receiving a message
void SensorSonoff::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_SET) {
    // retrieve from the message the value to set
    int value = message->getInt();
    if (value != 0 && value != 1 || value == _state) return;
    // toggle the state
    _toggle(child);
  }
  if (message->getCommand() == C_REQ) {
    // return the current state
    ((ChildInt*)child)->setValueInt(_state);
  }
}

// toggle the state
void SensorSonoff::_toggle(Child* child) {
  // toggle the state
  _state = _state ? false : true;
  // Change relay state
  digitalWrite(_relay_pin, _state? _relay_on: _relay_off);
  // Change LED state
  digitalWrite(_led_pin, _state? _led_on: _led_off);
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.print(F(" I="));
    Serial.print(child->child_id);
    Serial.print(F(" V="));
    Serial.println(_state);
  #endif
  ((ChildInt*)child)->setValueInt(_state);
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
   SensorHCSR04
*/
#ifdef MODULE_HCSR04
// contructor
SensorHCSR04::SensorHCSR04(NodeManager& node_manager, int pin): Sensor(node_manager, pin) {
  _name = "HCSR04";
  _trigger_pin = pin;
  _echo_pin = pin;
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

// what to do during before
void SensorHCSR04::onBefore() {
  // register the child
  new ChildInt(this,_node->getAvailableChildId(),S_DISTANCE,V_DISTANCE);
}

// what to do during setup
void SensorHCSR04::onSetup() {
  // initialize the library
  _sonar = new NewPing(_trigger_pin,_echo_pin,_max_distance);
}

// what to do during loop
void SensorHCSR04::onLoop(Child* child) {
  int distance = _node->getIsMetric() ? _sonar->ping_cm() : _sonar->ping_in();
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.print(F(" I="));
    Serial.print(child->child_id);
    Serial.print(F(" D="));
    Serial.println(distance);
  #endif
  ((ChildInt*)child)->setValueInt(distance);
}

// what to do as the main task when receiving a message
void SensorHCSR04::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
}
#endif

/*
   SensorMCP9808
*/
#ifdef MODULE_MCP9808
// contructor
SensorMCP9808::SensorMCP9808(NodeManager& node_manager): Sensor(node_manager) {
  _name = "MCP9808";
}

// what to do during before
void SensorMCP9808::onBefore() {
  new ChildFloat(this,_node->getAvailableChildId(),S_TEMP,V_TEMP);
}

// what to do during setup
void SensorMCP9808::onSetup() {
  _mcp = new Adafruit_MCP9808();
}

// what to do during loop
void SensorMCP9808::onLoop(Child* child) {
  float temperature = _mcp->readTempC();
  // convert it
  temperature = _node->celsiusToFahrenheit(temperature);
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.print(F(" I="));
    Serial.print(child->child_id);
    Serial.print(F(" T="));
    Serial.println(temperature);
  #endif
  // store the value
  if (! isnan(temperature)) ((ChildFloat*)child)->setValueFloat(temperature);
}

// what to do as the main task when receiving a message
void SensorMCP9808::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
}
#endif

/*
 * SensorMQ
 */
#ifdef MODULE_MQ

float SensorMQ::_default_LPGCurve[3] = {2.3,0.21,-0.47};
float SensorMQ::_default_COCurve[3] = {2.3,0.72,-0.34};
float SensorMQ::_default_SmokeCurve[3] = {2.3,0.53,-0.44};

SensorMQ::SensorMQ(NodeManager& node_manager, int pin): Sensor(node_manager, pin) {
  _name = "MQ";
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
  // register the child
  new ChildInt(this,_node->getAvailableChildId(),S_AIR_QUALITY,V_LEVEL);
}

// what to do during setup
void SensorMQ::onSetup() {
  _ro = _MQCalibration();
}

// what to do during loop
void SensorMQ::onLoop(Child* child) {
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
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.print(F(" I="));
    Serial.print(child->child_id);
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
  ((ChildInt*)child)->setValueInt((int16_t)ceil(value));
}

// what to do as the main task when receiving a message
void SensorMQ::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
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
#ifdef MODULE_MHZ19
// contructor
SensorMHZ19::SensorMHZ19(NodeManager& node_manager, int rxpin, int txpin): Sensor(node_manager, rxpin) {
  _name = "MHZ19";
  _rx_pin = rxpin;
  _tx_pin = txpin;
}

// what to do during before
void SensorMHZ19::onBefore() {
  // register the child
  new ChildInt(this,_node->getAvailableChildId(),S_AIR_QUALITY,V_LEVEL);
}

// what to do during setup
void SensorMHZ19::onSetup() {
  _ser = new SoftwareSerial(_rx_pin, _tx_pin);
  _ser->begin(9600);
  delay(2000);
  // clear CO2 buffer
  while (_ser->read()!=-1) {};  
}

// what to do during loop
void SensorMHZ19::onLoop(Child* child) {
  // Read the ppm value
  int co2ppm = _readCO2(); 
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.print(F(" I="));
    Serial.print(child->child_id);
    Serial.print(F(" ppm="));
    Serial.println(co2ppm);
  #endif
  // store the value
  ((ChildInt*)child)->setValueInt(co2ppm);
}


// what to do as the main task when receiving a message
void SensorMHZ19::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
}

// Read out the CO2 data
int SensorMHZ19::_readCO2() {
  while (_ser->read() != -1) {};  //clear serial buffer
  unsigned char response[9]; // for answer
  byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  // Command to ask for data.
  _ser->write(cmd, 9); //request PPM CO2
  // Then for 1 second listen for 9 bytes of data.
  _ser->readBytes(response, 9);
  #ifdef NODEMANAGER_DEBUG
  for (int i=0; i<9; i++) {
    Serial.print(response[i], HEX);
    Serial.print(F("-"));
  }
  Serial.println(F("END"));
  #endif
  if (response[0] != 0xFF) {
    Serial.println(F("ERR byte"));
    return -1;
  }
  if (response[1] != 0x86) {
    Serial.println(F("ERR command"));
    return -1;
  }
  int responseHigh = (int) response[2];
  int responseLow = (int) response[3];
  int ppm = (256 * responseHigh) + responseLow;
  return ppm;
}

#endif

/*
   SensorAM2320
*/
#ifdef MODULE_AM2320
// constructor
SensorAM2320::SensorAM2320(NodeManager& node_manager): Sensor(node_manager) {
  _name = "AM2320";
}

// what do to during before
void SensorAM2320::onBefore() {
  // register the child
  new ChildFloat(this,_node->getAvailableChildId(),S_TEMP,V_TEMP);
  new ChildFloat(this,_node->getAvailableChildId(),S_HUM,V_HUM);
}

// what do to during setup
void SensorAM2320::onSetup() {
  _th = new AM2320();
}

// what do to during loop
void SensorAM2320::onLoop(Child* child) {
  // read data from the sensor
  int status = _th->Read();
  if (status != 0) return;
  // temperature sensor
  if (child->type == V_TEMP) {
    float temperature = _th->t;
    #ifdef NODEMANAGER_DEBUG
      Serial.print(_name);
      Serial.print(F(" I="));
      Serial.print(child->child_id);
      Serial.print(F(" T="));
      Serial.println(temperature);
    #endif
    // store the value
    if (! isnan(temperature)) ((ChildFloat*)child)->setValueFloat(temperature);
  }
  // Humidity Sensor
  else if (child->type == V_HUM) {
    // read humidity
    float humidity = _th->h;
    #ifdef NODEMANAGER_DEBUG
      Serial.print(_name);
      Serial.print(F(" I="));
      Serial.print(child->child_id);
      Serial.print(F(" T="));
      Serial.println(humidity);
    #endif
    // store the value
    if (! isnan(humidity)) ((ChildFloat*)child)->setValueFloat(humidity);
  }
}

// what do to as the main task when receiving a message
void SensorAM2320::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
}
#endif

/*
   SensorTSL2561
*/
#ifdef MODULE_TSL2561
// contructor
SensorTSL2561::SensorTSL2561(NodeManager& node_manager): Sensor(node_manager) {
  _name = "TSL2561";
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
  // register the child
  new ChildInt(this,_node->getAvailableChildId(),S_LIGHT_LEVEL,V_LEVEL);
}

// what do to during setup
void SensorTSL2561::onSetup() {
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
    #ifdef NODEMANAGER_DEBUG
      Serial.println(F("ERROR"));
    #endif
  } 
}

// what do to during loop
void SensorTSL2561::onLoop(Child* child) {
  // request the light level
   switch (_tsl_spectrum) {
    case SensorTSL2561::VISIBLE:
      ((ChildInt*)child)->setValueInt(_tsl->getLuminosity(TSL2561_VISIBLE));
      break; 
    case SensorTSL2561::FULLSPECTRUM:
      ((ChildInt*)child)->setValueInt(_tsl->getLuminosity(TSL2561_FULLSPECTRUM));
      break; 
    case SensorTSL2561::INFRARED:
      ((ChildInt*)child)->setValueInt(_tsl->getLuminosity(TSL2561_INFRARED));
      break; 
    case SensorTSL2561::FULL:
      // request the full light level
      uint32_t lum = _tsl->getFullLuminosity(); 
      uint16_t ir, full;
      ir = lum >> 16;
      full = lum & 0xFFFF;
      ((ChildInt*)child)->setValueInt(_tsl->calculateLux(full, ir));
      #ifdef NODEMANAGER_DEBUG
        Serial.print(_name);
        Serial.print(F(" I="));
        Serial.print(child->child_id);
        Serial.print(F(" LUX="));
        Serial.print(((ChildInt*)child)->getValueInt());
        Serial.print(F(" IR="));
        Serial.print(ir);
        Serial.print(F(" FULL="));
        Serial.print(full);
        Serial.print(F(" VIS="));
        Serial.println(full-ir);
      #endif
      break; 
  }
  #ifdef NODEMANAGER_DEBUG
    if (_tsl_spectrum < 3) {
      Serial.print(_name);
      Serial.print(F(" I="));
      Serial.print(child->child_id);
      Serial.print(F(" L="));
      Serial.println(((ChildInt*)child)->getValueInt());
    }
  #endif
}

// what do to as the main task when receiving a message
void SensorTSL2561::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
}
#endif

/*
   SensorPT100
*/
#ifdef MODULE_PT100
// contructor
SensorPT100::SensorPT100(NodeManager& node_manager, int pin): Sensor(node_manager, pin) {
  _name = "PT100";
}

// setter/getter
void SensorPT100::setVoltageRef(float value) {
   _voltageRef = value;
}

// what to do during before
void SensorPT100::onBefore() {
  // register the child
  new ChildFloat(this,_node->getAvailableChildId(),S_TEMP,V_TEMP);
}

// what to do during setup
void SensorPT100::onSetup() {
  _PT100 = new DFRobotHighTemperature(_voltageRef); 
  // set the pin as input
  pinMode(_pin, INPUT);

}

// what to do during loop
void SensorPT100::onLoop(Child* child) {
  // read the PT100 sensor
  int temperature = _PT100->readTemperature(_pin);  
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.print(F(" I="));
    Serial.print(child->child_id);
    Serial.print(F(" T="));
    Serial.println(temperature);
  #endif
  // store the value
  if (! isnan(temperature)) ((ChildFloat*)child)->setValueFloat(temperature);
}

// what to do as the main task when receiving a message
void SensorPT100::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
}
#endif

/*
   SensorDimmer
*/

#ifdef MODULE_DIMMER
// contructor
SensorDimmer::SensorDimmer(NodeManager& node_manager, int pin): Sensor(node_manager, pin) {
  _name = "DIMMER";
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
  // register the child
  new ChildInt(this,_node->getAvailableChildId(),S_DIMMER,V_PERCENTAGE);
}

// what to do during setup
void SensorDimmer::onSetup() {
  pinMode(_pin, OUTPUT);
}

// what to do during loop
void SensorDimmer::onLoop(Child* child) {
}

// what to do as the main task when receiving a message
void SensorDimmer::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_SET && message->type == child->type) {
    int percentage = message->getInt();
    // normalize the provided percentage
    if (percentage < 0) percentage = 0;
    if (percentage > 100) percentage = 100;
    _fadeTo(child,percentage);
    ((ChildInt*)child)->setValueInt(_percentage);
  }
  if (message->getCommand() == C_REQ) {
    // return the current status
    ((ChildInt*)child)->setValueInt(_percentage);
  }
}

// fade to the provided value
void SensorDimmer::_fadeTo(Child* child, int target_percentage) {
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.print(F(" I="));
    Serial.print(child->child_id);
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

/*
   SensorPulseMeter
*/
#ifdef MODULE_PULSE_METER
// contructor
SensorPulseMeter::SensorPulseMeter(NodeManager& node_manager, int pin): Sensor(node_manager, pin) {
  _name = "PULSE";
}

// setter/getter
void SensorPulseMeter::setPulseFactor(float value) {
  _pulse_factor = value;
}
void SensorPulseMeter::setInitialValue(int value) {
  _initial_value = value;
}
void SensorPulseMeter::setInterruptMode(int value) {
  _interrupt_mode = value;
}

// what to do during before
void SensorPulseMeter::onBefore() {
  // register the child
  new ChildFloat(this,_node->getAvailableChildId(),S_CUSTOM,V_CUSTOM);
}

// what to do during setup
void SensorPulseMeter::onSetup() {
  // configure the interrupt pin so onInterrupt() will be called on tip
  setInterrupt(_pin,_interrupt_mode,_initial_value);
}

// what to do during loop
void SensorPulseMeter::onLoop(Child* child) {
  // do not report anything if called by an interrupt
  if (_node->getLastInterruptPin() == _interrupt_pin) return;
  // time to report the rain so far
  _reportTotal(child);
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.print(F(" I="));
    Serial.print(child->child_id);
    Serial.print(F(" T="));
    Serial.println(((ChildFloat*)child)->getValueFloat());
  #endif
  // reset the counter
  _count = 0;
}

// what to do as the main task when receiving a message
void SensorPulseMeter::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) {
    // report the total the last period
    _reportTotal(child);
  }
}

// what to do when receiving an interrupt
void SensorPulseMeter::onInterrupt() {
  // increase the counter
  _count++;
  #ifdef NODEMANAGER_DEBUG
    Serial.print(_name);
    Serial.println(F("+"));
  #endif
}

// return the total based on the pulses counted
void SensorPulseMeter::_reportTotal(Child* child) {
  ((ChildFloat*)child)->setValueFloat(_count / _pulse_factor);
}

/*
   SensorRainGauge
*/
// contructor
SensorRainGauge::SensorRainGauge(NodeManager& node_manager, int pin): SensorPulseMeter(node_manager, pin) {
  _name = "RAIN_GAUGE";
}

// what to do during before
void SensorRainGauge::onBefore() {
  // register the child
  new ChildFloat(this,_node->getAvailableChildId(),S_RAIN,V_RAIN);
  setPulseFactor(9.09);
}

/*
   SensorPowerMeter
*/
// contructor
SensorPowerMeter::SensorPowerMeter(NodeManager& node_manager, int pin): SensorPulseMeter(node_manager, pin) {
  _name = "POWER";
}

// what to do during before
void SensorPowerMeter::onBefore() {
  // register the child
  new ChildDouble(this,_node->getAvailableChildId(),S_POWER,V_KWH);
  setPulseFactor(1000);
}

// return the total based on the pulses counted
void SensorPowerMeter::_reportTotal(Child* child) {
  ((ChildDouble*)child)->setValueDouble(_count / _pulse_factor);
}

/*
   SensorWaterMeter
*/
// contructor
SensorWaterMeter::SensorWaterMeter(NodeManager& node_manager, int pin): SensorPulseMeter(node_manager, pin) {
  _name = "WATER";
}

// what to do during before
void SensorWaterMeter::onBefore() {
  // register the child
  new ChildDouble(this,_node->getAvailableChildId(),S_WATER,V_VOLUME);
  setPulseFactor(1000);
}

// return the total based on the pulses counted
void SensorWaterMeter::_reportTotal(Child* child) {
  ((ChildDouble*)child)->setValueDouble(_count / _pulse_factor);
}
#endif

/*
   SensorPlantowerPMS
*/
#ifdef MODULE_PMS
// contructor
SensorPlantowerPMS::SensorPlantowerPMS(NodeManager& node_manager, int rxpin, int txpin): Sensor(node_manager, rxpin) {
  _name = "PMS";
  _rx_pin = rxpin;
  _tx_pin = txpin;
}

// what to do during before
void SensorPlantowerPMS::onBefore() {
  // register the child
  new ChildInt(this, _node->getAvailableChildId(), S_AIR_QUALITY, V_LEVEL, "PM1.0");
  new ChildInt(this, _node->getAvailableChildId(), S_AIR_QUALITY, V_LEVEL, "PM2.5");
  new ChildInt(this, _node->getAvailableChildId(), S_AIR_QUALITY, V_LEVEL, "PM10.0");
}

// what to do during setup
void SensorPlantowerPMS::onSetup() {
  _ser = new SoftwareSerial(_rx_pin, _tx_pin);
  _pms = new PMS(*_ser);
  _ser->begin(9600);
}

// Clean _valuesRead flag at the beginning so the onLoop function knows when to read values (should be done only once for all children)
void SensorPlantowerPMS::loop(MyMessage* message) {
  _valuesRead = false;
  _valuesReadError = false;
  Sensor::loop(message);
}

// what to do during loop
void SensorPlantowerPMS::onLoop(Child* child) {
  // Read the ppm values
  if (!_valuesRead || _valuesReadError) {
    _valuesReadError = !_pms->read(_data, 1000);
    if (_valuesReadError) {
      Serial.println(F("ERR PMS read"));
      return;
    }
    _valuesRead = true;
  }
  int val = 0;
  if (child == children.get(1)) {
    // PM1.0 values
    val = _data.PM_AE_UG_1_0;
  } else if (child == children.get(2)) {
    // PM 2.5 values
    val = _data.PM_AE_UG_2_5;
  } else if (child == children.get(3)) {
    // PM 10.0 values
    val = _data.PM_AE_UG_10_0;
  } else {
    Serial.println(F("ERR child"));
    return;
  }
  // store the value
  ((ChildInt*)child)->setValueInt(val);
  #if DEBUG == 1
    Serial.print(_name);
    Serial.print(F(" I="));
    Serial.print(child->child_id);
    Serial.print(F(" µg/m³="));
    Serial.println(val);
  #endif
}

// what to do as the main task when receiving a message
void SensorPlantowerPMS::onReceive(MyMessage* message) {
  Child* child = getChild(message->sensor);
  if (child == nullptr) return;
  if (message->getCommand() == C_REQ && message->type == child->type) onLoop(child);
}
#endif

/*
   SensorConfiguration
*/
// contructor
SensorConfiguration::SensorConfiguration(NodeManager& node_manager): Sensor(node_manager) {
  _name = "CONFIG";
}

// what to do during before
void SensorConfiguration::onBefore() {
  new ChildInt(this,CONFIGURATION_CHILD_ID,S_CUSTOM,V_CUSTOM);
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
  Request request = Request(message->sensor,message->getString());
  int function = request.getFunction();
  int child_id = request.getChildId();
  // if the message is for the board itself
  if (child_id == 0) {
    switch(function) {
      case 1: _node->hello(); break;
      case 3: _node->setSleepSeconds(request.getValueInt()); break;
      case 4: _node->setSleepMinutes(request.getValueInt()); break;
      case 5: _node->setSleepHours(request.getValueInt()); break;
      case 29: _node->setSleepDays(request.getValueInt()); break;
      #ifndef MY_GATEWAY_ESP8266
        case 6: _node->reboot(); return;
      #endif
      case 7: _node->clearEeprom(); break;
      case 8: _node->sendMessage(CONFIGURATION_CHILD_ID,V_CUSTOM,VERSION); return;
      case 9: _node->wakeup(); break;
      case 10: _node->setRetries(request.getValueInt()); break;
      case 19: _node->setSleepInterruptPin(request.getValueInt()); break;
      case 20: _node->setSleepBetweenSend(request.getValueInt()); break;
      case 21: _node->setAck(request.getValueInt()); break;
      case 22: _node->setIsMetric(request.getValueInt()); break;
      case 24: _node->powerOn(); break;
      case 25: _node->powerOff(); break;
      case 27: _node->saveToMemory(0,request.getValueInt()); break;
      case 28: _node->setInterruptMinDelta(request.getValueInt()); break;
      case 30: _node->setSleepOrWait(request.getValueInt()); break;
      case 31: _node->setRebootPin(request.getValueInt()); break;
      case 32: _node->setADCOff(); break;
      case 36: _node->setReportIntervalSeconds(request.getValueInt()); break;
      case 37: _node->setReportIntervalMinutes(request.getValueInt()); break;
      case 38: _node->setReportIntervalHours(request.getValueInt()); break;
      case 39: _node->setReportIntervalDays(request.getValueInt()); break;
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
        case 7: sensor->setTrackLastValue(request.getValueInt()); break;
        case 9: sensor->setForceUpdateMinutes(request.getValueInt()); break;
        case 13: sensor->powerOn(); break;
        case 14: sensor->powerOff(); break;
        case 16: sensor->setReportIntervalMinutes(request.getValueInt()); break;
        case 17: sensor->setReportIntervalSeconds(request.getValueInt()); break;
        case 19: sensor->setReportIntervalHours(request.getValueInt()); break;
        case 20: sensor->setReportIntervalDays(request.getValueInt()); break;
        default: return;
      }
    } else {
      #ifndef MY_GATEWAY_ESP8266
      // the message is for a function specific to a sensor
      if (strcmp(sensor->getName(),"BATTERY") == 0) {
        SensorBattery* custom_sensor = (SensorBattery*)sensor;
        switch(function) {
          case 102: custom_sensor->setMinVoltage(request.getValueFloat()); break;
          case 103: custom_sensor->setMaxVoltage(request.getValueFloat()); break;
          case 104: custom_sensor->setBatteryInternalVcc(request.getValueInt()); break;
          case 105: custom_sensor->setBatteryPin(request.getValueInt()); break;
          case 106: custom_sensor->setBatteryVoltsPerBit(request.getValueFloat()); break;
          default: return;
        }
      }
      if (strcmp(sensor->getName(),"SIGNAL") == 0) {
        SensorSignal* custom_sensor = (SensorSignal*)sensor;
        switch(function) {
          case 101: custom_sensor->setSignalCommand(request.getValueInt()); break;
          default: return;
        }
      }
      #endif
      #ifdef MODULE_ANALOG_INPUT
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
      #ifdef MODULE_THERMISTOR
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
      #ifdef MODULE_ACS712
      if (strcmp(sensor->getName(),"ACS712") == 0) {
        SensorACS712* custom_sensor = (SensorACS712*)sensor;
        switch(function) {
          case 100: custom_sensor->setmVPerAmp(request.getValueInt()); break;
          case 102: custom_sensor->setOffset(request.getValueInt()); break;
          default: return;
        }
      }
      #endif
      #ifdef MODULE_DIGITAL_OUTPUT
      if (strcmp(sensor->getName(),"DIGITAL_O") == 0 || strcmp(sensor->getName(),"RELAY") == 0 || strcmp(sensor->getName(),"LATCHING") == 0) {
        SensorDigitalOutput* custom_sensor = (SensorDigitalOutput*)sensor;
        switch(function) {
            case 103: custom_sensor->setOnValue(request.getValueInt()); break;
            case 104: custom_sensor->setLegacyMode(request.getValueInt()); break;
            case 105: custom_sensor->setSafeguard(request.getValueInt()); break;
            case 106: custom_sensor->setInputIsElapsed(request.getValueInt()); break;
            case 107: custom_sensor->setWaitAfterSet(request.getValueInt()); break;
          default: return;
        }
        if (function > 200 && strcmp(sensor->getName(),"LATCHING") == 0) {
          SensorLatchingRelay* custom_sensor_2 = (SensorLatchingRelay*)sensor;
          switch(function) {
            case 201: custom_sensor_2->setPulseWidth(request.getValueInt()); break;
            case 202: custom_sensor_2->setPinOff(request.getValueInt()); break;
            case 203: custom_sensor_2->setPinOn(request.getValueInt()); break;
          default: return;
        }
        }
      }
      #endif
      #ifdef MODULE_SWITCH
      if (strcmp(sensor->getName(),"SWITCH") == 0 || strcmp(sensor->getName(),"DOOR") == 0 || strcmp(sensor->getName(),"MOTION") == 0) {
        SensorSwitch* custom_sensor = (SensorSwitch*)sensor;
        switch(function) {
          case 101: custom_sensor->setMode(request.getValueInt()); break;
          case 102: custom_sensor->setDebounce(request.getValueInt()); break;
          case 103: custom_sensor->setTriggerTime(request.getValueInt()); break;
          case 104: custom_sensor->setInitial(request.getValueInt()); break;
          default: return;
        }
      }
      #endif
      #ifdef MODULE_DS18B20
      if (strcmp(sensor->getName(),"DS18B20") == 0) {
        SensorDs18b20* custom_sensor = (SensorDs18b20*)sensor;
        switch(function) {
          case 101: custom_sensor->setResolution(request.getValueInt()); break;
          case 102: custom_sensor->setSleepDuringConversion(request.getValueInt()); break;
          default: return;
        }
      }
      #endif
      #ifdef MODULE_BH1750
      if (strcmp(sensor->getName(),"BH1750") == 0) {
        SensorBH1750* custom_sensor = (SensorBH1750*)sensor;
        switch(function) {
          case 101: custom_sensor->setMode(request.getValueInt()); break;
          default: return;
        }
      }
      #endif
      #if defined(MODULE_BME280) || defined(MODULE_BMP085) || defined(MODULE_BMP280)
      if (strcmp(sensor->getName(),"BMP085") == 0 || strcmp(sensor->getName(),"BME280") == 0 || strcmp(sensor->getName(),"BMP280") == 0) {
        SensorBosch* custom_sensor = (SensorBosch*)sensor;
        switch(function) {
          case 101: custom_sensor->setForecastSamplesCount(request.getValueInt()); break;
          default: return;
        }
      }
      #endif
      #ifdef MODULE_SONOFF
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
      #ifdef MODULE_HCSR04
      if (strcmp(sensor->getName(),"HCSR04") == 0) {
        SensorHCSR04* custom_sensor = (SensorHCSR04*)sensor;
        switch(function) {
          case 101: custom_sensor->setTriggerPin(request.getValueInt()); break;
          case 102: custom_sensor->setEchoPin(request.getValueInt()); break;
          case 103: custom_sensor->setMaxDistance(request.getValueInt()); break;
          default: return;
        }
      }
      #endif
      #ifdef MODULE_MQ
      if (strcmp(sensor->getName(),"MQ") == 0) {
        SensorMQ* custom_sensor = (SensorMQ*)sensor;
        switch(function) {
          case 101: custom_sensor->setTargetGas(request.getValueInt()); break;
          case 102: custom_sensor->setRlValue(request.getValueFloat()); break;
          case 103: custom_sensor->setRoValue(request.getValueFloat()); break;
          case 104: custom_sensor->setCleanAirFactor(request.getValueFloat()); break;
          case 105: custom_sensor->setCalibrationSampleTimes(request.getValueInt()); break;
          case 106: custom_sensor->setCalibrationSampleInterval(request.getValueInt()); break;
          case 107: custom_sensor->setReadSampleTimes(request.getValueInt()); break;
          case 108: custom_sensor->setReadSampleInterval(request.getValueInt()); break;
          default: return;
        }
      }
      #endif
      #ifdef MODULE_TSL2561
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
      #ifdef MODULE_PT100
      if (strcmp(sensor->getName(),"PT100") == 0) {
        SensorPT100* custom_sensor = (SensorPT100*)sensor;
        switch(function) {
          case 101: custom_sensor->setVoltageRef(request.getValueFloat()); break;
          default: return;
        }
      }
      #endif
      #ifdef MODULE_DIMMER
      if (strcmp(sensor->getName(),"DIMMER") == 0) {
        SensorDimmer* custom_sensor = (SensorDimmer*)sensor;
        switch(function) {
          case 101: custom_sensor->setEasing(request.getValueInt()); break;
          case 102: custom_sensor->setDuration(request.getValueInt()); break;
          case 103: custom_sensor->setStepDuration(request.getValueInt()); break;
          default: return;
        }
      }
      #endif
      #ifdef MODULE_PULSE_METER
      if (strcmp(sensor->getName(),"RAIN_GAUGE") == 0 || strcmp(sensor->getName(),"POWER") == 0 || strcmp(sensor->getName(),"WATER") == 0) {
        SensorPulseMeter* custom_sensor = (SensorPulseMeter*)sensor;
        switch(function) {
          case 102: custom_sensor->setPulseFactor(request.getValueFloat()); break;
          default: return;
        }
      }
      #endif
    }
  }
  _node->sendMessage(CONFIGURATION_CHILD_ID,V_CUSTOM,function);
}

/*******************************************
   NodeManager
*/

// initialize the node manager
NodeManager::NodeManager() {
  // setup the message container
  _message = MyMessage();
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

void NodeManager::setSleepSeconds(int value) {
  // set the status to AWAKE if the time provided is 0, SLEEP otherwise
  if (value == 0) _status = AWAKE;
  else _status = SLEEP;
  // store the time
  _sleep_time = value;
  // save sleep settings to eeprom
  if (_save_sleep_settings) _saveSleepSettings();
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
void NodeManager::setSaveSleepSettings(bool value) {
  _save_sleep_settings = value;
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

// register a sensor against NodeManager
void NodeManager::registerSensor(Sensor* sensor) {
  sensors.push(sensor);
}

// setup NodeManager
void NodeManager::before() {
  // print out the version
  #ifdef NODEMANAGER_DEBUG
    Serial.print(F("NodeManager v"));
    Serial.println(VERSION);
  #endif
  // setup the reboot pin if needed
  if (_reboot_pin > -1) {
    #ifdef NODEMANAGER_DEBUG
      Serial.print("REB P=");
      Serial.println(_reboot_pin);
    #endif
    pinMode(_reboot_pin, OUTPUT);
    digitalWrite(_reboot_pin, HIGH);
  }
  // print out MySensors' library capabilities
  #ifdef NODEMANAGER_DEBUG
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
  // restore the sleep settings saved in the eeprom
  if (_save_sleep_settings) _loadSleepSettings();
  // setup individual sensors
  for (List<Sensor*>::iterator itr = sensors.begin(); itr != sensors.end(); ++itr) {
    Sensor* sensor = *itr;
    // configure reporting interval
    if (! sensor->isReportIntervalConfigured()) sensor->setReportIntervalSeconds(_report_interval_seconds);
    // call each sensor's before()
    sensor->before();
  }
}

// present NodeManager and its sensors
void NodeManager::presentation() {
  #ifdef NODEMANAGER_DEBUG
    Serial.println(F("RADIO OK"));
  #endif
  // Send the sketch version information to the gateway and Controller
  if (_sleep_between_send > 0) sleep(_sleep_between_send);
  sendSketchInfo(SKETCH_NAME,SKETCH_VERSION);
  // present each sensor
  for (List<Sensor*>::iterator itr = sensors.begin(); itr != sensors.end(); ++itr) {
    Sensor* sensor = *itr;
    // call each sensor's presentation()
    if (_sleep_between_send > 0) sleep(_sleep_between_send);
    sensor->presentation();
  }
  #ifdef NODEMANAGER_DEBUG
    Serial.println(F("READY"));
    Serial.println("");
  #endif
}


// setup NodeManager
void NodeManager::setup() {
  // retrieve and store isMetric from the controller
  if (_get_controller_config) _is_metric = getControllerConfig().isMetric;
  #ifdef NODEMANAGER_DEBUG
    Serial.print(F("MY I="));
    Serial.print(getNodeId());
    Serial.print(F(" M="));
    Serial.println(_is_metric);
  #endif
  // run setup for all the registered sensors
  for (List<Sensor*>::iterator itr = sensors.begin(); itr != sensors.end(); ++itr) {
    Sensor* sensor = *itr;
    // call each sensor's setup()
    sensor->setup();
  }
  // setup the interrupt pins
  setupInterrupts();
}

// run the main function for all the register sensors
void NodeManager::loop() {
  // turn on the pin powering all the sensors
  powerOn();
  // run loop for all the registered sensors
  for (List<Sensor*>::iterator itr = sensors.begin(); itr != sensors.end(); ++itr) {
    Sensor* sensor = *itr;
    if (_last_interrupt_pin != -1 && sensor->getInterruptPin() == _last_interrupt_pin) {
      // if there was an interrupt for this sensor, call the sensor's interrupt() and then loop()
      _message.clear();
      sensor->interrupt();
      sensor->loop(nullptr);
        // reset the last interrupt pin
      _last_interrupt_pin = -1;
    }
    else if (_last_interrupt_pin == -1) {
      // if just at the end of a cycle, call the sensor's loop() 
      _message.clear();
      sensor->loop(nullptr);
    }
  }
  // turn off the pin powering all the sensors
  powerOff();
  // continue/start sleeping as requested
  if (isSleepingNode()) _sleep();
}

// dispacth inbound messages
void NodeManager::receive(MyMessage &message) {
  #ifdef NODEMANAGER_DEBUG
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
  // dispatch the message to the registered sensor
  Sensor* sensor = getSensorWithChild(message.sensor);
  if (sensor != nullptr) {
    // turn on the pin powering all the sensors
    powerOn();
    // call the sensor's receive()
    sensor->receive(message);
    // turn off the pin powering all the sensors
    powerOff();
  }
}

// request and return the current timestamp from the controller
long NodeManager::getTimestamp() {
  int retries = 3;
  _timestamp = -1;
  while (_timestamp == -1 && retries > 0) {
    #ifdef NODEMANAGER_DEBUG
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
  #ifdef NODEMANAGER_DEBUG
    Serial.print(F("TIME T="));
    Serial.print(_timestamp);
  #endif
}

// Send a hello message back to the controller
void NodeManager::hello() {
  // do nothing, the request will be echoed back
}

// reboot the board
void NodeManager::reboot() {
  #ifndef MY_GATEWAY_ESP8266
  #ifdef NODEMANAGER_DEBUG
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
  #endif
}

// clear the EEPROM
void NodeManager::clearEeprom() {
  #ifdef NODEMANAGER_DEBUG
    Serial.println(F("CLEAR"));
  #endif
  for (uint16_t i=0; i<EEPROM_LOCAL_CONFIG_ADDRESS; i++) saveState(i, 0xFF);
}

// wake up the board
void NodeManager::wakeup() {
  #ifdef NODEMANAGER_DEBUG
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
    #ifdef defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
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
  #ifdef NODEMANAGER_DEBUG
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
  #ifndef MY_GATEWAY_ESP8266
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
int NodeManager::getAvailableChildId() {
  for (int i = 1; i < 255; i++) {
    if (i == CONFIGURATION_CHILD_ID || i == BATTERY_CHILD_ID || i == SIGNAL_CHILD_ID) continue;
    Child* child = getChild(i);
    if (child == nullptr) return i;
  }
  return 254;
}

// handle an interrupt
void NodeManager::_onInterrupt_1() {
  long now = millis();
  if ( (now - _last_interrupt_1 > _interrupt_min_delta) || (now < _last_interrupt_1) ) {
    _last_interrupt_pin = INTERRUPT_PIN_1;
    #ifdef NODEMANAGER_DEBUG
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
    #ifdef NODEMANAGER_DEBUG
      Serial.print(F("INT P="));
      Serial.println(INTERRUPT_PIN_2);
    #endif
    _last_interrupt_2 = now;
  }
}

// send a message by providing the source child, type of the message and value
void NodeManager::sendMessage(int child_id, int type, int value) {
  _message.clear();
  _message.set(value);
  _sendMessage(child_id,type);
}
void NodeManager::sendMessage(int child_id, int type, float value) {
  _message.clear();
  _message.set(value,2);
  _sendMessage(child_id,type);
}
void NodeManager::sendMessage(int child_id, int type, double value) {
  _message.clear();
  _message.set(value,4);
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
    // if configured, sleep beetween each send
    if (_sleep_between_send > 0) sleep(_sleep_between_send);
    #ifdef NODEMANAGER_DEBUG
      Serial.print(F("SEND D="));
      Serial.print(_message.destination);
      Serial.print(F(" I="));
      Serial.print(_message.sensor);
      Serial.print(F(" C="));
      Serial.print(_message.getCommand());
      Serial.print(F(" T="));
      Serial.print(_message.type);
      Serial.print(F(" S="));
      Serial.print(_message.getString());
      Serial.print(F(" I="));
      Serial.print(_message.getInt());
      Serial.print(F(" F="));
      Serial.println(_message.getFloat());
    #endif
    send(_message, _ack);
  }
}

void NodeManager::setPowerManager(PowerManager& powerManager) {
  _powerManager = &powerManager;
}

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

// wrapper of smart sleep
void NodeManager::_sleep() {
  #ifdef NODEMANAGER_DEBUG
    Serial.print(F("SLEEP "));
    Serial.print(_sleep_time);
    Serial.println(F("s"));
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
    #ifdef NODEMANAGER_DEBUG
      Serial.print(F("INT P="));
      Serial.print(pin_number);
      Serial.print(F(", M="));
      Serial.println(interrupt_mode);
    #endif
    // when waking up from an interrupt on the wakup pin, stop sleeping
    if (_sleep_interrupt_pin == pin_number) _status = AWAKE;
  }
  // coming out of sleep
  #ifdef NODEMANAGER_DEBUG
    Serial.println(F("AWAKE"));
  #endif
}

// present the service
void NodeManager::_present(int child_id, int type) {
  #ifdef NODEMANAGER_DEBUG
    Serial.print(F("PRES I="));
    Serial.print(child_id);
    Serial.print(F(", T="));
    Serial.println(type);
  #endif
  if (_sleep_between_send > 0) sleep(_sleep_between_send);
  present(child_id,type,"",_ack);
}

// load the configuration stored in the eeprom
void NodeManager::_loadSleepSettings() {
  if (loadState(EEPROM_SLEEP_SAVED) == 1) {
    // load sleep settings
    int bit_1 = loadState(EEPROM_SLEEP_1);
    int bit_2 = loadState(EEPROM_SLEEP_2);
    int bit_3 = loadState(EEPROM_SLEEP_3);
    _sleep_time = bit_3*255*255 + bit_2*255 + bit_1;
    #ifdef NODEMANAGER_DEBUG
      Serial.print(F("LOADSLP T="));
      Serial.println(_sleep_time);
    #endif
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
