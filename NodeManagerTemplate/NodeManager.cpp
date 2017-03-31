/*
 * NodeManager
 */

#include "NodeManager.h"

/*
 * Constants
 */

const char* BATTERY = "BATTERY";
const char* AWAKE = "AWAKE";
const char* REBOOT = "REBOOT";
const char* CLEAR = "CLEAR";
const char* WAKEUP = "WAKEUP";

/***************************************
   PowerManager
*/

// set the vcc and ground pin the sensor is connected to
void PowerManager::setPowerPins(int ground_pin, int vcc_pin, long wait) {
  #if DEBUG == 1
    Serial.print("POWER G=");
    Serial.print(ground_pin);
    Serial.print(" V=");
    Serial.println(vcc_pin);
  #endif
  // configure the vcc pin as output and initialize to low (power off)
  _vcc_pin = vcc_pin;
  pinMode(_vcc_pin, OUTPUT);
  digitalWrite(_vcc_pin, LOW);
  // configure the ground pin as output and initialize to low
  _ground_pin = ground_pin;
  pinMode(_ground_pin, OUTPUT);
  digitalWrite(_ground_pin, LOW);
  _wait = wait;
}

// return true if power pins have been configured
bool PowerManager::_hasPowerManager() {
  if (_vcc_pin != -1 && _ground_pin != -1) return true;
  return false;
}

// turn on the sensor by activating its power pins
void PowerManager::powerOn() {
  if (! _hasPowerManager()) return;
  #if DEBUG == 1
    Serial.print("ON P=");
    Serial.println(_vcc_pin);
  #endif
  // power on the sensor by turning high the vcc pin
  digitalWrite(_vcc_pin, HIGH);
  // wait a bit for the device to settle down
  if (_wait > 0) sleep(_wait);
}

// turn off the sensor
void PowerManager::powerOff() {
  if (! _hasPowerManager()) return;
  #if DEBUG == 1
    Serial.print("OFF P=");
    Serial.println(_vcc_pin);
  #endif
  // power off the sensor by turning low the vcc pin
  digitalWrite(_vcc_pin, LOW);
}

/******************************************
    Sensors
*/

/*
   Sensor class
*/
// constructor
Sensor::Sensor(int child_id, int pin) {
  _child_id = child_id;
  _pin = pin;
  _msg = MyMessage(_child_id, _type);
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
void Sensor::setRetries(int value) {
  _retries = value;
}
void Sensor::setSamples(int value) {
  _samples = value;
}
void Sensor::setSamplesInterval(int value) {
  _samples_interval = value;
}
void Sensor::setTackLastValue(bool value) {
  _track_last_value = value;
}
void Sensor::setForceUpdate(int value) {
  _force_update = value;
}
void Sensor::setValueType(int value) {
  _value_type = value;
}
void Sensor::setFloatPrecision(int value) {
  _float_precision = value;
}
#if POWER_MANAGER == 1
    void Sensor::setPowerPins(int ground_pin, int vcc_pin, long wait) {
      _powerManager.setPowerPins(ground_pin, vcc_pin, wait);
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
void Sensor::setSleepBetweenSend(int value) {
  _sleep_between_send = value;
}

// present the sensor to the gateway and controller
void Sensor::presentation() {
  #if DEBUG == 1
    Serial.print("PRES I=");
    Serial.print(_child_id);
    Serial.print(" T=");
    Serial.println(_presentation);
  #endif
  present(_child_id, _presentation);
}

// call the sensor-specific implementation of before
void Sensor::before() {
  if (_pin == -1) return;
  onBefore();
}

// call the sensor-specific implementation of loop
void Sensor::loop(const MyMessage & message) {
  if (_pin == -1) return;
  #if POWER_MANAGER == 1
    // turn the sensor on
    if (_auto_power_pins) powerOn();
  #endif
  // for numeric sensor requiring multiple samples, keep track of the total
  float total = 0;
  // keep track of the number of cycles since the last update
  _cycles++;
  // collect multiple samples if needed
  for (int i = 0; i < _samples; i++) {
    // call the sensor-specific implementation of the main task which will store the result in the _value variable
    if (message.sender == 0 && message.sensor == 0 && message.getCommand() == 0 && message.type == 0) {
      // empty message, we'be been called from loop()
      onLoop();
    }
    else {
      // we've been called from receive(), pass the message along
      onReceive(message);
    }
    // for integers and floats, keep track of the total
    if (_value_type == TYPE_INTEGER) total += (float)_value_int;
    else if (_value_type == TYPE_FLOAT) total += _value_float;
    // wait between samples
    if (_samples_interval > 0) sleep(_samples_interval);
  }
  // process the result and send a response back. 
  if (_value_type == TYPE_INTEGER && total > -1) {
    // if the value is an integer, calculate the average value of the samples
    int avg = (int) (total / _samples);
    // if track last value is disabled or if enabled and the current value is different then the old value, send it back
    if (! _track_last_value || (_track_last_value && avg != _last_value_int) || (_track_last_value && _force_update > 0 && _cycles > _force_update)) {
      _cycles = 0;
      _last_value_int = avg;
      _send(_msg.set(avg));
    }
  }
  // process a float value
  else if (_value_type == TYPE_FLOAT && total > -1) {
    // calculate the average value of the samples
    float avg = total / _samples;
    // if track last value is disabled or if enabled and the current value is different then the old value, send it back
    if (! _track_last_value || (_track_last_value && avg != _last_value_float) || (_track_last_value && _cycles >= _force_update)) {
      _cycles = 0;
      _last_value_float = avg;
      _send(_msg.set(avg, _float_precision));
    }
  }
  // process a string value
  else if (_value_type == TYPE_STRING) {
    // if track last value is disabled or if enabled and the current value is different then the old value, send it back
    if (! _track_last_value || (_track_last_value && strcmp(_value_string, _last_value_string) != 0) || (_track_last_value && _cycles >= _force_update)) {
      _cycles = 0;
      _last_value_string = _value_string;
      _send(_msg.set(_value_string));
    }
  }
  // turn the sensor off
  #if POWER_MANAGER == 1
    if (_auto_power_pins) powerOff();
  #endif
}

// receive a message from the radio network
void Sensor::receive(const MyMessage &message) {
  // return if not for this sensor
  if (message.sensor != _child_id || message.type != _type) return;
  // a request would make the sensor executing its main task
  loop(message);
}

// send a message to the network
void Sensor::_send(MyMessage & message) {
  // send the message, multiple times if requested
  for (int i = 0; i < _retries; i++) {
    // if configured, sleep beetween each send
    if (_sleep_between_send > 0) sleep(_sleep_between_send);
    #if DEBUG == 1
      Serial.print("SEND D=");
      Serial.print(message.destination);
      Serial.print(" I=");
      Serial.print(message.sensor);
      Serial.print(" C=");
      Serial.print(message.getCommand());
      Serial.print(" T=");
      Serial.print(message.type);
      Serial.print(" S=");
      Serial.print(message.getString());
      Serial.print(" N=");
      Serial.print(message.getInt());
      Serial.print(" F=");
      Serial.println(message.getFloat());
    #endif
    send(message);
  }
}

/*
   SensorAnalogInput
*/

// contructor
SensorAnalogInput::SensorAnalogInput(int child_id, int pin): Sensor(child_id, pin) {
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

// what do to during setup
void SensorAnalogInput::onBefore() {
  // prepare the pin for input
  pinMode(_pin, INPUT);
}

// what do to during loop
void SensorAnalogInput::onLoop() {
  // read the input
  int adc = _getAnalogRead();
  // calculate the percentage
  int percentage = 0;
  if (_output_percentage) percentage = _getPercentage(adc);
  #if DEBUG == 1
    Serial.print("A-IN I=");
    Serial.print(_child_id);
    Serial.print(" V=");
    Serial.print(adc);
    Serial.print(" %=");
    Serial.println(percentage);
  #endif
  // store the result
  _value_int = _output_percentage ? percentage : adc;
}

// what do to during loop
void SensorAnalogInput::onReceive(const MyMessage & message) {
  onLoop();
}

// read the analog input
int SensorAnalogInput::_getAnalogRead() {
  // set the reference
  if (_reference != -1) {
    analogReference(_reference);
    sleep(100);
  }
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
SensorLDR::SensorLDR(int child_id, int pin): SensorAnalogInput(child_id, pin) {
  // set presentation and type and reverse (0: no light, 100: max light)
  setPresentation(S_LIGHT_LEVEL);
  setType(V_LIGHT_LEVEL);
  setReverse(true);
}

/*
   SensorThermistor
*/

// contructor
SensorThermistor::SensorThermistor(int child_id, int pin): Sensor(child_id, pin) {
  // set presentation, type and value type
  setPresentation(S_TEMP);
  setType(V_TEMP);
  setValueType(TYPE_FLOAT);
}

// setter/getter
void SensorThermistor::setNominalResistor(int value) {
  _nominal_resistor = value;
}
void SensorThermistor::setNominalTemperature(int value) {
  _nominal_temperature = value;
}
void SensorThermistor::setBCoefficient(int value) {
  _b_coefficient = value;
}
void SensorThermistor::setSeriesResistor(int value) {
  _series_resistor = value;
}
void SensorThermistor::setOffset(float value) {
  _offset = value;
}

// what do to during setup
void SensorThermistor::onBefore() {
  // set the pin as input
  pinMode(_pin, INPUT);
}

// what do to during loop
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
  if (! getControllerConfig().isMetric) temperature = temperature * 1.8 + 32;
  #if DEBUG == 1
    Serial.print("THER I=");
    Serial.print(_child_id);
    Serial.print(" V=");
    Serial.print(adc);
    Serial.print(" T=");
    Serial.println(temperature);
    Serial.print(" M=");
    Serial.println(getControllerConfig().isMetric);
  #endif
  // store the value
  _value_float = temperature;
}

// what do to as the main task when receiving a message
void SensorThermistor::onReceive(const MyMessage & message) {
  onLoop();
}

/*
   SensorDigitalInput
*/

// contructor
SensorDigitalInput::SensorDigitalInput(int child_id, int pin): Sensor(child_id, pin) {
}

// what do to during setup
void SensorDigitalInput::onBefore() {
  // set the pin for input
  pinMode(_pin, INPUT);
}

// what do to during loop
void SensorDigitalInput::onLoop() {
  // read the value
  int value = digitalRead(_pin);
  #if DEBUG == 1
    Serial.print("D-IN I=");
    Serial.print(_child_id);
    Serial.print(" P=");
    Serial.print(_pin);
    Serial.print(" V=");
    Serial.println(value);
  #endif
  // store the value
  _value_int = value;
}

// what do to as the main task when receiving a message
void SensorDigitalInput::onReceive(const MyMessage & message) {
  onLoop();
}


/*
   SensorDigitalOutput
*/

// contructor
SensorDigitalOutput::SensorDigitalOutput(int child_id, int pin): Sensor(child_id, pin) {
}

void SensorDigitalOutput::onBefore() {
  // set the pin as output and initialize it accordingly
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, _initial_value == LOW ? LOW : HIGH);
  // the initial value is now the current value
  _value_int = _initial_value;
}

// setter/getter
void SensorDigitalOutput::setInitialValue(int value) {
  _initial_value = value;
}
void SensorDigitalOutput::setPulseWidth(int value) {
  _pulse_width = value;
}

// main task
void SensorDigitalOutput::onLoop() {
  // do nothing on loop
}

// what do to as the main task when receiving a message
void SensorDigitalOutput::onReceive(const MyMessage & message) {
  // retrieve from the message the value to set
  int value = message.getInt();
  if (value != 0 && value != 1) return;
  #if DEBUG == 1
    Serial.print("DOUT I=");
    Serial.print(_child_id);
    Serial.print(" P=");
    Serial.print(_pin);
    Serial.print(" S=");
    Serial.print(_initial_value);
    Serial.print(" V=");
    Serial.print(value);
    Serial.print(" P=");
    Serial.println(_pulse_width);
  #endif
  // set the value
  digitalWrite(_pin, value);
  if (_pulse_width > 0) {
    // if this is a pulse output, restore the value to the original value after the pulse
    sleep(_pulse_width);
    digitalWrite(_pin, value == 0 ? HIGH: LOW);
  }
  // store the current value
  _value_int = value;
}

/*
   SensorRelay
*/

// contructor
SensorRelay::SensorRelay(int child_id, int pin): SensorDigitalOutput(child_id, pin) {
  // set presentation and type
  setPresentation(S_BINARY);
  setType(V_STATUS);
}

// define what to do during loop
void SensorRelay::onLoop() {
    // set the value to -1 so to avoid reporting to the gateway during loop
    _value_int = -1;
}

/*
   SensorLatchingRelay
*/

// contructor
SensorLatchingRelay::SensorLatchingRelay(int child_id, int pin): SensorRelay(child_id, pin) {
  // like a sensor with a default pulse set
  setPulseWidth(50);
}

/*
   SensorDHT
*/
#if MODULE_DHT == 1
// contructor
SensorDHT::SensorDHT(int child_id, int pin, DHT* dht, int sensor_type, int dht_type): Sensor(child_id, pin) {
  // store the dht object
  _dht = dht;
  // store the sensor type (0: temperature, 1: humidity)
  _sensor_type = sensor_type;
  _dht_type = dht_type;
  if (_sensor_type == 0) {
    // temperature sensor
    setPresentation(S_TEMP);
    setType(V_TEMP);
    setValueType(TYPE_FLOAT);
  }
  else if (_sensor_type == 1) {
    // humidity sensor
    setPresentation(S_HUM);
    setType(V_HUM);
    setValueType(TYPE_FLOAT);
  }
}

// what do to during setup
void SensorDHT::onBefore() {
    // initialize the dht library
    _dht->begin();
}

// what do to during loop
void SensorDHT::onLoop() {
  // temperature sensor
  if (_sensor_type == 0) {
    // read the temperature
    float temperature = _dht->readTemperature();
    // convert it
    if (! getControllerConfig().isMetric) temperature = temperature * 1.8 + 32;
    #if DEBUG == 1
      Serial.print("DHT I=");
      Serial.print(_child_id);
      Serial.print(" T=");
      Serial.println(temperature);
    #endif
    // store the value
    if (! isnan(temperature)) _value_float = temperature;
  }
  // humidity sensor
  else if (_sensor_type == 1) {
    // read humidity
    float humidity = _dht->readHumidity();
    if (isnan(humidity)) return;
    #if DEBUG == 1
      Serial.print("DHT I=");
      Serial.print(_child_id);
      Serial.print(" H=");
      Serial.println(humidity);
    #endif
    // store the value
    if (! isnan(humidity)) _value_float = humidity;
  }
}

// what do to as the main task when receiving a message
void SensorDHT::onReceive(const MyMessage & message) {
  onLoop();
}
#endif

/*
   SensorSHT21
*/
#if MODULE_SHT21 == 1
// contructor
SensorSHT21::SensorSHT21(int child_id, int sensor_type): Sensor(child_id,A2) {
  // store the sensor type (0: temperature, 1: humidity)
  _sensor_type = sensor_type;
  if (_sensor_type == 0) {
    // temperature sensor
    setPresentation(S_TEMP);
    setType(V_TEMP);
    setValueType(TYPE_FLOAT);
  }
  else if (_sensor_type == 1) {
    // humidity sensor
    setPresentation(S_HUM);
    setType(V_HUM);
    setValueType(TYPE_FLOAT);
  }
}

// what do to during setup
void SensorSHT21::onBefore() {
  // initialize the library
  Wire.begin();
}

// what do to during loop
void SensorSHT21::onLoop() {
  // temperature sensor
  if (_sensor_type == 0) {
    // read the temperature
    float temperature = SHT2x.GetTemperature();
    // convert it
    if (! getControllerConfig().isMetric) temperature = temperature * 1.8 + 32;
    #if DEBUG == 1
      Serial.print("SHT I=");
      Serial.print(_child_id);
      Serial.print(" T=");
      Serial.println(temperature);
    #endif
    // store the value
    if (! isnan(temperature)) _value_float = temperature;
  }
  // Humidity Sensor
  else if (_sensor_type == 1) {
    // read humidity
    float humidity = SHT2x.GetHumidity();
    if (isnan(humidity)) return;
    #if DEBUG == 1
      Serial.print("SHT I=");
      Serial.print(_child_id);
      Serial.print(" H=");
      Serial.println(humidity);
    #endif
    // store the value
    if (! isnan(humidity)) _value_float = humidity;
  }
}

// what do to as the main task when receiving a message
void SensorSHT21::onReceive(const MyMessage & message) {
  onLoop();
}
#endif

/*
 * SensorHTU21D
 */
 #if MODULE_SHT21 == 1
// constructor
SensorHTU21D::SensorHTU21D(int child_id, int pin): SensorSHT21(child_id, pin) {
}
#endif 

/*
 * SensorSwitch
 */
SensorSwitch::SensorSwitch(int child_id, int pin): Sensor(child_id,pin) {
  setType(V_TRIPPED);
}

// setter/getter
void SensorSwitch::setMode(int value) {
  _mode = value;
}
int SensorSwitch::getMode() {
  return _mode;
}
// setter/getter
void SensorSwitch::setDebounce(int value) {
  _debounce = value;
}
// setter/getter
void SensorSwitch::setTriggerTime(int value) {
  _trigger_time = value;
}

// what do to during setup
void SensorSwitch::onBefore() {
  // initialize the value
  if (_mode == RISING) _value_int = LOW;
  else if (_mode == FALLING) _value_int = HIGH;
}

// what do to during loop
void SensorSwitch::onLoop() {
  // wait to ensure the the input is not floating
  if (_debounce > 0) sleep(_debounce);
  // read the value of the pin
  int value = digitalRead(_pin);
  // process the value
  if ( (_mode == RISING && value == HIGH ) || (_mode == FALLING && value == LOW) || (_mode == CHANGE) )  {
    #if DEBUG == 1
      Serial.print("SWITCH I=");
      Serial.print(_child_id);
      Serial.print(" P=");
      Serial.print(_pin);
      Serial.print(" V=");
      Serial.println(value);
    #endif
    _value_int = value;
    // allow the signal to be restored to its normal value
    if (_trigger_time > 0) sleep(_trigger_time);
  } else {
    // invalid
    _value_int = -1;
  }
}
// what do to as the main task when receiving a message
void SensorSwitch::onReceive(const MyMessage & message) {
  onLoop();
}

/*
 * SensorDoor
 */
SensorDoor::SensorDoor(int child_id, int pin): SensorSwitch(child_id,pin) {
  setPresentation(S_DOOR);
}

/*
 * SensorMotion
 */
SensorMotion::SensorMotion(int child_id, int pin): SensorSwitch(child_id,pin) {
  setPresentation(S_MOTION);
  // capture only when it triggers
  setMode(RISING);
}

/*
   SensorDs18b20
*/
#if MODULE_DS18B20 == 1
// contructor
SensorDs18b20::SensorDs18b20(int child_id, int pin, DallasTemperature* sensors, int index): Sensor(child_id, pin) {
  setPresentation(S_TEMP);
  setType(V_TEMP);
  setValueType(TYPE_FLOAT);
  _index = index;
  _sensors = sensors;
}

// what do to during setup
void SensorDs18b20::onBefore() {
}

// what do to during loop
void SensorDs18b20::onLoop() {
  // request the temperature
  _sensors->requestTemperaturesByIndex(_index);
  // read the temperature
  float temperature = _sensors->getTempCByIndex(_index);
  // convert it
  if (! getControllerConfig().isMetric) temperature = temperature * 1.8 + 32;
  #if DEBUG == 1
    Serial.print("DS18 I=");
    Serial.print(_child_id);
    Serial.print(" T=");
    Serial.println(temperature);
  #endif
  // store the value
  _value_float = temperature;
}

// what do to as the main task when receiving a message
void SensorDs18b20::onReceive(const MyMessage & message) {
  onLoop();
}
#endif

/*
   SensorBH1750
*/
#if MODULE_BH1750 == 1
// contructor
SensorBH1750::SensorBH1750(int child_id): Sensor(child_id,A4) {
  setPresentation(S_LIGHT_LEVEL);
  setType(V_LEVEL);
  _lightSensor = new BH1750();
}

// what do to during setup
void SensorBH1750::onBefore() {
  _lightSensor->begin();
}

// what do to during loop
void SensorBH1750::onLoop() {
  // request the light level
  _value_int = _lightSensor->readLightLevel();
  #if DEBUG == 1
    Serial.print("BH1 I=");
    Serial.print(_child_id);
    Serial.print(" L=");
    Serial.println(_value_int);
  #endif
}

// what do to as the main task when receiving a message
void SensorBH1750::onReceive(const MyMessage & message) {
  onLoop();
}
#endif

/*
   SensorMLX90614
*/
#if MODULE_MLX90614 == 1
// contructor
SensorMLX90614::SensorMLX90614(int child_id, Adafruit_MLX90614* mlx, int sensor_type): Sensor(child_id,A4) {
  // store the sensor type (0: ambient, 1: object)
  _sensor_type = sensor_type;
  _mlx = mlx;
  // set presentation and type
  setPresentation(S_TEMP);
  setType(V_TEMP);
  setValueType(TYPE_FLOAT);
}

// what do to during setup
void SensorMLX90614::onBefore() {
  // initialize the library
  _mlx->begin();
}

// what do to during loop
void SensorMLX90614::onLoop() {
  float temperature = _sensor_type == 0 ? _mlx->readAmbientTempC() : _mlx->readObjectTempC();
  // convert it
  if (! getControllerConfig().isMetric) temperature = temperature * 1.8 + 32;
  #if DEBUG == 1
    Serial.print("MLX I=");
    Serial.print(_child_id);
    Serial.print(" T=");
    Serial.println(temperature);
  #endif
  if (! isnan(temperature)) _value_float = temperature;
}

// what do to as the main task when receiving a message
void SensorMLX90614::onReceive(const MyMessage & message) {
  onLoop();
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

// setter/getter
void NodeManager::setRebootPin(int value) {
    _reboot_pin = value;
}
void NodeManager::setRetries(int value) {
  _retries = value;
}
#if BATTERY_MANAGER == 1
  void NodeManager::setBatteryMin(float value) {
    _battery_min = value;
  }
  void NodeManager::setBatteryMax(float value) {
    _battery_max = value;
  }
  void NodeManager::setBatteryReportCycles(int value) {
    _battery_report_cycles = value;
  }
#endif
#if SLEEP_MANAGER == 1
  void NodeManager::setSleepMode(int value) {
    _sleep_mode = value;
  }
  void NodeManager::setSleepTime(int value) {
    _sleep_time = value;
  }
  void NodeManager::setSleepUnit(int value) {
    _sleep_unit = value;
  }
  void NodeManager::setSleep(int value1, int value2, int value3) {
    _sleep_mode = value1;
    _sleep_time = value2;
    _sleep_unit = value3;
  }
  void NodeManager::setSleepInterruptPin(int value) {
    _sleep_interrupt_pin = value;
  }
#endif
void NodeManager::setInterrupt(int pin, int mode, int pull) {
  if (pin == INTERRUPT_PIN_1) {
    _interrupt_1_mode = mode;
    _interrupt_1_pull = pull;
  }
  if (pin == INTERRUPT_PIN_2) {
    _interrupt_2_mode = mode;
    _interrupt_2_pull = pull;
  }
}
#if POWER_MANAGER == 1
  void NodeManager::setPowerPins(int ground_pin, int vcc_pin, long wait) {
    _powerManager.setPowerPins(ground_pin, vcc_pin, wait);
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

// register a sensor to this manager
int NodeManager::registerSensor(int sensor_type, int pin, int child_id) {
  #if DEBUG == 1
    if (_startup) {
      Serial.print("NodeManager v");
      Serial.println(VERSION);
      _startup = false;
    }
  #endif
  // get a child_id if not provided by the user
  if (child_id < 0) child_id = _getAvailableChildId();
  // based on the given sensor type instantiate the appropriate class
  if (sensor_type == 0) return -1;
  #if MODULE_ANALOG_INPUT == 1
    else if (sensor_type == SENSOR_ANALOG_INPUT) return registerSensor(new SensorAnalogInput(child_id, pin));
    else if (sensor_type == SENSOR_LDR) return registerSensor(new SensorLDR(child_id, pin));
    else if (sensor_type == SENSOR_THERMISTOR) return registerSensor(new SensorThermistor(child_id, pin));
  #endif
  #if MODULE_DIGITAL_INPUT == 1
    else if (sensor_type == SENSOR_DIGITAL_INPUT) return registerSensor(new SensorDigitalInput(child_id, pin));
  #endif
  #if MODULE_DIGITAL_OUTPUT == 1
    else if (sensor_type == SENSOR_DIGITAL_OUTPUT) return registerSensor(new SensorDigitalOutput(child_id, pin));
    else if (sensor_type == SENSOR_RELAY) return registerSensor(new SensorRelay(child_id, pin));
    else if (sensor_type == SENSOR_LATCHING_RELAY) return registerSensor(new SensorLatchingRelay(child_id, pin));
  #endif
  #if MODULE_DHT == 1
    else if (sensor_type == SENSOR_DHT11 || sensor_type == SENSOR_DHT22) {
      DHT* dht = new DHT(pin,DHT22);
      int dht_type = sensor_type == SENSOR_DHT11 ? DHT11 : DHT22;
      registerSensor(new SensorDHT(child_id,pin,dht,0,dht_type));
      child_id = _getAvailableChildId();
      return registerSensor(new SensorDHT(child_id,pin,dht,1,dht_type));
    }
  #endif
  #if MODULE_SHT21 == 1
    else if (sensor_type == SENSOR_SHT21) {
      registerSensor(new SensorSHT21(child_id,0));
      child_id = _getAvailableChildId();
      return registerSensor(new SensorSHT21(child_id,1));
    }
    else if (sensor_type == SENSOR_HTU21D) {
      registerSensor(new SensorHTU21D(child_id,0));
      child_id = _getAvailableChildId();
      return registerSensor(new SensorHTU21D(child_id,1));
    }
  #endif
  #if MODULE_SWITCH == 1
    else if (sensor_type == SENSOR_SWITCH || sensor_type == SENSOR_DOOR || sensor_type == SENSOR_MOTION) {
      // ensure an interrupt pin is provided
      if (pin != INTERRUPT_PIN_1 && pin != INTERRUPT_PIN_2) return;
      // register the sensor
      int index = 0;
      if (sensor_type == SENSOR_SWITCH) index = registerSensor(new SensorSwitch(child_id, pin));
      else if (sensor_type == SENSOR_DOOR) index = registerSensor(new SensorDoor(child_id, pin));
      else if (sensor_type == SENSOR_MOTION) index = registerSensor(new SensorMotion(child_id, pin));
      // set an interrupt on the pin and activate internal pull up
      setInterrupt(pin,((SensorSwitch*)get(index))->getMode(),HIGH);
      return index;
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
        index = registerSensor(new SensorDs18b20(child_id,pin,sensors,i));
      }
      return index;
    }
  #endif
  #if MODULE_BH1750 == 1
    else if (sensor_type == SENSOR_BH1750) {
      return registerSensor(new SensorBH1750(child_id));
    }
  #endif
  #if MODULE_MLX90614 == 1
    else if (sensor_type == SENSOR_MLX90614) {
      Serial.println("1");
      Adafruit_MLX90614* mlx = new Adafruit_MLX90614();
      Serial.println("2");
      registerSensor(new SensorMLX90614(child_id,mlx,0));
      Serial.println("3");
      child_id = _getAvailableChildId();
      return registerSensor(new SensorMLX90614(child_id,mlx,1));
    }
  #endif
  else {
    #if DEBUG == 1
      Serial.print("INVALID ");
      Serial.println(sensor_type);
    #endif
    return -1;
  };
}

// attach a built-in or custom sensor to this manager
int NodeManager::registerSensor(Sensor* sensor) {
  #if DEBUG == 1
    Serial.print("REG I=");
    Serial.print(sensor->getChildId());
    Serial.print(" P=");
    Serial.print(sensor->getPin());
    Serial.print(" P=");
    Serial.print(sensor->getPresentation());
    Serial.print(" T=");
    Serial.println(sensor->getType());
  #endif
  // set auto power pin
  sensor->setAutoPowerPins(_auto_power_pins);
  // add the sensor to the array of registered sensors
  _sensors[sensor->getChildId()] = sensor;
  // return the child_id
  return sensor->getChildId();
}

// return a sensor given its index
Sensor* NodeManager::get(int child_id) {
  // return a pointer to the sensor from the given child_id
  return _sensors[child_id];
}

// setup NodeManager
void NodeManager::before() {
  if (_reboot_pin > -1) {
    #if DEBUG == 1
      Serial.print("REB P=");
      Serial.println(_reboot_pin);
    #endif
    // setup the reboot pin
    pinMode(_reboot_pin, OUTPUT);
    digitalWrite(_reboot_pin, HIGH);
  }
  #if SLEEP_MANAGER == 1
    // setup the sleep interrupt pin
    if (_sleep_interrupt_pin > -1) {
      // set the interrupt when the pin is connected to ground
      setInterrupt(_sleep_interrupt_pin,FALLING,HIGH);
    }
  #endif
  // setup the interrupt pins
  if (_interrupt_1_mode != MODE_NOT_DEFINED) {
    pinMode(INTERRUPT_PIN_1,INPUT);
    if (_interrupt_1_pull > -1) digitalWrite(INTERRUPT_PIN_1,_interrupt_1_pull);
  }
  if (_interrupt_2_mode != MODE_NOT_DEFINED) {
    pinMode(INTERRUPT_PIN_2, INPUT);
    if (_interrupt_2_pull > -1) digitalWrite(INTERRUPT_PIN_2,_interrupt_2_pull);
  }
  #if DEBUG == 1
    Serial.print("INT1 M=");
    Serial.println(_interrupt_1_mode);
    Serial.print("INT2 M=");
    Serial.println(_interrupt_2_mode);
  #endif
  #if REMOTE_CONFIGURATION == 1 && SLEEP_MANAGER == 1 && PERSIST == 1
    // restore sleep configuration from eeprom
    if (loadState(EEPROM_SLEEP_SAVED) == 1) {
      // sleep settings found in the eeprom, restore them
      _sleep_mode = loadState(EEPROM_SLEEP_MODE);
      _sleep_time = loadState(EEPROM_SLEEP_TIME_MINOR);
      int major = loadState(EEPROM_SLEEP_TIME_MAJOR);
      if (major == 1) _sleep_time =  _sleep_time + 250;
      else if (major == 2) _sleep_time =  _sleep_time + 250 * 2;
      else if (major == 3) _sleep_time =  _sleep_time + 250 * 3;
      _sleep_unit = loadState(EEPROM_SLEEP_UNIT);
      #if DEBUG == 1
        Serial.print("LOADSLP M=");
        Serial.print(_sleep_mode);
        Serial.print(" T=");
        Serial.print(_sleep_time);
        Serial.print(" U=");
        Serial.println(_sleep_unit);
      #endif
    }
  #endif
  // setup individual sensors
  for (int i = 0; i < 255; i++) {
    if (_sensors[i] == 0) continue;
    // call each sensor's setup()
    _sensors[i]->before();
  }
}

// present NodeManager and its sensors
void NodeManager::presentation() {
  // present the service as a custom sensor to the controller
  #if DEBUG == 1
    Serial.print("PRES I=");
    Serial.print(CONFIGURATION_CHILD_ID);
    Serial.print(", T=");
    Serial.println(S_CUSTOM);
  #endif
  present(CONFIGURATION_CHILD_ID, S_CUSTOM);
  #if BATTERY_MANAGER == 1 && BATTERY_SENSOR == 1
    #if DEBUG == 1
      Serial.print("PRES I=");
      Serial.print(BATTERY_CHILD_ID);
      Serial.print(", T=");
      Serial.println(S_MULTIMETER);
    #endif
    // present the battery service
    present(BATTERY_CHILD_ID, S_MULTIMETER);
    // report battery level
    _process(BATTERY);
  #endif
  // present each sensor
  for (int i = 0; i < 255; i++) {
    if (_sensors[i] == 0) continue;
    // call each sensor's presentation()
    _sensors[i]->presentation();
  }
  #if DEBUG == 1
    Serial.println("READY");
    Serial.println("");
  #endif
}


// setup NodeManager
void NodeManager::setup() {
  #if DEBUG == 1
    Serial.print("MY I=");
    Serial.print(getNodeId());
    Serial.print(" M=");
    Serial.println(getControllerConfig().isMetric);
  #endif
}

// run the main function for all the register sensors
void NodeManager::loop() {
  #if SLEEP_MANAGER == 1
    MyMessage empty;
    if (_sleep_mode != IDLE &&  _sleep_time != 0) {
      #if POWER_MANAGER == 1
        // turn on the pin powering all the sensors
        if (_auto_power_pins) powerOn();
      #endif
      // run loop for all the registered sensors
      for (int i = 0; i < 255; i++) {
        if (_sensors[i] == 0) continue;
        // call each sensor's loop()
        _sensors[i]->loop(empty);
      }
      #if POWER_MANAGER == 1
        // turn off the pin powering all the sensors
        if (_auto_power_pins) powerOff();
      #endif
      #if SLEEP_MANAGER == 1
        // continue/start sleeping as requested
        _sleep();
      #endif
    }
  #endif
}

// dispacth inbound messages
void NodeManager::receive(const MyMessage &message) {
  #if DEBUG == 1
    Serial.print("RECV S=");
    Serial.print(message.sender);
    Serial.print(" I=");
    Serial.print(message.sensor);
    Serial.print(" C=");
    Serial.print(message.getCommand());
    Serial.print(" T=");
    Serial.print(message.type);
    Serial.print(" D=");
    Serial.println(message.getString());
  #endif
  // process incoming service messages
  if (message.sensor == CONFIGURATION_CHILD_ID && message.getCommand() == C_REQ && message.type == V_CUSTOM) {
    _process(message.getString());
  }
  // dispatch the message to the registered sensor
  else if (message.getCommand() == C_REQ && _sensors[message.sensor] != 0) {
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

// send a message to the network
void NodeManager::_send(MyMessage & message) {
  // send the message, multiple times if requested
  for (int i = 0; i < _retries; i++) {
    // if configured, sleep beetween each send
    if (_sleep_between_send > 0) sleep(_sleep_between_send);
    #if DEBUG == 1
      Serial.print("SEND D=");
      Serial.print(message.destination);
      Serial.print(" I=");
      Serial.print(message.sensor);
      Serial.print(" C=");
      Serial.print(message.getCommand());
      Serial.print(" T=");
      Serial.print(message.type);
      Serial.print(" S=");
      Serial.print(message.getString());
      Serial.print(" I=");
      Serial.print(message.getInt());
      Serial.print(" F=");
      Serial.println(message.getFloat());
    #endif
    send(message);
  }
}

// process a service message
void NodeManager::_process(const char * message) {
  // HELLO: hello request
  if (strcmp(message, "HELLO") == 0) {
    _send(_msg.set(message));
  }
  #if BATTERY_MANAGER == 1
    // BATTERY: return the battery level
    else if (strcmp(message, BATTERY) == 0) {
      // measure the board vcc
      float volt = _getVcc();
      // calculate the percentage
      int percentage = ((volt - _battery_min) / (_battery_max - _battery_min)) * 100;
      if (percentage > 100) percentage = 100;
      if (percentage < 0) percentage = 0;
      #if DEBUG == 1
        Serial.print("BATT V=");
        Serial.print(volt);
        Serial.print(" P=");
        Serial.println(percentage);
      #endif
      #if BATTERY_MANAGER == 1 && BATTERY_SENSOR == 1
        // report battery voltage
        MyMessage battery_msg(BATTERY_CHILD_ID, V_VOLTAGE);
        _send(battery_msg.set(volt, 2));
      #endif
      // report battery level percentage
      sendBatteryLevel(percentage);
    }
  #endif
  // REBOOT: reboot the board
  else if (strcmp(message, REBOOT) == 0 && _reboot_pin > -1) {
    #if DEBUG == 1
      Serial.println(REBOOT);
    #endif
    // set the reboot pin connected to RST to low so to reboot the board
    _send(_msg.set(message));
    digitalWrite(_reboot_pin, LOW);
  }
  // CLEAR: clear the user's eeprom
  else if (strcmp(message, CLEAR) == 0) {
    #if DEBUG == 1
      Serial.println(CLEAR);
    #endif
    for (int i = 0; i <= EEPROM_LAST_ID; i++) saveState(i, 0xFF);
    _send(_msg.set(message));
  }
  // VERSION: send back the extension's version
  else if (strcmp(message, "VERSION") == 0) {
    _send(_msg.set(VERSION, 1));
  }
  #if REMOTE_CONFIGURATION == 1
    // IDxxx: change the node id to the provided one. E.g. ID025: change the node id to 25. Requires a reboot/restart
    else if (strlen(message) == 5 && strncmp("ID", message, strlen("ID")) == 0) {
      // extract the node id
      char s[4];
      s[0] = message[2];
      s[1] = message[3];
      s[2] = message[4];
      s[3] = '\0';
      int node_id = atoi(s);
      #if DEBUG == 1
        Serial.print("MY I=");
        Serial.println(node_id);
      #endif
      // Save static ID to eeprom
      hwWriteConfig(EEPROM_NODE_ID_ADDRESS, (uint8_t)node_id);
      // reboot the board
      #if REBOOT_PIN == 1
        _process(REBOOT);
      #endif
    }
    #if SLEEP_MANAGER == 1
      // MODEx: change the way the board behaves. 0: stay awake, 1: go to sleep for the configured interval, 2: wait for the configured interval (e.g. MODE1)
      else if (strlen(message) == 5 && strncmp("MODE", message, strlen("MODE")) == 0) {
        // extract mode
        char s[2];
        s[0] = message[4];
        s[1] = '\0';
        _sleep_mode = atoi(s);
        #if DEBUG == 1
          Serial.print("SLEEP M=");
          Serial.println(_sleep_mode);
        #endif
        #if PERSIST == 1
          // save it to the eeprom
          saveState(EEPROM_SLEEP_SAVED, 1);
          saveState(EEPROM_SLEEP_MODE, _sleep_mode);
        #endif
        _send(_msg.set(message));
      }
      // INTVLnnnX: set and save the wait/sleep interval to nnn where X is S=Seconds, M=mins, H=Hours, D=Days. E.g. INTVL010M would be 10 minutes
      else if (strlen(message) == 9 && strncmp("INTVL", message, strlen("INTVL")) == 0) {
        // parse and set the sleep interval
        int offset = 5;
        // extract the unit (S=secs, M=mins, H=hours, D=Days)
        char unit[2];
        sprintf(unit, "%c", message[3 + offset]);
        unit[1] = '\0';
        if (strcmp(unit, "S") == 0) _sleep_unit = SECONDS;
        else if (strcmp(unit, "M") == 0) _sleep_unit = MINUTES;
        else if (strcmp(unit, "H") == 0) _sleep_unit = HOURS;
        else if (strcmp(unit, "D") == 0) _sleep_unit = DAYS;
        else return;
        // extract the requested time
        char s[4];
        s[0] = message[0 + offset];
        s[1] = message[1 + offset];
        s[2] = message[2 + offset];
        s[3] = '\0';
        _sleep_time = atoi(s);
        #if DEBUG == 1
          Serial.print("SLEEP T=");
          Serial.print(_sleep_time);
          Serial.print(" U=");
          Serial.println(_sleep_unit);
        #endif
        #if PERSIST == 1
          // save it to eeprom
          saveState(EEPROM_SLEEP_UNIT, _sleep_unit);
          // encode sleep time
          int major = 0;
          if (_sleep_time > 750) major = 3;
          else if (_sleep_time > 500) major = 2;
          else if (_sleep_time > 250) major = 1;
          int minor = _sleep_time - 250 * major;
          saveState(EEPROM_SLEEP_SAVED, 1);
          saveState(EEPROM_SLEEP_TIME_MINOR, minor);
          saveState(EEPROM_SLEEP_TIME_MAJOR, major);
        #endif
        // interval set, reply back with the same message to acknowledge.
        _send(_msg.set(message));
      }
    #endif
    // end remote configuration
  #endif
  // WAKEUP: when received after a sleeping cycle or during wait, abort the cycle and stay awake
  #if SLEEP_MANAGER == 1
    else if (strcmp(message, WAKEUP) == 0) {
      #if DEBUG == 1
        Serial.println(WAKEUP);
      #endif
      _send(_msg.set(message));
      _sleep_mode = IDLE;
    }
  #endif
}

#if SLEEP_MANAGER == 1
// wrapper of smart sleep
void NodeManager::_sleep() {
  // calculate the seconds to sleep
  long sleep_sec = _sleep_time;
  if (_sleep_unit == MINUTES) sleep_sec = sleep_sec * 60;
  else if (_sleep_unit == HOURS) sleep_sec = sleep_sec * 3600;
  else if (_sleep_unit == DAYS) sleep_sec = sleep_sec * 43200;
  long sleep_ms = sleep_sec * 1000;
  #if DEBUG == 1
    Serial.print("SLEEP ");
    Serial.print(sleep_sec);
    Serial.println("s");
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
  if (_sleep_mode == WAIT) {
    // wait for the given interval
    wait(sleep_ms);
  }
  else if (_sleep_mode == SLEEP) {
    // setup interrupt pins
    int interrupt_1_pin = _interrupt_1_mode == MODE_NOT_DEFINED ? INTERRUPT_NOT_DEFINED  : digitalPinToInterrupt(INTERRUPT_PIN_1);
    int interrupt_2_pin = _interrupt_2_mode == MODE_NOT_DEFINED ? INTERRUPT_NOT_DEFINED  : digitalPinToInterrupt(INTERRUPT_PIN_2);
    // enter smart sleep for the requested sleep interval and with the configured interrupts
    int ret = sleep(interrupt_1_pin,_interrupt_1_mode,interrupt_2_pin,_interrupt_2_mode,sleep_ms, true);
    if (ret > -1) {
      int pin_number = -1;
      int interrupt_mode = -1;      
      if (digitalPinToInterrupt(INTERRUPT_PIN_1) == ret) {
        pin_number = INTERRUPT_PIN_1;
        interrupt_mode = _interrupt_1_mode;
      }
      if (digitalPinToInterrupt(INTERRUPT_PIN_2) == ret) {
        pin_number = INTERRUPT_PIN_2;
        interrupt_mode = _interrupt_2_mode;
      }
      #if DEBUG == 1
        Serial.print("WAKE P=");
        Serial.print(pin_number);
        Serial.print(", M=");
        Serial.println(interrupt_mode);
      #endif
      #if SLEEP_MANAGER == 1
        // when waking up from an interrupt on the wakup pin, stop sleeping
        if (_sleep_interrupt_pin == pin_number) _sleep_mode = IDLE;
      #endif
    }
  }
  // coming out of sleep
  #if DEBUG == 1
    Serial.println(AWAKE);
  #endif
  #if SERVICE_MESSAGES == 1
    // notify the controller I am awake
    _send(_msg.set(AWAKE));
  #endif
  #if BATTERY_MANAGER == 1
    // keep track of the number of sleeping cycles
    _cycles++;
    // battery has to be reported after the configured number of sleep cycles
    if (_battery_report_cycles == _cycles) {
      // time to report the battery level again
      _process(BATTERY);
      _cycles = 0;
    }
  #endif
}
#endif


// return the next available child_id
int NodeManager::_getAvailableChildId() {
  for (int i = 1; i < 255; i++) {
    if (i == CONFIGURATION_CHILD_ID) continue;
    if (i == BATTERY_CHILD_ID) continue;
    // empty place, return it
    if (_sensors[i] == 0) return i;
  }
}

#if BATTERY_MANAGER == 1
// return vcc in V
float NodeManager::_getVcc() {
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
  delay(70);
  // Do conversion
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC)) {};
  // return Vcc in mV
  return (float)((1125300UL) / ADC) / 1000;
}
#endif

// guess the initial value of a digital output based on the configured interrupt mode
int NodeManager::_getInterruptInitialValue(int mode) {
  if (mode == RISING) return LOW; 
  if (mode == FALLING) return HIGH; 
  return -1;
}

