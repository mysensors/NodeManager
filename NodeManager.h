/*
 * NodeManager
 */
 
#ifndef NodeManager_h
#define NodeManager_h

#include <Arduino.h>

// define NodeManager version
#define VERSION "1.4"

/***********************************
   Constants
*/

// define sleep mode
#define IDLE 0
#define SLEEP 1
#define WAIT 2

// define time unit
#define SECONDS 0
#define MINUTES 1
#define HOURS 2
#define DAYS 3

// define value type
#define TYPE_INTEGER 0
#define TYPE_FLOAT 1
#define TYPE_STRING 2

// define interrupt pins
#define INTERRUPT_PIN_1 3
#define INTERRUPT_PIN_2 2

// define eeprom addresses
#define EEPROM_LAST_ID 4
#define EEPROM_SLEEP_SAVED 0
#define EEPROM_SLEEP_MODE 1
#define EEPROM_SLEEP_TIME_MAJOR 2
#define EEPROM_SLEEP_TIME_MINOR 3
#define EEPROM_SLEEP_UNIT 4

/************************************
 * Include user defined configuration settings
 */
 
#include "config.h"

/***********************************
   Default configuration settings
*/

// if enabled, will load the sleep manager library. Sleep mode and sleep interval have to be configured to make the board sleeping/waiting
#ifndef SLEEP_MANAGER
  #define SLEEP_MANAGER 1
#endif
// if enabled, enable the capability to power on sensors with the arduino's pins to save battery while sleeping
#ifndef POWER_MANAGER
  #define POWER_MANAGER 1
#endif
// if enabled, will load the battery manager library to allow the battery level to be reported automatically or on demand
#ifndef BATTERY_MANAGER
  #define BATTERY_MANAGER 1
#endif
// if enabled, allow modifying the configuration remotely by interacting with the configuration child id
#ifndef REMOTE_CONFIGURATION
  #define REMOTE_CONFIGURATION 1
#endif
// if enabled, persist the configuration settings on EEPROM
#ifndef PERSIST
  #define PERSIST 0
#endif

// if enabled, enable debug messages on serial port
#ifndef DEBUG
  #define DEBUG 1
#endif

// if enabled, send a SLEEPING and AWAKE service messages just before entering and just after leaving a sleep cycle
#ifndef SERVICE_MESSAGES
  #define SERVICE_MESSAGES 1
#endif
// if enabled, a battery sensor will be created at BATTERY_CHILD_ID and will report vcc voltage together with the battery level percentage
#ifndef BATTERY_SENSOR
  #define BATTERY_SENSOR 1
#endif

// the child id used to allow remote configuration
#ifndef CONFIGURATION_CHILD_ID
  #define CONFIGURATION_CHILD_ID 200
#endif
// the child id used to report the battery voltage to the controller
#ifndef BATTERY_CHILD_ID
  #define BATTERY_CHILD_ID 201
#endif

// Enable this module to use one of the following sensors: SENSOR_ANALOG_INPUT, SENSOR_LDR, SENSOR_THERMISTOR, SENSOR_MQ
#ifndef MODULE_ANALOG_INPUT
  #define MODULE_ANALOG_INPUT 0
#endif
// Enable this module to use one of the following sensors: SENSOR_DIGITAL_INPUT
#ifndef MODULE_DIGITAL_INPUT
  #define MODULE_DIGITAL_INPUT 0
#endif
// Enable this module to use one of the following sensors: SENSOR_DIGITAL_OUTPUT, SENSOR_RELAY, SENSOR_LATCHING_RELAY
#ifndef MODULE_DIGITAL_OUTPUT
  #define MODULE_DIGITAL_OUTPUT 0
#endif
// Enable this module to use one of the following sensors: SENSOR_SHT21, SENSOR_HTU21D
#ifndef MODULE_SHT21
  #define MODULE_SHT21 0
#endif
// Enable this module to use one of the following sensors: SENSOR_DHT11, SENSOR_DHT22
#ifndef MODULE_DHT
  #define MODULE_DHT 0
#endif
// Enable this module to use one of the following sensors: SENSOR_SWITCH, SENSOR_DOOR, SENSOR_MOTION
#ifndef MODULE_SWITCH
  #define MODULE_SWITCH 0
#endif
// Enable this module to use one of the following sensors: SENSOR_DS18B20
#ifndef MODULE_DS18B20
  #define MODULE_DS18B20 0
#endif
// Enable this module to use one of the following sensors: SENSOR_BH1750
#ifndef MODULE_BH1750
  #define MODULE_BH1750 0
#endif
// Enable this module to use one of the following sensors: SENSOR_MLX90614
#ifndef MODULE_MLX90614
  #define MODULE_MLX90614 0
#endif
// Enable this module to use one of the following sensors: SENSOR_BME280
#ifndef MODULE_BME280
  #define MODULE_BME280 0
#endif
/***********************************
   Sensors types
*/
#if MODULE_ANALOG_INPUT == 1
  // Generic analog sensor, return a pin's analog value or its percentage
  #define SENSOR_ANALOG_INPUT 1
  // LDR sensor, return the light level of an attached light resistor in percentage
  #define SENSOR_LDR 2
  // Thermistor sensor, return the temperature based on the attached thermistor
  #define SENSOR_THERMISTOR 3
  // MQ2 air quality sensor
  #define SENSOR_MQ 19
  // ML8511 UV sensor
  #define SENSOR_ML8511 20
#endif
#if MODULE_DIGITAL_INPUT == 1
  // Generic digital sensor, return a pin's digital value
  #define SENSOR_DIGITAL_INPUT 4
#endif
#if MODULE_DIGITAL_OUTPUT == 1
  // Generic digital output sensor, allows setting the digital output of a pin to the requested value
  #define SENSOR_DIGITAL_OUTPUT 5
  // Relay sensor, allows activating the relay
  #define SENSOR_RELAY 6
  // Latching Relay sensor, allows activating the relay with a pulse
  #define SENSOR_LATCHING_RELAY 7
#endif
#if MODULE_DHT == 1
  // DHT11/DHT22 sensors, return temperature/humidity based on the attached DHT sensor
  #define SENSOR_DHT11 8
  #define SENSOR_DHT22 9
#endif
#if MODULE_SHT21 == 1
  // SHT21 sensor, return temperature/humidity based on the attached SHT21 sensor
  #define SENSOR_SHT21 10
  #define SENSOR_HTU21D 15
#endif
#if MODULE_SWITCH == 1
  // Generic switch, wake up the board when a pin changes status
  #define SENSOR_SWITCH 11
  // Door sensor, wake up the board and report when an attached magnetic sensor has been opened/closed
  #define SENSOR_DOOR 12
  // Motion sensor, wake up the board and report when an attached PIR has triggered
  #define SENSOR_MOTION 13
#endif
#if MODULE_DS18B20 == 1
  // DS18B20 sensor, return the temperature based on the attached sensor
  #define SENSOR_DS18B20 14
#endif
#if MODULE_BH1750 == 1
  // BH1750 sensor, return light in lux
  #define SENSOR_BH1750 16
#endif
#if MODULE_MLX90614 == 1
  // MLX90614 sensor, contactless temperature sensor
  #define SENSOR_MLX90614 17
#endif
#if MODULE_BME280 == 1
  // MLX90614 sensor, contactless temperature sensor
  #define SENSOR_BME280 18
#endif
// last Id: 20
/***********************************
  Libraries
*/

// include MySensors libraries
#include <core/MySensorsCore.h>
#include <core/MyHwAVR.h>
//#include <core/MyHwATMega328.h>

// include third party libraries
#if MODULE_DHT == 1
  #include <DHT.h>
#endif
#if MODULE_SHT21 == 1
  #include <Wire.h>
  #include <Sodaq_SHT2x.h>
#endif
#if MODULE_DS18B20 == 1
  #include <OneWire.h>
  #include <DallasTemperature.h>
#endif
#if MODULE_BH1750 == 1
  #include <BH1750.h>
  #include <Wire.h>
#endif
#if MODULE_MLX90614 == 1
  #include <Wire.h>
  #include <Adafruit_MLX90614.h>
#endif
#if MODULE_BME280 == 1
  #include <Wire.h>
  #include <SPI.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
#endif

/**************************************
   Classes
*/

/*
   PowerManager
*/

class PowerManager {
  public:
    PowerManager() {};
    // to save battery the sensor can be optionally connected to two pins which will act as vcc and ground and activated on demand
    void setPowerPins(int ground_pin, int vcc_pin, int wait_time = 50);
    void powerOn();
    void powerOff();
    float getVcc();
    bool isConfigured();
  private:
    int _vcc_pin = -1;
    int _ground_pin = -1;
    long _wait = 0;
};


/***************************************
   Sensor: generic sensor class
*/
class Sensor {
  public:
    Sensor(int child_id, int pin);
    // where the sensor is attached to (default: not set)
    void setPin(int value);
    int getPin();
    // child_id of this sensor (default: not set)
    void setChildId(int value);
    int getChildId();
    // presentation of this sensor (default: S_CUSTOM)
    void setPresentation(int value);
    int getPresentation();
    // type of this sensor (default: V_CUSTOM)
    void setType(int value);
    int getType();
    // when queried, send the message multiple times (default: 1)
    void setRetries(int value);
    // For some sensors, the measurement can be queried multiple times and an average is returned (default: 1)
    void setSamples(int value);
    // If more then one sample has to be taken, set the interval in milliseconds between measurements (default: 0)
    void setSamplesInterval(int value);
    // if true will report the measure only if different then the previous one (default: false)
    void setTackLastValue(bool value);
    // if track last value is enabled, force to send an update after the configured number of cycles (default: -1)
    void setForceUpdate(int value);
    // the value type of this sensor (default: TYPE_INTEGER)
    void setValueType(int value);
    // for float values, set the float precision (default: 2)
    void setFloatPrecision(int value);
    // optionally sleep interval in milliseconds before sending each message to the radio network (default: 0)
    void setSleepBetweenSend(int value);
    #if POWER_MANAGER == 1
      // to save battery the sensor can be optionally connected to two pins which will act as vcc and ground and activated on demand
      void setPowerPins(int ground_pin, int vcc_pin, int wait_time = 50);
      // if enabled the pins will be automatically powered on while awake and off during sleeping (default: true)
      void setAutoPowerPins(bool value);
      // manually turn the power on
      void powerOn();
      // manually turn the power off
      void powerOff();
    #endif
    // define what to do at each stage of the sketch
    virtual void before();
    virtual void presentation();
    virtual void setup();
    virtual void loop(const MyMessage & message);
    virtual void receive(const MyMessage & message);
    // abstract functions, subclasses need to implement
    virtual void onBefore() = 0;
    virtual void onSetup() = 0;
    virtual void onLoop() = 0;
    virtual void onReceive(const MyMessage & message) = 0;
  protected:
    MyMessage _msg;
    int _sleep_between_send = 0;
    int _pin = -1;
    int _child_id;
    int _presentation = S_CUSTOM;
    int _type = V_CUSTOM;
    int _retries = 1;
    int _samples = 1;
    int _samples_interval = 0;
    bool _track_last_value = false;
    int _cycles = 0;
    int _force_update = -1;
    void _send(MyMessage & msg);
    #if POWER_MANAGER  == 1
      PowerManager _powerManager;
      bool _auto_power_pins = true;
    #endif
    int _value_type = TYPE_INTEGER;
    int _float_precision = 2;
    int _value_int = -1;
    float _value_float = -1;
    char * _value_string = "";
    int _last_value_int = -1;
    float _last_value_float = -1;
    char * _last_value_string = "";
};

/*
   SensorAnalogInput: read the analog input of a configured pin
*/
class SensorAnalogInput: public Sensor {
  public:
    SensorAnalogInput(int child_id, int pin);
    // the analog reference to use (default: not set, can be either INTERNAL or DEFAULT)
    void setReference(int value);
    // reverse the value or the percentage (e.g. 70% -> 30%) (default: false)
    void setReverse(bool value);
    // when true returns the value as a percentage (default: true)
    void setOutputPercentage(bool value);
    // minimum value for calculating the percentage (default: 0)
    void setRangeMin(int value);
    // maximum value for calculating the percentage (default: 1024)
    void setRangeMax(int value);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
  protected:
    int _reference = -1;
    bool _reverse = false;
    bool _output_percentage = true;
    int _range_min = 0;
    int _range_max = 1024;
    int _getPercentage(int value);
    int _getAnalogRead();
};

/*
   SensorLDR: return the percentage of light from a Light dependent resistor
*/
class SensorLDR: public SensorAnalogInput {
  public:
    SensorLDR(int child_id, int pin);
};

/*
   SensorThermistor: read the temperature from a thermistor
*/
class SensorThermistor: public Sensor {
  public:
    SensorThermistor(int child_id, int pin);
    // resistance at 25 degrees C (default: 10000)
    void setNominalResistor(int value);
    // temperature for nominal resistance (default: 25)
    void setNominalTemperature(int value);
    // The beta coefficient of the thermistor (default: 3950)
    void setBCoefficient(int value);
    // the value of the resistor in series with the thermistor (default: 10000)
    void setSeriesResistor(int value);
    // set a temperature offset
    void setOffset(float value);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
  protected:
    int _nominal_resistor = 10000;
    int _nominal_temperature = 25;
    int _b_coefficient = 3950;
    int _series_resistor = 10000;
    float _offset = 0;
};

/*
    SensorMQ
 */
class SensorMQ: public Sensor {
  public:
    SensorMQ(int child_id, int pin);
    // define the target gas whose ppm has to be returned. 0: LPG, 1: CO, 2: Smoke (default: 1);
    void setTargetGas(int value);
    // define the load resistance on the board, in kilo ohms (default: 1);
    void setRlValue(float value);
    // define the Ro resistance on the board (default: 10000);
    void setRoValue(float value);
    // Sensor resistance in clean air (default: 9.83);
    void setCleanAirFactor(float value);
    // define how many samples you are going to take in the calibration phase (default: 50);
    void setCalibrationSampleTimes(int value);
    // define the time interal(in milisecond) between each samples in the cablibration phase (default: 500);
    void setCalibrationSampleInterval(int value);
    // define how many samples you are going to take in normal operation (default: 50);
    void setReadSampleTimes(int value);
    // define the time interal(in milisecond) between each samples in the normal operations (default: 5);
    void setReadSampleInterval(int value);
    // set the LPGCurve array (default: {2.3,0.21,-0.47})
    void setLPGCurve(float *value);
    // set the COCurve array (default: {2.3,0.72,-0.34})
    void setCOCurve(float *value);
    // set the SmokeCurve array (default: {2.3,0.53,-0.44})
    void setSmokeCurve(float *value);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
  protected:
    float _rl_value = 1.0;
    float _ro_clean_air_factor = 9.83;
    int _calibration_sample_times = 50;
    int _calibration_sample_interval = 500;
    int _read_sample_interval = 50;
    int _read_sample_times = 5;
    float _ro = 10000.0;
    float _LPGCurve[3] = {2.3,0.21,-0.47};
    float _COCurve[3] = {2.3,0.72,-0.34};
    float _SmokeCurve[3] = {2.3,0.53,-0.44};
    float _MQResistanceCalculation(int raw_adc);
    float _MQCalibration();
    float _MQRead();
    int _MQGetGasPercentage(float rs_ro_ratio, int gas_id);
    int  _MQGetPercentage(float rs_ro_ratio, float *pcurve);
    int _gas_lpg = 0;
    int _gas_co = 1;
    int _gas_smoke = 2;
    int _target_gas = _gas_co;
};

/*
    SensorML8511
 */

class SensorML8511: public Sensor {
  public:
    SensorML8511(int child_id, int pin);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
  protected:
    float _mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
};
  

/*
   SensorDigitalInput: read the digital input of the configured pin
*/
class SensorDigitalInput: public Sensor {
  public:
    SensorDigitalInput(int child_id, int pin);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
};

/*
   SensorDigitalOutput: control a digital output of the configured pin
*/
class SensorDigitalOutput: public Sensor {
  public:
    SensorDigitalOutput(int child_id, int pin);
    // set how to initialize the output (default: LOW)
    void setInitialValue(int value);
    // if greater than 0, send a pulse of the given duration in ms and then restore the output back to the original value (default: 0)
    void setPulseWidth(int value);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
  protected:
    int _initial_value = LOW;
    int _pulse_width = 0;
};


/*
   SensorRelay
*/
class SensorRelay: public SensorDigitalOutput {
  public:
    SensorRelay(int child_id, int pin);
    // define what to do at each stage of the sketch
    void onLoop();
};

/*
   SensorLatchingRelay
*/
class SensorLatchingRelay: public SensorRelay {
  public:
    SensorLatchingRelay(int child_id, int pin);
};

/*
   SensorDHT
*/
#if MODULE_DHT == 1
class SensorDHT: public Sensor {
  public:
    SensorDHT(int child_id, int pin, DHT* dht, int sensor_type, int dht_type);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
  protected:
    DHT* _dht;
    int _dht_type = DHT11;
    float _offset = 0;
    int _sensor_type = 0;
};
#endif

/*
   SensorSHT21: temperature and humidity sensor
*/
#if MODULE_SHT21 == 1
class SensorSHT21: public Sensor {
  public:
    SensorSHT21(int child_id, int sensor_type);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
  protected:
    float _offset = 0;
    int _sensor_type = 0;
};

/*
   SensorHTU21D: temperature and humidity sensor
*/

class SensorHTU21D: public SensorSHT21 {
  public:
    SensorHTU21D(int child_id, int pin);
};
#endif

/*
 * SensorSwitch
 */
class SensorSwitch: public Sensor {
  public:
    SensorSwitch(int child_id, int pin);
    // set the interrupt mode. Can be CHANGE, RISING, FALLING (default: CHANGE)
    void setMode(int value);
    int getMode();
    // milliseconds to wait before reading the input (default: 0)
    void setDebounce(int value);
    // time to wait in milliseconds after a change is detected to allow the signal to be restored to its normal value (default: 0)
    void setTriggerTime(int value);
    // Set initial value on the interrupt pin (default: HIGH)
    void setInitial(int value);
    int getInitial();
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
  protected:
    int _debounce = 0;
    int _trigger_time = 0;
    int _mode = CHANGE;
    int _initial = HIGH;
};

/*
 * SensorDoor
 */
class SensorDoor: public SensorSwitch {
  public:
    SensorDoor(int child_id, int pin);
};

/*
 * SensorMotion
 */
class SensorMotion: public SensorSwitch {
  public:
    SensorMotion(int child_id, int pin);
};

/*
   SensorDs18b20
*/
#if MODULE_DS18B20 == 1
class SensorDs18b20: public Sensor {
  public:
    SensorDs18b20(int child_id, int pin, DallasTemperature* sensors, int index);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    // return the sensors' device address
    DeviceAddress* getDeviceAddress();
    // returns the sensor's resolution in bits
    int getResolution();
    // set the sensor's resolution in bits
    void setResolution(int value);
  protected:
    float _offset = 0;
    int _index;
    DallasTemperature* _sensors;
    DeviceAddress _device_address;
};
#endif

/*
   SensorBH1750
*/
#if MODULE_BH1750 == 1
class SensorBH1750: public Sensor {
  public:
    SensorBH1750(int child_id);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
  protected:
    BH1750* _lightSensor;
};
#endif

/*
   SensorMLX90614
*/
#if MODULE_MLX90614 == 1
class SensorMLX90614: public Sensor {
  public:
    SensorMLX90614(int child_id, Adafruit_MLX90614* mlx, int sensor_type);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
  protected:
    Adafruit_MLX90614* _mlx;
    int _sensor_type;
};
#endif

/*
   SensorBME280
*/
#if MODULE_BME280 == 1
class SensorBME280: public Sensor {
  public:
    SensorBME280(int child_id, Adafruit_BME280* bme, int sensor_type);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
  protected:
    Adafruit_BME280* _bme;
    int _sensor_type;
};
#endif



/***************************************
   NodeManager: manages all the aspects of the node
*/
class NodeManager {
  public:
    NodeManager();
    // the pin to connect to the RST pin to reboot the board (default: 4)
    void setRebootPin(int value);
    // send the same service message multiple times (default: 1)
    void setRetries(int value);
    #if BATTERY_MANAGER == 1
      // the expected vcc when the batter is fully discharged, used to calculate the percentage (default: 2.7)
      void setBatteryMin(float value);
      // the expected vcc when the batter is fully charged, used to calculate the percentage (default: 3.3)
      void setBatteryMax(float value);
      // after how many sleeping cycles report the battery level to the controller. When reset the battery is always reported (default: 10)
      void setBatteryReportCycles(int value);
      // if true, the battery level will be evaluated by measuring the internal vcc without the need to connect any pin, if false the voltage divider methon will be used (default: true)
      void setBatteryInternalVcc(bool value);
      // if setBatteryInternalVcc() is set to false, the analog pin to which the battery's vcc is attached (https://www.mysensors.org/build/battery) (default: -1)
      void setBatteryPin(int value);
      // if setBatteryInternalVcc() is set to false, the volts per bit ratio used to calculate the battery voltage (default: 0.003363075)
      void setBatteryVoltsPerBit(float value);
      // If true, wake up by an interrupt counts as a valid cycle for battery reports otherwise only uninterrupted sleep cycles would contribute (default: true)
      void setBatteryReportWithInterrupt(bool value);
    #endif
    #if SLEEP_MANAGER == 1
      // define if the board has to sleep every time entering loop (default: IDLE). It can be IDLE (no sleep), SLEEP (sleep at every cycle), WAIT (wait at every cycle)
      void setSleepMode(int value);
      // define for how long the board will sleep (default: 0)
      void setSleepTime(int value);
      // define the unit of SLEEP_TIME. It can be SECONDS, MINUTES, HOURS or DAYS (default: MINUTES)
      void setSleep(int value1, int value2, int value3);
      void setSleepUnit(int value);
      // if enabled, when waking up from the interrupt, the board stops sleeping. Disable it when attaching e.g. a motion sensor (default: true)
      void setSleepInterruptPin(int value);
    #endif
    // configure the interrupt pin and mode. Mode can be CHANGE, RISING, FALLING (default: MODE_NOT_DEFINED)
    void setInterrupt(int pin, int mode, int pull = -1);
    // optionally sleep interval in milliseconds before sending each message to the radio network (default: 0)
    void setSleepBetweenSend(int value);
    // register a built-in sensor
    int registerSensor(int sensor_type, int pin = -1, int child_id = -1);
    // register a custom sensor
    int registerSensor(Sensor* sensor);
    // return a sensor by its index
    Sensor* get(int sensor_index);
    Sensor* getSensor(int sensor_index);
    // assign a different child id to a sensor
    bool renameSensor(int old_child_id, int new_child_id);
    #if POWER_MANAGER == 1
      // to save battery the sensor can be optionally connected to two pins which will act as vcc and ground and activated on demand
      void setPowerPins(int ground_pin, int vcc_pin, int wait_time = 50);
      // if enabled the pins will be automatically powered on while awake and off during sleeping (default: true)
      void setAutoPowerPins(bool value);
      // manually turn the power on
      void powerOn();
      // manually turn the power off
      void powerOff();
    #endif
    // hook into the main sketch functions
    void before();
    void presentation();
    void setup();
    void loop();
    void receive(const MyMessage & msg);
  private:
    #if SLEEP_MANAGER == 1
      int _sleep_mode = IDLE;
      int _sleep_time = 0;
      int _sleep_unit = MINUTES;
      int _sleep_interrupt_pin = -1;
    #endif
    #if BATTERY_MANAGER == 1
      float _battery_min = 2.6;
      float _battery_max = 3.3;
      int _battery_report_cycles = 10;
      bool _battery_report_with_interrupt = true;
      bool _battery_internal_vcc = true;
      int _battery_pin = -1;
      float _battery_volts_per_bit = 0.003363075;
      int _cycles = 0;
      float _getVcc();
    #endif
    #if POWER_MANAGER == 1
      // to optionally controller power pins
      PowerManager _powerManager;
      bool _auto_power_pins = true;
    #endif
    MyMessage _msg;
    void _send(MyMessage & msg);
    int _sleep_between_send = 0;
    int _retries = 1;
    int _interrupt_1_mode = MODE_NOT_DEFINED;
    int _interrupt_2_mode = MODE_NOT_DEFINED;
    int _interrupt_1_pull = -1;
    int _interrupt_2_pull = -1;
    int _reboot_pin = -1;
    Sensor* _sensors[255] = {0};
    void _process(const char * message);
    void _sleep();
    int _getAvailableChildId();
    int _getInterruptInitialValue(int mode);
};

#endif
