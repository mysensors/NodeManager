/*
 * NodeManager
 */
#ifndef NodeManager_h
#define NodeManager_h

#include <Arduino.h>

// define NodeManager version
#define VERSION "1.6-dev"

/***********************************
   Constants
*/

// define board status
#define AWAKE 0
#define SLEEP 1

// define time unit
#define SECONDS 0
#define MINUTES 1
#define HOURS 2
#define DAYS 3

// define on/off
#define OFF 0
#define ON 1

// define value type
#define TYPE_INTEGER 0
#define TYPE_FLOAT 1
#define TYPE_STRING 2

// define interrupt pins
#define INTERRUPT_PIN_1 3
#define INTERRUPT_PIN_2 2

// define eeprom addresses
#define EEPROM_SLEEP_SAVED 0
#define EEPROM_SLEEP_1 5
#define EEPROM_SLEEP_2 6
#define EEPROM_SLEEP_3 7
#define EEPROM_USER_START 100

// define requests

/************************************
 * Include user defined configuration settings
 */
 
#include "config.h"

/***********************************
   Default configuration settings
*/
// if enabled, enable debug messages on serial port
#ifndef DEBUG
  #define DEBUG 1
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

// if enabled, send a SLEEPING and AWAKE service messages just before entering and just after leaving a sleep cycle
#ifndef SERVICE_MESSAGES
  #define SERVICE_MESSAGES 0
#endif
// if enabled, a battery sensor will be created at BATTERY_CHILD_ID (201 by default) and will report vcc voltage together with the battery level percentage
#ifndef BATTERY_SENSOR
  #define BATTERY_SENSOR 1
#endif
// if enabled, a RSSI sensor will be created at SIGNAL_CHILD_ID (202 by default) and will report the signal quality of the transport layer
#ifndef SIGNAL_SENSOR
  #define SIGNAL_SENSOR 1
#endif

// the child id used to allow remote configuration
#ifndef CONFIGURATION_CHILD_ID
  #define CONFIGURATION_CHILD_ID 200
#endif
// the child id used to report the battery voltage to the controller
#ifndef BATTERY_CHILD_ID
  #define BATTERY_CHILD_ID 201
#endif
// the child id used to report the rssi level to the controller
#ifndef SIGNAL_CHILD_ID
  #define SIGNAL_CHILD_ID 202
#endif
// define the maximum number of sensors that can be managed
#ifndef MAX_SENSORS
  #define MAX_SENSORS 10
#endif
// define default sketch name and version
#ifndef SKETCH_NAME
  #define SKETCH_NAME "NodeManager"
#endif
#ifndef SKETCH_VERSION
  #define SKETCH_VERSION "1.0"
#endif


/***********************************
   Default module settings
*/

// Enable this module to use one of the following sensors: SENSOR_ANALOG_INPUT, SENSOR_LDR, SENSOR_THERMISTOR, SENSOR_ML8511, SENSOR_ACS712, SENSOR_RAIN_GAUGE, SENSOR_RAIN, SENSOR_SOIL_MOISTURE
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
// Enable this module to use one of the following sensors: SENSOR_DHT11, SENSOR_DHT22
#ifndef MODULE_DHT
  #define MODULE_DHT 0
#endif
// Enable this module to use one of the following sensors: SENSOR_SHT21, SENSOR_HTU21D
#ifndef MODULE_SHT21
  #define MODULE_SHT21 0
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
// Enable this module to use one of the following sensors: SENSOR_SONOFF
#ifndef MODULE_SONOFF
  #define MODULE_SONOFF 0
#endif
// Enable this module to use one of the following sensors: SENSOR_BMP085
#ifndef MODULE_BMP085
  #define MODULE_BMP085 0
#endif
// Enable this module to use one of the following sensors: SENSOR_HCSR04
#ifndef MODULE_HCSR04
  #define MODULE_HCSR04 0
#endif
// Enable this module to use one of the following sensors: SENSOR_MCP9808
#ifndef MODULE_MCP9808
  #define MODULE_MCP9808 0
#endif
// Enable this module to use one of the following sensors: SENSOR_MQ
#ifndef MODULE_MQ
  #define MODULE_MQ 0
#endif
// Enable this module to use one of the following sensors: SENSOR_MHZ19
#ifndef MODULE_MHZ19
  #define MODULE_MHZ19 0
#endif
// Enable this module to use one of the following sensors: SENSOR_AM2320
#ifndef MODULE_AM2320
  #define MODULE_AM2320 0
#endif
// Enable this module to use one of the following sensors: SENSOR_TSL2561
#ifndef MODULE_TSL2561
  #define MODULE_TSL2561 0
#endif
// Enable this module to use one of the following sensors: SENSOR_PT100
#ifndef MODULE_PT100
  #define SENSOR_PT100 0
#endif
// Enable this module to use one of the following sensors: SENSOR_BMP280
#ifndef MODULE_BMP280
  #define MODULE_BMP280 0
#endif
// Enable this module to use one of the following sensors: SENSOR_DIMMER
#ifndef MODULE_DIMMER
  #define MODULE_DIMMER 0
#endif

/***********************************
   Supported Sensors
*/
enum supported_sensors {
  #if MODULE_ANALOG_INPUT == 1
    // Generic analog sensor, return a pin's analog value or its percentage
    SENSOR_ANALOG_INPUT,
    // LDR sensor, return the light level of an attached light resistor in percentage
    SENSOR_LDR,
    // Thermistor sensor, return the temperature based on the attached thermistor
    SENSOR_THERMISTOR,
    // ML8511 UV sensor
    SENSOR_ML8511,
    // Current sensor
    SENSOR_ACS712,
    // rain gauge sensor
    SENSOR_RAIN_GAUGE,
    // Rain sensor, return the percentage of rain from an attached analog sensor
    SENSOR_RAIN,
    // Soil moisture sensor, return the percentage of moisture from an attached analog sensor
    SENSOR_SOIL_MOISTURE,
  #endif
  #if MODULE_DIGITAL_INPUT == 1
    // Generic digital sensor, return a pin's digital value
    SENSOR_DIGITAL_INPUT,
  #endif
  #if MODULE_DIGITAL_OUTPUT == 1
    // Generic digital output sensor, allows setting the digital output of a pin to the requested value
    SENSOR_DIGITAL_OUTPUT,
    // Relay sensor, allows activating the relay
    SENSOR_RELAY,
    // Latching Relay sensor, allows activating the relay with a pulse
    SENSOR_LATCHING_RELAY,
  #endif
  #if MODULE_DHT == 1
    // DHT11/DHT22 sensors, return temperature/humidity based on the attached DHT sensor
    SENSOR_DHT11,
    SENSOR_DHT22,
  #endif
  #if MODULE_SHT21 == 1
    // SHT21 sensor, return temperature/humidity based on the attached SHT21 sensor
    SENSOR_SHT21,
    SENSOR_HTU21D,
  #endif
  #if MODULE_SWITCH == 1
    // Generic switch, wake up the board when a pin changes status
    SENSOR_SWITCH,
    // Door sensor, wake up the board and report when an attached magnetic sensor has been opened/closed
    SENSOR_DOOR,
    // Motion sensor, wake up the board and report when an attached PIR has triggered
    SENSOR_MOTION,
  #endif
  #if MODULE_DS18B20 == 1
    // DS18B20 sensor, return the temperature based on the attached sensor
    SENSOR_DS18B20,
  #endif
  #if MODULE_BH1750 == 1
    // BH1750 sensor, return light in lux
    SENSOR_BH1750,
  #endif
  #if MODULE_MLX90614 == 1
    // MLX90614 sensor, contactless temperature sensor
    SENSOR_MLX90614,
  #endif
  #if MODULE_BME280 == 1
    // BME280 sensor, return temperature, humidity and pressure
    SENSOR_BME280,
  #endif
  #if MODULE_SONOFF == 1
    // Sonoff wireless smart switch
    SENSOR_SONOFF,
  #endif
  #if MODULE_BMP085 == 1
    // BMP085/BMP180 sensor, return temperature and pressure
    SENSOR_BMP085,
  #endif
  #if MODULE_HCSR04 == 1
    // HC-SR04 sensor, return the distance between the sensor and an object
    SENSOR_HCSR04,
  #endif
  #if MODULE_MCP9808 == 1
    // MCP9808 sensor, precision temperature sensor
    SENSOR_MCP9808,
  #endif
  #if MODULE_MQ == 1
    // MQ2 air quality sensor
    SENSOR_MQ,
  #endif
  #if MODULE_MHZ19 == 1
    // MH-Z19 CO2 sensor
    SENSOR_MHZ19,
  #endif
  #if MODULE_TSL2561 == 1
    // TSL2561 sensor, return light in lux
    SENSOR_TSL2561,
  #endif
  #if MODULE_AM2320 == 1
    // AM2320 sensors, return temperature/humidity based on the attached AM2320 sensor
    SENSOR_AM2320,
  #endif
   #if MODULE_PT100 == 1
    // High temperature sensor associated with DFRobot Driver, return the temperature in C° from the attached PT100 sensor
    SENSOR_PT100,
  #endif
  #if MODULE_BMP280 == 1
    // BMP280 sensor, return temperature and pressure
    SENSOR_BMP280,
  #endif
  #if MODULE_DIMMER == 1
    // Generic dimmer sensor used to drive a pwm output
    SENSOR_DIMMER,
  #endif
};
 
/***********************************
  Libraries
*/

// include supporting libraries
#ifdef MY_USE_UDP
    #include <WiFiUdp.h>
#endif
#ifdef MY_GATEWAY_ESP8266
  #include <ESP8266WiFi.h>
#endif

// include MySensors libraries
#include <core/MySensorsCore.h>
#include <core/MyCapabilities.h>
#include <core/MyTransport.h>
#include <core/Version.h>

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
#if MODULE_SONOFF == 1
  #include <Bounce2.h>
#endif
#if MODULE_BMP085 == 1
  #include <Wire.h>
  #include <Adafruit_BMP085.h>
#endif
#if MODULE_HCSR04 == 1
  #include <NewPing.h>
#endif
#if MODULE_MCP9808 == 1
  #include <Wire.h>
  #include "Adafruit_MCP9808.h"
#endif
#if MODULE_MHZ19 == 1
  #include <SoftwareSerial.h>
#endif
#if MODULE_AM2320 == 1
  #include <Wire.h>
  #include <AM2320.h>
#endif
#if MODULE_TSL2561 == 1
  #include <TSL2561.h>
  #include <Wire.h>
#endif
#if MODULE_PT100 == 1
  #include <DFRobotHighTemperatureSensor.h>
#endif
#if MODULE_BMP280 == 1
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BMP280.h>
#endif
#if MODULE_DIMMER == 1
  #include <math.h>
#endif

/*******************************************************************
   Classes
*/
class NodeManager;

/*
   PowerManager
*/

class PowerManager {
  public:
    PowerManager() {};
    // to save battery the sensor can be optionally connected to two pins which will act as vcc and ground and activated on demand
    void setPowerPins(int ground_pin, int vcc_pin, int wait_time = 50);
    // turns the power pins on
    void powerOn();
    // turns the power pins on
    void powerOff();
  private:
    int _vcc_pin = -1;
    int _ground_pin = -1;
    long _wait = 0;
};

/*
   Timer
*/

class Timer {
  public:
    Timer(NodeManager* node_manager);
    // start the timer which will be over when the configured target passes by
    void start(int target, int unit);
    void start();
    // stop the timer
    void stop();
    // reset the timer
    void reset();
    // reset the timer and start over
    void restart();
    // set the timer configuration but do not start it
    void set(int target, int unit);
    void unset();
    // update the timer. To be called at every cycle
    void update();
    // return true if the time is over
    bool isOver();
    // return true if the timer is running
    bool isRunning();
    // return true if the timer has been configured
    bool isConfigured();
    // return true if this is the first time the timer runs
    bool isFirstRun();
    // return the current elapsed time
    float getElapsed();
   private:
    NodeManager* _node_manager;
    int _target = 0;
    long _elapsed = 0;
    long _last_millis = 0;
    bool _is_running = false;
    bool _is_configured = false;
    bool _first_run = true;
};

/*
   Request
*/

class Request {
  public:
    Request(const char* string);
    // return the parsed function
    int getFunction();
    // return the value as an int
    int getValueInt();
    // return the value as a float
    float getValueFloat();
    // return the value as a string
    char* getValueString();
   private:
    NodeManager* _node_manager;
    int _function;
    char* _value;
};

/***************************************
   Sensor: generic sensor class
*/
class Sensor {
  public:
    Sensor(NodeManager* node_manager, int child_id, int pin);
    // [1] where the sensor is attached to (default: not set)
    void setPin(int value);
    int getPin();
    // [2] child_id of this sensor (default: not set)
    void setChildId(int value);
    int getChildId();
    // presentation of this sensor (default: S_CUSTOM)
    void setPresentation(int value);
    int getPresentation();
    // [3] type of this sensor (default: V_CUSTOM)
    void setType(int value);
    int getType();
    // [4] description of the sensor (default: '')
    void setDescription(char *value);
    // [5] For some sensors, the measurement can be queried multiple times and an average is returned (default: 1)
    void setSamples(int value);
    // [6] If more then one sample has to be taken, set the interval in milliseconds between measurements (default: 0)
    void setSamplesInterval(int value);
    // [7] if true will report the measure only if different than the previous one (default: false)
    void setTrackLastValue(bool value);
    // [9] if track last value is enabled, force to send an update after the configured number of minutes
    void setForceUpdateMinutes(int value);
    // [19] if track last value is enabled, force to send an update after the configured number of hours
    void setForceUpdateHours(int value);
    // [10] the value type of this sensor (default: TYPE_INTEGER)
    void setValueType(int value);
    int getValueType();
    // [11] for float values, set the float precision (default: 2)
    void  setFloatPrecision(int value);
    #if POWER_MANAGER == 1
      // to save battery the sensor can be optionally connected to two pins which will act as vcc and ground and activated on demand
      void setPowerPins(int ground_pin, int vcc_pin, int wait_time = 50);
      // [12] if enabled the pins will be automatically powered on while awake and off during sleeping (default: true)
      void setAutoPowerPins(bool value);
      // [13] manually turn the power on
      void powerOn();
      // [14] manually turn the power off
      void powerOff();
    #endif
    // get the latest recorded value from the sensor
    int getValueInt();
    float getValueFloat();
    char* getValueString();
    // [17] After how many minutes the sensor will report back its measure (default: 10 minutes)
    void setReportIntervalSeconds(int value);
    // [16] After how many minutes the sensor will report back its measure (default: 10 minutes)
    void setReportIntervalMinutes(int value);
    // [19] After how many hours the sensor will report back its measure (default: 10 minutes)
    void setReportIntervalHours(int value);
    // [20] After how many days the sensor will report back its measure (default: 10 minutes)
    void setReportIntervalDays(int value);
    // return true if the report interval has been already configured
    bool isReportIntervalConfigured();
    // process a remote request
    void process(Request & request);
    // return the pin the interrupt is attached to
    int getInterruptPin();
    // listen for interrupts on the given pin so interrupt() will be called when occurring
    void setInterrupt(int pin, int mode, int initial);
    // define what to do at each stage of the sketch
    virtual void before();
    virtual void presentation();
    virtual void setup();
    virtual void loop(const MyMessage & message);
    virtual void interrupt();
    virtual void receive(const MyMessage & message);
    // abstract functions, subclasses need to implement
    virtual void onBefore() = 0;
    virtual void onSetup() = 0;
    virtual void onLoop() = 0;
    virtual void onReceive(const MyMessage & message) = 0;
    virtual void onProcess(Request & request) = 0;
    virtual void onInterrupt() = 0;
  protected:
    MyMessage _msg;
    MyMessage _msg_service;
    NodeManager* _node_manager;
    int _pin = -1;
    int _child_id;
    int _presentation = S_CUSTOM;
    int _type = V_CUSTOM;
    char* _description = "";
    int _samples = 1;
    int _samples_interval = 0;
    bool _track_last_value = false;
    int _value_type = TYPE_INTEGER;
    int _float_precision = 2;
    int _value_int = -1;
    float _value_float = -1;
    char * _value_string = "";
    int _last_value_int = -1;
    float _last_value_float = -1;
    char * _last_value_string = "";
    int _interrupt_pin = -1;
    #if POWER_MANAGER  == 1
      PowerManager _powerManager;
      bool _auto_power_pins = true;
    #endif
    Timer* _report_timer;
    Timer* _force_update_timer;
    void _send(MyMessage & msg);
    bool _isReceive(const MyMessage & message);
    bool _isWorthSending(bool comparison);
};

#if MODULE_ANALOG_INPUT == 1
/*
   SensorAnalogInput: read the analog input of a configured pin
*/
class SensorAnalogInput: public Sensor {
  public:
    SensorAnalogInput(NodeManager* node_manager, int child_id, int pin);
    // [101] the analog reference to use (default: not set, can be either INTERNAL or DEFAULT)
    void setReference(int value);
    // [102] reverse the value or the percentage (e.g. 70% -> 30%) (default: false)
    void setReverse(bool value);
    // [103] when true returns the value as a percentage (default: true)
    void setOutputPercentage(bool value);
    // [104] minimum value for calculating the percentage (default: 0)
    void setRangeMin(int value);
    // [105] maximum value for calculating the percentage (default: 1024)
    void setRangeMax(int value);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
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
    SensorLDR(NodeManager* node_manager, int child_id, int pin);
};

/*
   SensorThermistor: read the temperature from a thermistor
*/
class SensorThermistor: public Sensor {
  public:
    SensorThermistor(NodeManager* node_manager, int child_id, int pin);
    // [101] resistance at 25 degrees C (default: 10000)
    void setNominalResistor(long value);
    // [102] temperature for nominal resistance (default: 25)
    void setNominalTemperature(int value);
    // [103] The beta coefficient of the thermistor (default: 3950)
    void setBCoefficient(int value);
    // [104] the value of the resistor in series with the thermistor (default: 10000)
    void setSeriesResistor(long value);
    // [105] set a temperature offset
    void setOffset(float value);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
  protected:
    long _nominal_resistor = 10000;
    int _nominal_temperature = 25;
    int _b_coefficient = 3950;
    long _series_resistor = 10000;
    float _offset = 0;
};

/*
    SensorML8511
*/

class SensorML8511: public Sensor {
  public:
    SensorML8511(NodeManager* node_manager, int child_id, int pin);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
  protected:
    float _mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
};

/*
    SensorACS712
*/

class SensorACS712: public Sensor {
  public:
    SensorACS712(NodeManager* node_manager, int child_id, int pin);
    // [101] set how many mV are equivalent to 1 Amp. The value depends on the module (100 for 20A Module, 66 for 30A Module) (default: 185);
    void setmVPerAmp(int value);
    // [102] set ACS offset (default: 2500);
    void setOffset(int value);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
  protected:
    int _ACS_offset = 2500;
    int _mv_per_amp = 185;
};

/*
    SensorRainGauge
*/

class SensorRainGauge: public Sensor {
  public:
    SensorRainGauge(NodeManager* node_manager, int child_id, int pin);
    // [102] set how many mm of rain to count for each tip (default: 0.11)
    void setSingleTip(float value);
    // set initial value - internal pull up (default: HIGH)
    void setInitialValue(int value);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
  protected:
    long _count = 0;
    float _single_tip = 0.11;
    int _initial_value = HIGH;
};

/*
   SensorRain
*/
class SensorRain: public SensorAnalogInput {
  public:
    SensorRain(NodeManager* node_manager, int child_id, int pin);
};

/*
   SensorSoilMoisture
*/
class SensorSoilMoisture: public SensorAnalogInput {
  public:
    SensorSoilMoisture(NodeManager* node_manager, int child_id, int pin);
};
#endif


#if MODULE_DIGITAL_INPUT == 1
/*
   SensorDigitalInput: read the digital input of the configured pin
*/
class SensorDigitalInput: public Sensor {
  public:
    SensorDigitalInput(NodeManager* node_manager, int child_id, int pin);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
};
#endif

#if MODULE_DIGITAL_OUTPUT == 1
/*
   SensorDigitalOutput: control a digital output of the configured pin
*/
class SensorDigitalOutput: public Sensor {
  public:
    SensorDigitalOutput(NodeManager* node_manager, int child_id, int pin);
    // [103] define which value to set to the output when set to on (default: HIGH)
    void setOnValue(int value);
    // [104] when legacy mode is enabled expect a REQ message to trigger, otherwise the default SET (default: false)
    void setLegacyMode(bool value);
    // [105] automatically turn the output off after the given number of minutes
    void setSafeguard(int value);
    // [106] if true the input value becomes a duration in minutes after which the output will be automatically turned off (default: false)
    void setInputIsElapsed(bool value);
    // [107] optionally wait for the given number of milliseconds after changing the status (default: 0)
    void setWaitAfterSet(int value);
    // manually switch the output to the provided value
    void setStatus(int value);
    // get the current state
    int getStatus();
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
  protected:
    int _on_value = HIGH;
    int _status = OFF;
    bool _legacy_mode = false;
    bool _input_is_elapsed = false;
    int _wait_after_set = 0;
    Timer* _safeguard_timer;
    void _setupPin(int pin);
    virtual void _setStatus(int value);
    int _getValueToWrite(int value);
};


/*
   SensorRelay
*/
class SensorRelay: public SensorDigitalOutput {
  public:
    SensorRelay(NodeManager* node_manager, int child_id, int pin);
};

/*
   SensorLatchingRelay
*/
class SensorLatchingRelay: public SensorRelay {
  public:
    SensorLatchingRelay(NodeManager* node_manager, int child_id, int pin);
    // [201] set the duration of the pulse to send in ms to activate the relay (default: 50)
    void setPulseWidth(int value);
    // [202] set the pin which turns the relay off (default: the pin provided while registering the sensor)
    void setPinOff(int value);
    // [203] set the pin which turns the relay on (default: the pin provided while registering the sensor + 1)
    void setPinOn(int value);
    // define what to do at each stage of the sketch
    void onBefore();
    void onProcess(Request & request);
  protected:
    int _pin_on;
    int _pin_off;
    int _pulse_width = 50;
    void _setStatus(int value);
};
#endif

/*
   SensorDHT
*/
#if MODULE_DHT == 1
class SensorDHT: public Sensor {
  public:
    SensorDHT(NodeManager* node_manager, int child_id, int pin, DHT* dht, int sensor_type, int dht_type);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
    // constants
    const static int TEMPERATURE = 0;
    const static int HUMIDITY = 1;
  protected:
    DHT* _dht;
    int _dht_type;
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
    SensorSHT21(NodeManager* node_manager, int child_id, int sensor_type);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
    // constants
    const static int TEMPERATURE = 0;
    const static int HUMIDITY = 1;
  protected:
    float _offset = 0;
    int _sensor_type = 0;
};

/*
   SensorHTU21D: temperature and humidity sensor
*/

class SensorHTU21D: public SensorSHT21 {
  public:
    SensorHTU21D(NodeManager* node_manager, int child_id, int pin);
};
#endif

/*
 * SensorSwitch
 */
#if MODULE_SWITCH == 1
class SensorSwitch: public Sensor {
  public:
    SensorSwitch(NodeManager* node_manager, int child_id, int pin);
    // [101] set the interrupt mode. Can be CHANGE, RISING, FALLING (default: CHANGE)
    void setMode(int value);
    // [102] milliseconds to wait before reading the input (default: 0)
    void setDebounce(int value);
    // [103] time to wait in milliseconds after a change is detected to allow the signal to be restored to its normal value (default: 0)
    void setTriggerTime(int value);
    // [104] Set initial value on the interrupt pin (default: HIGH)
    void setInitial(int value);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
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
    SensorDoor(NodeManager* node_manager, int child_id, int pin);
};

/*
 * SensorMotion
 */
class SensorMotion: public SensorSwitch {
  public:
    SensorMotion(NodeManager* node_manager, int child_id, int pin);
};
#endif
/*
   SensorDs18b20
*/
#if MODULE_DS18B20 == 1
class SensorDs18b20: public Sensor {
  public:
    SensorDs18b20(NodeManager* node_manager, int child_id, int pin, DallasTemperature* sensors, int index);
    // returns the sensor's resolution in bits
    int getResolution();
    // [101] set the sensor's resolution in bits
    void setResolution(int value);
    // [102] sleep while DS18B20 calculates temperature (default: false)
    void setSleepDuringConversion(bool value);
    // return the sensors' device address
    DeviceAddress* getDeviceAddress();
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
  protected:
    float _offset = 0;
    int _index;
    bool _sleep_during_conversion = false;
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
    SensorBH1750(NodeManager* node_manager, int child_id);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
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
    SensorMLX90614(NodeManager* node_manager, int child_id, Adafruit_MLX90614* mlx, int sensor_type);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
    // constants
    const static int TEMPERATURE_AMBIENT = 0;
    const static int TEMPERATURE_OBJECT = 1;
  protected:
    Adafruit_MLX90614* _mlx;
    int _sensor_type;
};
#endif


/*
 * SensorBosch
*/

#if MODULE_BME280 == 1 || MODULE_BMP085 == 1 || MODULE_BMP280 == 1
class SensorBosch: public Sensor {
  public:
    SensorBosch(NodeManager* node_manager, int child_id, int sensor_type);
    // [101] define how many pressure samples to keep track of for calculating the forecast (default: 5)
    void setForecastSamplesCount(int value);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
    // constants
    const static int TEMPERATURE = 0;
    const static int HUMIDITY = 1;
    const static int PRESSURE = 2;
    const static int FORECAST = 3;
    static uint8_t GetI2CAddress(uint8_t chip_id);
  protected:
    int _sensor_type;
    char* _weather[6] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
    int _forecast_samples_count = 5;
    float* _forecast_samples;
    int _minute_count = 0;
    float _pressure_avg;
    float _pressure_avg2;
    float _dP_dt;
    bool _first_round = true;
    float _getLastPressureSamplesAverage();
    void _forecast(float pressure);
};
#endif

/*
   SensorBME280
*/
#if MODULE_BME280 == 1
class SensorBME280: public SensorBosch {
  public:
    SensorBME280(NodeManager* node_manager, int child_id, Adafruit_BME280* bme, int sensor_type);
    void onLoop();
  protected:
    Adafruit_BME280* _bme;
};
#endif

/*
   SensorBMP085
*/
#if MODULE_BMP085 == 1
class SensorBMP085: public SensorBosch {
  public:
    SensorBMP085(NodeManager* node_manager, int child_id, Adafruit_BMP085* bmp, int sensor_type);
    void onLoop();
  protected:
    Adafruit_BMP085* _bmp;
};
#endif

/*
   SensorBMP280
*/
#if MODULE_BMP280 == 1
class SensorBMP280: public SensorBosch {
  public:
    SensorBMP280(NodeManager* node_manager, int child_id, Adafruit_BMP280* bmp, int sensor_type);
    void onLoop();
  protected:
    Adafruit_BMP280* _bmp;
};
#endif

/*
   SensorHCSR04
*/
#if MODULE_HCSR04 == 1
class SensorHCSR04: public Sensor {
  public:
    SensorHCSR04(NodeManager* node_manager, int child_id, int pin);
    // [101] Arduino pin tied to trigger pin on the ultrasonic sensor (default: the pin set while registering the sensor)
    void setTriggerPin(int value);
    // [102] Arduino pin tied to echo pin on the ultrasonic sensor (default: the pin set while registering the sensor)
    void setEchoPin(int value);
    // [103] Maximum distance we want to ping for (in centimeters) (default: 300)
    void setMaxDistance(int value);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
  protected:
    int _trigger_pin;
    int _echo_pin;
    int _max_distance = 300;
    NewPing* _sonar;
};
#endif

/*
   SensorSonoff
*/
#if MODULE_SONOFF == 1
class SensorSonoff: public Sensor {
  public:
    SensorSonoff(NodeManager* node_manager, int child_id);
    // [101] set the button's pin (default: 0)
    void setButtonPin(int value);
    // [102] set the relay's pin (default: 12)
    void setRelayPin(int value);
    // [103] set the led's pin (default: 13)
    void setLedPin(int value);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
  protected:
    Bounce _debouncer = Bounce();
    int _button_pin = 0;
    int _relay_pin = 12;
    int _led_pin = 13;
    int _old_value = 0;
    bool _state = false;
    int _relay_on = 1;
    int _relay_off = 0;
    int _led_on = 0;
    int _led_off = 1;
    void _blink();
    void _toggle();
};
#endif

/*
   SensorMCP9808
*/
#if MODULE_MCP9808 == 1
class SensorMCP9808: public Sensor {
  public:
    SensorMCP9808(NodeManager* node_manager, int child_id, Adafruit_MCP9808* mcp);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
  protected:
    Adafruit_MCP9808* _mcp;
};
#endif

/*
    SensorMQ
 */
 #if MODULE_MQ == 1
class SensorMQ: public Sensor {
  public:
    SensorMQ(NodeManager* node_manager, int child_id, int pin);
    // [101] define the target gas whose ppm has to be returned. 0: LPG, 1: CO, 2: Smoke (default: 1);
    void setTargetGas(int value);
    // [102] define the load resistance on the board, in kilo ohms (default: 1);
    void setRlValue(float value);
    // [103] define the Ro resistance on the board (default: 10000);
    void setRoValue(float value);
    // [104] Sensor resistance in clean air (default: 9.83);
    void setCleanAirFactor(float value);
    // [105] define how many samples you are going to take in the calibration phase (default: 50);
    void setCalibrationSampleTimes(int value);
    // [106] define the time interal(in milisecond) between each samples in the cablibration phase (default: 500);
    void setCalibrationSampleInterval(int value);
    // [107] define how many samples you are going to take in normal operation (default: 50);
    void setReadSampleTimes(int value);
    // [108] define the time interal(in milisecond) between each samples in the normal operations (default: 5);
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
    void onProcess(Request & request);
    void onInterrupt();
  protected:
    float _rl_value = 1.0;
    float _ro_clean_air_factor = 9.83;
    int _calibration_sample_times = 50;
    int _calibration_sample_interval = 500;
    int _read_sample_interval = 50;
    int _read_sample_times = 5;
    float _ro = 10000.0;
    static float _default_LPGCurve[3];
    static float _default_COCurve[3];
    static float _default_SmokeCurve[3];
    float *_LPGCurve;
    float *_COCurve;
    float *_SmokeCurve;
    float _MQResistanceCalculation(int raw_adc);
    float _MQCalibration();
    float _MQRead();
    int _MQGetGasPercentage(float rs_ro_ratio, int gas_id);
    int  _MQGetPercentage(float rs_ro_ratio, float *pcurve);
    const static int _gas_lpg = 0;
    const static int _gas_co = 1;
    const static int _gas_smoke = 2;
    int _target_gas = _gas_co;
};
#endif

/*
   SensorMHZ19
*/
#if MODULE_MHZ19 == 1
class SensorMHZ19: public Sensor {
  public:
    SensorMHZ19(NodeManager* node_manager, int child_id, int pin);
    // set the pins for RX and TX of the SoftwareSerial (default: Rx=6, Tx=7)
    void setRxTx(int rxpin, int txpin);
    int readCO2();
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
  protected:
    SoftwareSerial* _ser;
    int _tx_pin = 6;
    int _rx_pin = 7;
};
#endif

/*
   SensorAM2320
*/
#if MODULE_AM2320 == 1
class SensorAM2320: public Sensor {
  public:
    SensorAM2320(NodeManager* node_manager, int child_id, AM2320* th, int sensor_type);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
    // constants
    const static int TEMPERATURE = 0;
    const static int HUMIDITY = 1;
  protected:
    AM2320* _th;
    int _sensor_type = 0;
};
#endif

/*
   SensorTSL2561
*/
#if MODULE_TSL2561 == 1
class SensorTSL2561: public Sensor {
  public:
    SensorTSL2561(NodeManager* node_manager, int child_id);
    // [101] set the gain, possible values are SensorTSL2561::GAIN_0X (0), SensorTSL2561::GAIN_16X (1) (default 16x)
    void setGain(int value);
    // [102] set the timing, possible values are SensorTSL2561::INTEGRATIONTIME_13MS (0), SensorTSL2561::INTEGRATIONTIME_101MS (1), SensorTSL2561::INTEGRATIONTIME_402MS (2) (default: 13ms)
    void setTiming(int value);
    // [103] set the spectrum, possible values are SensorTSL2561::VISIBLE (0), SensorTSL2561::FULLSPECTRUM (1), SensorTSL2561::INFRARED (2), SensorTSL2561::FULL (3) (default: visible)
    void setSpectrum(int value);
    // [104] set the i2c address values are SensorTSL2561::ADDR_FLOAT, SensorTSL2561::ADDR_LOW, SensorTSL2561::ADDR_HIGH
    void setAddress(int value);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
    // constants
    const static int ADDR_FLOAT = 0;
    const static int ADDR_LOW = 1;
    const static int ADDR_HIGH = 2;
    const static int GAIN_0X = 0;
    const static int GAIN_16X = 1;
    const static int INTEGRATIONTIME_13MS = 0;
    const static int INTEGRATIONTIME_101MS = 1;
    const static int INTEGRATIONTIME_402MS = 2;
    const static int VISIBLE = 0;
    const static int FULLSPECTRUM = 1;
    const static int INFRARED = 2;
    const static int FULL = 3;
  protected:
    TSL2561* _tsl;
    int _tsl_address = 0;
    int _tsl_gain = 1;
    int _tsl_timing = 0;
    int _tsl_spectrum = 0;
};
#endif

/*
    SensorPT100
*/
#if MODULE_PT100 == 1
class SensorPT100: public Sensor {
  public:
    SensorPT100(NodeManager* node_manager, int child_id, int pin);
    // [101] set the voltageRef used to compare with analog measures
    void setVoltageRef(float value);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
  protected:
    DFRobotHighTemperature* _PT100;
    float _voltageRef = 3.3;
};
#endif

/*
    SensorPT100
*/
#if MODULE_DIMMER == 1
class SensorDimmer: public Sensor {
  public:
    SensorDimmer(NodeManager* node_manager, int child_id, int pin);
    // [101] set the effect to use for a smooth transition, can be one of SensorDimmer::EASE_LINEAR, SensorDimmer::EASE_INSINE, SensorDimmer::EASE_OUTSINE, SensorDimmer::EASE_INOUTSINE (default: EASE_LINEAR)
    void setEasing(int value);
    // [102] the duration of entire the transition in seconds (default: 1)
    void setDuration(int value);
    // [103] the duration of a single step of the transition in milliseconds (default: 100)
    void setStepDuration(int value);
    // fade the output from the current value to the target provided in the range 0-100
    void fadeTo(int value);
    enum easing {
      EASE_LINEAR,
      EASE_INSINE,
      EASE_OUTSINE,
      EASE_INOUTSINE,
    };
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop();
    void onReceive(const MyMessage & message);
    void onProcess(Request & request);
    void onInterrupt();
  protected:
    int _percentage = 0;
    int _easing = EASE_LINEAR;
    int _duration = 1000;
    int _step_duration = 100;
    float _getEasing(float t, float b, float c, float d);
};
#endif

/***************************************
   NodeManager: manages all the aspects of the node
*/
class NodeManager {
  public:
    NodeManager();
    // [10] send the same service message multiple times (default: 1)
    void setRetries(int value);
    int getRetries();
    #if BATTERY_MANAGER == 1
      // [11] the expected vcc when the batter is fully discharged, used to calculate the percentage (default: 2.7)
      void setBatteryMin(float value);
      // [12] the expected vcc when the batter is fully charged, used to calculate the percentage (default: 3.3)
      void setBatteryMax(float value);
      // [14] after how many minutes report the battery level to the controller. When reset the battery is always reported (default: 60 minutes)
      void setBatteryReportMinutes(int value);
      // [40] after how many minutes report the battery level to the controller. When reset the battery is always reported (default: 60 minutes)
      void setBatteryReportSeconds(int value);
      // [41] after how many minutes report the battery level to the controller. When reset the battery is always reported (default: 60 minutes)
      void setBatteryReportHours(int value);
      // [42] after how many minutes report the battery level to the controller. When reset the battery is always reported (default: 60 minutes)
      void setBatteryReportDays(int value);
      // [15] if true, the battery level will be evaluated by measuring the internal vcc without the need to connect any pin, if false the voltage divider methon will be used (default: true)
      void setBatteryInternalVcc(bool value);
      // [16] if setBatteryInternalVcc() is set to false, the analog pin to which the battery's vcc is attached (https://www.mysensors.org/build/battery) (default: -1)
      void setBatteryPin(int value);
      // [17] if setBatteryInternalVcc() is set to false, the volts per bit ratio used to calculate the battery voltage (default: 0.003363075)
      void setBatteryVoltsPerBit(float value);
      // [18] If true, wake up by an interrupt counts as a valid cycle for battery reports otherwise only uninterrupted sleep cycles would contribute (default: true)
      void setBatteryReportWithInterrupt(bool value);
      // [2] Send a battery level report to the controller
      void batteryReport();
    #endif
    // [3] set the duration (in seconds) of a sleep cycle
    void setSleepSeconds(int value);
    long getSleepSeconds();
    // [4] set the duration (in minutes) of a sleep cycle
    void setSleepMinutes(int value);
    // [5] set the duration (in hours) of a sleep cycle
    void setSleepHours(int value);
    // [29] set the duration (in days) of a sleep cycle
    void setSleepDays(int value);
    // [19] if enabled, when waking up from the interrupt, the board stops sleeping. Disable it when attaching e.g. a motion sensor (default: true)
    void setSleepInterruptPin(int value);
    // configure the interrupt pin and mode. Mode can be CHANGE, RISING, FALLING (default: MODE_NOT_DEFINED)
    void setInterrupt(int pin, int mode, int initial = -1);
    // [28] ignore two consecutive interrupts if happening within this timeframe in milliseconds (default: 100)
    void setInterruptMinDelta(long value);
    // [20] optionally sleep interval in milliseconds before sending each message to the radio network (default: 0)
    void setSleepBetweenSend(int value);
    int getSleepBetweenSend();
    // register a built-in sensor
    int registerSensor(int sensor_type, int pin = -1, int child_id = -1);
    // register a custom sensor
    int registerSensor(Sensor* sensor);
    // [26] un-register a sensor
    void unRegisterSensor(int sensor_index);
    // return a sensor by its index
    Sensor* get(int sensor_index);
    Sensor* getSensor(int sensor_index);
    // assign a different child id to a sensor
    bool renameSensor(int old_child_id, int new_child_id);
    #if POWER_MANAGER == 1
      // to save battery the sensor can be optionally connected to two pins which will act as vcc and ground and activated on demand
      void setPowerPins(int ground_pin, int vcc_pin, int wait_time = 50);
      // [23] if enabled the pins will be automatically powered on while awake and off during sleeping (default: true)
      void setAutoPowerPins(bool value);
      // [24] manually turn the power on
      void powerOn();
      // [25] manually turn the power off
      void powerOff();
    #endif
    // [21] set this to true if you want destination node to send ack back to this node (default: false)
    void setAck(bool value);
    bool getAck();
    // request and return the current timestamp from the controller
    long getTimestamp();
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
    // [8] send NodeManager's the version back to the controller
    void version();
    // [7] clear the EEPROM
    void clearEeprom();
    // [9] wake up the board
    void wakeup();
    // process a remote request
    void process(Request & request);
    // return the value stored at the requested index from the EEPROM
    int loadFromMemory(int index);
    // [27] save the given index of the EEPROM the provided value
    void saveToMemory(int index, int value);
    // return vcc in V
    float getVcc();
    // setup the configured interrupt pins
    void setupInterrupts();
    // return the pin from which the last interrupt came
    int getLastInterruptPin();
    // [36] set the default interval in minutes all the sensors will report their measures. If the same function is called on a specific sensor, this will not change the previously set value. or sleeping sensors, the elapsed time can be evaluated only upon wake up (default: 10 minutes)
    void setReportIntervalSeconds(int value);
    // [37] set the default interval in minutes all the sensors will report their measures. If the same function is called on a specific sensor, this will not change the previously set value. or sleeping sensors, the elapsed time can be evaluated only upon wake up (default: 10 minutes)
    void setReportIntervalMinutes(int value);
    // [38] set the default interval in minutes all the sensors will report their measures. If the same function is called on a specific sensor, this will not change the previously set value. or sleeping sensors, the elapsed time can be evaluated only upon wake up (default: 10 minutes)
    void setReportIntervalHours(int value);
    // [39] set the default interval in minutes all the sensors will report their measures. If the same function is called on a specific sensor, this will not change the previously set value. or sleeping sensors, the elapsed time can be evaluated only upon wake up (default: 10 minutes)
    void setReportIntervalDays(int value);
    // [30] if set and when the board is battery powered, sleep() is always called instead of wait() (default: true)
    void setSleepOrWait(bool value);
    // sleep if the node is a battery powered or wait if it is not for the given number of milliseconds 
    void sleepOrWait(long value);
    // [31] set which pin is connected to RST of the board to reboot the board when requested. If not set the software reboot is used instead (default: -1)
    void setRebootPin(int value);
    // [32] turn the ADC off so to save 0.2 mA
    void setADCOff();
    #if SIGNAL_SENSOR == 1 && defined(MY_SIGNAL_REPORT_ENABLED)
      // [33] How frequenly to send a signal report to the controller (default: 60 minutes)
      void setSignalReportMinutes(int value);
      // [43] How frequenly to send a signal report to the controller (default: 60 minutes)
      void setSignalReportSeconds(int value);
      // [44] How frequenly to send a signal report to the controller (default: 60 minutes)
      void setSignalReportHours(int value);
      // [45] How frequenly to send a signal report to the controller (default: 60 minutes)
      void setSignalReportDays(int value);
      // [34] define which signal report to send. Possible values are SR_UPLINK_QUALITY, SR_TX_POWER_LEVEL, SR_TX_POWER_PERCENT, SR_TX_RSSI, SR_RX_RSSI, SR_TX_SNR, SR_RX_SNR (default: SR_RX_RSSI)
      void setSignalCommand(int value);
      // [35] report the signal level to the controller
      void signalReport();
    #endif
    // hook into the main sketch functions
    void before();
    void presentation();
    void setup();
    void loop();
    void receive(const MyMessage & msg);
    void receiveTime(unsigned long ts);
    // handle interrupts
    static void _onInterrupt_1();
    static void _onInterrupt_2();
  private:
    #if BATTERY_MANAGER == 1
      float _battery_min = 2.6;
      float _battery_max = 3.3;
      Timer _battery_report_timer = Timer(this);
      bool _battery_report_with_interrupt = true;
      bool _battery_internal_vcc = true;
      int _battery_pin = -1;
      float _battery_volts_per_bit = 0.003363075;
    #endif
    #if POWER_MANAGER == 1
      // to optionally controller power pins
      PowerManager _powerManager;
      bool _auto_power_pins = true;
    #endif
    #if SIGNAL_SENSOR == 1 && defined(MY_SIGNAL_REPORT_ENABLED)
      Timer _signal_report_timer = Timer(this);
      int _signal_command = SR_RX_RSSI;
    #endif
    MyMessage _msg;
    void _send(MyMessage & msg);
    int _status = AWAKE;
    long _sleep_time = 0;
    int _sleep_interrupt_pin = -1;
    int _sleep_between_send = 0;
    int _retries = 1;
    int _interrupt_1_mode = MODE_NOT_DEFINED;
    int _interrupt_2_mode = MODE_NOT_DEFINED;
    int _interrupt_1_initial = -1;
    int _interrupt_2_initial = -1;
    static int _last_interrupt_pin;
    static long _interrupt_min_delta;
    static long _last_interrupt_1;
    static long _last_interrupt_2;
    long _timestamp = -1;
    Sensor* _sensors[MAX_SENSORS+1] = {0};
    bool _ack = false;
    void _sleep();
    void _present(int child_id, int type);
    int _getAvailableChildId();
    int _getInterruptInitialValue(int mode);
    bool _get_controller_config = true;
    int _is_metric = 1;
    int _report_interval_seconds = 10*60;
    bool _sleep_or_wait = true;
    int _reboot_pin = -1;
    void _loadConfig();
    void _saveConfig();
};

#endif
