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
// define board sleep status
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

// include third party libraries
#ifdef USE_SIGNAL
  #define MY_SIGNAL_REPORT_ENABLED
#endif
#ifdef USE_DHT
  #include <DHT.h>
#endif
#ifdef USE_SHT21
  #include <Wire.h>
  #include <Sodaq_SHT2x.h>
#endif
#ifdef USE_DS18B20
  #include <OneWire.h>
  #include <DallasTemperature.h>
#endif
#ifdef USE_BH1750
  #include <BH1750.h>
  #include <Wire.h>
#endif
#ifdef USE_MLX90614
  #include <Wire.h>
  #include <Adafruit_MLX90614.h>
#endif
#ifdef USE_BME280
  #include <Wire.h>
  #include <SPI.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
#endif
#ifdef USE_SONOFF
  #include <Bounce2.h>
#endif
#ifdef USE_BMP085_180
  #include <Wire.h>
  #include <Adafruit_BMP085.h>
#endif
#ifdef USE_HCSR04
  #include <NewPing.h>
#endif
#ifdef USE_MCP9808
  #include <Wire.h>
  #include "Adafruit_MCP9808.h"
#endif
#ifdef USE_MHZ19
  #include <SoftwareSerial.h>
#endif
#ifdef USE_AM2320
  #include <Wire.h>
  #include <AM2320.h>
#endif
#ifdef USE_TSL2561
  #include <TSL2561.h>
  #include <Wire.h>
#endif
#ifdef USE_PT100
  #include <DFRobotHighTemperatureSensor.h>
#endif
#ifdef USE_BMP280
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BMP280.h>
#endif
#ifdef USE_DIMMER
  #include <math.h>
#endif
#if defined(USE_PCA9685W) || defined(USE_PCA9685RGB) || defined(USE_PCA9685RGBW)
  #include <math.h>
  #include <Wire.h>
  #include <Adafruit_PWMServoDriver.h>
#endif
#ifdef USE_PMS
  #include <PMS.h>
  #include <SoftwareSerial.h> 
#endif
#ifdef USE_VL53L0X
  #include <Wire.h>
  #include <VL53L0X.h>
#endif
#ifdef USE_SSD1306
  #include <SSD1306Ascii.h>
  #include <SSD1306AsciiAvrI2c.h>
#endif
#ifdef USE_SHT31
  #include <Wire.h>
  #include "Adafruit_SHT31.h"
#endif
#ifdef USE_SI7021
  #include <Wire.h>
  #include "SparkFun_Si7021_Breakout_Library.h"
#endif
#ifdef USE_CHIRP
  #include <Wire.h>
  #include <I2CSoilMoistureSensor.h>
#endif
#ifdef USE_HD44780
  #include <Wire.h> 
  #include <LiquidCrystal_I2C.h>
#endif
#ifdef USE_SERVO
  #include <Servo.h>
#endif
#ifdef USE_APDS9960
  #include <Wire.h>
  #include <SparkFun_APDS9960.h>
#endif
#ifdef USE_NEOPIXEL
#if defined(ARDUINO_ARCH_STM32F0) || defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F3) || defined(ARDUINO_ARCH_STM32F4) || defined(ARDUINO_ARCH_STM32L4)
  #include <NeoMaple.h>
#else
  #include <Adafruit_NeoPixel.h>
#endif
#endif
#ifdef USE_SDS011
  #include <SDS011.h>
#endif
#ifdef USE_FPM10A
  #include <Adafruit_Fingerprint.h>
  #include <SoftwareSerial.h>
#endif

// include third party libraries for enabled features
#ifdef MY_GATEWAY_SERIAL
  #define FEATURE_SLEEP OFF
  #define FEATURE_DEBUG OFF
#endif
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
    NodeManager* _node;
    int _target = 0;
    long _elapsed = 0;
    bool _is_running = false;
    bool _is_configured = false;
    bool _first_run = true;
    long _last = 0;
};

/*
   Request
*/

class Request {
  public:
    Request(int child_id, const char* string);
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
    void setForceUpdateMinutes(int value);
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
    void setValueInt(int value);
    int getValueInt();
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
    void setValueFloat(float value);
    float getValueFloat();
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
    void setValueDouble(double value);
    double getValueDouble();
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
    void setValueString(const char* value);
    const char* getValueString();
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
    // [21] enable/disable reporting to the gateway (default: true)
    void setReporting(bool value);
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
#if FEATURE_INTERRUPTS == ON
    // return the pin the interrupt is attached to
    int getInterruptPin();
    // listen for interrupts on the given pin so interrupt() will be called when occurring
    void setInterrupt(int pin, int mode, int initial);
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
    void interrupt();
#endif
    Child* getChild(int child_id);
    // register a child
    void registerChild(Child* child);
    NodeManager* _node;
    // define what to do at each stage of the sketch
    void before();
    void presentation();
    void setup();
    void loop(MyMessage* message);
#if FEATURE_RECEIVE == ON
    void receive(MyMessage* message);
#endif
    // abstract functions, subclasses need to implement
    virtual void onBefore();
    virtual void onSetup();
    virtual void onLoop(Child* child);
    virtual void onReceive(MyMessage* message);
    virtual void onInterrupt();
  protected:
    const char* _name = "";
    int _pin = -1;
    int _samples = 1;
    int _samples_interval = 0;
    bool _reporting = true;
#if FEATURE_INTERRUPTS == ON
    int _interrupt_pin = -1;
#endif
#if FEATURE_POWER_MANAGER == ON
    PowerManager* _powerManager = nullptr;
#endif
    Timer* _report_timer;
#if FEATURE_HOOKING == ON
    void (*_setup_hook)(Sensor* sensor);
    void (*_pre_loop_hook)(Sensor* sensor);
    void (*_post_loop_hook)(Sensor* sensor);
    void (*_interrupt_hook)(Sensor* sensor);
    void (*_receive_hook)(Sensor* sensor, MyMessage* message);
#endif
};

#ifdef USE_BATTERY
/*
   SensorBattery: report battery level
*/
class SensorBattery: public Sensor {
  public:
    SensorBattery(NodeManager& nodeManager, int child_id = BATTERY_CHILD_ID);
    // [102] the expected vcc when the batter is fully discharged, used to calculate the percentage (default: 2.7)
    void setMinVoltage(float value);
    // [103] the expected vcc when the batter is fully charged, used to calculate the percentage (default: 3.3)
    void setMaxVoltage(float value);
    // [104] if true, the battery level will be evaluated by measuring the internal vcc without the need to connect any pin, if false the voltage divider methon will be used (default: true)
    void setBatteryInternalVcc(bool value);
    // [105] if setBatteryInternalVcc() is set to false, the analog pin to which the battery's vcc is attached (https://www.mysensors.org/build/battery) (default: -1)
    void setBatteryPin(int value);
    // [106] if setBatteryInternalVcc() is set to false, the volts per bit ratio used to calculate the battery voltage (default: 0.003363075)
    void setBatteryVoltsPerBit(float value);
    // [107] change battery voltage calibration factor
    void setBatteryCalibrationFactor(float value);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
      float _battery_min = 2.6;
      float _battery_max = 3.3;
      bool _battery_internal_vcc = true;
      int _battery_pin = -1;
      float _battery_volts_per_bit = 0.003363075;
      float _battery_adj_factor = 1.0;
};
#endif

#ifdef USE_SIGNAL
/*
   SensorSignal: report RSSI signal strength from the radio
*/
class SensorSignal: public Sensor {
  public:
    SensorSignal(NodeManager& nodeManager, int child_id = SIGNAL_CHILD_ID);
    // [101] define which signal report to send. Possible values are SR_UPLINK_QUALITY, SR_TX_POWER_LEVEL, SR_TX_POWER_PERCENT, SR_TX_RSSI, SR_RX_RSSI, SR_TX_SNR, SR_RX_SNR (default: SR_RX_RSSI)
    void setSignalCommand(int value);
    // define what to do at each stage of the sketch
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    int _signal_command = SR_RX_RSSI;
};
#endif

#ifdef USE_CONFIGURATION
/*
   SensorConfiguration: allow remote configuration of the board and any configured sensor
*/
class SensorConfiguration: public Sensor {
  public:
    SensorConfiguration(NodeManager& nodeManager);
    // define what to do at each stage of the sketch
    void onBefore();
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
};
#endif

#ifdef USE_ANALOG_INPUT
/*
   SensorAnalogInput: read the analog input of a configured pin
*/
class SensorAnalogInput: public Sensor {
  public:
    SensorAnalogInput(NodeManager& node_manager, int pin, int child_id = -255);
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
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
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
    SensorLDR(NodeManager& node_manager, int pin, int child_id = -255);
    // define what to do at each stage of the sketch
    void onSetup();
};

/*
   SensorRain
*/
class SensorRain: public SensorAnalogInput {
  public:
    SensorRain(NodeManager& node_manager, int pin, int child_id = -255);
    // define what to do at each stage of the sketch
    void onSetup();
};

/*
   SensorSoilMoisture
*/
class SensorSoilMoisture: public SensorAnalogInput {
  public:
    SensorSoilMoisture(NodeManager& node_manager, int pin, int child_id = -255);
    // define what to do at each stage of the sketch
    void onSetup();
};
#endif

#ifdef USE_THERMISTOR
/*
   SensorThermistor: read the temperature from a thermistor
*/
class SensorThermistor: public Sensor {
  public:
    SensorThermistor(NodeManager& node_manager, int pin, int child_id = -255);
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
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    long _nominal_resistor = 10000;
    int _nominal_temperature = 25;
    int _b_coefficient = 3950;
    long _series_resistor = 10000;
    float _offset = 0;
};
#endif

#ifdef USE_ML8511
/*
    SensorML8511
*/

class SensorML8511: public Sensor {
  public:
    SensorML8511(NodeManager& node_Manager, int pin, int child_id = -255);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    float _mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
};
#endif

#ifdef USE_ACS712
/*
    SensorACS712
*/

class SensorACS712: public Sensor {
  public:
    SensorACS712(NodeManager& node_manager, int pin, int child_id = -255);
    // [101] set how many mV are equivalent to 1 Amp. The value depends on the module (100 for 20A Module, 66 for 30A Module) (default: 185);
    void setmVPerAmp(int value);
    // [102] set ACS offset (default: 2500);
    void setOffset(int value);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    int _ACS_offset = 2500;
    int _mv_per_amp = 185;
};
#endif

#ifdef USE_DIGITAL_INPUT
/*
   SensorDigitalInput: read the digital input of the configured pin
*/
class SensorDigitalInput: public Sensor {
  public:
    SensorDigitalInput(NodeManager& node_manager, int pin, int child_id = -255);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
};
#endif

#ifdef USE_DIGITAL_OUTPUT
/*
   SensorDigitalOutput: control a digital output of the configured pin
*/
class SensorDigitalOutput: public Sensor {
  public:
    SensorDigitalOutput(NodeManager& node_manager, int pin, int child_id = -255);
    // [104] when legacy mode is enabled expect a REQ message to trigger, otherwise the default SET (default: false)
    void setLegacyMode(bool value);
    // [105] automatically turn the output off after the given number of minutes
    void setSafeguard(int value);
    // [106] if true the input value becomes a duration in minutes after which the output will be automatically turned off (default: false)
    void setInputIsElapsed(bool value);
    // [107] optionally wait for the given number of milliseconds after changing the status (default: 0)
    void setWaitAfterSet(int value);
    // [108] when switching on, turns the output off after the given number of milliseconds. For latching relay controls the pulse width (default: 0)
    void setPulseWidth(int value);
    // [109] Invert the value to write. E.g. if ON is received, write LOW (default: false) 
    void setInvertValueToWrite(bool value);
    // [110] for a 2-pins latching relay, set the pin which turns the relay off (default: -1)
    void setPinOff(int value);
    // manually switch the output to the provided status (ON or OFF)
    void setStatus(int value);
    // toggle the status
    void toggleStatus();
    // get the current state
    int getStatus();
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    int _status = OFF;
    int _pin_off = -1;
    bool _legacy_mode = false;
    bool _input_is_elapsed = false;
    int _wait_after_set = 0;
    int _pulse_width = 0;
    bool _invert_value_to_write = false;
    Timer* _safeguard_timer = new Timer(_node);
    virtual void _switchOutput(int value);
};

/*
   SensorRelay
*/
class SensorRelay: public SensorDigitalOutput {
  public:
    SensorRelay(NodeManager& node_manager, int pin, int child_id = -255);
};

/*
   SensorLatchingRelay1Pin
*/
class SensorLatchingRelay1Pin: public SensorRelay {
  public:
    SensorLatchingRelay1Pin(NodeManager& node_manager, int pin, int child_id = -255);
};

/*
   SensorLatchingRelay2Pins
*/
class SensorLatchingRelay2Pins: public SensorRelay {
  public:
    SensorLatchingRelay2Pins(NodeManager& node_manager, int pin_off, int pin_on, int child_id = -255);
};
#endif

/*
   SensorDHT
*/
#ifdef USE_DHT
class SensorDHT: public Sensor {
  public:
    SensorDHT(NodeManager& node_manager, int pin, int child_id = -255);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    DHT* _dht;
    int _dht_type;
    float _offset = 0;
};

/*
   SensorDHT11
*/
class SensorDHT11: public SensorDHT {
  public:
    SensorDHT11(NodeManager& node_manager, int pin, int child_id = -255);
};

/*
   SensorDHT22
*/
class SensorDHT22: public SensorDHT {
  public:
    SensorDHT22(NodeManager& node_manager, int pin, int child_id = -255);
};
#endif

/*
   SensorSHT21: temperature and humidity sensor
*/
#ifdef USE_SHT21
class SensorSHT21: public Sensor {
  public:
    SensorSHT21(NodeManager& node_manager, int child_id = -255);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
};

/*
   SensorHTU21D: temperature and humidity sensor
*/

class SensorHTU21D: public SensorSHT21 {
  public:
    SensorHTU21D(NodeManager& nodeManager, int child_id = -255);
};
#endif

/*
 * SensorInterrupt
 */
#ifdef USE_INTERRUPT
class SensorInterrupt: public Sensor {
  public:
    SensorInterrupt(NodeManager& node_manager, int pin, int child_id = -255);
    // [101] set the interrupt mode. Can be CHANGE, RISING, FALLING (default: CHANGE)
    void setInterruptMode(int value);
    // [103] milliseconds to wait/sleep after the interrupt before reporting (default: 0)
    void setWaitAfterTrigger(int value);
    // [104] Set initial value on the interrupt pin. Can be used for internal pull up (default: HIGH)
    void setInitialValue(int value);
    // [105] Invert the value to report. E.g. if FALLING and value is LOW, report HIGH (default: false) 
    void setInvertValueToReport(bool value);
    // [106] Set armed, if false the sensor will not trigger until armed (default: true) 
    void setArmed(bool value);
#if FEATURE_TIME == ON
    // [107] when keeping track of the time, trigger only after X consecutive interrupts within the same minute (default: 1)
    void setThreshold(int value);
#endif
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
    void onInterrupt();
  protected:
    int _wait_after_trigger = 0;
    int _interrupt_mode = CHANGE;
    int _initial_value = HIGH;
    bool _invert_value_to_report = false;
    bool _armed = true;
#if FEATURE_TIME == ON
    int _threshold = 1;
    int _counter = 0;
    int _current_minute = minute();
#endif
};

/*
 * SensorDoor
 */
class SensorDoor: public SensorInterrupt {
  public:
    SensorDoor(NodeManager& node_manager, int pin, int child_id = -255);
};

/*
 * SensorMotion
 */
class SensorMotion: public SensorInterrupt {
  public:
    SensorMotion(NodeManager& node_manager, int pin, int child_id = -255);
};
#endif
/*
   SensorDs18b20
*/
#ifdef USE_DS18B20
class SensorDs18b20: public Sensor {
  public:
    SensorDs18b20(NodeManager& node_manager, int pin, int child_id = -255);
    // returns the sensor's resolution in bits
    int getResolution();
    // [101] set the sensor's resolution in bits
    void setResolution(int value);
    // [102] sleep while DS18B20 calculates temperature (default: false)
    void setSleepDuringConversion(bool value);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    bool _sleep_during_conversion = false;
    DallasTemperature* _sensors;
    char* _getAddress(int index);
};
#endif

/*
   SensorBH1750
*/
#ifdef USE_BH1750
class SensorBH1750: public Sensor {
  public:
    SensorBH1750(NodeManager& node_manager, int child_id = -255);
    // [101] set sensor reading mode, e.g. BH1750_ONE_TIME_HIGH_RES_MODE
    void setMode(uint8_t mode);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    BH1750* _lightSensor;
};
#endif

/*
   SensorMLX90614
*/
#ifdef USE_MLX90614
class SensorMLX90614: public Sensor {
  public:
    SensorMLX90614(NodeManager& node_manager, int child_id = -255);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    Adafruit_MLX90614* _mlx;
    int _sensor_type;
};
#endif


/*
 * SensorBosch
*/

#if defined(USE_BME280) || defined(USE_BMP085_180) || defined(USE_BMP280)
class SensorBosch: public Sensor {
  public:
    SensorBosch(NodeManager& node_manager, int child_id = -255);
    // [101] define how many pressure samples to keep track of for calculating the forecast (default: 5)
    void setForecastSamplesCount(int value);
    // define what to do at each stage of the sketch
    void onReceive(MyMessage* message);
    uint8_t detectI2CAddress(uint8_t chip_id);
  protected:
    const char* _weather[6] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
    int _forecast_samples_count = 5;
    float* _forecast_samples;
    int _minute_count = 0;
    float _pressure_avg;
    float _pressure_avg2;
    float _dP_dt;
    bool _first_round = true;
    float _getLastPressureSamplesAverage();
    const char* _forecast(float pressure);
};
#endif

/*
   SensorBME280
*/
#ifdef USE_BME280
class SensorBME280: public SensorBosch {
  public:
    SensorBME280(NodeManager& node_manager, int child_id = -255);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
  protected:
    Adafruit_BME280* _bm;
};
#endif

/*
   SensorBMP085
*/
#ifdef USE_BMP085_180
class SensorBMP085: public SensorBosch {
  public:
    SensorBMP085(NodeManager& node_manager, int child_id = -255);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
  protected:
    Adafruit_BMP085* _bm;
};

/*
   SensorBMP180
*/
class SensorBMP180: public SensorBMP085 {
  public:
    SensorBMP180(NodeManager& node_manager, int child_id = -255);
};
#endif

/*
   SensorBMP280
*/
#ifdef USE_BMP280
class SensorBMP280: public SensorBosch {
  public:
    SensorBMP280(NodeManager& node_manager, int child_id = -255);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
  protected:
    Adafruit_BMP280* _bm;
};
#endif

/*
   SensorSonoff
*/
#ifdef USE_SONOFF
class SensorSonoff: public Sensor {
  public:
    SensorSonoff(NodeManager& node_manager, int child_id = -255);
    // [101] set the button's pin (default: 0)
    void setButtonPin(int value);
    // [102] set the relay's pin (default: 12)
    void setRelayPin(int value);
    // [103] set the led's pin (default: 13)
    void setLedPin(int value);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
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
    void _toggle(Child* child);
};
#endif

/*
   SensorHCSR04
*/
#ifdef USE_HCSR04
class SensorHCSR04: public Sensor {
  public:
    SensorHCSR04(NodeManager& node_manager, int echo_pin, int trigger_pin, int child_id = -255);
    // [103] Maximum distance we want to ping for (in centimeters) (default: 300)
    void setMaxDistance(int value);
    // [104] Report the measure even if is invalid (e.g. 0) (default: true)
    void setReportIfInvalid(bool value);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
    void onProcess(Request & request);
  protected:
    int _trigger_pin;
    int _echo_pin;
    bool _report_if_invalid = true;
    int _max_distance = 300;
    NewPing* _sonar;
};
#endif

/*
   SensorMCP9808
*/
#ifdef USE_MCP9808
class SensorMCP9808: public Sensor {
  public:
    SensorMCP9808(NodeManager& node_manager, int child_id = -255);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    Adafruit_MCP9808* _mcp;
};
#endif

/*
    SensorMQ
 */
 #ifdef USE_MQ
class SensorMQ: public Sensor {
  public:
    SensorMQ(NodeManager& node_manager, int pin, int child_id = -255);
    // [102] set the load resistance on the board, in ohms (default: 1000);
    void setRlValue(float value);
    // [103] set the Ro resistance in ohms. By default will be calculated at startup during the calibration phase using the known ppm provided
    void setRoValue(float value);
    // [104] set the ppm used during the calibration (default: 411);
    void setKnownPpm(float value);
    // [105] define how many samples we are going to take in the calibration phase (default: 50);
    void setCalibrationSamples(int value);
    // [106] define the time (in milisecond) between each sample in the cablibration phase (default: 500);
    void setCalibrationSampleInterval(int value);
    // [107] define how many samples you are going to take in normal operation (default: 50);
    void setSamples(int value);
    // [108] define the time (in milisecond) between each sample in the normal operations (default: 5);
    void setSampleInterval(int value);
    // [109] set the ppm (x) of a random point on the gas curve (default: 200)
    void setPoint1Ppm(float value); 
    // [110] set the Rs/Ro ratio (y) of the same random point on the gas curve (default: 5)
    void setPoint1Ratio(float value);
    // [111] set the ppm (x) of another random point on the gas curve (default: 10000)
    void setPoint2Ppm(float value);
    // [112] set the Rs/Ro ratio (y) of the same random point on the gas curve (default: 1.2)
    void setPoint2Ratio(float value);
    // [113] with ppm = scaling_factor*x^exponent set the value manually, otherwise will be calculated automatically based on the two points provided
    void setCurveScalingFactor(float value); 
    // [114] with ppm = scaling_factor*x^exponent set the value manually, otherwise will be calculated automatically based on the two points provided
    void setCurveExponent(float value); 
    // do not report for the given number of minutes, waiting for the sensor to warm up (default: 0);
    void setWarmupMinutes(int value);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    long _rl = 1000;
    long _ro = 0;
    int _known_ppm = 411;
    int _calibration_samples = 50;
    int _calibration_sample_interval = 500;
    int _sample_interval = 50;
    int _samples = 5;
    unsigned long _warmup_minutes = 0;
    float _point1_ppm = 200;
    float _point1_ratio = 5;
    float _point2_ppm = 10000;
    float _point2_ratio = 1.2;
    float _curve_scaling_factor = 0;
    float _curve_exponent = 0;
    float _getRsValue(int samples, int sample_interval);
};
#endif

/*
   SensorMHZ19
*/
#ifdef USE_MHZ19
class SensorMHZ19: public Sensor {
  public:
    SensorMHZ19(NodeManager& node_manager, int rxpin, int txpin, int child_id = -255);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    int _readCO2();
    SoftwareSerial* _ser;
    int _tx_pin = 6;
    int _rx_pin = 7;
};
#endif

/*
   SensorAM2320
*/
#ifdef USE_AM2320
class SensorAM2320: public Sensor {
  public:
    SensorAM2320(NodeManager& node_manager, int child_id = -255);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    AM2320* _th;
};
#endif

/*
   SensorTSL2561
*/
#ifdef USE_TSL2561
class SensorTSL2561: public Sensor {
  public:
    SensorTSL2561(NodeManager& node_manager, int child_id = -255);
    // [101] set the gain, possible values are SensorTSL2561::GAIN_0X (0), SensorTSL2561::GAIN_16X (1) (default 16x)
    void setGain(int value);
    // [102] set the timing, possible values are SensorTSL2561::INTEGRATIONTIME_13MS (0), SensorTSL2561::INTEGRATIONTIME_101MS (1), SensorTSL2561::INTEGRATIONTIME_402MS (2) (default: 13ms)
    void setTiming(int value);
    // [103] set the spectrum, possible values are SensorTSL2561::VISIBLE (0), SensorTSL2561::FULLSPECTRUM (1), SensorTSL2561::INFRARED (2), SensorTSL2561::FULL (3) (default: visible)
    void setSpectrum(int value);
    // [104] set the i2c address values are SensorTSL2561::ADDR_FLOAT, SensorTSL2561::ADDR_LOW, SensorTSL2561::ADDR_HIGH
    void setAddress(int value);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
    void onProcess(Request & request);
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
#ifdef USE_PT100
class SensorPT100: public Sensor {
  public:
    SensorPT100(NodeManager& node_manager, int pin, int child_id = -255);
    // [101] set the voltageRef used to compare with analog measures
    void setVoltageRef(float value);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    DFRobotHighTemperature* _PT100;
    float _voltageRef = 3.3;
};
#endif

/*
    SensorPT100
*/
#ifdef USE_DIMMER
class SensorDimmer: public Sensor {
  public:
    SensorDimmer(NodeManager& node_manager, int pin, int child_id = -255);
    // [101] set the effect to use for a smooth transition, can be one of SensorDimmer::EASE_LINEAR, SensorDimmer::EASE_INSINE, SensorDimmer::EASE_OUTSINE, SensorDimmer::EASE_INOUTSINE (default: EASE_LINEAR)
    void setEasing(int value);
    // [102] the duration of entire the transition in seconds (default: 1)
    void setDuration(int value);
    // [103] the duration of a single step of the transition in milliseconds (default: 100)
    void setStepDuration(int value);
    // [104] reverse cathod and anode (default: false)
    void setReverse(bool value);
    // set the status of the dimmer
    void setStatus(int value);
    // set the percentage of the dimmer
    void setPercentage(int value);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    // fade the output from the current value to the target provided in the range 0-100
    void _fadeTo(Child* child, int value);
    enum _easing_list {
      EASE_LINEAR,
      EASE_INSINE,
      EASE_OUTSINE,
      EASE_INOUTSINE,
    };
    int _percentage = 100;
    int _status = OFF;
    int _easing = EASE_LINEAR;
    int _duration = 1000;
    int _step_duration = 100;
    int _reverse = false;
    float _getEasing(float t, float b, float c, float d);
};
#endif

/*
    SensorPca9685LED
*/
#if defined(USE_PCA9685W) || defined(USE_PCA9685RGB) || defined(USE_PCA9685RGBW)
class SensorPca9685Led {
  public:
    SensorPca9685Led(int channel = 0, uint8_t i2c_addr = 0x40, Adafruit_PWMServoDriver* pca9685 = NULL);
    void onSetup();
    
    void setEasing(int value);
    void setDuration(int value);
    void setStepDuration(int value);
    
    // getter/setter of color values of the dimmer
    void setVal(int value);
    void setValHex(String hexstring);
    void setValPercentage(int percentage);
    int getVal();
    int getValPercentage();
    String getValHex();

    //get fading delta for one step
    void faderInc();
    
    enum _easing_list {
      EASE_LINEAR = 0,
      EASE_INSINE = 1,
      EASE_OUTSINE = 2,
      EASE_INOUTSINE = 3
    };
  protected:
    // pca9685
    Adafruit_PWMServoDriver* _pca9685;
    bool _ownPca9685 = false;
    uint8_t _i2c_addr = 0xFF;
    int _pwm_ch = 0;
    int _target_color = 0;
    int _start_color = 0;
    int _cur_color = 0;
    unsigned long _start_time = 0;
    
    int _status = OFF;
    int _easing = EASE_LINEAR;
    int _duration = 1000;
    int _step_duration = 100;
    float _getEasing(float t, float b, float c, float d);
};
#endif

/*
    SensorPca9685W
*/
#ifdef USE_PCA9685W
class SensorPca9685W: public Sensor {
  public:
    SensorPca9685W(NodeManager& node_manager,  int child_id = -255, int channel = 0,uint8_t i2c_addr = 0x40, Adafruit_PWMServoDriver* pca9685 = NULL);
    // [101] set the effect to use for a smooth transition, can be one of SensorPca9685Rgbw::EASE_LINEAR (0), SensorPca9685Rgbw::EASE_INSINE (1), SensorPca9685Rgbw::EASE_OUTSINE (2), SensorPca9685Rgbw::EASE_INOUTSINE (3) (default: EASE_LINEAR)
    void setEasing(int value);
    // [102] the duration of entire the transition in milliseconds (default: 1000)
    void setDuration(int value);
    // [103] the duration of a single step of the transition in milliseconds (default: 100)
    void setStepDuration(int value);

    //get instance of PCA9685-board
    Adafruit_PWMServoDriver* getPWMServoDriver();
    //set instance of PCA9685-board, if using more than one pca9685-dimmer sensor on the same pca9685-board
    void setPWMServoDriver(Adafruit_PWMServoDriver* servoDriver);
    // set the W value as status -- STATUS: 0=0%; 1=100%
    void setStatus(bool mystatus);
    // get the W value as status -- STATUS: 0=0%; 1 .. if >0%
    bool getStatus();
    // set the W value in percentage
    void setWVal(int percentage);
    // get the W value in percentage
    int getWVal();
   
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    SensorPca9685Led* _pca9685w;
    int _channel = -1;
    int _i2c_addr = 0xFF;
    Adafruit_PWMServoDriver* _pca9685 = NULL;
    int _status = OFF;
    int _easing = SensorPca9685Led::EASE_LINEAR;
    int _duration = 1000;
    int _step_duration = 100;
};
#endif

/*
    SensorPca9685RGB
*/
#ifdef USE_PCA9685RGB
class SensorPca9685Rgb: public Sensor {
  public:
    SensorPca9685Rgb(NodeManager& node_manager,  int child_id = -255, int ch_r = 0,int ch_g = 0,int ch_b = 0,uint8_t i2c_addr = 0x40, Adafruit_PWMServoDriver* pca9685 = NULL);
    // [101] set the effect to use for a smooth transition, can be one of SensorPca9685Rgbw::EASE_LINEAR (0), SensorPca9685Rgbw::EASE_INSINE (1), SensorPca9685Rgbw::EASE_OUTSINE (2), SensorPca9685Rgbw::EASE_INOUTSINE (3) (default: EASE_LINEAR)
    void setEasing(int value);
    // [102] the duration of entire the transition in milliseconds (default: 1000)
    void setDuration(int value);
    // [103] the duration of a single step of the transition in milliseconds (default: 100)
    void setStepDuration(int value);

    //get instance of PCA9685-board
    Adafruit_PWMServoDriver* getPWMServoDriver();
    //set instance of PCA9685-board, if using more than one pca9685-dimmer sensor on the same pca9685-board
    void setPWMServoDriver(Adafruit_PWMServoDriver* servoDriver);
    //set the RGB (red/green/blue) value as hex-string (e.g. 000000...black/off, ff0000...red, 00ff00...green, 0000ff...blue, ffa500...orange, ffffff...white)
    void setRgbVal(String hexstring);
    //get the current RGB (red/green/blue) value as hex-string
    String getRgbVal();
    
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    SensorPca9685Led* _pca9685r;
    SensorPca9685Led* _pca9685g;
    SensorPca9685Led* _pca9685b;
    int _ch_r = -1;
    int _ch_g = -1;
    int _ch_b = -1;
    int _i2c_addr = 0xFF;
    Adafruit_PWMServoDriver* _pca9685 = NULL;
    int _status = OFF;
    int _easing = SensorPca9685Led::EASE_LINEAR;
    int _duration = 1000;
    int _step_duration = 100;
};
#endif

/*
    SensorPca9685RGBW
*/
#ifdef USE_PCA9685RGBW
class SensorPca9685Rgbw: public Sensor {
  public:
    SensorPca9685Rgbw(NodeManager& node_manager,  int child_id = -255, int ch_r = 0,int ch_g = 0,int ch_b = 0,int ch_w = 0, uint8_t i2c_addr = 0x40, Adafruit_PWMServoDriver* pca9685 = NULL);
    // [101] set the effect to use for a smooth transition, can be one of SensorPca9685Rgbw::EASE_LINEAR (0), SensorPca9685Rgbw::EASE_INSINE (1), SensorPca9685Rgbw::EASE_OUTSINE (2), SensorPca9685Rgbw::EASE_INOUTSINE (3) (default: EASE_LINEAR)
    void setEasing(int value);
    // [102] the duration of entire the transition in milliseconds (default: 1000)
    void setDuration(int value);
    // [103] the duration of a single step of the transition in milliseconds (default: 100)
    void setStepDuration(int value);

    //get instance of PCA9685-board
    Adafruit_PWMServoDriver* getPWMServoDriver();
    //set instance of PCA9685-board, if using more than one pca9685-dimmer sensor on the same pca9685-board
    void setPWMServoDriver(Adafruit_PWMServoDriver* servoDriver);
    //set the RGBW (red/green/blue/white) value as hex-string (see RGB-examples from PCA9685RGB and add "00".."ff" for the white-channel)
    void setRgbwVal(String hexstring);
    //get the current RGBW (red/green/blue/white) value as hex-string
    String getRgbwVal();
        
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    SensorPca9685Led* _pca9685r;
    SensorPca9685Led* _pca9685g;
    SensorPca9685Led* _pca9685b;
    SensorPca9685Led* _pca9685w;
    int _ch_r = -1;
    int _ch_g = -1;
    int _ch_b = -1;
    int _ch_w = -1;
    int _i2c_addr = 0xFF;
    Adafruit_PWMServoDriver* _pca9685 = NULL;
    int _status = OFF;
    int _easing = SensorPca9685Led::EASE_LINEAR;
    int _duration = 1000;
    int _step_duration = 100;
};
#endif


/*
    SensorPulseMeter
*/
#ifdef USE_PULSE_METER
class SensorPulseMeter: public Sensor {
  public:
    SensorPulseMeter(NodeManager& node_manager, int pin, int child_id = -255);
    // [102] set how many pulses for each unit (e.g. 1000 pulses for 1 kwh of power, 9 pulses for 1 mm of rain, etc.)
    void setPulseFactor(float value);
    // Set initial value on the interrupt pin. Can be used for internal pull up (default: HIGH)
    void setInitialValue(int value);
    // set the interrupt mode. Can be CHANGE, RISING, FALLING (default: FALLING)
    void setInterruptMode(int value);
    // milliseconds to wait/sleep after the interrupt before reporting (default: 0)
    void setWaitAfterTrigger(int value);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
    void onProcess(Request & request);
    void onInterrupt();
  protected:
    long _count = 0;
    float _pulse_factor;
    int _initial_value = HIGH;
    int _interrupt_mode = FALLING;
    int _wait_after_trigger = 0;
    virtual void _reportTotal(Child* child);
};

/*
    SensorRainGauge
*/
class SensorRainGauge: public SensorPulseMeter {
  public:
    SensorRainGauge(NodeManager& node_manager, int pin, int child_id = -255);
};

/*
    SensorPowerMeter
*/
class SensorPowerMeter: public SensorPulseMeter {
  public:
    SensorPowerMeter(NodeManager& node_manager, int pin, int child_id = -255);
  protected:
    void _reportTotal(Child* child);
};

/*
    SensorWaterMeter
*/
class SensorWaterMeter: public SensorPulseMeter {
  public:
    SensorWaterMeter(NodeManager& node_manager, int pin, int child_id = -255);
  protected:
    void _reportTotal(Child* child);
};
#endif

/*
   SensorPlantowerPMS
*/
#ifdef USE_PMS
class SensorPlantowerPMS: public Sensor {
  public:
    SensorPlantowerPMS(NodeManager& node_manager, int rxpin, int txpin, int child_id = -255);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    int _readSensorValues();
    SoftwareSerial* _ser;
    int _tx_pin = 4;
    int _rx_pin = 3;
    PMS *_pms;
    PMS::DATA _data;
    bool _valuesRead = false;
    bool _valuesReadError = false;
};
#endif

/*
 * VL53L0X Laser distance sensor
 */
#ifdef USE_VL53L0X
class SensorVL53L0X: public Sensor {
  public:
    SensorVL53L0X(NodeManager& node_manager, int xshut_pin, int child_id = -255);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    int _getDistance();
    VL53L0X *_lox;
};
#endif

/*
 * Display class
 */
#if defined(USE_SSD1306) || defined(USE_HD44780)
class Display: public Sensor {
  public:
    Display(NodeManager& node_manager, int child_id = -255);
    // set a static caption text on header of the display
    void setCaption(const char* value);
    // print a static caption on top of the screen
    virtual void printCaption(const char* value);
    // print the given string
    virtual void print(const char* value);
    // print the given string and goes to the next line
    virtual void println(const char* value);
    // print the value of the given child
    virtual void printChild(Child* child);
    // clear the display
    virtual void clear();
    // set the cursor to the given position
    virtual void setCursor(int col,int row);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
  const char* _caption = "";
};
#endif

/*
 * SSD1306 OLED display
 */
#ifdef USE_SSD1306
class DisplaySSD1306: public Display {
  public:
    DisplaySSD1306(NodeManager& node_manager, int child_id = -255);
    // set device
    void setDev(const DevType* dev);
    // set i2c address
    void setI2CAddress(uint8_t i2caddress);
    // set text font (default: &Adafruit5x7)
    void setFont(const uint8_t* font);
    // [102] set the contrast of the display (0-255)
    void setContrast(uint8_t value);
    // [104] Rotate the display 180 degree (use rotate=false to revert)
    void rotateDisplay(bool rotate = true);
    // [105] Text font size (possible are 1 and 2; default is 1)
    void setFontSize(int fontsize);
    // [106] Text caption font size (possible are 1 and 2; default is 2)
    void setCaptionFontSize(int fontsize);
    // [107] Invert display (black text on color background; use invert=false to revert)
    void invertDisplay(bool invert = true);
    // display specific functions
    void printCaption(const char* value);
    void print(const char* value);
    void println(const char* value);
    void printChild(Child* child);
    void clear();
    void setCursor(int col,int row);
    // define what to do at each stage of the sketch
    void onSetup();
  protected:
    SSD1306AsciiAvrI2c *_oled;
    const DevType* _dev = &Adafruit128x64;
    uint8_t _i2caddress = 0x3c;
    int _fontsize = 1;
    int _caption_fontsize = 2;
    const uint8_t* _font = Adafruit5x7;
    uint8_t _contrast = -1;
};
#endif

/*
   SensorSHT31: temperature and humidity sensor
*/
#ifdef USE_SHT31
class SensorSHT31: public Sensor {
  public:
    SensorSHT31(NodeManager& node_manager, int child_id = -255);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    Adafruit_SHT31* _sht31;
};
#endif

/*
   SensorSI7021: temperature and humidity sensor
*/
#ifdef USE_SI7021
class SensorSI7021: public Sensor {
  public:
    SensorSI7021(NodeManager& node_manager, int child_id = -255);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    Weather* _si7021;
};
#endif

/*
   SensorChirp: Chirp soil moisture sensor (includes temperature and light sensors)  
*/
#ifdef USE_CHIRP
class SensorChirp: public Sensor {
  public:
    SensorChirp(NodeManager& node_manager, int child_id = -255);
    // [101] set the soil moisture offset (default: 0)
    void setMoistureOffset(int value);
    // [102] set the soil moisture range (default: 0)
    void setMoistureRange(int value);
    // [103] return the soil moisture normalized (default: false)
    void setReturnMoistureNormalized(bool value);
    // [104] reverse the light value (default: true)
    void setReturnLightReversed(bool value); 
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
  I2CSoilMoistureSensor* _chirp;
  int _chirp_moistureoffset = 0;
  int _chirp_moisturerange = 0;
  bool _chirp_moisturenormalized = false;  
  bool _chirp_lightreversed = true;
};
#endif

/*
 * Hitachi HD44780 display
 */
#ifdef USE_HD44780
class DisplayHD44780: public Display {
  public:
    DisplayHD44780(NodeManager& node_manager, int child_id = -255);
    // set i2c address (default: 0x38)
    void setI2CAddress(uint8_t i2caddress);
    // set the backlight (default: HIGH)
    void setBacklight(uint8_t value);
    // display specific functions
    void printCaption(const char* value);
    void print(const char* value);
    void println(const char* value);
    void printChild(Child* child);
    void clear();
    void setCursor(int col,int row);
    // define what to do at each stage of the sketch
    void onSetup();
  protected:
    LiquidCrystal_I2C* _lcd;
    uint8_t _i2caddress = 0x38;
    int _column = 0;
};
#endif

/*
   SensorTTP: TTP226/TTP229 Touch control sensor
*/
#ifdef USE_TTP
class SensorTTP: public Sensor {
  public:
    SensorTTP(NodeManager& node_manager, int child_id = -255);
    // set the passcode length. Passcode will be sent to the controller only after this number of digits have been pressed (default: 4)
    void setPasscodeLength(int value);
    // set the clock pin (default: 6)
    void setClockPin(int value);
    // set the SDO pin (default: 5)
    void setSdoPin(int value);
    // set the DV pin (default: 3)
    void setDvPin(int value);
    // set the RST pin (default: 4)
    void setRstPin(int value);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onInterrupt();
    void onReceive(MyMessage* message);
  protected:
    int _clock_pin = 6;
    int _sdo_pin = 5;
    int _dv_pin = 3;
    int _rst_pin = 4;
    int _fetchData();
    List<int> _passcode;
    int _passcode_length = 4;
};
#endif

/*
 * Servo motor sensor
 */
#ifdef USE_SERVO
class SensorServo: public Sensor {
  public:
    SensorServo(NodeManager& node_manager, int pin, int child_id = -255);
    // set the servo to the given percentage
    void setPercentage(int value);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
    Servo _servo;
    int _value;
};
#endif

/*
 * SparkFun RGB and Gesture Sensor
 */
#ifdef USE_APDS9960
class SensorAPDS9960: public Sensor {
  public:
    SensorAPDS9960(NodeManager& node_manager, int pin, int child_id = -255);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onInterrupt();
  protected:
  SparkFun_APDS9960* _apds;
};
#endif

/*
 * Neopixel LED Sensor
 */
#ifdef USE_NEOPIXEL
class SensorNeopixel: public Sensor {
  public:
    SensorNeopixel(NodeManager& node_manager, int pin, int child_id = -255);
    // set how many NeoPixels are attached
    void setNumPixels(int value);
    // format expeted is "<pixel_number>,<RGB color in a packed 32 bit format>"
    void setColor(char* string);
    // define what to do at each stage of the sketch
    void onSetup();
    void onLoop(Child* child);
    void onReceive(MyMessage* message);
  protected:
#if defined(ARDUINO_ARCH_STM32F0) || defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F3) || defined(ARDUINO_ARCH_STM32F4) || defined(ARDUINO_ARCH_STM32L4)
  NeoMaple* _pixels;
#else  
  Adafruit_NeoPixel* _pixels;
#endif
  int _num_pixels = 16;
};
#endif

/*
  SensorSDS011
*/
#ifdef USE_SDS011
 class SensorSDS011: public Sensor {
 public:
   SensorSDS011(NodeManager& node_manager, int rxpin, int txpin, int child_id = -255);
   // Sleep sensor during measurment aka stop fan.
   void setSleep(bool value);
   // define what to do at each stage of the sketch
   void onSetup();
   void onLoop(Child* child);
   void onReceive(MyMessage* message);
 protected:
   SDS011* _sds;
   float _p10 = 0.;
   float _p25 = 0.;
   int _rx_pin = 6;
   int _tx_pin = 7;
   bool _slp = true;
 };
#endif

/*
  SensorFPM10A
*/
#ifdef USE_FPM10A
 class SensorFPM10A: public Sensor {
 public:
   SensorFPM10A(NodeManager& node_manager, int rxpin, int txpin, int child_id = -255);
   // set the baud rate of the serial port for connecting to the sensor (default: 57600)
   void setBaudRate(uint32_t value);
   // set the password for connecting to the sensor (default: 0)
   void setPassword(uint32_t value);
   // [101] set the minimum confidence below which the match is not considered valid (default: 0)
   void setMinConfidence(uint16_t value);
   // [102] wait for a valid fingerprint for the given amount of seconds. Useful when battery powered (default: 0)
   void setWaitFingerForSeconds(int value);
   // define what to do at each stage of the sketch
   void onSetup();
   void onLoop(Child* child);
 protected:
   Adafruit_Fingerprint* _finger;
   SoftwareSerial* _serial;
   int _rx_pin;
   int _tx_pin;
   Timer* _timer;
   uint32_t _baud_rate = 57600;
   uint32_t _password = 0;
   uint16_t _min_confidence = 0;
   int _readFingerprint();
 };
#endif

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
    int _status = AWAKE;
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
    static long _interrupt_debounce;
    static long _last_interrupt_1;
    static long _last_interrupt_2;
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

#endif
