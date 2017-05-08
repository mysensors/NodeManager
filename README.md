# Introduction

NodeManager is intended to take care on your behalf of all those common tasks a MySensors node has to accomplish, speeding up the development cycle of your projects.

NodeManager includes the following main components:

* Sleep manager: allows managing automatically the complexity behind battery-powered sensors spending most of their time sleeping
* Power manager: allows powering on your sensors only while the node is awake
* Battery manager: provides common functionalities to read and report the battery level
* Remote configuration: allows configuring remotely the node without the need to have physical access to it
* Built-in sensors: for the most common sensors, provide embedded code so to allow their configuration with a single line

## Features

* Manage all the aspects of a sleeping cycle by leveraging smart sleep
* Allow configuring the sleep mode and the sleep duration remotely
* Allow waking up a sleeping node remotely at the end of a sleeping cycle
* Allow powering on each connected sensor only while the node is awake to save battery
* Report battery level periodically and automatically
* Calculate battery level without requiring an additional pin and the resistors
* Report battery voltage through a built-in sensor
* Can report battery level on demand
* Allow rebooting the board remotely
* Provide out-of-the-box sensors personalities and automatically execute their main task at each cycle

# Installation
* Download the package or clone the git repository at https://github.com/mysensors/NodeManager
* Open the provided sketch template and save it under a different name
* Open `config.h` and customize both MySensors configuration and NodeManager global settings
* Register your sensors in the sketch file
* Upload the sketch to your arduino board

Please note NodeManager cannot be used as an arduino library since requires access to your MySensors configuration directives, hence its files have to be placed into the same directory of your sketch.

## Upgrade
* Download the package
* Replace the NodeManager.cpp and NodeManager.h of your project with those just downloaded
* Review the release notes in case there is any manual change required to the existing sketch or config.h file

# Configuration
NodeManager configuration includes compile-time configuration directives (which can be set in config.h), runtime global and per-sensor configuration settings (which can be set in your sketch) and settings that can be customized remotely (via a special child id).

## Setup MySensors
Since NodeManager has to communicate with the MySensors gateway on your behalf, it has to know how to do it. Place on top of the `config.h` file all the MySensors typical directives you are used to set on top of your sketch so both your sketch AND NodeManager will be able to share the same configuration. For example:
~~~c
/**********************************
 * Sketch configuration
 */

#define SKETCH_NAME "NodeManagerTemplate"
#define SKETCH_VERSION "1.0"

/**********************************
 * MySensors node configuration
 */

// General settings
#define MY_BAUD_RATE 9600
//#define MY_DEBUG
//#define MY_NODE_ID 100

// NRF24 radio settings
#define MY_RADIO_NRF24
//#define MY_RF24_ENABLE_ENCRYPTION
//#define MY_RF24_CHANNEL 76
//#define MY_RF24_PA_LEVEL RF24_PA_HIGH

// RFM69 radio settings
//#define MY_RADIO_RFM69
//#define MY_RFM69_FREQUENCY RF69_868MHZ
//#define MY_IS_RFM69HW
//#define MY_RFM69_NEW_DRIVER
//#define MY_RFM69_ENABLE_ENCRYPTION
//#define MY_RFM69_NETWORKID 100
//#define MY_RF69_IRQ_PIN D1
//#define MY_RF69_IRQ_NUM MY_RF69_IRQ_PIN
//#define MY_RF69_SPI_CS D2

/**********************************
 * MySensors gateway configuration
 */
// Common gateway settings
//#define MY_REPEATER_FEATURE

// Serial gateway settings
//#define MY_GATEWAY_SERIAL

// Ethernet gateway settings
//#define MY_GATEWAY_W5100

// ESP8266 gateway settings
//#define MY_GATEWAY_ESP8266
//#define MY_ESP8266_SSID "MySSID"
//#define MY_ESP8266_PASSWORD "MyVerySecretPassword"

// Gateway networking settings
//#define MY_IP_ADDRESS 192,168,178,87
//#define MY_IP_GATEWAY_ADDRESS 192,168,178,1
//#define MY_IP_SUBNET_ADDRESS 255,255,255,0
//#define MY_PORT 5003
//#define MY_GATEWAY_MAX_CLIENTS 2
//#define MY_USE_UDP

// Gateway MQTT settings
//#define MY_GATEWAY_MQTT_CLIENT
//#define MY_CONTROLLER_IP_ADDRESS 192, 168, 178, 68
//#define MY_PORT 1883
//#define MY_MQTT_USER "username"
//#define MY_MQTT_PASSWORD "password"
//#define MY_MQTT_CLIENT_ID "mysensors-1"
//#define MY_MQTT_PUBLISH_TOPIC_PREFIX "mygateway1-out"
//#define MY_MQTT_SUBSCRIBE_TOPIC_PREFIX "mygateway1-in"

// Gateway inclusion mode
//#define MY_INCLUSION_MODE_FEATURE
//#define MY_INCLUSION_BUTTON_FEATURE
//#define MY_INCLUSION_MODE_DURATION 60
//#define MY_DEFAULT_LED_BLINK_PERIOD 300

// Gateway Leds settings
//#define MY_DEFAULT_ERR_LED_PIN 4
//#define MY_DEFAULT_RX_LED_PIN  5
//#define MY_DEFAULT_TX_LED_PIN  6
~~~

## Enable/Disable NodeManager's modules

Those NodeManager's directives in the `config.h` file control which module/library/functionality will be made available to your sketch. Enable (e.g. set to 1) only what you need to ensure enough space is left to your custom code.

~~~c
// if enabled, enable the capability to power on sensors with the arduino's pins to save battery while sleeping
#define POWER_MANAGER 1
// if enabled, will load the battery manager library to allow the battery level to be reported automatically or on demand
#define BATTERY_MANAGER 1
// if enabled, allow modifying the configuration remotely by interacting with the configuration child id
#define REMOTE_CONFIGURATION 1
// if enabled, persist the configuration settings on EEPROM
#define PERSIST 0

// if enabled, enable debug messages on serial port
#define DEBUG 1

// if enabled, send a SLEEPING and AWAKE service messages just before entering and just after leaving a sleep cycle and STARTED when starting/rebooting
#define SERVICE_MESSAGES 1
// if enabled, a battery sensor will be created at BATTERY_CHILD_ID and will report vcc voltage together with the battery level percentage
#define BATTERY_SENSOR 1

// Enable this module to use one of the following sensors: SENSOR_ANALOG_INPUT, SENSOR_LDR, SENSOR_THERMISTOR, SENSOR_MQ, SENSOR_ML8511, SENSOR_ACS712
#define MODULE_ANALOG_INPUT 1
// Enable this module to use one of the following sensors: SENSOR_DIGITAL_INPUT
#define MODULE_DIGITAL_INPUT 1
// Enable this module to use one of the following sensors: SENSOR_DIGITAL_OUTPUT, SENSOR_RELAY, SENSOR_LATCHING_RELAY
#define MODULE_DIGITAL_OUTPUT 1
// Enable this module to use one of the following sensors: SENSOR_SHT21
#define MODULE_SHT21 0
// Enable this module to use one of the following sensors: SENSOR_DHT11, SENSOR_DHT22
#define MODULE_DHT 0
// Enable this module to use one of the following sensors: SENSOR_SWITCH, SENSOR_DOOR, SENSOR_MOTION
#define MODULE_SWITCH 0
// Enable this module to use one of the following sensors: SENSOR_DS18B20
#define MODULE_DS18B20 0
// Enable this module to use one of the following sensors: SENSOR_BH1750
#define MODULE_BH1750 0
// Enable this module to use one of the following sensors: SENSOR_MLX90614
#define MODULE_MLX90614 0
// Enable this module to use one of the following sensors: SENSOR_BME280
#define MODULE_BME280 0
// Enable this module to use one of the following sensors: SENSOR_SONOFF
#define MODULE_SONOFF 0
// Enable this module to use one of the following sensors: SENSOR_BMP085
#define MODULE_BMP085 0
// Enable this module to use one of the following sensors: SENSOR_HCSR04
#define MODULE_HCSR04 0
~~~

## Installing the dependencies

Some of the modules above rely on third party libraries. Those libraries are not included within NodeManager and have to be installed from the Arduino IDE Library Manager (Sketch -> Include Library -> Manager Libraries). You need to install the library ONLY if the module is enabled:

Module  | Required Library
 ------------- | -------------
MODULE_SHT21 | https://github.com/SodaqMoja/Sodaq_SHT2x
MODULE_DHT | https://github.com/adafruit/DHT-sensor-library
MODULE_DS18B20 | https://github.com/milesburton/Arduino-Temperature-Control-Library
MODULE_BH1750 | https://github.com/claws/BH1750
MODULE_MLX90614 | https://github.com/adafruit/Adafruit-MLX90614-Library
MODULE_BME280 | https://github.com/adafruit/Adafruit_BME280_Library
MODULE_SONOFF | https://github.com/thomasfredericks/Bounce2
MODULE_BMP085 | https://github.com/adafruit/Adafruit-BMP085-Library
MODULE_HCSR04 | https://github.com/mysensors/MySensorsArduinoExamples/tree/master/libraries/NewPing

## Configure NodeManager

Node Manager comes with a reasonable default configuration. If you want/need to change its settings, this can be done in your sketch, inside the `before()` function and just before registering your sensors. The following methods are exposed for your convenience:

~~~c
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
    // define the way the node should behave. It can be IDLE (stay awake withtout executing each sensors' loop), SLEEP (go to sleep for the configured interval), WAIT (wait for the configured interval), ALWAYS_ON (stay awake and execute each sensors' loop)
    void setSleepMode(int value);
    void setMode(int value);
    // define for how long the board will sleep (default: 0)
    void setSleepTime(int value);
    // define the unit of SLEEP_TIME. It can be SECONDS, MINUTES, HOURS or DAYS (default: MINUTES)
    void setSleepUnit(int value);
    // configure the node's behavior, parameters are mode, time and unit
    void setSleep(int value1, int value2, int value3);
    // if enabled, when waking up from the interrupt, the board stops sleeping. Disable it when attaching e.g. a motion sensor (default: true)
    void setSleepInterruptPin(int value);
    #endif
    // configure the interrupt pin and mode. Mode can be CHANGE, RISING, FALLING (default: MODE_NOT_DEFINED)
    void setInterrupt(int pin, int mode, int pull = -1);
    // optionally sleep interval in milliseconds before sending each message to the radio network (default: 0)
    void setSleepBetweenSend(int value);
    // set the interrupt pin the sensor is attached to so its loop() will be executed only upon that interrupt (default: -1)
    void setInterruptPin(int value);
    // register a built-in sensor
    int registerSensor(int sensor_type, int pin = -1, int child_id = -1);
    // un-register a sensor
    void unRegisterSensor(int sensor_index);
    // register a custom sensor
    int registerSensor(Sensor* sensor);
    // return a sensor by its index
    Sensor* get(int sensor_index);
	Sensor* getSensor(int sensor_index);
	// assign a different child id to a sensor
    bool renameSensor(int old_child_id, int new_child_id);
    #if POWER_MANAGER == 1
      // to save battery the sensor can be optionally connected to two pins which will act as vcc and ground and activated on demand
      void setPowerPins(int ground_pin, int vcc_pin, long wait = 0);
      // if enabled the pins will be automatically powered on while awake and off during sleeping (default: true)
      void setAutoPowerPins(bool value);
      // manually turn the power on
      void powerOn();
      // manually turn the power off
      void powerOff();
    #endif
    // set this to true if you want destination node to send ack back to this node (default: false)
    void setAck(bool value);
    // request and return the current timestamp from the controller
    long getTimestamp();
~~~

For example

~~~c
	nodeManager.setBatteryMin(1.8);
~~~

## Register your sensors
In your sketch, inside the `before()` function and just before calling `nodeManager.before()`, you can register your sensors against NodeManager. The following built-in sensor types are available:

Sensor type  | Description
 ------------- | -------------
SENSOR_ANALOG_INPUT | Generic analog sensor, return a pin's analog value or its percentage
SENSOR_LDR | LDR sensor, return the light level of an attached light resistor in percentage
SENSOR_THERMISTOR | Thermistor sensor, return the temperature based on the attached thermistor
SENSOR_DIGITAL_INPUT |  Generic digital sensor, return a pin's digital value
SENSOR_DIGITAL_OUTPUT | Generic digital output sensor, allows setting the digital output of a pin to the requested value
SENSOR_RELAY | Relay sensor, allows activating the relay
SENSOR_LATCHING_RELAY| Latching Relay sensor, allows activating the relay with a pulse
SENSOR_DHT11 | DHT11 sensor, return temperature/humidity based on the attached DHT sensor
SENSOR_DHT22 | DHT22 sensor, return temperature/humidity based on the attached DHT sensor
SENSOR_SHT21 | SHT21 sensor, return temperature/humidity based on the attached SHT21 sensor
SENSOR_SWITCH | Generic switch, wake up the board when a pin changes status
SENSOR_DOOR | Door sensor, wake up the board and report when an attached magnetic sensor has been opened/closed
SENSOR_MOTION | Motion sensor, wake up the board and report when an attached PIR has triggered
SENSOR_DS18B20 | DS18B20 sensor, return the temperature based on the attached sensor
SENSOR_HTU21D | HTU21D sensor, return temperature/humidity based on the attached HTU21D sensor
SENSOR_BH1750 | BH1750 sensor, return light level in lux
SENSOR_MLX90614 | MLX90614 contactless temperature sensor, return ambient and object temperature
SENSOR_BME280 | BME280 sensor, return temperature/humidity/pressure based on the attached BME280 sensor
SENSOR_MQ | MQ sensor, return ppm of the target gas
SENSOR_ML8511 | ML8511 sensor, return UV intensity
SENSOR_SONOFF | Sonoff wireless smart switch
SENSOR_BMP085 | BMP085/BMP180 sensor, return temperature and pressure
SENSOR_HCSR04 | HC-SR04 sensor, return the distance between the sensor and an object
SENSOR_ACS712 | ACS712 sensor, measure the current going through the attached module

To register a sensor simply call the NodeManager instance with the sensory type and the pin the sensor is conncted to. For example:
~~~c
	nodeManager.registerSensor(SENSOR_THERMISTOR,A2);
	nodeManager.registerSensor(SENSOR_DOOR,3);
~~~

Once registered, your job is done. NodeManager will assign a child id automatically, present each sensor for you to the controller, query each sensor and report the value back to the gateway/controller at at the end of each sleep cycle. An optional child id can be provided as a third argument if you want to assign it manually. For actuators (e.g. relays) those can be triggered by sending a `REQ` message to their assigned child id.

When called, registerSensor returns the child_id of the sensor so you will be able to retrieve it later if needed. If you want to set a child_id manually, this can be passed as third argument to the function.

### Creating a custom sensor

If you want to create a custom sensor and register it with NodeManager so it can take care of all the common tasks, you can create a class inheriting from `Sensor` and implement the following methods:
~~~c
    // define what to do during before() to setup the sensor
    void onBefore();
	// define what to do during setup() by executing the sensor's main task
    void onSetup();
    // define what to do during loop() by executing the sensor's main task
    void onLoop();
    // define what to do during receive() when the sensor receives a message
    void onReceive(const MyMessage & message);
~~~

You can then instantiate your newly created class and register with NodeManager:
~~~c
	nodeManager.registerSensor(new SensorCustom(child_id, pin));
~~~

## Configuring the sensors
Each built-in sensor class comes with reasonable default settings. In case you want/need to customize any of those settings, after having registered the sensor, you can retrieve it back and call set functions common to all the sensors or specific for a given class.

To do so, use `nodeManager.getSensor(child_id)` which will return a pointer to the sensor. Remeber to cast it to the right class before calling their functions. For example:

~~~c
	((SensorLatchingRelay*)nodeManager.getSensor(2))->setPulseWidth(50);
~~~


### Sensor's general configuration

The following methods are available for all the sensors:
~~~c
    // where the sensor is attached to (default: not set)
    void setPin(int value);
    // child_id of this sensor (default: not set)
    void setChildId(int value);
    // presentation of this sensor (default: S_CUSTOM)
    void setPresentation(int value);
    // type of this sensor (default: V_CUSTOM)
    void setType(int value);
    // description of the sensor (default: '')
    void setDescription(char *value);
    // set this to true if you want destination node to send ack back to this node (default: false)
    void setAck(bool value);
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
      void setPowerPins(int ground_pin, int vcc_pin, long wait = 0);
      // if enabled the pins will be automatically powered on while awake and off during sleeping (default: true)
      void setAutoPowerPins(bool value);
      // manually turn the power on
      void powerOn();
      // manually turn the power off
      void powerOff();
    #endif
~~~

### Sensor's specific configuration

Each sensor class can expose additional methods.

#### SensorAnalogInput / SensorLDR
~~~c
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
~~~

#### SensorThermistor
~~~c
    // resistance at 25 degrees C (default: 10000)
    void setNominalResistor(long value);
    // temperature for nominal resistance (default: 25)
    void setNominalTemperature(int value);
    // The beta coefficient of the thermistor (default: 3950)
    void setBCoefficient(int value);
    // the value of the resistor in series with the thermistor (default: 10000)
    void setSeriesResistor(long value);
    // set a temperature offset
    void setOffset(float value);
~~~

#### SensorMQ
~~~c
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
~~~

#### SensorDigitalOutput / SensorRelay / SensorLatchingRelay
~~~c
    // set how to initialize the output (default: LOW)
    void setInitialValue(int value);
    // if greater than 0, send a pulse of the given duration in ms and then restore the output back to the original value (default: 0)
    void setPulseWidth(int value);
    // define which value to set to the output when set to on (default: HIGH)
    void setOnValue(int value);
~~~

#### SensorSwitch / SensorDoor / SensorMotion
~~~c
    // set the interrupt mode. Can be CHANGE, RISING, FALLING (default: CHANGE)
    void setMode(int value);
    // milliseconds to wait before reading the input (default: 0)
    void setDebounce(int value);
    // time to wait in milliseconds after a change is detected to allow the signal to be restored to its normal value (default: 0)
    void setTriggerTime(int value);
    // Set initial value on the interrupt pin (default: HIGH)
    void setInitial(int value);
~~~

#### SensorDs18b20
~~~c
    // return the sensors' device address
    DeviceAddress* getDeviceAddress();
    // returns the sensor's resolution in bits
    int getResolution();
    // set the sensor's resolution in bits
    void setResolution(int value);
    // sleep while DS18B20 calculates temperature (default: false)
    void setSleepDuringConversion(bool value);
~~~

#### SensorBME280
~~~c
    // define how many pressure samples to keep track of for calculating the forecast (default: 5)
    void setForecastSamplesCount(int value);
~~~

#### SensorSonoff
~~~c
    // set the button's pin (default: 0)
    void setButtonPin(int value);
    // set the relay's pin (default: 12)
    void setRelayPin(int value);
    // set the led's pin (default: 13)
    void setLedPin(int value);
~~~

#### SensorBMP085
~~~c
    // define how many pressure samples to keep track of for calculating the forecast (default: 5)
    void setForecastSamplesCount(int value);
~~~

#### SensorHCSR04
~~~c
    // Arduino pin tied to trigger pin on the ultrasonic sensor (default: the pin set while registering the sensor)
    void setTriggerPin(int value);
    // Arduino pin tied to echo pin on the ultrasonic sensor (default: the pin set while registering the sensor)
    void setEchoPin(int value);
    // Maximum distance we want to ping for (in centimeters) (default: 300)
    void setMaxDistance(int value);
~~~

#### SensorACS712
~~~c
    // set how many mV are equivalent to 1 Amp. The value depends on the module (100 for 20A Module, 66 for 30A Module) (default: 185);
    void setmVPerAmp(int value);
    // set ACS offset (default: 2500);
    void setOffset(int value);
~~~

## Upload your sketch

Upload your sketch to your arduino board as you are used to.

## Verify if everything works fine

Check your gateway's logs to ensure the node is working as expected. You should see the node presenting itself, reporting battery level, presenting all the registered sensors and the configuration child id service.
When `DEBUG` is enabled, detailed information is available through the serial port. Remember to disable debug once the tests have been completed.

## Communicate with each sensor

You can interact with each registered sensor asking to execute their main tasks by sending to the child id a `REQ` command. For example to request the temperature to node_id 254 and child_id 1:

`254;1;2;0;0;`

To activate a relay connected to the same node, child_id 100:

`254;100;2;0;2;1`

No need to implement anything on your side since for built-in sensor types this is handled automatically. 
Once the node will be sleeping, it will report automatically each measure at the end of every sleep cycle.

## Communicate with the node

NodeManager exposes a configuration service by default on child_id 200 so you can interact with it by sending `V_CUSTOM` type of messages and commands within the payload. For each `REQ` message, the node will respond with a `SET` message.
The following custom commands are available:

NodeManager command  | Description
 ------------- | -------------
BATTERY | Report the battery level back to the gateway/controller
HELLO | Hello request
REBOOT | Reboot the board
CLEAR | Wipe from the EEPROM NodeManager's settings
VERSION | Respond with NodeManager's version
IDxxx |  Change the node id to the provided one. E.g. ID025: change the node id to 25. Requires a reboot to take effect
INTVLnnnX | Set the wait/sleep interval to nnn where X is S=Seconds, M=mins, H=Hours, D=Days. E.g. INTVL010M would be 10 minutes
MODEx | change the way the node behaves. 0: stay awake withtout executing each sensors' loop(), 1: go to sleep for the configured interval, 2: wait for the configured interval, 3: stay awake and execute each sensors' loop()
AWAKE | When received after a sleeping cycle or during wait, abort the cycle and stay awake

For example, to request the battery level to node id 254:

`254;200;2;0;48;BATTERY`

To set the sleeping cycle to 1 hour:

`254;200;2;0;48;INTVL001H`

To ask the node to start sleeping (and waking up based on the previously configured interval):

`254;200;2;0;48;MODE1`

To wake up a node previously configured with `MODE1`, send the following just after reporting `AWAKE`:

`254;200;2;0;48;WAKEUP`

In addition, NodeManager will report with custom messages every time the board is going to sleep (`SLEEPING`) or it is awake (`AWAKE`).

If `PERSIST` is enabled, the settings provided with `INTVLnnnX` and `MODEx` are saved to the EEPROM to be persistent even after rebooting the board.

# How it works

A NodeManager object must be created and called from within your sketch during `before()`, `presentation()`, `loop()` and `receive()` to work properly. NodeManager will do the following during each phase:

## NodeManager::before()
* Configure the reboot pin so to allow rebooting the board
* Setup the interrupt pins to wake up the board based on the configured interrupts (e.g. stop sleeping when the pin is connected to ground or wake up and notify when a motion sensor has trigger)
* If persistance is enabled, restore from the EEPROM the latest sleeping settings
* Call `before()` of each registered sensor

### Sensor::before()
* Call sensor-specific implementation of before by invoking `onBefore()` to initialize the sensor

## NodeManager::setup()
* Send a custom message with a STARTED payload to the controller
* Call `setup()` of each registered sensor

### Sensor::setup()
* Call sensor-specific implementation of setup by invoking `onSetup()` to initialize the sensor

## NodeManager::loop()
* If all the sensors are powered by an arduino pin, this is set to HIGH
* Call `loop()` of each registered sensor
* If all the sensors are powered by an arduino pin, this is set to LOW

### Sensor::loop()
* If the sensor is powered by an arduino pin, this is set to HIGH
* For each registered sensor, the sensor-specific `onLoop()` is called. If multiple samples are requested, this is run multiple times.
* In case multiple samples have been collected, the average is calculated
* A message is sent to the gateway with the calculated value. Depending on the configuration, this is not sent if it is the same as the previous value or sent anyway after a given number of cycles. These functionalies are not sensor-specific and common to all the sensors inheriting from the `Sensor` class.
* If the sensor is powered by an arduino pin, this is set to LOW

## NodeManager::receive()
* Receive a message from the radio network 
* If the destination child id is the configuration node, it will handle the incoming message, otherwise will dispatch the message to the recipient sensor

### Sensor::receive()
* Invoke `Sensor::loop()` which will execute the sensor main taks and eventually call `Sensor::onReceive()`

# Examples
All the examples below takes place within the before() function in the main sketch, just below the "Register below your sensors" comment.

Enable reboot pin, connect pin 4 to RST to enable rebooting the board with the REBOOT message:

~~~c
  nodeManager.setRebootPin(4);
~~~

Set battery minimum and maxium voltage. This will be used to calculate the level percentage:

~~~c
    nodeManager.setBatteryMin(1.8);
    nodeManager.setBatteryMin(3.2);
~~~

Instruct the board to sleep for 10 minutes at each cycle:

~~~c
    nodeManager.setSleep(SLEEP,10,MINUTES);
~~~

Configure a wake up pin. When pin 3 is connected to ground, the board will stop sleeping:

~~~c
    nodeManager.setSleepInterruptPin(3);
~~~

Use the arduino pins to power on and off the attached sensors. All the sensors' vcc and ground are connected to pin 6 (ground) and 7 (vcc). NodeManager will enable the vcc pin every time just before loop() and wait for 100ms for the power to settle before running loop() of each sensor:

~~~c
   nodeManager.setPowerPins(6,7,100);
~~~

Register a thermistor sensor attached to pin A2. NodeManager will then send the temperature to the controller at the end of each sleeping cycle:

~~~c
   nodeManager.registerSensor(SENSOR_THERMISTOR,A2);
~~~

Register a SHT21 temperature/humidity sensor; since using i2c for communicating with the sensor, the pins used are implicit (A4 and A5). NodeManager will then send the temperature and the humidity to the controller at the end of each sleeping cycle:

~~~c
   nodeManager.registerSensor(SENSOR_SHT21);
~~~

Register a LDR sensor attached to pin A1 and send to the gateway the average of 3 samples:

~~~c
  int sensor_ldr = nodeManager.registerSensor(SENSOR_LDR,A1);
  ((SensorLDR*)nodeManager.getSensor(sensor_ldr))->setSamples(3);
~~~

Register a rain sensor connected to A0. This will be powered with via pins 4 (ground) and 5 (vcc) just before reading its value at each cycle, it will be presented as S_RAIN. sending V_RAINRATE messages, the output will be a percentage (calculated between 200 and 1024) and the value will be reversed (so that no rain will be 0%):

~~~c
  int rain = nodeManager.registerSensor(SENSOR_ANALOG_INPUT,A0);
  SensorAnalogInput* rainSensor = ((SensorAnalogInput*)nodeManager.getSensor(rain));
  rainSensor->setPowerPins(4,5,300);
  rainSensor->setPresentation(S_RAIN);
  rainSensor->setType(V_RAINRATE);
  rainSensor->setOutputPercentage(true);
  rainSensor->setRangeMin(200);
  rainSensor->setRangeMax(1024);
  rainSensor->setReverse(true);
~~~

Register a latching relay connecting to pin 6 (set) and pin 7 (unset):

~~~c
  nodeManager.registerSensor(SENSOR_LATCHING_RELAY,6);
  nodeManager.registerSensor(SENSOR_LATCHING_RELAY,7);
~~~

# Example Sketches

## Analog Light and Temperature Sensor

The following sketch can be used to report the temperature and the light level based on a thermistor and LDR sensors attached to two analog pins of the arduino board (A1 and A2). Both the thermistor and the LDR are connected to ground on one side and to vcc via a resistor on the other so to measure the voltage drop across each of them through the analog pins. 

The sensor will be put to sleep after startup and will report both the measures every 10 minutes. NodeManager will take care of presenting the sensors, managing the sleep cycle, reporting the battery level every 10 cycles and report the measures in the appropriate format. This sketch requires MODULE_ANALOG_INPUT enabled in the global config.h file.

Even if the sensor is sleeping most of the time, it can be potentially woke up by sending a V_CUSTOM message with a WAKEUP payload to NodeManager service child id (200 by default) just after having reported its heartbeat. At this point the node will report AWAKE and the user can interact with it by e.g. sending REQ messages to its child IDs, changing the duration of a sleep cycle with a V_CUSTOM message to the NodeManager service child id, etc.

~~~c
/*
NodeManager is intended to take care on your behalf of all those common tasks a MySensors node has to accomplish, speeding up the development cycle of your projects.

NodeManager includes the following main components:
- Sleep manager: allows managing automatically the complexity behind battery-powered sensors spending most of their time sleeping
- Power manager: allows powering on your sensors only while the node is awake
- Battery manager: provides common functionalities to read and report the battery level
- Remote configuration: allows configuring remotely the node without the need to have physical access to it
- Built-in personalities: for the most common sensors, provide embedded code so to allow their configuration with a single line 

Documentation available on: https://github.com/mysensors/NodeManager
 */

 
// load user settings
#include "config.h"
// load MySensors library
#include <MySensors.h>
// load NodeManager library
#include "NodeManager.h"

// create a NodeManager instance
NodeManager nodeManager;

// before
void before() {
  // setup the serial port baud rate
  Serial.begin(MY_BAUD_RATE);  
  /*
   * Register below your sensors
  */
  nodeManager.setSleep(SLEEP,10,MINUTES); 
  nodeManager.registerSensor(SENSOR_THERMISTOR,A1);
  nodeManager.registerSensor(SENSOR_LDR,A2);
  /*
   * Register above your sensors
  */
  nodeManager.before();
}

// presentation
void presentation() {
  // Send the sketch version information to the gateway and Controller
	sendSketchInfo(SKETCH_NAME,SKETCH_VERSION);
  // call NodeManager presentation routine
  nodeManager.presentation();

}

// setup
void setup() {
  // call NodeManager setup routine
  nodeManager.setup();
}

// loop
void loop() {
  // call NodeManager loop routine
  nodeManager.loop();

}

// receive
void receive(const MyMessage &message) {
  // call NodeManager receive routine
  nodeManager.receive(message);
}
~~~

## Motion Sensor

The following sketch can be used to report back to the controller when a motion sensor attached to the board's pin 3 triggers. In this example, the board will be put to sleep just after startup and will report a heartbeat every hour. NodeManager will take care of configuring an interrupt associated to the provided pin so automatically wake up when a motion is detected and report a V_TRIPPED message back. This sketch requires MODULE_SWITCH to be enabled in the global config.h file.

~~~c
/*
NodeManager is intended to take care on your behalf of all those common tasks a MySensors node has to accomplish, speeding up the development cycle of your projects.

NodeManager includes the following main components:
- Sleep manager: allows managing automatically the complexity behind battery-powered sensors spending most of their time sleeping
- Power manager: allows powering on your sensors only while the node is awake
- Battery manager: provides common functionalities to read and report the battery level
- Remote configuration: allows configuring remotely the node without the need to have physical access to it
- Built-in personalities: for the most common sensors, provide embedded code so to allow their configuration with a single line 

Documentation available on: https://github.com/mysensors/NodeManager 
 */

 
// load user settings
#include "config.h"
// load MySensors library
#include <MySensors.h>
// load NodeManager library
#include "NodeManager.h"

// create a NodeManager instance
NodeManager nodeManager;

// before
void before() {
  // setup the serial port baud rate
  Serial.begin(MY_BAUD_RATE);  
  /*
   * Register below your sensors
  */
  nodeManager.setSleep(SLEEP,60,MINUTES); 
  nodeManager.registerSensor(SENSOR_MOTION,3);

  /*
   * Register above your sensors
  */
  nodeManager.before();
}

// presentation
void presentation() {
  // Send the sketch version information to the gateway and Controller
	sendSketchInfo(SKETCH_NAME,SKETCH_VERSION);
  // call NodeManager presentation routine
  nodeManager.presentation();

}

// setup
void setup() {
  // call NodeManager setup routine
  nodeManager.setup();
}

// loop
void loop() {
  // call NodeManager loop routine
  nodeManager.loop();

}

// receive
void receive(const MyMessage &message) {
  // call NodeManager receive routine
  nodeManager.receive(message);
}
~~~

## Boiler Sensor

The following sketch controls a latching relay connected to a boiler. A latching relay (requiring only a pulse to switch) has been chosen to minimize the power consumption required by a traditional relay to stay on. This relay has normally two pins, one for closing and the other for opening the controlled circuit, connected to pin 6 and 7 of the arduino board. This is why we have to register two sensors against NodeManager so to control the two funtions indipendently. Since using a SENSOR_LATCHING_RELAY type of sensor, NodeManager will take care of just sending out a single pulse only when a REQ command of type V_STATUS is sent to one or the other child id.

In this example, the board also runs at 1Mhz so it can go down to 1.8V: by setting setBatteryMin() and setBatteryMax(), the battery percentage will be calculated and reported (by default, automatically every 10 sleeping cycles) based on these custom boundaries.

The board will be put to sleep just after startup and will report back to the controller every 5 minutes. It is the controller's responsability to catch when the board reports its heartbeat (using smart sleep behind the scene) and send a command back if needed. This sketch requires MODULE_DIGITAL_OUTPUT to be enabled in the config.h file.

~~~c
/*
NodeManager is intended to take care on your behalf of all those common tasks a MySensors node has to accomplish, speeding up the development cycle of your projects.

NodeManager includes the following main components:
- Sleep manager: allows managing automatically the complexity behind battery-powered sensors spending most of their time sleeping
- Power manager: allows powering on your sensors only while the node is awake
- Battery manager: provides common functionalities to read and report the battery level
- Remote configuration: allows configuring remotely the node without the need to have physical access to it
- Built-in personalities: for the most common sensors, provide embedded code so to allow their configuration with a single line 

Documentation available on: https://github.com/mysensors/NodeManager 
 */

 
// load user settings
#include "config.h"
// load MySensors library
#include <MySensors.h>
// load NodeManager library
#include "NodeManager.h"

// create a NodeManager instance
NodeManager nodeManager;

// before
void before() {
  // setup the serial port baud rate
  Serial.begin(MY_BAUD_RATE);  
  /*
   * Register below your sensors
  */
  nodeManager.setBatteryMin(1.8);
  nodeManager.setBatteryMax(3.2);
  nodeManager.setSleep(SLEEP,5,MINUTES);
  nodeManager.registerSensor(SENSOR_LATCHING_RELAY,6);
  nodeManager.registerSensor(SENSOR_LATCHING_RELAY,7);

  /*
   * Register above your sensors
  */
  nodeManager.before();
}

// presentation
void presentation() {
  // Send the sketch version information to the gateway and Controller
	sendSketchInfo(SKETCH_NAME,SKETCH_VERSION);
  // call NodeManager presentation routine
  nodeManager.presentation();

}

// setup
void setup() {
  // call NodeManager setup routine
  nodeManager.setup();
}

// loop
void loop() {
  // call NodeManager loop routine
  nodeManager.loop();

}

// receive
void receive(const MyMessage &message) {
  // call NodeManager receive routine
  nodeManager.receive(message);
}
~~~


## Rain and Soil Moisture Sensor

The following sketch can be used to report the rain level and the soil moisture based on two sensors connected to the board's analog pins (A1 and A2). In this case we are customizing the out-of-the-box SENSOR_ANALOG_INPUT sensor type since we just need to measure an analog input but we also want to provide the correct type and presentation for each sensor. 

We register the sensors first with registerSensor() which returns the child id assigned to the sensor. We then retrieve the sensor's reference by calling getSensor() so we can invoke the sensor-specific functions, like setPresentation() and setType().

In this example, the two sensors are not directly connected to the battery's ground and vcc but, to save additional power, are powered through two arduino's pins. By using e.g. setPowerPins(4,5,300), NodeManger will assume pin 4 is ground and pin 5 is vcc for that specific sensor so it will turn on the power just before reading the analog input (and waiting 300ms for the sensor to initialize) and back off before going to sleep.

For both the sensors we want a percentage output and with setRangeMin() and setRangeMax() we define the boundaries for calculating the percentage (if we read e.g. 200 when the rain sensor is completely into the water, we know for sure it will not go below this value which will represent the new lower boundary). 
Finally, since both the sensors reports low when wet and high when dry but we need the opposite, we set setReverse() so to have 0% reported when there is no rain/moisture, 100% on the opposite situation.

~~~c
/*
NodeManager is intended to take care on your behalf of all those common tasks a MySensors node has to accomplish, speeding up the development cycle of your projects.

NodeManager includes the following main components:
- Sleep manager: allows managing automatically the complexity behind battery-powered sensors spending most of their time sleeping
- Power manager: allows powering on your sensors only while the node is awake
- Battery manager: provides common functionalities to read and report the battery level
- Remote configuration: allows configuring remotely the node without the need to have physical access to it
- Built-in personalities: for the most common sensors, provide embedded code so to allow their configuration with a single line 

Documentation available on: https://github.com/mysensors/NodeManager 
 */

 
// load user settings
#include "config.h"
// load MySensors library
#include <MySensors.h>
// load NodeManager library
#include "NodeManager.h"

// create a NodeManager instance
NodeManager nodeManager;

// before
void before() {
  // setup the serial port baud rate
  Serial.begin(MY_BAUD_RATE);  
  /*
   * Register below your sensors
  */
  analogReference(DEFAULT);
  nodeManager.setSleep(SLEEP,10,MINUTES);
  
  int rain = nodeManager.registerSensor(SENSOR_ANALOG_INPUT,A1);
  int soil = nodeManager.registerSensor(SENSOR_ANALOG_INPUT,A2);
  
  SensorAnalogInput* rainSensor = ((SensorAnalogInput*)nodeManager.getSensor(rain));
  SensorAnalogInput* soilSensor = ((SensorAnalogInput*)nodeManager.getSensor(soil));
  
  rainSensor->setPresentation(S_RAIN);
  rainSensor->setType(V_RAINRATE);
  rainSensor->setPowerPins(4,5,300);
  rainSensor->setOutputPercentage(true);
  rainSensor->setRangeMin(200);
  rainSensor->setRangeMax(1024);
  rainSensor->setReverse(true);
  
  soilSensor->setPresentation(S_MOISTURE);
  soilSensor->setType(V_LEVEL);
  soilSensor->setPowerPins(6,7,300);
  soilSensor->setOutputPercentage(true);
  soilSensor->setRangeMin(300);
  soilSensor->setRangeMax(1024);
  soilSensor->setReverse(true);

  /*
   * Register above your sensors
  */
  nodeManager.before();
}

// presentation
void presentation() {
  // Send the sketch version information to the gateway and Controller
	sendSketchInfo(SKETCH_NAME,SKETCH_VERSION);
  // call NodeManager presentation routine
  nodeManager.presentation();

}

// setup
void setup() {
  // call NodeManager setup routine
  nodeManager.setup();
}

// loop
void loop() {
  // call NodeManager loop routine
  nodeManager.loop();

}

// receive
void receive(const MyMessage &message) {
  // call NodeManager receive routine
  nodeManager.receive(message);
}
~~~

# Release Notes

v1.0:

* Initial release

v1.1:
* Added ability to sleep between send() so to save additional battery
* Bug fixes

v1.2:

* Added out-of-the-box support for BH1750 light sensor
* Added out-of-the-box support for HTU21D temperature and humidity sensor
* Added out-of-the-box support for MLX90614 contactless temperature sensor
* Added a few examples to the documentation
* Fixed a few bugs

v1.3:

* Added support for BME280 temperature/humudity/pressure sensor
* Added option to measure battery level via a pin in addition to internal Vcc
* Added example sketches to the documentation
* Fixed a few bugs

v1.4:

* Added support for ML8511 UV intensity sensor
* Added support for MQ air quality sensor
* Added ability to manually assign a child id to a sensor
* Ensured compatibility for non-sleeping nodes
* Ability to control if waking up from an interrupt counts for a battery level report
* When power pins are set the sensor is powered on just after
* Service messages are disabled by default
* Bug fixes

v1.5:

* Added support for ACS712 current sensor
* Added support for HC-SR04 distance sensor
* Added support for BMP085/BMP180 temperature and pressure sensor
* Added support for Sonoff smart switch
* Added forecast output to BME280 sensor
* Added capability to retrieve the time from the controller
* Added support for running as a gateway
* A heartbeat is sent when waking up from a wait cycle
* Allowed combining sensors waking up from an interrupt and sensors reporting periodically
* Optimized battery life for DS18B20 sensors
* SLEEP_MANAGER has been deprecated and setMode() replaces setSleepMode()
* New mode ALWAYS_ON to let the node staying awake and executing each sensors' loop
* ESP8266WiFi.h has to be included in the main sketch if MY_GATEWAY_ESP8266 is defined
* Added receiveTime() wrapper in the main sketch
* Fixed the logic for output sensors
* Added common gateway settings in config.h