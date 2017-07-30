NodeManager is intended to take care on your behalf of all those common tasks a MySensors node has to accomplish, speeding up the development cycle of your projects.

NodeManager includes the following main components:

* Sleep manager: allows managing automatically the complexity behind battery-powered sensors spending most of their time sleeping
* Power manager: allows powering on your sensors only while the node is awake
* Battery manager: provides common functionalities to read and report the battery level
* Remote configuration: allows configuring remotely the node without the need to have physical access to it
* Built-in sensors: for the most common sensors, provide embedded code so to allow their configuration with a single line

## Features

* Manage all the aspects of a sleeping cycle by leveraging smart sleep
* Allow configuring the node and any attached sensors remotely
* Allow waking up a sleeping node remotely at the end of a sleeping cycle
* Allow powering on each connected sensor only while the node is awake to save battery
* Report battery level periodically and automatically or on demand
* Report signal level periodically and automatically or on demand
* Calculate battery level without requiring an additional pin and the resistors
* Allow rebooting the board remotely
* Provide out-of-the-box sensors personalities and automatically execute their main task at each cycle
* Allow collecting and averaging multiple samples, tracking the last value and forcing periodic updates for any sensor
* Provide buil-in capabilities to handle interrupt-based sensors 

## Installation
* Download the package or clone the git repository from https://github.com/mysensors/NodeManager
* Open the provided sketch and save it under a different name
* Open `config.h` and customize both MySensors configuration and NodeManager global settings
* Register your sensors in the sketch file
* Upload the sketch to your arduino board

Please note NodeManager cannot be used as an arduino library since requires access to your MySensors configuration directives, hence its files have to be placed into the same directory of your sketch.

### Upgrade
* Download the package
* Replace the NodeManager.cpp and NodeManager.h of your project with those just downloaded
* Review the release notes in case there is any manual change required to the existing sketch or config.h file

## Configuration
NodeManager configuration includes compile-time configuration directives (which can be set in config.h), runtime global and per-sensor configuration settings (which can be set in your sketch).

### Setup MySensors
Since NodeManager has to communicate with the MySensors gateway on your behalf, it has to know how to do it. Place on top of the `config.h` file all the MySensors typical directives you are used to set on top of your sketch so both your sketch AND NodeManager will be able to share the same configuration. For example:
~~~c
/**********************************
 * Sketch configuration
 */

#define SKETCH_NAME "NodeManager"
#define SKETCH_VERSION "1.0"

/**********************************
 * MySensors node configuration
 */

// General settings
#define MY_BAUD_RATE 9600
//#define MY_DEBUG
//#define MY_NODE_ID 100
//#define MY_SMART_SLEEP_WAIT_DURATION_MS 500

// NRF24 radio settings
#define MY_RADIO_NRF24
//#define MY_RF24_ENABLE_ENCRYPTION
//#define MY_RF24_CHANNEL 76
//#define MY_RF24_PA_LEVEL RF24_PA_HIGH
//#define MY_DEBUG_VERBOSE_RF24
//#define MY_RF24_DATARATE RF24_250KBPS

// RFM69 radio settings
//#define MY_RADIO_RFM69
//#define MY_RFM69_FREQUENCY RF69_868MHZ
//#define MY_IS_RFM69HW
//#define MY_DEBUG_VERBOSE_RFM69
//#define MY_RFM69_NEW_DRIVER
//#define MY_RFM69_ENABLE_ENCRYPTION
//#define MY_RFM69_NETWORKID 100
//#define MY_RF69_IRQ_PIN D1
//#define MY_RF69_IRQ_NUM MY_RF69_IRQ_PIN
//#define MY_RF69_SPI_CS D2

// RS485 serial transport settings
//#define MY_RS485
//#define MY_RS485_BAUD_RATE 9600
//#define MY_RS485_DE_PIN 2
//#define MY_RS485_MAX_MESSAGE_LENGTH 40
//#define MY_RS485_HWSERIAL Serial1

// Message signing settings
//#define MY_SIGNING_SOFT
//#define MY_SIGNING_SOFT_RANDOMSEED_PIN 7
//#define MY_SIGNING_REQUEST_SIGNATURES
//#define MY_SIGNING_ATSHA204

// OTA Firmware update settings
//#define MY_OTA_FIRMWARE_FEATURE
//#define OTA_WAIT_PERIOD 300
//#define FIRMWARE_MAX_REQUESTS 2
//#define MY_OTA_RETRY 2

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
//#define MY_ESP8266_SSID ""
//#define MY_ESP8266_PASSWORD ""

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

### Enable/Disable NodeManager's modules

The next step is to enable NodeManager's additional functionalities and the modules required for your sensors. The directives in the `config.h` file control which module/library/functionality will be made available to your sketch. Enable (e.g. set to 1) only what you need to ensure enough storage is left to your custom code.

~~~c
/***********************************
 * NodeManager configuration
 */

// if enabled, enable debug messages on serial port
#define DEBUG 1

// if enabled, enable the capability to power on sensors with the arduino's pins to save battery while sleeping
#define POWER_MANAGER 1
// if enabled, will load the battery manager library to allow the battery level to be reported automatically or on demand
#define BATTERY_MANAGER 1
// if enabled, allow modifying the configuration remotely by interacting with the configuration child id
#define REMOTE_CONFIGURATION 1
// if enabled, persist the remote configuration settings on EEPROM
#define PERSIST 0
// if enabled, a battery sensor will be created at BATTERY_CHILD_ID and will report vcc voltage together with the battery level percentage
#define BATTERY_SENSOR 1
// if enabled, a signal sensor will be created at RSSI_CHILD_ID (202 by default) and will report the signal quality of the transport layer
#define SIGNAL_SENSOR 1
// if enabled, send a SLEEPING and AWAKE service messages just before entering and just after leaving a sleep cycle and STARTED when starting/rebooting
#define SERVICE_MESSAGES 0

// Enable this module to use one of the following sensors: SENSOR_ANALOG_INPUT, SENSOR_LDR, SENSOR_THERMISTOR, SENSOR_ML8511, SENSOR_ACS712, SENSOR_RAIN_GAUGE, SENSOR_RAIN, SENSOR_SOIL_MOISTURE
#define MODULE_ANALOG_INPUT 1
// Enable this module to use one of the following sensors: SENSOR_DIGITAL_INPUT
#define MODULE_DIGITAL_INPUT 1
// Enable this module to use one of the following sensors: SENSOR_DIGITAL_OUTPUT, SENSOR_RELAY, SENSOR_LATCHING_RELAY
#define MODULE_DIGITAL_OUTPUT 1
// Enable this module to use one of the following sensors: SENSOR_DHT11, SENSOR_DHT22
#define MODULE_DHT 0
// Enable this module to use one of the following sensors: SENSOR_SHT21
#define MODULE_SHT21 0
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
// Enable this module to use one of the following sensors: SENSOR_MCP9808
#define MODULE_MCP9808 0
// Enable this module to use one of the following sensors: SENSOR_MQ
#define MODULE_MQ 0
// Enable this module to use one of the following sensors: SENSOR_MHZ19
#define MODULE_MHZ19 0
// Enable this module to use one of the following sensors: SENSOR_AM2320    
#define MODULE_AM2320 0
// Enable this module to use one of the following sensors: SENSOR_TSL2561    
#define MODULE_TSL2561 0
// Enable this module to use one of the following sensors: SENSOR_PT100
#define MODULE_PT100 0
// Enable this module to use one of the following sensors: SENSOR_BMP280
#define MODULE_BMP280 0
// Enable this module to use one of the following sensors: SENSOR_DIMMER
#define MODULE_DIMMER 0
~~~

### Installing the dependencies

Some of the modules above rely on third party libraries. Those libraries are not included within NodeManager and have to be installed from the Arduino IDE Library Manager (Sketch -> Include Library -> Manager Libraries) or manually. You need to install the library ONLY if the module is enabled:

Module  | Required Library
 ------------- | -------------
MODULE_SHT21 | https://github.com/SodaqMoja/Sodaq_SHT2x
MODULE_DHT | https://github.com/mysensors/MySensorsArduinoExamples/tree/master/libraries/DHT
MODULE_DS18B20 | https://github.com/milesburton/Arduino-Temperature-Control-Library
MODULE_BH1750 | https://github.com/claws/BH1750
MODULE_MLX90614 | https://github.com/adafruit/Adafruit-MLX90614-Library
MODULE_BME280 | https://github.com/adafruit/Adafruit_BME280_Library
MODULE_SONOFF | https://github.com/thomasfredericks/Bounce2
MODULE_BMP085 | https://github.com/adafruit/Adafruit-BMP085-Library
MODULE_HCSR04 | https://github.com/mysensors/MySensorsArduinoExamples/tree/master/libraries/NewPing
MODULE_MCP9808 | https://github.com/adafruit/Adafruit_MCP9808_Library
MODULE_AM2320 | https://github.com/thakshak/AM2320
MODULE_TSL2561 | https://github.com/adafruit/TSL2561-Arduino-Library
MODULE_BMP280 | https://github.com/adafruit/Adafruit_BMP280_Library

### Configure NodeManager

The next step is to configure NodeManager with settings which will instruct how the node should behave. To do so, go to the main sketch, inside the `before()` function and add call one or more of the functions below just before registering your sensors. The following methods are exposed for your convenience and can be called on the `nodeManager` object already created for you:

~~~c
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
~~~

### Set reporting intervals and sleeping cycles

If not instructed differently, the node will stay in awake, all the sensors will report every 10 minutes. Battery level and signal level will be automatically reported every 60 minutes. To change those settings, you can call the following functions on the nodeManager object:

Function  | Description
------------ | -------------
setSleepSeconds()/setSleepMinutes()/setSleepHours()/setSleepDays() | the time interval the node will spend in a (smart) sleep cycle
setReportIntervalSeconds()/setReportIntervalMinutes()/setReportIntervalHours()/setReportIntervalDays() | the time interval the node will report the measures of all the attached sensors
setBatteryReportSeconds()/setBatteryReportMinutes()/setBatteryReportHours()/setBatteryReportDays() | the time interval the node will report the battery level
setSignalReportSeconds()/setSignalReportMinutes()/setSignalReportHours()/setSignalReportDays() | the time interval the node will report the radio signal level

For example, to put the node to sleep in cycles of 10 minutes:

~~~c
	nodeManager.setSleepMinutes(10);
~~~

If you need every sensor to report at a different time interval, you can call `setReportIntervalMinutes()` or `setReportIntervalSeconds()` on the sensor's object. For example to have a DHT sensor reporting every 60 seconds while all the other sensors every 20 minutes:
~~~c
int id = nodeManager.registerSensor(SENSOR_DHT22,6);
SensorDHT* dht = (SensorDHT*)nodeManager.get(id);
dht->setReportIntervalSeconds(60);
nodeManager.setReportIntervalMinutes(20);
~~~

Please note, if you configure a sleep cycle, this may have an impact on the reporting interval since the sensor will be able to report its measures ONLY when awake. For example if you set a report interval of 5 minutes and a sleep cycle of 10 minutes, the sensors will report every 10 minutes.

### Register your sensors
Once configured the node, it is time to tell NodeManager which sensors are attached to the board and where. In your sketch, inside the `before()` function and just before calling `nodeManager.before()`, you can register your sensors against NodeManager. The following built-in sensor types are available. Remember the corresponding module should be enabled in `config.h` for a successful compilation: 

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
SENSOR_MCP9808 | MCP9808 sensor, measure the temperature through the attached module
SENSOR_RAIN_GAUGE | Rain gauge sensor
SENSOR_RAIN | Rain sensor, return the percentage of rain from an attached analog sensor
SENSOR_SOIL_MOISTURE | Soil moisture sensor, return the percentage of moisture from an attached analog sensor
SENSOR_MHZ19 | MH-Z19 CO2 sensor via UART (SoftwareSerial, default on pins 6(Rx) and 7(Tx)
SENSOR_TSL2561 | TSL2561 sensor, return light in lux
SENSOR_AM2320 | AM2320 sensors, return temperature/humidity based on the attached AM2320 sensor
SENSOR_PT100 | High temperature sensor associated with DFRobot Driver, return the temperature in C° from the attached PT100 sensor
SENSOR_BMP280 | BMP280 sensor, return temperature/pressure based on the attached BMP280 sensor
SENSOR_DIMMER | Generic dimmer sensor used to drive a pwm output

To register a sensor simply call the NodeManager instance with the sensory type and the pin the sensor is conncted to and optionally a child id. For example:
~~~c
	nodeManager.registerSensor(SENSOR_THERMISTOR,A2);
	nodeManager.registerSensor(SENSOR_DOOR,3,1);
~~~

Once registered, your job is done. NodeManager will assign a child id automatically if not instructed differently, present each sensor for you to the controller, query each sensor and report the measure back to the gateway/controller. For actuators (e.g. relays) those can be triggered by sending a `REQ` message with the expected type to their assigned child id.

When called, registerSensor returns the child_id of the sensor so you will be able to retrieve it later if needed. Please note for sensors creating multiple child IDs (like a DHT sensor which creates a temperature and humidity sensor with different IDs), the last id is returned.

#### Creating a custom sensor

If you want to create a custom sensor and register it with NodeManager so it can take care of all the common tasks, you can create an inline class inheriting from `Sensor` or other subclasses and implement the following methods:
~~~c
    // define what to do during before() to setup the sensor
    void onBefore();
	// define what to do during setup() by executing the sensor's main task
    void onSetup();
    // define what to do during loop() by executing the sensor's main task
    void onLoop();
    // define what to do during receive() when the sensor receives a message
    void onReceive(const MyMessage & message);
	// define what to do when receiving a remote configuration message
	void onProcess(Request & request);
	// define what to do when receiving an interrupt
	void onInterrupt();
~~~

You can then instantiate your newly created class and register with NodeManager:
~~~c
	nodeManager.registerSensor(new SensorCustom(&nodeManager,child_id, pin));
~~~

### Configuring the sensors
Each built-in sensor class comes with reasonable default settings. In case you want/need to customize any of those settings, after having registered the sensor, you can retrieve it back and call set functions common to all the sensors or specific for a given class.

To do so, use `nodeManager.getSensor(child_id)` which will return a pointer to the sensor. Remeber to cast it to the right class before calling their functions. For example:

~~~c
	SensorLatchingRelay* relay = (SensorLatchingRelay*) nodeManager.getSensor(2);
	relay->setPulseWidth(50);
~~~


#### Sensor's general configuration

The following methods are available for all the sensors and can be called on the object reference as per the example above:
~~~c
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
~~~

#### Sensor's specific configuration

Each sensor class can expose additional methods.

* SensorAnalogInput / SensorLDR / SensorRain / SensorSoilMoisture
~~~c
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
~~~

* SensorThermistor
~~~c
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
~~~

* SensorMQ
~~~c
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
~~~

* SensorACS712
~~~c
    // [101] set how many mV are equivalent to 1 Amp. The value depends on the module (100 for 20A Module, 66 for 30A Module) (default: 185);
    void setmVPerAmp(int value);
    // [102] set ACS offset (default: 2500);
    void setOffset(int value);
~~~

* SensorRainGauge
~~~c
    // [102] set how many mm of rain to count for each tip (default: 0.11)
    void setSingleTip(float value);
    // set initial value - internal pull up (default: HIGH)
    void setInitialValue(int value);
~~~

* SensorDigitalOutput / SensorRelay
~~~c
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
~~~

* SensorLatchingRelay (in addition to those available for SensorDigitalOutput / SensorRelay)
~~~c
    // [201] set the duration of the pulse to send in ms to activate the relay (default: 50)
    void setPulseWidth(int value);
    // [202] set the pin which turns the relay off (default: the pin provided while registering the sensor)
    void setPinOff(int value);
    // [203] set the pin which turns the relay on (default: the pin provided while registering the sensor + 1)
    void setPinOn(int value);
~~~

*  SensorSwitch / SensorDoor / SensorMotion
~~~c
    // [101] set the interrupt mode. Can be CHANGE, RISING, FALLING (default: CHANGE)
    void setMode(int value);
    // [102] milliseconds to wait before reading the input (default: 0)
    void setDebounce(int value);
    // [103] time to wait in milliseconds after a change is detected to allow the signal to be restored to its normal value (default: 0)
    void setTriggerTime(int value);
    // [104] Set initial value on the interrupt pin (default: HIGH)
    void setInitial(int value);
~~~

*  SensorDs18b20
~~~c
    // returns the sensor's resolution in bits
    int getResolution();
    // [101] set the sensor's resolution in bits
    void setResolution(int value);
    // [102] sleep while DS18B20 calculates temperature (default: false)
    void setSleepDuringConversion(bool value);
    // return the sensors' device address
    DeviceAddress* getDeviceAddress();
~~~

*  SensorBME280 / SensorBMP085 / SensorBMP280
~~~c
    // [101] define how many pressure samples to keep track of for calculating the forecast (default: 5)
    void setForecastSamplesCount(int value);
~~~

* SensorHCSR04
~~~c
    // [101] Arduino pin tied to trigger pin on the ultrasonic sensor (default: the pin set while registering the sensor)
    void setTriggerPin(int value);
    // [102] Arduino pin tied to echo pin on the ultrasonic sensor (default: the pin set while registering the sensor)
    void setEchoPin(int value);
    // [103] Maximum distance we want to ping for (in centimeters) (default: 300)
    void setMaxDistance(int value);
~~~

*  SensorSonoff
~~~c
    // [101] set the button's pin (default: 0)
    void setButtonPin(int value);
    // [102] set the relay's pin (default: 12)
    void setRelayPin(int value);
    // [103] set the led's pin (default: 13)
    void setLedPin(int value);
~~~

* SensorMHZ19
~~~c
    // set the RX and TX pins for the software serial port to talk to the sensor
    void setRxTx(int rxpin, int txpin);
~~~

* SensorTSL2561
~~~c
    // [101] set the gain, possible values are SensorTSL2561::GAIN_0X (0), SensorTSL2561::GAIN_16X (1) (default 16x)
    void setGain(int value);
    // [102] set the timing, possible values are SensorTSL2561::INTEGRATIONTIME_13MS (0), SensorTSL2561::INTEGRATIONTIME_101MS (1), SensorTSL2561::INTEGRATIONTIME_402MS (2) (default: 13ms)
    void setTiming(int value);
    // [103] set the spectrum, possible values are SensorTSL2561::VISIBLE (0), SensorTSL2561::FULLSPECTRUM (1), SensorTSL2561::INFRARED (2), SensorTSL2561::FULL (3) (default: visible)
    void setSpectrum(int value);
    // [104] set the i2c address values are SensorTSL2561::ADDR_FLOAT, SensorTSL2561::ADDR_LOW, SensorTSL2561::ADDR_HIGH
    void setAddress(int value);
~~~

* SensorDimmer
~~~c
    // [101] set the effect to use for a smooth transition, can be one of SensorDimmer::EASE_LINEAR, SensorDimmer::EASE_INSINE, SensorDimmer::EASE_OUTSINE, SensorDimmer::EASE_INOUTSINE (default: EASE_LINEAR)
    void setEasing(int value);
    // [102] the duration of entire the transition in seconds (default: 1)
    void setDuration(int value);
    // [103] the duration of a single step of the transition in milliseconds (default: 100)
    void setStepDuration(int value);
    // fade the output from the current value to the target provided in the range 0-100
    void fadeTo(int value);
~~~

### Upload your sketch

Upload your sketch to your arduino board as you are used to.

Check your gateway's logs to ensure the node is working as expected. You should see the node presenting itself, reporting battery level, presenting all the registered sensors and the configuration child id service.
When `DEBUG` is enabled, detailed information is available through the serial port. Remember to disable debug once the tests have been completed.

### Communicate with NodeManager and its sensors

You can interact with each registered sensor by sending to the child id a `REQ` command (or a `SET` for output sensors like relays). For example to request the temperature to node_id 254 and child_id 1:

`254;1;2;0;0;`

To activate a relay connected to the same node, child_id 100 we need to send a `SET` command with payload set to 1:

`254;100;1;0;2;1`

No need to implement anything on your side since for built-in sensors this is handled automatically. 

NodeManager exposes also a configuration service which is by default on child_id 200 so you can interact with it by sending `V_CUSTOM` type of messages and commands within the payload. For each `REQ` message, the node will respond with a `SET` message if successful. 
Almost all the functions made available through the API can be called remotely. To do so, the payload must be in the format `<function_id>[,<value_to_set>]` where `function_id` is the number between square brackets you can find in the description above and, if the function takes and argument, this can be passed along in `value_to_set`. 
For example, to request a battery report, find the function you need to call remotely within the documentation:
~~~c
    // [2] Send a battery level report to the controller
    void batteryReport();
~~~
In this case `function_id` will be 2. To request a battery report to the node_id 100, send the following message:
`<node_id>;<configuration_child_id>;<req>;0;<V_CUSTOM>;<function_id>`
`100;200;2;0;48;2`

The change the sleep time to e.g. 10 minutes:
~~~c
    // [4] set the duration (in minutes) of a sleep cycle
    void setSleepMinutes(int value);
~~~
`<node_id>;<configuration_child_id>;<req>;0;<V_CUSTOM>;<function_id>,<value>`
`100;200;2;0;48;4,10`

To wake up a node previously configured as sleeping, send the following as the node wakes up next:
~~~c
    // [9] wake up the board
    void wakeup();
~~~
`100;200;2;0;48;9`

The same protocol can be used to execute remotely also sensor-specific functions. In this case the message has to be sent to the sensor's child_id, with a `V_CUSTOM` type of message. For example if you want to collect and average 10 samples for child_id 1:
~~~c
    // [5] For some sensors, the measurement can be queried multiple times and an average is returned (default: 1)
    void setSamples(int value);
~~~
`100;1;2;0;48;5,10`

If you want to decrease the temperature offset of a thermistor sensor to -2:
~~~c
    // [105] set a temperature offset
    void setOffset(float value);
~~~
`100;1;2;0;48;105,-2`

Please note that anything set remotely will NOT persist a reboot apart from those setting the sleep interval which are saved to the EEPROM (provided `PERSIST` is enabled).

## Understanding NodeManager: how it works

A NodeManager object is created for you at the beginning of your sketch and its main functions must called from within `before()`, `presentation()`, `loop()` and `receive()` to work properly. NodeManager will do the following during each phase:

NodeManager::before():
* Setup the interrupt pins to wake up the board based on the configured interrupts
* If persistance is enabled, restore from the EEPROM the latest sleeping settings
* Call `before()` of each registered sensor

Sensor::before():
* Call sensor-specific implementation of before by invoking `onBefore()` to initialize the sensor

NodeManager::setup():
* Send a custom message with a STARTED payload to the controller
* Call `setup()` of each registered sensor

Sensor::setup():
* Call sensor-specific implementation of setup by invoking `onSetup()` to initialize the sensor

NodeManager::loop():
* If all the sensors are powered by an arduino pin, this is turned on
* Call `loop()` of each registered sensor
* If all the sensors are powered by an arduino pin, this is turned off

Sensor::loop():
* If the sensor is powered by an arduino pin, this is set to on
* For each registered sensor, the sensor-specific `onLoop()` is called. If multiple samples are requested, this is run multiple times. `onLoop()` is not intended to send out any message but just sets a new value to a local variable
* In case multiple samples have been collected, the average is calculated
* A message is sent to the gateway with the calculated value. Depending on the configuration, this is not sent if it is the same as the previous value or sent anyway after a given number of cycles. These functionalies are not sensor-specific and common to all the sensors inheriting from the `Sensor` class.
* If the sensor is powered by an arduino pin, this is turned off

NodeManager::receive():
* Receive a message from the radio network 
* If the destination child id is the configuration node, it will handle the incoming message, otherwise will dispatch the message to the recipient sensor

Sensor::receive(): 
* Invoke `Sensor::loop()` which will execute the sensor main taks and eventually call `Sensor::onReceive()`

NodeManager::process():
* Process an incoming remote configuration request

Sensor::process():
* Process a sensor-generic incoming remote configuration request
* Calls `onProcess()` for sensor-specific incoming remote configuration request

Sensor::interrupt():
* Calls the sensor's implementation of `onInterrupt()` to handle the interrupt

## Examples
All the examples below takes place within the before() function in the main sketch, just below the "Register below your sensors" comment.

Set battery minimum and maxium voltage. This will be used to calculate the level percentage:

~~~c
    nodeManager.setBatteryMin(1.8);
    nodeManager.setBatteryMin(3.2);
~~~

Instruct the board to sleep for 10 minutes at each cycle:

~~~c
    nodeManager.setSleepMinutes(10);
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

Register a SHT21 temperature/humidity sensor; since using I2C for communicating with the sensor, the pins used are implicit (A4 and A5). NodeManager will then send the temperature and the humidity to the controller at the end of each sleeping cycle:

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
~~~

## Example Sketches

*  Analog Light and Temperature Sensor

The following sketch can be used to report the temperature and the light level based on a thermistor and LDR sensors attached to two analog pins of the arduino board (A1 and A2). Both the thermistor and the LDR are connected to ground on one side and to vcc via a resistor on the other so to measure the voltage drop across each of them through the analog pins. 

The sensor will be put to sleep after startup and will report both the measures every 10 minutes. NodeManager will take care of presenting the sensors, managing the sleep cycle, reporting the battery level every hour and report the measures in the appropriate format. This sketch requires MODULE_ANALOG_INPUT enabled in the global config.h file.

Even if the sensor is sleeping most of the time, it can be potentially woke up by sending a V_CUSTOM message to NodeManager service child id (200 by default) just after having reported its heartbeat. At this point the node will report awake and the user can interact with it by e.g. sending REQ messages to its child IDs, changing the duration of a sleep cycle, etc.

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
  nodeManager.setSleepMinutes(10);
  nodeManager.setReportIntervalMinutes(10);
  nodeManager.registerSensor(SENSOR_THERMISTOR,A1);
  nodeManager.registerSensor(SENSOR_LDR,A2);
  /*
   * Register above your sensors
  */
  nodeManager.before();
}

// presentation
void presentation() {
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

// receiveTime
void receiveTime(unsigned long ts) {
  // call NodeManager receiveTime routine
  nodeManager.receiveTime(ts);
}
~~~

*  Motion Sensor

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
// include supporting libraries
#ifdef MY_GATEWAY_ESP8266
  #include <ESP8266WiFi.h>
#endif
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
  nodeManager.setSleepHours(1);
  nodeManager.registerSensor(SENSOR_MOTION,3);

  /*
   * Register above your sensors
  */
  nodeManager.before();
}

// presentation
void presentation() {
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

// receiveTime
void receiveTime(unsigned long ts) {
  // call NodeManager receiveTime routine
  nodeManager.receiveTime(ts);
}
~~~

*  Boiler Sensor

The following sketch controls a latching relay connected to a boiler. A latching relay (requiring only a pulse to switch) has been chosen to minimize the power consumption required by a traditional relay to stay on. This relay has normally two pins, one for closing and the other for opening the controlled circuit, connected to pin 6 and 7 of the arduino board. Since using a SENSOR_LATCHING_RELAY type of sensor, NodeManager will automatically consider the provided pin as the ON pin and the one just after as the OFF pin and will take care of just sending out a single pulse only when a SET command of type V_STATUS is sent to the child id. The appropriate pin will be then used.

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
// include supporting libraries
#ifdef MY_GATEWAY_ESP8266
  #include <ESP8266WiFi.h>
#endif
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
  nodeManager.setSleepMinutes(5);
  nodeManager.registerSensor(SENSOR_LATCHING_RELAY,6);

  /*
   * Register above your sensors
  */
  nodeManager.before();
}

// presentation
void presentation() {
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

// receiveTime
void receiveTime(unsigned long ts) {
  // call NodeManager receiveTime routine
  nodeManager.receiveTime(ts);
}
~~~


*  Rain and Soil Moisture Sensor

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
// include supporting libraries
#ifdef MY_GATEWAY_ESP8266
  #include <ESP8266WiFi.h>
#endif
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
  nodeManager.setSleepMinutes(10);
  nodeManager.setReportIntervalMinutes(10);
  
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

// receiveTime
void receiveTime(unsigned long ts) {
  // call NodeManager receiveTime routine
  nodeManager.receiveTime(ts);
}
~~~

## Contributing

Contributes to NodeManager are of course more than welcome. 

### Reporting an issue or request an enhancement

For reporting an issue, requesting support for a new sensor or any other kind of enhancement, please drop a message either on the project's main page (<https://www.mysensors.org/download/node-manager>) or directly on Github (<https://github.com/mysensors/NodeManager/issues>).


### Contributing to the code

If you want to contribute to the code, a pull request on Github is the way to go. First of all setup your development environment:

* Create a copy of the project in your Github account by clicking on the "Fork" button on `https://github.com/mysensors/NodeManager` 
* Check the copy actually exists on `https://github.com/<username>/NodeManager`
* Clone your repository on your computer: `git clone https://github.com/<username>/NodeManager.git`
* Configure the main project's repository as an upstream: `git remote add upstream https://github.com/mysensors/NodeManager.git`
* Create and switch to a local development branch: `git checkout -b development origin/development`

Before applying any change, ensure you have the latest development version available:
* Switch to your local development branch: `git checkout development`
* Fetch the latest version from the main project's repository: `git fetch upstream`
* Merge into your development copy all the changes from the main repository: `git merge development upstream/development`
* Update the development branch of your repository: `git push origin development`

Create a branch for the fix/feature you want to work on and apply changes to the code:
* Create and switch to a new branch (give it a significant name, e.g. fix/enum-sensors): `git checkout -b <yourbranch>`
* Do any required change to the code
* Include all the files changed for your commit: `git add .`
* Ensure both the main sketch and the config.h file do not present any change
* Commit the changes: `git  commit -m"Use enum instead of define for defining each sensor #121"`
* Push the branch with the changes to your repository: `git push origin <yourbranch>`
* Visit `https://github.com/<username>/NodeManager/branches` and click the "New pull request" button just aside your newly created branch
* Fill in the request with a significant title and description and select the "development" branch from the main repository to be compared against your branch. Ensure there is one or more issues the pull request will fix and make it explicit within the description
* Submit the request and start the discussion
* Any additional commits to your branch which will be presented within the same pull request
* When the pull request is merged, delete your working branch: `git branch -D <yourbranch>`
* Update your local and remote development branch as per the instructions above

If there are changes introduced to the development branch that conflicts with an open pull request, you will have to resolve the conflicts and update the PR:
* Fetch and merge into development any change from upstream/development as detailed above
* Switch to your branch: `git checkout <yourbranch>`
* Rebase the branch you filed the PR from against your updated development branch: `git rebase development`
* Resolve the conflicts and commit again
* Force push your updated branch so the PR gets updated: `git push HEAD:<yourbranch> -f`

## Release Notes

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
* Added support for Rain Gauge sensor
* Added support for MCP9808 temperature sensor
* Added forecast output to all Bosch sensors
* Added I2C address auto-discovery for all Bosch sensors
* Added support for running as a gateway
* Added option to retrieve the latest value of a sensor from outside NodeManager
* Remote reboot now does not need a reboot pin configured
* A heartbeat is now sent also when waking up from a wait cycle
* When waking up for an interrupt, only the code of the sensor expecting that interrupt is executed
* Added capability to retrieve the time from the controller
* Optimized battery life for DS18B20 sensors
* SLEEP_MANAGER has been deprecated (now always enabled) and setMode() replaces setSleepMode()
* New mode ALWAYS_ON to let the node staying awake and executing each sensors' loop
* ESP8266WiFi.h has to be included in the main sketch if MY_GATEWAY_ESP8266 is defined
* Added receiveTime() wrapper in the main sketch
* Fixed the logic for output sensors
* Added common gateway settings in config.h

v1.6:
* Introduced new remote API to allow calling all NodeManager's and sensors' functions remotely
* Decoupled reporting intervals from sleeping cycles
* All intervals (measure/battery reports) are now time-based
* Added support for BMP280 temperature and pressure sensor
* Added support for RS485 serial transport 
* Added support for TSL2561 light sensor
* Added support for DHT21 temperature/humidity sensor
* Added support for AM2320 temperature/humidity sensor
* Added support for PT100 high temperature sensor
* Added support for MH-Z19 CO2 sensor
* Added buil-in rain and soil moisture analog sensors
* Added support for generic dimmer sensor (PWM output)
* Radio signal level is reported automatically and on demand through child 202
* SensorRainGauge now supports sleep mode
* SensorSwitch now supports awake mode
* SensorLatchingRealy now handles automatically both on and off commands
* SensorMQ now depends on its own module
* Added automatic off capability (safeguard) to SensorDigitalOutput
* Any sensor can now access all NodeManager's functions
* DHT sensor now using MySensors' DHT library