# MySensors NodeManager [![Build Status](https://travis-ci.org/mysensors/NodeManager.svg?branch=master)](https://travis-ci.org/mysensors/NodeManager)

NodeManager is intended to take care on your behalf of all those common tasks that a MySensors node has to accomplish, speeding up the development cycle of your projects.
Consider it as a sort of frontend for your MySensors projects. When you need to add a sensor (which requires just uncommeting a single line),
NodeManager will take care of importing the required library, presenting the sensor to the gateway/controller, executing periodically the main function of the sensor
(e.g. measure a temperature, detect a motion, etc.), allowing you to interact with the sensor and even configuring it remotely.

## Features

* Allows managing automatically the complexity behind battery-powered sensors spending most of their time sleeping
* Provides common functionalities to read and report the battery level
* For the most common sensors, provide embedded code so to allow their configuration with a single line
* Manage all the aspects of a sleeping cycle by leveraging smart sleep
* Allow configuring the node and any attached sensors remotely
* Allow waking up a sleeping node remotely at the end of a sleeping cycle
* Allow powering on each connected sensor only while the node is awake to save battery
* Report battery level periodically and automatically or on demand
* Calculate battery level without requiring an additional pin and the resistors
* Report signal level periodically and automatically or on demand
* Allow collecting and averaging multiple samples, tracking the last value and forcing periodic updates for any sensor
* Provide built-in capabilities to handle interrupt-based sensors

## Installation

* Download the package or clone the git repository from https://github.com/mysensors/NodeManager
* Install NodeManager as an Arduino library (https://www.arduino.cc/en/Guide/Libraries)

### Upgrade

* Make a backup copy of the library, remove it, download the latest version of NodeManager and install the new library
* Review the release notes in case there is any manual change required to the main sketch

Please be aware when upgrading to v1.8 from an older version this procedure is not supported and the code should be migrated manually.

## Configuration

* Open the Template sketch from the Arduino IDE under File -> Examples -> MySensors Nodemanager -> Template
* Alternatively, open one of the provided example
* Customize NodeManager's and your sensors settings (see below)

### MySensors configuration

Since NodeManager has to communicate with the MySensors network on your behalf, it has to know how to do it. On top of the template sketch you will find the typical MySensors directives you are used to which can be customized to configure the board to act as a MySensors node or a MySensors gateway.

### NodeManager configuration

NodeManager built-in capabilities can be enabled/disabled also when you need to save some storage for your code.
To enable/disable a built-in feature:
* Install the required dependency if any
* Enable the corresponding capability by setting it to ON in the main sketch. To disable it, set it to OFF
* When a capability is enabled additional functions may be made available. Have a look at the API documentation for details

A list of the supported capabilities and the required dependencies is presented below:

Capability                      | Default | Description                                                                                      | Dependencies
--------------------------------|---------|--------------------------------------------------------------------------------------------------|----------------------------------------------------------
NODEMANAGER_DEBUG               | ON      | NodeManager's debug output on serial console                                                     | -
NODEMANAGER_DEBUG_VERBOSE       | OFF     | increase NodeManager's debug output on the serial console                                        | -
NODEMANAGER_POWER_MANAGER       | OFF     | allow powering on your sensors only while the node is awake                                      | -
NODEMANAGER_INTERRUPTS          | ON      | allow managing interrupt-based sensors like a PIR or a door sensor                               | -
NODEMANAGER_CONDITIONAL_REPORT  | OFF     | allow reporting a measure only when different from the previous or above/below a given threshold | -
NODEMANAGER_EEPROM              | OFF     | allow keeping track of some information in the EEPROM                                            | -
NODEMANAGER_SLEEP               | ON      | allow managing automatically the complexity behind battery-powered sleeping sensors              | -
NODEMANAGER_RECEIVE             | ON      | allow the node to receive messages; can be used by the remote API or for triggering the sensors  | -
NODEMANAGER_TIME                | OFF     | allow keeping the current system time in sync with the controller                                | https://github.com/PaulStoffregen/Time
NODEMANAGER_RTC                 | OFF     | allow keeping the current system time in sync with an attached RTC device                        | https://github.com/JChristensen/DS3232RTC
NODEMANAGER_SD                  | OFF     | allow reading from and writing to SD cards                                                       | -
NODEMANAGER_HOOKING             | OFF     | allow custom code to be hooked in the out of the box sensors                                     | -
NODEMANAGER_OTA_CONFIGURATION   | OFF     | allow over-the-air configuration of the sensors                                                  | -
NODEMANAGER_SERIAL_INPUT        | OFF     | read from the serial port at the end of each loop cycle expecting a serial protocol command      | -

Once the NodeManager library header file is included, a global instance of the NodeManager class called `nodeManager` is made available and can be used all along the sketch.

### Add your sensors

NodeManager provides built-in implementation of a number of sensors through ad-hoc classes located within the "sensors" directory of the library.
To use a built-in sensor:
* Install the required dependencies, if any manually or through the Arduino IDE Library Manager (see below)
* Include the sensor header file (e.g. `#include <sensors/SensorBattery.h>`)
* Create an instance of the sensor's class (e.g. `SensorBattery battery(node)`)

Once created, the sensor will automatically present one or more child to the gateway and controller.
A list of built-in sensors, required dependencies and the number of child automatically created is presented below:

Sensor/Class Name        |#Child | Description                                                                                       | Dependencies
-------------------------|-------|---------------------------------------------------------------------------------------------------|----------------------------------------------------------
SensorBattery            | 1     | Built-in sensor for automatic battery reporting                                                   | -
SensorSignal             | 1     | Built-in sensor for automatic signal level reporting                                              | -
SensorAnalogInput        | 1     | Generic analog sensor, return a pin's analog value or its percentage                              | -
SensorLDR                | 1     | LDR sensor, return the light level of an attached light resistor in percentage                    | -
SensorRain               | 1     | Rain sensor, return the percentage of rain from an attached analog sensor                         | -
SensorSoilMoisture       | 1     | Soil moisture sensor, return the percentage of moisture from an attached analog sensor            | -
SensorThermistor         | 1     | Thermistor sensor, return the temperature based on the attached thermistor                        | -
SensorTMP102             | 1     | Temperature sensor, return the temperature based on the TMP102 sensor	                         | https://github.com/Yannicked/Sensor_TMP102
SensorML8511             | 1     | ML8511 sensor, return UV intensity                                                                | -
SensorACS712             | 1     | ACS712 sensor, measure the current going through the attached module                              | -
SensorDigitalInput       | 1     | Generic digital sensor, return a pin's digital value                                              | -
SensorDigitalOutput      | 1     | Generic digital output sensor, allows setting the digital output of a pin to the requested value  | -
SensorRelay              | 1     | Relay sensor, allows activating the relay                                                         | -
SensorLatchingRelay1Pin  | 1     | Latching Relay sensor, allows toggling the relay with a pulse on the configured pin               | -
SensorLatchingRelay2Pins | 1     | Latching Relay sensor, allows turing the relay on and off with a pulse on the configured pins     | -
SensorDHT11              | 2     | DHT11 sensor, return temperature/humidity based on the attached DHT sensor                        | https://github.com/markruys/arduino-DHT
SensorDHT22              | 2     | DHT22 sensor, return temperature/humidity based on the attached DHT sensor                        | https://github.com/markruys/arduino-DHT
SensorSHT21              | 2     | SHT21 sensor, return temperature/humidity based on the attached SHT21 sensor                      | https://github.com/SodaqMoja/Sodaq_SHT2x
SensorHTU21D             | 2     | HTU21D sensor, return temperature/humidity based on the attached HTU21D sensor                    | https://github.com/SodaqMoja/Sodaq_SHT2x
SensorInterrupt          | 1     | Generic interrupt-based sensor, wake up the board when a pin changes status                       | -
SensorDoor               | 1     | Door sensor, wake up the board and report when an attached magnetic sensor has been opened/closed | -
SensorMotion             | 1     | Motion sensor, wake up the board and report when an attached PIR has triggered                    | -
SensorDs18b20            | 1+    | DS18B20 sensor, return the temperature based on the attached sensor                               | https://github.com/milesburton/Arduino-Temperature-Control-Library
SensorBH1750             | 1     | BH1750 sensor, return light level in lux                                                          | https://github.com/claws/BH1750
SensorMLX90614           | 2     | MLX90614 contactless temperature sensor, return ambient and object temperature                    | https://github.com/adafruit/Adafruit-MLX90614-Library
SensorBME280             | 4     | BME280 sensor, return temperature/humidity/pressure based on the attached BME280 sensor           | https://github.com/adafruit/Adafruit_BME280_Library / https://github.com/adafruit/Adafruit_Sensor
SensorBMP085             | 3     | BMP085 sensor, return temperature and pressure                                                    | https://github.com/adafruit/Adafruit-BMP085-Library / https://github.com/adafruit/Adafruit_Sensor
SensorBMP180             | 3     | BMP180 sensor, return temperature and pressure                                                    | https://github.com/adafruit/Adafruit-BMP085-Library / https://github.com/adafruit/Adafruit_Sensor
SensorBMP280             | 3     | BMP280 sensor, return temperature/pressure based on the attached BMP280 sensor                    | https://github.com/adafruit/Adafruit_BMP280_Library / https://github.com/adafruit/Adafruit_Sensor
SensorSonoff             | 1     | Sonoff wireless smart switch                                                                      | https://github.com/thomasfredericks/Bounce2
SensorHCSR04             | 1     | HC-SR04 sensor, return the distance between the sensor and an object                              | https://github.com/mysensors/MySensorsArduinoExamples/tree/master/libraries/NewPing
SensorMCP9808            | 1     | MCP9808 sensor, measure the temperature through the attached module                               | https://github.com/adafruit/Adafruit_MCP9808_Library
SensorMQ                 | 1     | MQ sensor, return ppm of the target gas. Tuned by default for MQ135 and CO2                       | -
SensorMHZ19              | 1     | MH-Z19 CO2 sensor via UART (SoftwareSerial, default on pins 6(Rx) and 7(Tx)                       | -
SensorAM2320             | 2     | AM2320 sensors, return temperature/humidity based on the attached AM2320 sensor                   | https://github.com/thakshak/AM2320
SensorTSL2561            | 1     | TSL2561 sensor, return light in lux                                                               | https://github.com/adafruit/TSL2561-Arduino-Library
SensorPT100              | 1     | DFRobot Driver high temperature sensor, return the temperature from the attached PT100 sensor     | https://github.com/nxcosa/HighTemperatureSensor
SensorDimmer             | 1     | Generic dimmer sensor used to drive a pwm output                                                  | -
SensorRainGauge          | 1     | Rain gauge sensor                                                                                 | -
SensorPowerMeter         | 1     | Power meter pulse sensor                                                                          | -
SensorWaterMeter         | 1     | Water meter pulse sensor                                                                          | -
SensorPlantowerPMS       | 3     | Plantower PMS particulate matter sensors (reporting PM<=1.0, PM<=2.5 and PM<=10.0 in µg/m³)       | https://github.com/fu-hsi/pms
SensorVL53L0X            | 1     | VL53L0X laser time-of-flight distance sensor via I²C, sleep pin supported (optional)              | https://github.com/pololu/vl53l0x-arduino
DisplaySSD1306           | 1     | SSD1306 128x64 OLED display (I²C); By default displays values of all sensors and children         | https://github.com/greiman/SSD1306Ascii.git
SensorSHT31              | 2     | SHT31 sensor, return temperature/humidity based on the attached SHT31 sensor                      | https://github.com/adafruit/Adafruit_SHT31
SensorSI7021             | 2     | SI7021 sensor, return temperature/humidity based on the attached SI7021 sensor                    | https://github.com/sparkfun/SparkFun_Si701_Breakout_Arduino_Library
SensorChirp              | 3     | Chirp soil moisture sensor (includes temperature and light sensors)                               | https://github.com/Apollon77/I2CSoilMoistureSensor
DisplayHD44780           | 1     | Supports most Hitachi HD44780 based LCDs, by default displays values of all sensors and children  | https://github.com/cyberang3l/NewLiquidCrystal
SensorTTP                | 1     | TTP226/TTP229 Touch control sensor                                                                | -
SensorServo              | 1     | Control a generic Servo motor sensor                                                              | -
SensorAPDS9960           | 1     | SparkFun RGB and Gesture Sensor                                                                   | https://github.com/sparkfun/APDS-9960_RGB_and_Gesture_Sensor
SensorNeopixel           | 1     | Control a Neopixel LED                                                                            | https://github.com/adafruit/Adafruit_NeoPixel
SensorSDS011             | 2     | SDS011 air quality sensor, return concentrations of 2.5 and 10 micrometer particles.              | https://github.com/ricki-z/SDS011
SensorFPM10A             | 1     | FPM10A fingerprint sensor                                                                         | https://github.com/adafruit/Adafruit-Fingerprint-Sensor-Library
SensorPH                 | 1     | PH ( SKU SEN161 ) sensor, measure the analog value from the amplifier module                      | -
SensorPca9685W 	         | 2     | Generic dimmer sensor (S_DIMMER) used to drive a single channel pwm output of PCA9685             | https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
SensorPca9685Rgb         | 2     | Generic RGB-dimmer sensor (S_RGB_LIGHT) used to drive RGB resp. 3-channel pwm output of PCA9685   | https://github.com/adafruit/Adafruit-PWM-Servo- Driver-Library
SensorPca9685Rgbw        | 2     | Generic RGBW-dimmer sensor (S_RGBW_LIGHT) used to drive RGBW resp. 4-channel pwm output of PCA9685| https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
SensorDSM501A            | 1     | Dust sensor module DSM501A for PM1.0 and PM2.5 particles                                          | -
SensorPN532              | 1     | PN532 NFC RFID Module                                                                             | https://github.com/elechouse/PN532
SensorCCS811             | 1     | CCS811 gas/Air Quality sensor. Measure VOC and eCO2                                               | https://github.com/adafruit/Adafruit_CCS811
SensorMPR121             | 1     | MPR121-based capacitive touch control sensor                                                      | https://github.com/adafruit/Adafruit_MPR121
SensorGSM                | 1     | Send SMS through an attached serial modem (e.g. SIM900)                                           | -
SensorHVAC               | 3     | Control Air conditioner through an attached IR led                                                | https://github.com/ToniA/arduino-heatpumpir
SensorWaterLeak          | 1     | Water leak sensor; via an interrupt, wake up the board and report when a leak is detected         | -
SensorIRremote           | 1     | Send signals to infrared devices through an attached IR sensor                                    | https://github.com/z3t0/Arduino-IRremote / https://github.com/crankyoldgit/IRremoteESP8266

Those sensors requiring a pin to operate would take it as an argument in the constructor.
NodeManager automatically creates all the child_ids, assigning an incremental counter. If you need to set your own child_id, pass it as the last argument to the constructor

Examples:

~~~c
// Add a thermistor sensor attached to pin A0
#include <sensors/SensorThermistor.h>
SensorThermistor thermistor(A0);

// Add a LDR sensor attached to pin A0 and assing child_id 5
#include <sensors/SensorLDR.h>
SensorLDR ldr(A1,5);

// Add a temperature/humidity sensor SHT21 sensor. No pin required since using i2c
#include <sensors/SensorSHT21.h>
SensorSHT21 sht21;
~~~

The sensor will be then registered automatically with NodeManager which will take care of it all along its lifecycle.
NodeManager will present each sensor for you to the controller, query each sensor and report the measure back to the gateway/controller. For actuators (e.g. relays) those can be triggered by sending a `REQ`/`SET` message with the expected type to their assigned child id.

### Installing the dependencies

Some of the sensors and buit-in capabilities rely on third party libraries. Those libraries are not included within NodeManager and have to be installed from the Arduino IDE Library Manager (Sketch -> Include Library -> Manager Libraries) or manually.
You need to install the library ONLY if you are planning to enable to use the sensor.


### Configure your sensors

NodeManager and all the sensors can be configured from within `before()` in the main sketch. Find `Configure your sensors below` to customize the behavior of any sensor by invoking one of the functions available.

Examples:

~~~c
// report measures of every attached sensors every 10 minutes
nodeManager.setReportIntervalMinutes(10);
// set the node to sleep in 5 minutes cycles
nodeManager.setSleepMinutes(5);
// report battery level every 10 minutes
battery.setReportIntervalMinutes(10);
// set an offset to -1 to a thermistor sensor
thermistor.setOffset(-1);
// Change the id of a the first child of a sht21 sensor
sht21.children.get(1)->child_id = 5;
// power all the nodes through dedicated pins
nodeManager.setPowerManager(power);
~~~

If not instructed differently, the node will stay awake and all the sensors will report every 10 minutes, battery level and signal level will be automatically reported every 60 minutes (if the corresponding sensors have been added).

Please note, if you configure a sleep cycle, this may have an impact on the reporting interval since the sensor will be able to report its measures ONLY when awake. For example if you set a report interval of 5 minutes and a sleep cycle of 10 minutes, the sensors will report every 10 minutes.

## Running your code

Once finished configuring your node, upload your sketch to your Arduino board as you are used to.

Check your gateway's logs to ensure the node is working as expected. You should see the node presenting itself, presenting all the registered sensors and reporting new measures at the configured reporting interval.
When `NODEMANAGER_DEBUG` is enabled, detailed information will be available through the serial port. The better understand the logs generaged by NodeManager, paste them into MySensors Log Parser (https://mysensors.org/build/parser).
Remember to disable debug once the tests have been completed to save additional storage.

## Communicate with the sensors

You can interact with each registered sensor by sending to the child id a `REQ` command (or a `SET` for output sensors like relays). For example to request the temperature to node_id 254 and child_id 1:

`254;1;2;0;0;`

To activate a relay connected to the same node, child_id 100 we need to send a `SET` command with payload set to 1:

`254;100;1;0;2;1`

No need to implement anything on your side since for built-in sensors this is handled automatically.

## API

You can interact with each class provided by NodeManager through a set of API functions.

### NodeManager API

~~~c
	// instantiate a NodeManager object. An optional fixed number of sensors can be passed as an argument
	NodeManager(uint8_t sensorcount = 0);
	// [10] send the same message multiple times (default: 1)
	void setRetries(uint8_t value);
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
    // [36] set the default interval in minutes all the sensors will report their measures. If the same function is called on a specific sensor, this will not change the previously set value. On sleeping sensors, the elapsed time can be evaluated only upon wake up (default: 10 minutes)
    void setReportIntervalSeconds(unsigned long value);
	unsigned long getReportIntervalSeconds();
    // [37] set the default interval in minutes all the sensors will report their measures. If the same function is called on a specific sensor, this will not change the previously set value. On sleeping sensors, the elapsed time can be evaluated only upon wake up (default: 10 minutes)
    void setReportIntervalMinutes(unsigned long value);
    // [38] set the default interval in minutes all the sensors will report their measures. If the same function is called on a specific sensor, this will not change the previously set value. On sleeping sensors, the elapsed time can be evaluated only upon wake up (default: 10 minutes)
    void setReportIntervalHours(unsigned int value);
    // [39] set the default interval in minutes all the sensors will report their measures. If the same function is called on a specific sensor, this will not change the previously set value. On sleeping sensors, the elapsed time can be evaluated only upon wake up (default: 10 minutes)
    void setReportIntervalDays(uint8_t value);
	// [30] if set and when the board is battery powered, sleep() is always called instead of wait() (default: true)
	void setSleepOrWait(bool value);
	// sleep if the node is a battery powered or wait if it is not for the given number of milliseconds
	void sleepOrWait(unsigned long value);
	// [31] set which pin is connected to RST of the board to reboot the board when requested. If not set the software reboot is used instead (default: -1)
	void setRebootPin(int8_t value);
	// [32] turn the ADC off so to save 0.2 mA
	void setADCOff();
	// send a message by providing the source child, type of the message and value
	void sendMessage(uint8_t child_id, uint8_t type, int value);
	void sendMessage(uint8_t child_id, uint8_t type, float value, uint8_t precision);
	void sendMessage(uint8_t child_id, uint8_t type, double value, uint8_t precision);
	void sendMessage(uint8_t child_id, uint8_t type, const char* value);
	// register a sensor
	void registerSensor(Sensor* sensor);
#if NODEMANAGER_SLEEP == ON
	// register a timer
	void registerTimer(InternalTimer* timer);
#endif
	// return the next-available child id
	uint8_t getAvailableChildId(uint8_t child_id = 0);
	// list containing all the registered sensors
	List<Sensor*> sensors;
	// return the Child object of the given child_id
	Child* getChild(uint8_t child_id);
	// return the sensor object of the given child_id
	Sensor* getSensorWithChild(uint8_t child_id);
	// sleep between send()
	void sleepBetweenSend();
	// set the analog reference to the given value and optionally perform some fake reading on the given pin
	void setAnalogReference(uint8_t value, uint8_t pin = -1);
	// send the configured unit prefix just before sending the first measure (default: false)
	void setSendUnitPrefix(bool value);
	// return the default unit prefix for the given sensor presentation and type
	const char* getDefaultUnitPrefix(uint8_t presentation, uint8_t type);
#if NODEMANAGER_SLEEP == ON
	// [3] set the duration (in seconds) of a sleep cycle
	void setSleepSeconds(unsigned long value);
	unsigned long getSleepSeconds();
	// [4] set the duration (in minutes) of a sleep cycle
	void setSleepMinutes(unsigned long value);
	// [5] set the duration (in hours) of a sleep cycle
	void setSleepHours(unsigned int value);
	// [29] set the duration (in days) of a sleep cycle
	void setSleepDays(uint8_t value);
	// [20] optionally sleep interval in milliseconds before sending each message to the radio network (default: 0)
	void setSleepBetweenSend(unsigned int value);
	// [43] when sleep between send is set, by default the node will only wait, set it to true to make it sleeping for long intervals (default: false)
	void setSleepBetweenSendSleepOrWait(bool value);
	// [9] wake up the board
	void wakeup();
	// use smart sleep for sleeping boards (default: true)
	void setSmartSleep(bool value);
#endif
#if NODEMANAGER_INTERRUPTS == ON
	// [19] if enabled, when waking up from the interrupt, the board stops sleeping. Disable it when attaching e.g. a motion sensor (default: true)
	void setSleepInterruptPin(int8_t value);
	// configure the interrupt pin and mode. Mode can be CHANGE, RISING, FALLING (default: MODE_NOT_DEFINED)
	void setInterrupt(int8_t pin, uint8_t mode, int8_t initial = -1);
	// [28] ignore two consecutive interrupts if happening within this timeframe in milliseconds (default: 100)
	void setInterruptDebounce(unsigned long value);
	// return the pin from which the last interrupt came
	int8_t getLastInterruptPin();
	// return the value of the pin from which the last interrupt came
	int8_t getLastInterruptValue();
	// setup the interrupt pins
	void setupInterrupts(bool from_setup);
#endif
#if NODEMANAGER_POWER_MANAGER == ON
	// configure a PowerManager common to all the sensors
	void setPowerManager(PowerManager& powerManager);
	// to save battery the sensor can be optionally connected to two pins which will act as vcc and ground and activated on demand
	void setPowerPins(int8_t ground_pin, int8_t vcc_pin, unsigned long wait_time = 50);
	// [24] manually turn the power on
	void powerOn();
	// [25] manually turn the power off
	void powerOff();
#endif
#if NODEMANAGER_EEPROM == ON
	// [7] clear the EEPROM
	void clearEeprom();
	// return the value stored at the requested index from the EEPROM
	int loadFromMemory(int index);
	// [27] save the given index of the EEPROM the provided value
	void saveToMemory(int index, int value);
	// [40] if set save the sleep settings in memory, also when changed remotely (default: false)
	void setSaveSleepSettings(bool value);
	// keep track in the eeprom of enabled/disabled status for each sensor (default: false)
	void setPersistEnabledSensors(bool value);
	bool getPersistEnabledSensors();
#endif
#if NODEMANAGER_TIME == ON
	// [41] synchronize the local time with the controller
	void syncTime();
	// [42] returns the current system time
	unsigned long getTime();
	// [43] set the hour offset for when syncronizing the time (default: 0)
	void setTimezone(int8_t value);
	// request the current time to the controller during setup(). Time with a RTC if configured is always synchronized (default: true)
	void setSyncTimeOnSetup(bool value);
	// request the current time to the controller just after a sleep cycle. Time with a RTC if configured is always synchronized (default: true)
	void setSyncTimeAfterSleep(bool value);
	// request the current time to the controller after the configured number of minutes (default: 0)
	void setSyncTimeAfterInterval(unsigned long value);
	// receiveTime() callback
	void receiveTime(unsigned long ts);
#endif
#if NODEMANAGER_SD == ON
	// SD card variables
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
#if NODEMANAGER_RECEIVE == ON
	void receive(const MyMessage & msg);
~~~

### Sensor API

The following methods are available for all the sensors:
~~~c
	Sensor(int8_t pin = -1);
	// return the name of the sensor
	const char* getName();
	// [1] where the sensor is attached to (default: not set)
	void setPin(int8_t value);
	// [5] For some sensors, the measurement can be queried multiple times and an average is returned (default: 1)
	void setSamples(unsigned int value);
	// [6] If more then one sample has to be taken, set the interval in milliseconds between measurements (default: 0)
	void setSamplesInterval(unsigned long value);
    // [17] After how many seconds the sensor will report back its measure (default: 10 minutes)
    void setReportIntervalSeconds(unsigned long value);
    // [16] After how many minutes the sensor will report back its measure (default: 10 minutes)
    void setReportIntervalMinutes(unsigned long value);
    // [19] After how many hours the sensor will report back its measure (default: 10 minutes)
    void setReportIntervalHours(unsigned int value);
    // [20] After how many days the sensor will report back its measure (default: 10 minutes)
    void setReportIntervalDays(uint8_t value);
	// [24] Set the way the timer used for reporting to the gateway should operate. It can be either TIME_INTERVAL (e.g. report every X seconds with the amount of time set with setReportTimerValue()), IMMEDIATELY (e.g. report at every cycle, useful for sensors like actuators which should report as soon as the value has changed), DO_NOT_REPORT (e.g. never report, useful for when there is no need to report, like a Display) and when NODEMANAGER_TIME is ON, EVERY_MINUTE/EVERY_HOUR/EVERY_DAY (e.g. to report the value set in the previous timeframe, useful for sensors reporting an accumulated value linked to a timeframe at regular intervals), AT_MINUTE/AT_HOUR/AT_DAY (e.g. report at a given minute/hour/day, useful if the measure is expected at a specified time, set with setReportTimerValue())
	void setReportTimerMode(timer_mode value);
	// [25] Set the value for the reporting timer's mode which has been set with setReportTimerMode()
	void setReportTimerValue(unsigned long value);
	// [26] Set the way the timer used for taking measures should operate. Takes the same parameters as setReportTimerMode(). If not set explicitly, will be set as the reporting timer
	void setMeasureTimerMode(timer_mode value);
	// [27] Set the value for the reporting timer's mode which has been set with setReportTimerMode() If not set explicitely, will be set with the same value as the reporting timer
	void setMeasureTimerValue(unsigned long value);
	// list of configured child
	List<Child*> children;
	// return the child object based on the provided child_id
	Child* getChild(uint8_t child_id);
	// register a child
	void registerChild(Child* child);
	// [28] enabler/disable the sensor (default: true)
	void setEnabled(bool value, bool just_set = false);
	bool getEnabled();
#if NODEMANAGER_INTERRUPTS == ON
	// return the pin the interrupt is attached to
	int8_t getInterruptPin();
	// set initial value of the configured pin. Can be used for internal pull up
	void setPinInitialValue(int8_t value);
	// for interrupt-based sensor, set the interrupt mode. Can be CHANGE, RISING, FALLING (default: CHANGE)
	void setInterruptMode(uint8_t value);
	// [22] for interrupt-based sensor, milliseconds to wait/sleep after the interrupt before reporting (default: 0)
	void setWaitAfterInterrupt(unsigned long value);
	// [23] for interrupt-based sensor, the value of the pin is checked and the interrupt ignored if RISING and not HIGH or FALLING and not LOW (default: true)
	void setInterruptStrict(bool value);
#endif
#if NODEMANAGER_POWER_MANAGER == ON
	// set a previously configured PowerManager to the sensor so to powering it up with custom pins
	void setPowerManager(PowerManager& powerManager);
	// to save battery the sensor can be optionally connected to two pins which will act as vcc and ground and activated on demand
	void setPowerPins(int8_t ground_pin, int8_t vcc_pin, unsigned long wait_time = 50);
	// [13] manually turn the power on
	void powerOn();
	// [14] manually turn the power off
	void powerOff();
#endif
#if NODEMANAGER_HOOKING == ON
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
	// define what to do at each stage of the sketch
	void presentation();
	void setup();
	void loop(MyMessage* message);
#if NODEMANAGER_INTERRUPTS == ON
	bool interrupt();
#endif
#if NODEMANAGER_RECEIVE == ON
	void receive(MyMessage* message);
#endif
	// abstract functions, subclasses may implement
	virtual void onSetup();
	virtual void onLoop(Child* child);
	virtual void onReceive(MyMessage* message);
	virtual void onInterrupt();
#if NODEMANAGER_OTA_CONFIGURATION == ON
	virtual void onOTAConfiguration(ConfigurationRequest* request);
#endif
~~~

### Child API

The following methods are available for all the child:
~~~c
	Child(Sensor* sensor, value_format format, uint8_t child_id, uint8_t presentation, uint8_t type, const char* description = "", const char* unit_prefix = "", , bool request_initial_value = false);
	// set child id used to communicate with the gateway/controller
	void setChildId(uint8_t value);
	uint8_t getChildId();
	// set sensor format
	void setFormat(value_format value);
	value_format getFormat();
	// set sensor presentation (default: S_CUSTOM)
	void setPresentation(uint8_t value);
	uint8_t getPresentation();
	// set sensor type (default: V_CUSTOM)
	void setType(uint8_t value);
	uint8_t getType();
	// set how many decimal digits to use (default: 2 for ChildFloat, 4 for ChildDouble)
	void setFloatPrecision(uint8_t value);
	// set sensor description
	void setDescription(const char* value);
	const char* getDescription();
	// configure the behavior of the child when setValue() is called multiple times. It can be NONE (ignore the previous values but the last one),  AVG (averages the values), SUM (sum up the values) (default: AVG)
	void setValueProcessing(child_processing value);
	// send the value to the gateway even if there have been no samples collected (default: false)
	void setSendWithoutValue(bool value);
	// set the value of the child
	void setValue(int value);
	void setValue(float value);
	void setValue(double value);
	void setValue(const char* value);
	// get the value of the child
	int getValueInt();
	float getValueFloat();
	double getValueDouble();
	const char* getValueString();
	// check if the value must be sended back to the controller
	bool valueReadyToSend();
	// send the current value to the gateway
	void sendValue(bool force = 0);
	// print the current value on a LCD display
	void print(Print& device);
	// reset all the counters
	void reset();
	// if set request the controller the initial value of this child (default: false)
	void setRequestInitialValue(bool value);
	bool getRequestInitialValue();
#if NODEMANAGER_CONDITIONAL_REPORT == ON
	// force to send an update after the configured number of minutes
	void setForceUpdateTimerValue(unsigned long value);
	// never report values below this threshold (default: -FLT_MAX)
	void setMinThreshold(float value);
	// never report values above this threshold (default: FLT_MAX)
	void setMaxThreshold(float value);
	// do not report values if too close to the previous one (default: 0)
	void setValueDelta(float value);
	// set when the last value is updated. Possible values are UPDATE_ALWAYS (at every cycle), UPDATE_ON_SEND (only after sending) (default: UPDATE_ON_SEND)
	void setUpdateLastValue(last_value_mode value);
	// get the last value of the child
	int getLastValueInt();
	float getLastValueFloat();
	double getLastValueDouble();
	const char* getLastValueString();
#endif
#if NODEMANAGER_EEPROM == ON
	// persist the child's value in EEPROM. The value will be saved at each update and loaded at boot time (default: false)
	void setPersistValue(bool value);
	bool getPersistValue();
	// load old value from EEPROM
	void loadValue();
	// load current value to EEPROM
	void saveValue();
#endif
~~~

### Built-in sensors API

Each sensor class may expose additional methods.

* SensorBattery
~~~c
	// [102] the expected vcc when the batter is fully discharged, used to calculate the percentage (default: 2.7)
	void setMinVoltage(float value);
	// [103] the expected vcc when the batter is fully charged, used to calculate the percentage (default: 3.3)
	void setMaxVoltage(float value);
	// [104] if true, the battery level will be evaluated by measuring the internal vcc without the need to connect any pin, if false the voltage divider methon will be used (default: true)
	void setBatteryInternalVcc(bool value);
	// [105] if setBatteryInternalVcc() is set to false, the analog pin to which the battery's vcc is attached (https://www.mysensors.org/build/battery) (default: -1)
	void setBatteryPin(int8_t value);
	// [106] if setBatteryInternalVcc() is set to false, the volts per bit ratio used to calculate the battery voltage (default: 0.003363075)
	void setBatteryVoltsPerBit(float value);
	// [107] change battery voltage calibration factor
	void setBatteryCalibrationFactor(float value);
	// if true call sendBatteryLevel() in addition to send the measured voltage back (default: true)
	void setSendBatteryLevel(bool value);
~~~

* SensorSignal
~~~c
    // [101] define which signal report to send. Possible values are SR_UPLINK_QUALITY, SR_TX_POWER_LEVEL, SR_TX_POWER_PERCENT, SR_TX_RSSI, SR_RX_RSSI, SR_TX_SNR, SR_RX_SNR (default: SR_RX_RSSI)
    void setSignalCommand(int value);
~~~

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

* SensorTMP102
~~~c
    /* Can be:
    Sensor_TMP102::RATE_SINGLE_SHOT (reads temperature only when requested) takes about 50 ms
    Sensor_TMP102::RATE_025HZ (reads temperature at 0.25Hz intervals) faster than single shot
    Sensor_TMP102::RATE_1HZ (reads temperature at 1Hz intervals)
    etc.
    */
    void setConversionRate(Sensor_TMP102::conversion_rate rate);
    // Set the TMP102 to extended mode (temperatures above 128*C)
    void setExtended(bool extended);
~~~

* SensorACS712
~~~c
	// [101] set how many mV are equivalent to 1 Amp. The value depends on the module (185 for 5A Module, 100 for 20A Module, 66 for 30A Module) (default: 185);
	void setmVPerAmp(int value);
	// [102] set ACS offset (default: 2500);
	void setOffset(int value);
	// [103] set AC Measurement mode
	void setACMode(bool value);
	// [104] set AC noise
	void setACNoise(int value);
	// [105] Adjust AC noise
	void computeACNoise();
~~~

* SensorDigitalInput
~~~c
	// Invert the value to report. E.g. report 1 if value is LOW, report 0 if HIGH (default: false)
	void setInvertValueToReport(bool value);
	// Set optional internal pull up/down
	void setInitialValue(int value);
~~~

* SensorDigitalOutput / SensorRelay / SensorLatchingRelay1Pin / SensorLatchingRelay2Pins
~~~c
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
    void setPinOff(int8_t value);
    // manually switch the output to the provided status (ON or OFF)
    void setStatus(int value);
    // toggle the status
    void toggleStatus(int value);
~~~

*  SensorInterrupt / SensorDoor / SensorMotion / SensorWaterLeak
~~~c
    // [105] Invert the value to report. E.g. if FALLING and value is LOW, report HIGH (default: false)
    void setInvertValueToReport(bool value);
#if NODEMANAGER_TIME == ON
    // [107] when keeping track of the time, trigger only after X consecutive interrupts within the same minute (default: 1)
    void setThreshold(int value);
#endif
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

*  SensorBH1750
~~~c
    // [101] set sensor reading mode, e.g. BH1750_ONE_TIME_HIGH_RES_MODE
    void setMode(uint8_t mode);
~~~

*  SensorBME280 / SensorBMP085 / SensorBMP280
~~~c
    // [101] define how many pressure samples to keep track of for calculating the forecast (default: 5)
    void setForecastSamplesCount(int value);
~~~

*  SensorBME280
~~~c
	// set custom sampling to the sensor
	void setSampling(Adafruit_BME280::sensor_mode mode, Adafruit_BME280::sensor_sampling tempSampling, Adafruit_BME280::sensor_sampling pressSampling, Adafruit_BME280::sensor_sampling humSampling, Adafruit_BME280::sensor_filter filter, Adafruit_BME280::standby_duration duration);
~~~

*  SensorSonoff
~~~c
    // [101] set the button's pin (default: 0)
    void setButtonPin(int8_t value);
    // [102] set the relay's pin (default: 12)
    void setRelayPin(int8_t value);
    // [103] set the led's pin (default: 13)
    void setLedPin(int8_t value);
~~~

* SensorHCSR04
~~~c
    // [103] Maximum distance we want to ping for (in centimeters) (default: 300)
    void setMaxDistance(int value);
    // [104] Report the measure even if is invalid (e.g. 0) (default: true)
    void setReportIfInvalid(bool value);
~~~

* SensorMQ
~~~c
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

* SensorPT100
~~~c
    // [101] set the voltageRef used to compare with analog measures
    void setVoltageRef(float value);
~~~

* SensorSDS011
~~~c
    // Sleep sensor after measurment. This powers down the fan but increases measurment time. (default: true)
    void setSleep(bool value);
~~~

* SensorDimmer
~~~c
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
~~~

* SensorPulseMeter / SensorRainGauge / SensorPowerMeter / SensorWaterMeter
~~~c
    // [102] set how many pulses for each unit (e.g. 1000 pulses for 1 kwh of power, 9 pulses for 1 mm of rain, etc.)
    void setPulseFactor(float value);
~~~

* DisplaySSD1306
~~~c
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
    void setHeaderFontSize(int fontsize);
    // [107] Invert display (black text on color background; use invert=false to revert)
    void invertDisplay(bool invert = true);
	// set a static caption text on header of the display
	void setCaption(const char* value);
	// print a static caption on top of the screen
	void printCaption(const char* value);
	// print the given string
	void print(const char* value);
	// print the given string and goes to the next line
	void println(const char* value);
	// print the value of the given child
	void printChild(Child* child);
	// clear the display
	void clear();
	// set the cursor to the given position
	void setCursor(int col,int row);
	// return the display object
	SSD1306AsciiAvrI2c* getDisplay();
~~~

* SensorChirp
~~~c
    // [101] set the soil moisture offset (default: 0)
    void setMoistureOffset(int value);
    // [102] set the soil moisture range (default: 0)
    void setMoistureRange(int value);
    // [103] return the soil moisture normalized (default: false)
    void setReturnMoistureNormalized(bool value);
    // [104] reverse the light value (default: true)
    void setReturnLightReversed(bool value);
~~~

* DisplayHD44780
~~~c
    // set i2c address (default: 0x38)
    void setI2CAddress(uint8_t i2caddress);
    // set the backlight (default: HIGH)
    void setBacklight(uint8_t value);
	// set a static caption text on header of the display
	void setCaption(const char* value);
	// print a static caption on top of the screen
	void printCaption(const char* value);
	// print the given string
	void print(const char* value);
	// print the given string and goes to the next line
	void println(const char* value);
	// print the value of the given child
	void printChild(Child* child);
	// clear the display
	void clear();
	// set the cursor to the given position
	void setCursor(int col,int row);
	// return the display object
	LiquidCrystal_I2C* getDisplay();
~~~

* SensorTTP
~~~c
    // set the passcode length. Passcode will be sent to the controller only after this number of digits have been pressed (default: 4)
    void setPasscodeLength(int value);
    // set the clock pin (default: 6)
    void setClockPin(int8_t value);
    // set the SDO pin (default: 5)
    void setSdoPin(int8_t value);
    // set the DV pin (default: 3)
    void setDvPin(int8_t value);
    // set the RST pin (default: 4)
    void setRstPin(int8_t value);
~~~

* SensorServo
~~~c
    // set the servo to the given percentage
    void setPercentage(int value);
~~~

* SensorNeopixel
~~~c
    // set how many NeoPixels are attached
    void setNumPixels(int value);
	// set default brightness
	void setDefaultBrightness(int value) {
    // format expected is:
    //<pixel_number from>-<pixel_number to>,<RGB color in a packed 24 bit format>
    //<pixel_number>,<RGB color in a packed 24 bit format>
    //<RGB color in a packed 24 bit format>
    void setColor(char* string);
	// brightness for all LEDs
	void setBrightness(int value)
~~~

* SensorFPM10A
~~~c
   // set the baud rate of the serial port for connecting to the sensor (default: 57600)
   void setBaudRate(uint32_t value);
   // set the password for connecting to the sensor (default: 0)
   void setPassword(uint32_t value);
   // [101] set the minimum confidence below which the match is not considered valid (default: 0)
   void setMinConfidence(uint16_t value);
   // [102] wait for a valid fingerprint for the given amount of seconds. Useful when battery powered (default: 0)
   void setWaitFingerForSeconds(int value);
	// return true if the fingerprint was recognized successfully, false otherwise. Useful when a hook function needs to act upon the result
	bool getFingerprintIsValid();
~~~

* SensorPH
~~~c
	// setting AnalogRefValue (default: 5.0)
	void setVoltageRef(float value);
	// setting the voltage value @ph = 7 (default: 2.52)
	void setPH7Voltage(float value);
	// setting the voltage value @ph = 4 (default: 3.04)
    void setPH4Voltage(float value);
~~~

* SensorPca9685W
~~~c
    // [101] set the effect to use for a smooth transition, can be one of SensorDimmer::EASE_LINEAR (0), SensorDimmer::EASE_INSINE (1), SensorDimmer::EASE_OUTSINE (2), SensorDimmer::EASE_INOUTSINE (3) (default: EASE_LINEAR)
    void setEasing(int value);
    // [102] the duration of entire the transition in seconds (default: 1)
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
~~~

* SensorPca9685Rgb
~~~c
    // [101] set the effect to use for a smooth transition, can be one of SensorDimmer::EASE_LINEAR (0), SensorDimmer::EASE_INSINE (1), SensorDimmer::EASE_OUTSINE (2), SensorDimmer::EASE_INOUTSINE (3) (default: EASE_LINEAR)
    void setEasing(int value);
    // [102] the duration of entire the transition in seconds (default: 1)
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
~~~

* SensorPca9685Rgbw
~~~c
    // [101] set the effect to use for a smooth transition, can be one of SensorDimmer::EASE_LINEAR (0), SensorDimmer::EASE_INSINE (1), SensorDimmer::EASE_OUTSINE (2), SensorDimmer::EASE_INOUTSINE (3) (default: EASE_LINEAR)
    void setEasing(int value);
    // [102] the duration of entire the transition in seconds (default: 1)
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
~~~

* SensorDSM501A
~~~c
    // [101] set the reference temperature for calculating PM1.0
    void setTemperature(int value);
~~~

* SensorPN532
~~~c
    // [101] wait for a valid card for the given amount of seconds. Useful when battery powered (default: 0)
	void setWaitCardForSeconds(int value);
	// return true if the card was recognized successfully, false otherwise. Useful when a hook function needs to act upon the result
	bool getCardIsValid();
~~~

* SensorCCS811
~~~c
    // [101] set the temperature for calibrating the sensor
	void setTemperature(float value);
	// Set to true if the board has a temperature sensor embedded that can be used for calibration (default: false)
	void setTemperatureSensor(bool value);
~~~

* SensorMPR121
~~~c
    // set the passcode length. Passcode will be sent to the controller only after this number of digits have been pressed (default: 4)
	void setPasscodeLength(int value);
	// [101] wait for a valid code for the given amount of seconds. Useful when battery powered (default: 0)
	void setWaitCodeForSeconds(int value);
	// return true if the code was recognized successfully, false otherwise. Useful when a hook function needs to act upon the result
	bool getCodeIsValid();
~~~

* SensorGSM
~~~c
	// set the baud rate of the serial port for connecting to the sensor (default: 115200)
	void setBaudRate(uint32_t value);
	// [101] set the recipient phone number
	void setRecipient(const char* value);
	// send the provided text via SMS to the configured recipient
	void sendSMS(const char* text);
~~~

* SensorMCP9808
~~~c
    // set I2C address (default: 0x18)
    void setI2CAddress(uint8_t i2caddress);
    // set temperature resolution (default: 3)
    void setResolution(uint8_t resolution);
    // sleep sensor after measurment (default: true)
    void setSleep(bool value);
~~~ 

### OTA Configuration

When `NODEMANAGER_OTA_CONFIGURATION` is set to ON the API presented above can be also called remotely through `SensorConfiguration`, which is automatically added to NodeManager. SensorConfiguration exposes by default child id 200 that can be used to interact with the service by sending `V_CUSTOM` type of messages and commands within the payload. For each `REQ` message, the node will respond with a `SET` message if successful.

Almost all the functions made available through the API can be called remotely. To do so, the payload must be in the format `<child_id>,<function_id>[,<value_to_set>]` where `child_id` is the recipient child id you want to communicate with (the node has child id 0), `function_id` is the number between square brackets you can find in the API documentation and, if the function takes and argument, this can be passed along in `value_to_set`.
For example, to change the sleep time to e.g. 10 minutes:
~~~c
    // [4] set the duration (in minutes) of a sleep cycle
	void setSleepMinutes(unsigned long value);
~~~
`<node_id>;<configuration_child_id>;<req>;0;<V_CUSTOM>;<child_id>,<function_id>,<value>`
`100;200;2;0;48;0,4,10`

To wake up a node previously configured as sleeping, send the following as the node wakes up next:
~~~c
    // [9] wake up the board
    void wakeup();
~~~
`100;200;2;0;48;0,9`

if you want to collect and average 10 samples for the sensor on child_id 1:
~~~c
    // [5] For some sensors, the measurement can be queried multiple times and an average is returned (default: 1)
    void setSamples(unsigned int value);
~~~
`100;200;2;0;48;1,5,10`

If you want to decrease the temperature offset of a thermistor sensor to -2:
~~~c
    // [105] set a temperature offset
    void setOffset(float value);
~~~
`100;200;2;0;48;1,105,-2`

Please note that anything set remotely will NOT persist a reboot apart from the sleep interval which is saved to the EEPROM if setSaveSleepSettings() is set.

## How it works

* Your sketch should begin with the standard MySensors directives
* NodeManager's settings can be customized through `NODEMANAGER_*` defines, just after. If not set, the default value will be used
* To make use of the NodeManager library you have to include it with `#include <MySensors_NodeManager.h>`. This automatically includes the required MySensors libraries and creates an object called `nodeManager`
* To add a sensor, just include the corresponding header file and creates an instance of the class
* Interaction with your code happens through callback functions, placed at the end of `before()`, `presentation()`, `loop()`, `receive()` and `receiveTimes()`.

The following is detailed what happens when the different callback functions are called:

`NodeManager::before()`:
* Setup the interrupt pins to wake up the board based on the configured interrupts
* Restore from the EEPROM the latest sleeping settings, if setSaveSleepSettings() was set
* Create and register an instance of SensorConfiguration

`NodeManager::presentation()`:
* Present the sketch
* Present each child of each of each registered sensors

`NodeManager::setup()`:
* Sync the time with the controller (if requested)
* Setup the SD card reader (if requested)
* Call `setup()` of each registered sensor
* Setup the interrupts as requested by the sensors

`Sensor::setup()`:
* Initialize the timers
* Call the sensor-specific implementation of setup by invoking `onSetup()`

`NodeManager::loop()`:
* Sync the time with the controller if not done recently
* If all the sensors are powered by a pin, this is turned on
* Call `loop()` of each of the registered sensor
* If all the sensors are powered by a pin, this is turned off
* Go to sleep (if requested)

`Sensor::loop()`:
* If it is time to take a new measure he sensor-specific `onLoop()` is called. If multiple samples are requested, this is run multiple times. `onLoop()` is not intended to send out any message but just sets a new value to the requested child
* If it is time to report, a message is sent to the gateway with the value. Depending on the configuration, this is not sent if it is the same as the previous value or sent anyway after a given number of cycles. These functionalities are not sensor-specific and common to all the sensors inheriting from the `Sensor` class.

`NodeManager::receive()`:
* Receive a message from the radio network
* Dispatch the message to the recipient sensor

`Sensor::receive()`:
* Invoke `Sensor::loop()` which will execute the sensor main task and eventually call `Sensor::onReceive()`

`Sensor::interrupt()`:
* Check if the value from the interrupt is matching the one expected
* Calls the sensor's implementation of `onInterrupt()` to handle the interrupt

Each sensor is in dedicated header file under the `sensors` directory of the library which has be directly included if needed into the main sketch. The implementation of the class is inline. Required libraries and OTA configuration request handling is everything happening inside the class.

## Contributing

Contributes to NodeManager are of course more than welcome.

### Reporting an issue or request an enhancement

For reporting an issue, requesting support for a new sensor or any other kind of enhancement, please drop a message either on the project's main page (<https://www.mysensors.org/download/node-manager>), on the MySensors Forum (<https://forum.mysensors.org/category/43/nodemanager>) or open an issue directly on Github (<https://github.com/mysensors/NodeManager/issues>).


### Contributing to the code

If you want to contribute to the code, a pull request on Github is the way to go. First of all setup your development environment:

* Create a copy of the project in your Github account by clicking on the "Fork" button on `https://github.com/mysensors/NodeManager`
* Check the copy actually exists on `https://github.com/<username>/NodeManager`
* Clone your repository on your computer: `git clone https://github.com/<username>/NodeManager.git`
* Configure the main project's repository as an upstream: `git remote add upstream https://github.com/mysensors/NodeManager.git`
* Create and switch to a local development branch: `git checkout -b development origin/development`

Before applying any change, always ensure you have the latest development version available:
* Switch to your local development branch: `git checkout development`
* Fetch the latest version from the main project's repository: `git fetch upstream`
* Merge into your development copy all the changes from the main repository: `git merge development upstream/development`
* Update the development branch of your repository: `git push origin development`

Create a branch for the fix/feature you want to work on and apply changes to the code:
* Create and switch to a new branch (give it a significant name, e.g. fix/enum_sensors): `git checkout -b <yourbranch>`
* Do any required change to the code
* Include all the files changed for your commit: `git add .`
* Commit the changes: `git  commit -m"Use enum instead of define for defining each sensor #121"`
* Push the branch with the changes to your repository: `git push origin <yourbranch>`
* A compilation test under Github Actions should trigger, wait for the results to ensure your changes are still making the code compiling successfully
* Visit `https://github.com/<username>/NodeManager/branches` and click the "New pull request" button just aside your newly created branch
* Fill in the request with a significant title and description and select the "development" branch (this step is very important)
* Submit the request and start the discussion. After a few seconds, the configured Continuous Integration tool will automatically compile your code and check for errors
* Any additional commits to your branch which will be presented within the same pull request
* When the pull request is merged, feel free delete your local branch: `git branch -D <yourbranch>`
* Update your local and remote development branch as per the instructions above

If there are changes introduced to the development branch that conflicts with an open pull request, you will have to resolve the conflicts and update the PR:
* Fetch and merge into development any change from upstream/development as detailed above
* Switch to your branch: `git checkout <yourbranch>`
* Rebase the branch you filed the PR from against your updated development branch: `git rebase development`
* Resolve the conflicts and commit again
* Force push your updated branch so the PR gets updated: `git push HEAD:<yourbranch> -f`

### Contributing with a new sensor

When contributing with a new sensor follows the same guidelines presented above and proceed with the following steps:
* Define your class is in a header file named `SensorNAME_OF_THE_SENSOR.h` under the `sensors` directory
* Implement your sensor inline with the class. See `SensorExample.h` or other sensors for more commented examples and details
* Add your sensor in `examples/Templates/Template.ino`, just after the last sensor. Ensure your lines are commented out, like the other sensors
* Add an additional step for the CI/CD pipeline configured in `.github/workflows/test.yml` for testing out your new code taking inspiration from other sensors. The code has to create a new sketch out of the template and uncomment the lines required for your new sensor to be tested
* Add the sensor's specs in "Add your sensors" and in "Built-in sensors API" of the README.md file
* Add the name of the class of your sensor in the keywords.txt file

## Compatibility

This version of NodeManager has been tested and is compatible with the following MySensors library:
* v2.3.2

You don't necessarily need a NodeManager gateway to interact with a NodeManager node. A NodeManager node is fully compatible with any existing gateway you are currently operating with.

There are generally speaking no compatibility issues in having in your network nodes running different versions of NodeManager.

Starting from v1.8 NodeManager is released as an Arduino library hence your code has to be migrated manually from previous versions.

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
* Introduced new remote API to allow calling almost ALL NodeManager's and its sensors' functions remotely
* Reporting interval configuration is now indipendent from the sleep cycle
* Reporting interval can be customized per-sensor
* All intervals (measure/battery reports) are now time-based
* Added support for BMP280 temperature and pressure sensor
* Added support for RS485 serial transport
* Added support for TSL2561 light sensor
* Added support for DHT21 temperature/humidity sensor
* Added support for AM2320 temperature/humidity sensor
* Added support for PT100 high temperature sensor
* Added support for MH-Z19 CO2 sensor
* Added support for analog rain and soil moisture sensors
* Added support for generic dimmer sensor (PWM output)
* Added support for power and water meter pulse sensors
* Radio signal level (RSSI) is now reported automatically like the battery level
* SensorRainGauge now supports sleep mode
* SensorSwitch now supports awake mode
* SensorLatchingRealy now handles automatically both on and off commands
* SensorMQ now depends on its own module
* Added safeguard (automatic off) to SensorDigitalOutput
* Any sensor can now access all NodeManager's functions
* DHT sensor now using MySensors' DHT library

v1.7:
* Reviewed the entire NodeManager's architecture with children now automatically created from within each sensor
* Optimized the code so to use the memory in a more efficient manner
* Improved the overall user experience, also with sensors' patterns in the main sketch
* Sensors can now be enabled by uncommenting the corresponding USE_* define and requiring a single line to be created and initialized
* NodeManager's advanced features can be enabled/disabled by setting the corresponding NODEMANAGER_* define
* Simplified the configuration of each sensor, now without the need of getting the sensor back through a nasty casting
* Merged config.h into the main sketch so to centralize the configuration in a single place
* Added time-aware capability, with or without an attached RTC
* Intra-sensor communication now possible with the possibility for the user to nicely hook into the sensor's code
* Batery and signal reports are now available through the regular sensors SensorBattery and SensorSignal
* Remote API interaction for all the sensors has been moved into the regular sensor SensorConfiguration
* Fixed bug preventing negative temperatures to be reported for all the sensors
* Added ability for each sensor to report only when value is above or below a configured threshold
* Addded support for SD card reader
* Added support for RFM95 radio
* Added supoport for MySensors Sensebender Gateway and Sensebender Micro boards
* Added support for generic LCD devices through an abstract Display class
* SensorDimmer now supports both V_STATUS and V_PERCENTAGE
* SensorPulseMeter now supports running on batteries
* SensorDs18B20 optimized and now supporting V_ID
* SensorSwitch (now renamed into SensorInterrupt) now catches interrupt in a more reliable way
* SensorLatchingRelay now specialized and renamed into SensorLatchingRelay1Pin and SensorLatchingRelay2Pins
* Added support for HD44780 i2c LCD
* Added support for MG996R Servo sensor
* Added support for VL53L0X laser time-of-flight distance sensor
* Added support for SensorPlantowerPMS particulate matter sensors
* Added support for SHT31 temperature and humidity sensor
* Added support for SI7021 temperature and humidity sensor
* Added support for for Neopixel LED
* Added support for Chirp Sensor soil moisture sensor
* Added support for SparkFun RGB and Gesture Sensor
* Added support for TTP226/TTP229 Touch control sensor

v1.8:
* Split NodeManager's core classes in individual files and each sensor code in its own dedicated header file
* New Arduino-compatible library structure to allow easier integration and more consistent updates across version
* Included a complete set of examples which can be loaded directly from the Arduino IDE
* Simplified the template sketch with a global nodeManager object and sensors that can be imported directly from there
* Debug output is now fully compatible with the one used by the MySensors library and integrated into MySensors LogParser
* Better control on how often, if and when to sync the time with the controller for time-aware nodes
* Added a Measure Timer so to allow splitting between taking measures and reporting
* Added support for every sensor to keep track of the last value assigned to a child in EEPROM and restoring it upon a reboot
* Introduced new capabilities for reporting every minute/hour/day or only at a given minute/hour/day
* Added ability to read from the serial port at the end of each loop cycle, useful for debugging interactive sensors
* Added support for pH sensor
* Added support for PCA9685 as RGB/RGBW/W dimmer
* Added support for DSM501A dust sensor
* Added support for PN532 NFC RFID module
* Added support for CCS811 CO2/VOC sensor
* Added support for MPR121 Capacitive Touch Sensor
* Added support for serial GSM/SMS device
* Added support for FPM10A fingerprint sensor
* Added support for SDS011 Air quality sensor
* Added support for ESP32 devices
* Added support for nRF52 radio
* Improved SensorDigitalInput and NeoPixelSensor
* Si7021 sensor is now using the library from the MySensors example
* Reviewed the MQ Sensor implementation
* Optimized memory utilization
* Added Travis Continuous Integration tests
* Fixed wrong battery report when using battery pin and SensorRain/SensorSoilMoisture
* Fixed DigitalOutput safeguard not working as expected
* Fixed radio signal level reporting wrong values
* Fixed SensorLatchingRelay2Pins wrong pin selection
* Other minor bug fixes
