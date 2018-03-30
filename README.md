# NodeManager

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
* Provide buil-in capabilities to handle interrupt-based sensors 

### Built-in sensors

NodeManager provides built-in implementation of a number of sensors through ad-hoc classes. 
To use a buil-in sensor:
* Install the required library if any
* Enable the corresponding module (uncomment it) in the main sketch
* Declare the sensor (uncomment it) in the main sketch

Once created, the sensor will automatically present one or more child to the gateway and controller.
A list of buil-in sensors, module to enable, required dependencies and the number of child automatically created is presented below:

Sensor Name              |#Child | Module to enable   | Description                                                                                       | Dependencies
-------------------------|-------|--------------------|---------------------------------------------------------------------------------------------------|----------------------------------------------------------
SensorBattery            | 1     | USE_BATTERY        | Built-in sensor for automatic battery reporting                                                   | - 
SensorSignal             | 1     | USE_SIGNAL         | Built-in sensor for automatic signal level reporting                                              | -
SensorConfiguration      | 1     | USE_CONFIGURATION  | Built-in sensor for OTA remote configuration of any registered sensor                             | -
SensorAnalogInput        | 1     | USE_ANALOG_INPUT   | Generic analog sensor, return a pin's analog value or its percentage                              | -
SensorLDR                | 1     | USE_ANALOG_INPUT   | LDR sensor, return the light level of an attached light resistor in percentage                    | -
SensorRain               | 1     | USE_ANALOG_INPUT   | Rain sensor, return the percentage of rain from an attached analog sensor                         | -
SensorSoilMoisture       | 1     | USE_ANALOG_INPUT   | Soil moisture sensor, return the percentage of moisture from an attached analog sensor            | -
SensorThermistor         | 1     | USE_THERMISTOR     | Thermistor sensor, return the temperature based on the attached thermistor                        | -
SensorML8511             | 1     | USE_ML8511         | ML8511 sensor, return UV intensity                                                                | -
SensorACS712             | 1     | USE_ACS712         | ACS712 sensor, measure the current going through the attached module                              | -
SensorDigitalInput       | 1     | USE_DIGITAL_INPUT  | Generic digital sensor, return a pin's digital value                                              | -
SensorDigitalOutput      | 1     | USE_DIGITAL_OUTPUT | Generic digital output sensor, allows setting the digital output of a pin to the requested value  | -
SensorRelay              | 1     | USE_DIGITAL_OUTPUT | Relay sensor, allows activating the relay                                                         | -
SensorLatchingRelay1Pin  | 1     | USE_DIGITAL_OUTPUT | Latching Relay sensor, allows toggling the relay with a pulse on the configured pin               | -
SensorLatchingRelay2Pins | 1     | USE_DIGITAL_OUTPUT | Latching Relay sensor, allows turing the relay on and off with a pulse on the configured pins     | -
SensorDHT11              | 2     | USE_DHT            | DHT11 sensor, return temperature/humidity based on the attached DHT sensor                        | https://github.com/mysensors/MySensorsArduinoExamples/tree/master/libraries/DHT
SensorDHT22              | 2     | USE_DHT            | DHT22 sensor, return temperature/humidity based on the attached DHT sensor                        | https://github.com/mysensors/MySensorsArduinoExamples/tree/master/libraries/DHT
SensorSHT21              | 2     | USE_SHT21          | SHT21 sensor, return temperature/humidity based on the attached SHT21 sensor                      | https://github.com/SodaqMoja/Sodaq_SHT2x
SensorHTU21D             | 2     | USE_SHT21          | HTU21D sensor, return temperature/humidity based on the attached HTU21D sensor                    | https://github.com/SodaqMoja/Sodaq_SHT2x
SensorInterrupt          | 1     | USE_INTERRUPT      | Generic interrupt-based sensor, wake up the board when a pin changes status                       | -
SensorDoor               | 1     | USE_INTERRUPT      | Door sensor, wake up the board and report when an attached magnetic sensor has been opened/closed | -
SensorMotion             | 1     | USE_INTERRUPT      | Motion sensor, wake up the board and report when an attached PIR has triggered                    | -
SensorDs18b20            | 1+    | USE_DS18B20        | DS18B20 sensor, return the temperature based on the attached sensor                               | https://github.com/milesburton/Arduino-Temperature-Control-Library
SensorBH1750             | 1     | USE_BH1750         | BH1750 sensor, return light level in lux                                                          | https://github.com/claws/BH1750
SensorMLX90614           | 2     | USE_MLX90614       | MLX90614 contactless temperature sensor, return ambient and object temperature                    | https://github.com/adafruit/Adafruit-MLX90614-Library
SensorBME280             | 4     | USE_BME280         | BME280 sensor, return temperature/humidity/pressure based on the attached BME280 sensor           | https://github.com/adafruit/Adafruit_BME280_Library
SensorBMP085             | 3     | USE_BMP085_180     | BMP085 sensor, return temperature and pressure                                                    | https://github.com/adafruit/Adafruit-BMP085-Library
SensorBMP180             | 3     | USE_BMP085_180     | BMP180 sensor, return temperature and pressure                                                    | https://github.com/adafruit/Adafruit-BMP085-Library
SensorBMP280             | 3     | USE_BMP280         | BMP280 sensor, return temperature/pressure based on the attached BMP280 sensor                    | https://github.com/adafruit/Adafruit_BMP280_Library
SensorSonoff             | 1     | USE_SONOFF         | Sonoff wireless smart switch                                                                      | https://github.com/thomasfredericks/Bounce2
SensorHCSR04             | 1     | USE_HCSR04         | HC-SR04 sensor, return the distance between the sensor and an object                              | https://github.com/mysensors/MySensorsArduinoExamples/tree/master/libraries/NewPing
SensorMCP9808            | 1     | USE_MCP9808        | MCP9808 sensor, measure the temperature through the attached module                               | https://github.com/adafruit/Adafruit_MCP9808_Library
SensorMQ                 | 1     | USE_MQ             | MQ sensor, return ppm of the target gas                                                           | -
SensorMHZ19              | 1     | USE_MHZ19          | MH-Z19 CO2 sensor via UART (SoftwareSerial, default on pins 6(Rx) and 7(Tx)                       | -
SensorAM2320             | 2     | USE_AM2320         | AM2320 sensors, return temperature/humidity based on the attached AM2320 sensor                   | https://github.com/thakshak/AM2320
SensorTSL2561            | 1     | USE_TSL2561        | TSL2561 sensor, return light in lux                                                               | https://github.com/adafruit/TSL2561-Arduino-Library
SensorPT100              | 1     | USE_PT100          | DFRobot Driver high temperature sensor, return the temperature from the attached PT100 sensor     | -
SensorDimmer             | 1     | USE_DIMMER         | Generic dimmer sensor used to drive a pwm output                                                  | -
SensorRainGauge          | 1     | USE_PULSE_METER    | Rain gauge sensor                                                                                 | -
SensorPowerMeter         | 1     | USE_PULSE_METER    | Power meter pulse sensor                                                                          | -
SensorWaterMeter         | 1     | USE_PULSE_METER    | Water meter pulse sensor                                                                          | -
SensorPlantowerPMS       | 3     | USE_PMS            | Plantower PMS particulate matter sensors (reporting PM<=1.0, PM<=2.5 and PM<=10.0 in µg/m³)       | https://github.com/fu-hsi/pms
SensorVL53L0X            | 1     | USE_VL53L0X        | VL53L0X laser time-of-flight distance sensor via I²C, sleep pin supported (optional)              | https://github.com/pololu/vl53l0x-arduino
DisplaySSD1306           | 1     | USE_SSD1306        | SSD1306 128x64 OLED display (I²C); By default displays values of all sensors and children         | https://github.com/greiman/SSD1306Ascii.git
SensorSHT31              | 2     | USE_SHT31          | SHT31 sensor, return temperature/humidity based on the attached SHT31 sensor                      | https://github.com/adafruit/Adafruit_SHT31
SensorSI7021             | 2     | USE_SI7021         | SI7021 sensor, return temperature/humidity based on the attached SI7021 sensor                    | https://github.com/sparkfun/SparkFun_Si701_Breakout_Arduino_Library
SensorChirp              | 3     | USE_CHIRP          | Chirp soil moisture sensor (includes temperature and light sensors)                               | https://github.com/Apollon77/I2CSoilMoistureSensor
DisplayHD44780           | 1     | USE_HD44780        | Supports most Hitachi HD44780 based LCDs, by default displays values of all sensors and children  | https://github.com/cyberang3l/NewLiquidCrystal
SensorTTP                | 1     | USE_TTP            | TTP226/TTP229 Touch control sensor                                                                | -
SensorServo              | 1     | USE_SERVO          | Control a generic Servo motor sensor                                                              | -
SensorAPDS9960           | 1     | USE_APDS9960       | SparkFun RGB and Gesture Sensor                                                                   | https://github.com/sparkfun/APDS-9960_RGB_and_Gesture_Sensor
SensorNeopixel           | 1     | USE_NEOPIXEL       | Control a Neopixel LED                                                                            | https://github.com/adafruit/Adafruit_NeoPixel
SensorSDS011             | 2     | USE_SDS011         | SDS011 air quality sensor, return concentrations of 2.5 and 10 micrometer particles.              | https://github.com/ricki-z/SDS011

### Built-in features

NodeManager built-in features can be enabled/disabled also when you need to save some storage for your code. 
To enable/disable a buil-in feature:
* Install the required library if any
* Enable the corresponding feature by setting it to ON in the main sketch. To disable it, set it to OFF
* When a feature is enabled additional functions may be made available. Have a look at the API documentation for details

A list of buil-in features and the required dependencies is presented below:

Feature                     | Default | Description                                                                                      | Dependencies
----------------------------|---------|--------------------------------------------------------------------------------------------------|----------------------------------------------------------
FEATURE_DEBUG               | ON      | NodeManager's debug output on serial console                                                     | - 
FEATURE_POWER_MANAGER       | OFF     | allow powering on your sensors only while the node is awake                                      | - 
FEATURE_INTERRUPTS          | ON      | allow managing interrupt-based sensors like a PIR or a door sensor                               | - 
FEATURE_CONDITIONAL_REPORT  | OFF     | allow reporting a measure only when different from the previous or above/below a given threshold | - 
FEATURE_EEPROM              | OFF     | allow keeping track of some information in the EEPROM                                            | - 
FEATURE_SLEEP               | ON      | allow managing automatically the complexity behind battery-powered sleeping sensors              | - 
FEATURE_RECEIVE             | ON      | allow the node to receive messages; can be used by the remote API or for triggering the sensors  | - 
FEATURE_TIME                | OFF     | allow keeping the current system time in sync with the controller                                | https://github.com/PaulStoffregen/Time
FEATURE_RTC                 | OFF     | allow keeping the current system time in sync with an attached RTC device (requires FEATURE_TIME)| https://github.com/JChristensen/DS3232RTC
FEATURE_SD                  | OFF     | allow reading from and writing to SD cards                                                       | -
FEATURE_HOOKING             | OFF     | allow custom code to be hooked in the out of the box sensors                                     | -

## Installation

* Download the package or clone the git repository from https://github.com/mysensors/NodeManager
* Open the NodeManager.ino sketch and save it under a different name
* Configure you sensors and upload the sketch to your arduino board

Please note NodeManager cannot be used as an arduino library since requires access to your MySensors configuration directives, hence its files have to be placed into the same directory of your sketch.

### Installing the dependencies

Some of the sensors rely on third party libraries. Those libraries are not included within NodeManager and have to be installed from the Arduino IDE Library Manager (Sketch -> Include Library -> Manager Libraries) or manually. 
You need to install the library ONLY if you are planning to enable to use the sensor.

### Upgrade

* Download the latest version of NodeManager
* Replace the NodeManagerLibrary.ino and NodeManagerLibrary.h of your project with those just downloaded
* Review the release notes in case there is any manual change required to the main sketch

Please be aware when upgrading to v1.7 from an older version this procedure is not supported and must be manual.

## Configuration

Configuring a sketch with is using NodeManager requires a few steps. All the configuration directives are located within the main sketch.

### MySensors configuration

Since NodeManager has to communicate with the MySensors network on your behalf, it has to know how to do it. On top of the main sketch you will find the typical MySensors directives you are used to which can be customized to configure the board to act as a MySensors node or a MySensors gateway. 
Please note you don't necessarily need a NodeManager gateway to interact with a NodeManager node. A NodeManager node is fully compatible with any existing gateway you are currently operating with.

### NodeManager configuration

The next step is to enable NodeManager's modules required for your sensors. When a module is enabled, the required library will be loaded and the corresponding sensor will be made available. To enable it, uncomment the line. Enabled only what you need to ensure enough storage is left for your custom code.

### Add your sensors

Find in the main sketch `Add your sensors below` and add your sensors to NodeManager. To add a sensor, just create an instance of the class, passing it `node` as an argument. 
Those sensors requiring a pin to operate would take it as a second argument in the constructor. 
NodeManager automatically creates all the child_ids assigning an incremental counter. If you need to set your own child_id, pass it as the last argument to the constructor

~~~c
// Add a thermistor sensor attached to pin A0
SensorThermistor thermistor(node,A0);
// Add a LDR sensor attached to pin A0 and assing child_id 5
SensorLDR ldr(node,A1,5);
// Add a temperature/humidity sensor SHT21 sensor. No pin required since using i2c
SensorSHT21 sht21(node);
~~~

The sensor will be then registered automatically with NodeManager which will take care of it all along its lifecycle. Please ensure the corresponding module has been previously enabled for a successful compilation of the code.
NodeManager will present each sensor for you to the controller, query each sensor and report the measure back to the gateway/controller. For actuators (e.g. relays) those can be triggered by sending a `REQ` message with the expected type to their assigned child id.

### Configure your sensors

NodeManager and all the sensors can be configured from within `before()` in the main sketch. Find `Configure your sensors below` to customize the behavior of any sensor by calling one of the functions available.

~~~c
// report measures of every attached sensors every 10 minutes
node.setReportIntervalMinutes(10);
// set the node to sleep in 5 minutes cycles
node.setSleepMinutes(5);
// report battery level every 10 minutes
battery.setReportIntervalMinutes(10);
// set an offset to -1 to a thermistor sensor
thermistor.setOffset(-1);
// Change the id of a the first child of a sht21 sensor
sht21.children.get(1)->child_id = 5;
// power all the nodes through dedicated pins
node.setPowerManager(power);
~~~

If not instructed differently, the node will stay awake and all the sensors will report every 10 minutes, battery level and signal level will be automatically reported every 60 minutes (if the corresponding sensors have been added). 

Please note, if you configure a sleep cycle, this may have an impact on the reporting interval since the sensor will be able to report its measures ONLY when awake. For example if you set a report interval of 5 minutes and a sleep cycle of 10 minutes, the sensors will report every 10 minutes.

## Running your code

Once finished configuring your node, upload your sketch to your arduino board as you are used to.

Check your gateway's logs to ensure the node is working as expected. You should see the node presenting itself, presenting all the registered sensors and reporting new measures at the configured reporting interval.
When `FEATURE_DEBUG` is enabled, detailed information will be available through the serial port. Remember to disable debug once the tests have been completed to save additional storage.

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
~~~

### Sensor API

The following methods are available for all the sensors:
~~~c
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
~~~

### Child API

The following methods are available for all the child:
~~~c
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
~~~

### Built-in sensors API

Each sensor class exposes additional methods.

* SensorBattery
~~~c
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

* SensorACS712
~~~c
    // [101] set how many mV are equivalent to 1 Amp. The value depends on the module (100 for 20A Module, 66 for 30A Module) (default: 185);
    void setmVPerAmp(int value);
    // [102] set ACS offset (default: 2500);
    void setOffset(int value);
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
    void setPinOff(int value);
    // manually switch the output to the provided status (ON or OFF)
    void setStatus(int value);
    // toggle the status
    void toggleStatus(int value);
~~~

*  SensorInterrupt / SensorDoor / SensorMotion
~~~c
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

*  SensorSonoff
~~~c
    // [101] set the button's pin (default: 0)
    void setButtonPin(int value);
    // [102] set the relay's pin (default: 12)
    void setRelayPin(int value);
    // [103] set the led's pin (default: 13)
    void setLedPin(int value);
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
    // Set initial value on the interrupt pin. Can be used for internal pull up (default: HIGH)
    void setInitialValue(int value);
    // set the interrupt mode. Can be CHANGE, RISING, FALLING (default: FALLING)
    void setInterruptMode(int value);
    // milliseconds to wait/sleep after the interrupt before reporting (default: 0)
    void setWaitAfterTrigger(int value);
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
~~~

* SensorTTP
~~~c
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
    // format expected is:
    //<pixel_number from>-<pixel_number to>,<RGB color in a packed 24 bit format>
    //<pixel_number>,<RGB color in a packed 24 bit format>
    //<RGB color in a packed 24 bit format>
    void setColor(char* string);
~~~

### Remote API

If SensorConfiguration is added to NodeManager, the API can be also called remotely. SensorConfiguration exposes child id 200 that can be used to interact with the service by sending `V_CUSTOM` type of messages and commands within the payload. For each `REQ` message, the node will respond with a `SET` message if successful. 

Almost all the functions made available through the API can be called remotely. To do so, the payload must be in the format `<child_id>,<function_id>[,<value_to_set>]` where `child_id` is the recipient child id you want to communicate with (the board has child id 0), `function_id` is the number between square brackets you can find in the API documentation and, if the function takes and argument, this can be passed along in `value_to_set`. 
For example, to change the sleep time to e.g. 10 minutes:
~~~c
    // [4] set the duration (in minutes) of a sleep cycle
    void setSleepMinutes(int value);
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
    void setSamples(int value);
~~~
`100;200;2;0;48;1,5,10`

If you want to decrease the temperature offset of a thermistor sensor to -2:
~~~c
    // [105] set a temperature offset
    void setOffset(float value);
~~~
`100;200;2;0;48;1,105,-2`

Please note that anything set remotely will NOT persist a reboot apart from the sleep interval which is saved to the EEPROM if setSaveSleepSettings() is set.

## Creating a new sensor

### NodeManager's internal architecture review

A NodeManager object is created for you at the beginning of your sketch and its main functions must be called from within `before()`, `presentation()`, `loop()` and `receive()` to work properly. NodeManager will do the following during each phase:

NodeManager::before():
* Setup the interrupt pins to wake up the board based on the configured interrupts
* Restore from the EEPROM the latest sleeping settings, if setSaveSleepSettings() was set
* Call `before()` of each registered sensor

Sensor::before():
* Call sensor-specific implementation of before by invoking `onBefore()` to initialize the sensor. 

NodeManager::setup():
* Call `setup()` of each registered sensor

Sensor::setup():
* Call sensor-specific implementation of setup by invoking `onSetup()` to initialize the sensor

NodeManager::loop():
* If all the sensors are powered by an arduino pin, this is turned on
* Call `loop()` of each registered sensor
* If all the sensors are powered by an arduino pin, this is turned off

Sensor::loop():
* If the sensor is powered by an arduino pin, this is set to on
* For each registered sensor, the sensor-specific `onLoop()` is called. If multiple samples are requested, this is run multiple times. `onLoop()` is not intended to send out any message but just sets a new value to the requested child
* A message is sent to the gateway with the value. Depending on the configuration, this is not sent if it is the same as the previous value or sent anyway after a given number of cycles. These functionalies are not sensor-specific and common to all the sensors inheriting from the `Sensor` class.
* If the sensor is powered by an arduino pin, this is turned off

NodeManager::receive():
* Receive a message from the radio network 
* Dispatch the message to the recipient sensor

Sensor::receive(): 
* Invoke `Sensor::loop()` which will execute the sensor main taks and eventually call `Sensor::onReceive()`

Sensor::interrupt():
* Calls the sensor's implementation of `onInterrupt()` to handle the interrupt

### Custom sensors

If you want to create a new sensor, you can create a new class inheriting from Sensor or other subclasses. The constructor is supposed to assign to assign the sensor a name through the `_name` variable. The following methods have to be implemented:
~~~c
    // define what to do during before(). Usually creates all the Child(ren) which belong to the sensor
    void onBefore();
	// define what to do during setup(). Usually initialize the required libraries
    void onSetup();
    // define what to do during loop() by executing the sensor's main task. Usually does a calculation and store the value to send back to the given Child class.
    void onLoop(Child* child);
    // define what to do during receive() when the sensor receives a message
    void onReceive(MyMessage* message);
    // define what to do when receiving an interrupt
    void onInterrupt();
~~~

If the sensor implements a remote API, this has to be made available in SensorConfiguration::onReceive.

## Examples

* Analog Light and Temperature Sensor
The following sketch can be used to report the temperature and the light level based on a thermistor and LDR sensors attached to two analog pins of the arduino board (A1 and A2). Both the thermistor and the LDR are connected to ground on one side and to vcc via a resistor on the other so to measure the voltage drop across each of them through the analog pins.

The sensor will be put to sleep after startup and will report both the measures every 10 minutes. NodeManager will take care of presenting the sensors, managing the sleep cycle, reporting the battery level every hour and report the measures in the appropriate format. 

Even if the sensor is sleeping most of the time, it can be potentially woke up by sending a V_CUSTOM message to NodeManager service child id (200 by default) just after having reported its heartbeat. At this point the node will report awake and the user can interact with it by e.g. sending REQ messages to its child IDs, changing the duration of a sleep cycle, etc.

~~~c

/**********************************
 * MySensors node configuration
 */

// General settings
#define SKETCH_NAME "LightTemperatureSensor"
#define SKETCH_VERSION "1.0"
#define MY_BAUD_RATE 9600
#define MY_NODE_ID 99

// NRF24 radio settings
#define MY_RADIO_NRF24

/***********************************
 * NodeManager modules
 */

#define USE_ANALOG_INPUT
#define USE_THERMISTOR

/***********************************
 * Load NodeManager Library
 */

// include NodeManager's library
#include "NodeManagerLibrary.h"
NodeManager node;

/***********************************
 * Add your sensors below
 */

SensorBattery battery(node);
SensorConfiguration configuration(node);

SensorLDR ldr(node,A1);
SensorThermistor thermistor(node,A2);

/***********************************
 * Main Sketch
 */

// before
void before() {
  // setup the serial port baud rate
  Serial.begin(MY_BAUD_RATE);

  /*
  * Configure your sensors below
  */

  node.setReportIntervalMinutes(10);
  node.setSleepMinutes(10);
  
  /*
  * Configure your sensors above
  */
  node.before();
}

// presentation
void presentation() {
  // call NodeManager presentation routine
  node.presentation();
}

// setup
void setup() {
  // call NodeManager setup routine
  node.setup();
}

// loop
void loop() {
  // call NodeManager loop routine
  node.loop();
}

#if FEATURE_RECEIVE == ON
// receive
void receive(const MyMessage &message) {
  // call NodeManager receive routine
  node.receive(message);
}
#endif

#if FEATURE_TIME == ON
// receiveTime
void receiveTime(unsigned long ts) {
  // call NodeManager receiveTime routine
  node.receiveTime(ts);
}
#endif
~~~

* Motion Sensor
The following sketch can be used to report back to the controller when a motion sensor attached to the board's pin 3 triggers. In this example, the board will be put to sleep just after startup and will report a heartbeat every hour. NodeManager will take care of configuring an interrupt associated to the provided pin so automatically wake up when a motion is detected and report a V_TRIPPED message back.

~~~c

/**********************************
 * MySensors node configuration
 */

// General settings
#define SKETCH_NAME "MotionSensor"
#define SKETCH_VERSION "1.0"
#define MY_BAUD_RATE 9600
#define MY_NODE_ID 99

// NRF24 radio settings
#define MY_RADIO_NRF24

/***********************************
 * NodeManager modules
 */

#define USE_INTERRUPT

/***********************************
 * Load NodeManager Library
 */

// include NodeManager's library
#include "NodeManagerLibrary.h"
NodeManager node;

/***********************************
 * Add your sensors below
 */

SensorMotion motion(node,3);

/***********************************
 * Main Sketch
 */

// before
void before() {
  // setup the serial port baud rate
  Serial.begin(MY_BAUD_RATE);

  /*
  * Configure your sensors below
  */

  node.setSleepMinutes(60);
  
  /*
  * Configure your sensors above
  */
  node.before();
}

// presentation
void presentation() {
  // call NodeManager presentation routine
  node.presentation();
}

// setup
void setup() {
  // call NodeManager setup routine
  node.setup();
}

// loop
void loop() {
  // call NodeManager loop routine
  node.loop();
}

#if FEATURE_RECEIVE == ON
// receive
void receive(const MyMessage &message) {
  // call NodeManager receive routine
  node.receive(message);
}
#endif

#if FEATURE_TIME == ON
// receiveTime
void receiveTime(unsigned long ts) {
  // call NodeManager receiveTime routine
  node.receiveTime(ts);
}
#endif
~~~

* Boiler Sensor

The following sketch controls a latching relay connected to a boiler. A latching relay (requiring only a pulse to switch) has been chosen to minimize the power consumption required by a traditional relay to stay on. This relay has normally two pins, one for closing and the other for opening the controlled circuit, connected to pin 6 (off) and 7 (on) of the arduino board. Since using a SensorLatchingRelay2Pins type of sensor, NodeManager will automatically consider the provided pin as the ON pin and the one just after as the OFF pin and will take care of just sending out a single pulse only when a SET command of type V_STATUS is sent to the child id. The appropriate pin will be then used.

In this example, the board also runs at 1Mhz so it can go down to 1.8V: by setting setBatteryMin() and setBatteryMax(), the battery percentage will be calculated and reported (by default, automatically every hour) based on these custom boundaries.

The board will be put to sleep just after startup and will report back to the controller every 5 minutes. It is the controller's responsability to catch when the board reports its heartbeat (using smart sleep behind the scene) and send a command back if needed.

~~~c

/**********************************
 * MySensors node configuration
 */

// General settings
#define SKETCH_NAME "BoilerSensor"
#define SKETCH_VERSION "1.0"
#define MY_BAUD_RATE 9600
#define MY_NODE_ID 99

// NRF24 radio settings
#define MY_RADIO_NRF24

/***********************************
 * NodeManager modules
 */

#define USE_DIGITAL_OUTPUT

/***********************************
 * Load NodeManager Library
 */

// include NodeManager's library
#include "NodeManagerLibrary.h"
NodeManager node;

/***********************************
 * Add your sensors below
 */

SensorBattery battery(node);
SensorLatchingRelay2Pin latching2pins(node,6,7);

/***********************************
 * Main Sketch
 */

// before
void before() {
  // setup the serial port baud rate
  Serial.begin(MY_BAUD_RATE);

  /*
  * Configure your sensors below
  */

  node.setSleepMinutes(5);
  
  battery.setBatteryMin(1.8);
  battery.setBatteryMax(3.2);
  
  /*
  * Configure your sensors above
  */
  node.before();
}

// presentation
void presentation() {
  // call NodeManager presentation routine
  node.presentation();
}

// setup
void setup() {
  // call NodeManager setup routine
  node.setup();
}

// loop
void loop() {
  // call NodeManager loop routine
  node.loop();
}

#if FEATURE_RECEIVE == ON
// receive
void receive(const MyMessage &message) {
  // call NodeManager receive routine
  node.receive(message);
}
#endif

#if FEATURE_TIME == ON
// receiveTime
void receiveTime(unsigned long ts) {
  // call NodeManager receiveTime routine
  node.receiveTime(ts);
}
#endif
~~~

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

## Compatibility

This version of NodeManager has been tested and is compatibile with the following MySensors library:
* v2.1.1
* v2.2.0-beta
* v2.2.0

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
* NodeManager's advanced features can be enabled/disabled by setting the corresponding FEATURE_* define
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
