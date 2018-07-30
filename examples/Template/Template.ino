/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2016 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************

 DESCRIPTION

NodeManager is intended to take care on your behalf of all those common tasks that a 
MySensors node has to accomplish, speeding up the development cycle of your projects. 
Consider it as a sort of frontend for your MySensors projects. When you need to add 
a sensor (which requires just uncommeting a single line),
NodeManager will take care of importing the required library, presenting the sensor 
to the gateway/controller, executing periodically the main function of the sensor 
(e.g. measure a temperature, detect a motion, etc.), allowing you to interact with 
the sensor and even configuring it remotely.

Documentation available on: https://github.com/mysensors/NodeManager
NodeManager provides built-in implementation of a number of sensors through ad-hoc 
classes. 

To use a buil-in sensor:
* Install the required library if any
* Enable the corresponding module (uncomment it) in the main sketch
* Declare the sensor (uncomment it) in the main sketch

Once created, the sensor will automatically present one or more child to the gateway 
and controller. A list of buil-in sensors, module to enable, required dependencies 
and the number of child automatically created is presented below:

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
SensorMQ                 | 1     | USE_MQ             | MQ sensor, return ppm of the target gas. Tuned by default for MQ135 and CO2                       | -
SensorMHZ19              | 1     | USE_MHZ19          | MH-Z19 CO2 sensor via UART (SoftwareSerial, default on pins 6(Rx) and 7(Tx)                       | -
SensorAM2320             | 2     | USE_AM2320         | AM2320 sensors, return temperature/humidity based on the attached AM2320 sensor                   | https://github.com/thakshak/AM2320
SensorTSL2561            | 1     | USE_TSL2561        | TSL2561 sensor, return light in lux                                                               | https://github.com/adafruit/TSL2561-Arduino-Library
SensorPT100              | 1     | USE_PT100          | DFRobot Driver high temperature sensor, return the temperature from the attached PT100 sensor     | https://github.com/nxcosa/HighTemperatureSensor
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
SensorFPM10A             | 1     | USE_FPM10A         | FPM10A fingerprint sensor                                                                         | https://github.com/adafruit/Adafruit-Fingerprint-Sensor-Library

NodeManager provides useful built-in features which can be disabled if you need 
to save some storage for your code. To enable/disable a buil-in feature:
* Install the required library if any
* Enable the corresponding feature by setting it to ON in the main sketch. To 
disable it, set it to OFF
* When a feature is enabled additional functions may be made available. Have a look 
at the API documentation for details

A list of buil-in features and the required dependencies is presented below:

Feature                     | Default | Description                                                                                      | Dependencies
----------------------------|---------|--------------------------------------------------------------------------------------------------|----------------------------------------------------------
NODEMANAGER_DEBUG               | ON      | NodeManager's debug output on serial console                                                     | - 
NODEMANAGER_POWER_MANAGER       | ON      | allow powering on your sensors only while the node is awake                                      | - 
NODEMANAGER_INTERRUPTS          | ON      | allow managing interrupt-based sensors like a PIR or a door sensor                               | - 
NODEMANAGER_CONDITIONAL_REPORT  | ON      | allow reporting a measure only when different from the previous or above/below a given threshold | - 
NODEMANAGER_EEPROM              | ON      | allow keeping track of some information in the EEPROM                                            | - 
NODEMANAGER_SLEEP               | ON      | allow managing automatically the complexity behind battery-powered sleeping sensors              | - 
NODEMANAGER_RECEIVE             | ON      | allow the node to receive messages; can be used by the remote API or for triggering the sensors  | - 
NODEMANAGER_TIME                | OFF     | allow keeping the current system time in sync with the controller                                | https://github.com/PaulStoffregen/Time
NODEMANAGER_RTC                 | OFF     | allow keeping the current system time in sync with an attached RTC device (requires NODEMANAGER_TIME)| https://github.com/JChristensen/DS3232RTC
NODEMANAGER_SD                  | OFF     | allow reading from and writing to SD cards                                                       | -
NODEMANAGER_HOOKING             | OFF     | allow custom code to be hooked in the out of the box sensors                                     | -
**/

/**********************************
 * MySensors node configuration
 */

// General settings
#define SKETCH_NAME "NodeManager"
#define SKETCH_VERSION "1.0"
//#define MY_DEBUG
//#define MY_NODE_ID 99

// NRF24 radio settings
#define MY_RADIO_NRF24
//#define MY_RF24_ENABLE_ENCRYPTION
//#define MY_RF24_CHANNEL 125
//#define MY_RF24_PA_LEVEL RF24_PA_HIGH
//#define MY_DEBUG_VERBOSE_RF24
//#define MY_RF24_DATARATE RF24_250KBPS

// RFM69 radio settings
//#define MY_RADIO_RFM69
//#define MY_RFM69_FREQUENCY RFM69_433MHZ
//#define MY_IS_RFM69HW
//#define MY_RFM69_NEW_DRIVER
//#define MY_RFM69_ENABLE_ENCRYPTION
//#define MY_RFM69_NETWORKID 100
//#define MY_DEBUG_VERBOSE_RFM69
//#define MY_RF69_IRQ_PIN D1
//#define MY_RF69_IRQ_NUM MY_RF69_IRQ_PIN
//#define MY_RF69_SPI_CS D2
//#define MY_RFM69_ATC_MODE_DISABLED

// RFM95 radio settings
//#define MY_RADIO_RFM95
//#define MY_RFM95_FREQUENCY (RFM95_868MHZ)
//#define MY_DEBUG_VERBOSE_RFM95
//#define MY_RFM95_MAX_POWER_LEVEL_DBM (20)
//#define MY_RFM95_IRQ_PIN D1
//#define MY_RFM95_IRQ_NUM MY_RFM95_IRQ_PIN
//#define MY_RFM95_CS_PIN D8

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
//#define MY_SIGNING_ATSHA204_PIN 4
//#define MY_SIGNING_REQUEST_SIGNATURES

// OTA Firmware update settings
//#define MY_OTA_FIRMWARE_FEATURE
//#define OTA_WAIT_PERIOD 300
//#define FIRMWARE_MAX_REQUESTS 2
//#define MY_OTA_RETRY 2

// OTA debug output
//#define MY_DEBUG_OTA (0)
//#define MY_OTA_LOG_SENDER_FEATURE
//#define MY_OTA_LOG_RECEIVER_FEATURE
//#define MY_DEBUG_OTA_DISABLE_ACK

// Advanced settings
#define MY_BAUD_RATE 9600
//#define MY_SMART_SLEEP_WAIT_DURATION_MS 500
#define MY_SPLASH_SCREEN_DISABLED
//#define MY_DISABLE_RAM_ROUTING_TABLE_FEATURE
//#define MY_SIGNAL_REPORT_ENABLED

// Optimizations when running on 2032 Coin Cell. Also set node.setSleepBetweenSend(500) and run the board at 1Mhz
//#define MY_TRANSPORT_UPLINK_CHECK_DISABLED
//#define MY_TRANSPORT_WAIT_READY_MS  5000
//#define MY_SLEEP_TRANSPORT_RECONNECT_TIMEOUT_MS 2000
//#define MY_PARENT_NODE_ID 0
//#define MY_PARENT_NODE_IS_STATIC

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

/***********************************
 * NodeManager configuration
 */

#define NODEMANAGER_DEBUG ON
#define NODEMANAGER_POWER_MANAGER OFF
#define NODEMANAGER_INTERRUPTS ON
#define NODEMANAGER_CONDITIONAL_REPORT OFF
#define NODEMANAGER_EEPROM OFF
#define NODEMANAGER_SLEEP ON
#define NODEMANAGER_RECEIVE ON
#define NODEMANAGER_TIME OFF
#define NODEMANAGER_RTC OFF
#define NODEMANAGER_SD OFF
#define NODEMANAGER_HOOKING OFF
#define NODEMANAGER_OTA_CONFIGURATION OFF

#include <MySensors_NodeManager.h>
NodeManager node;

/***********************************
 * Add your sensors
 */

//PowerManager power(5,6);
 
//#include <sensors/SensorBattery.h>
//SensorBattery battery(node);

//#include <sensors/SensorConfiguration.h>
//SensorConfiguration configuration(node);

//#include <sensors/SensorSignal.h>
//SensorSignal signal(node);

//#include <sensors/SensorAnalogInput.h>
//SensorAnalogInput analog(node,A0);

//#include <sensors/SensorLDR.h>
//SensorLDR ldr(node,A0);

//#include <sensors/SensorRain.h>
//SensorRain rain(node,A0);

//#include <sensors/SensorSoilMoisture.h>
//SensorSoilMoisture soil(node,A0);

//#include <sensors/SensorThermistor.h>
//SensorThermistor thermistor(node,A0);

//#include <sensors/SensorML8511.h>
//SensorML8511 ml8511(node,A0);

//#include <sensors/SensorACS712.h>
//SensorACS712 acs712(node,A0);

//#include <sensors/SensorDigitalInput.h>
//SensorDigitalInput digitalIn(node,6);

//#include <sensors/SensorDigitalOutput.h>
//SensorDigitalOutput digitalOut(node,6);

//#include <sensors/SensorRelay.h>
//SensorRelay relay(node,6);

//#include <sensors/SensorLatchingRelay1Pin.h>
//SensorLatchingRelay1Pin latching1pin(node,6);

//#include <sensors/SensorLatchingRelay2Pins.h>
//SensorLatchingRelay2Pins latching2pins(node,6,7);

//#include <sensors/SensorDHT11.h>
//SensorDHT11 dht11(node,6);

//#include <sensors/SensorDHT22.h>
//SensorDHT22 dht22(node,6);

//#include <sensors/SensorSHT21.h>
//SensorSHT21 sht21(node);

//#include <sensors/SensorHTU21D.h>
//SensorHTU21D htu21(node);

//#include <sensors/SensorInterrupt.h>
//SensorInterrupt interrupt(node,3);

//#include <sensors/SensorDoor.h>
//SensorDoor door(node,3);

//#include <sensors/SensorMotion.h>
//SensorMotion motion(node,3);

//#include <sensors/SensorDs18b20.h>
//SensorDs18b20 ds18b20(node,6);

//#include <sensors/SensorBH1750.h>
//SensorBH1750 bh1750(node);

//#include <sensors/SensorMLX90614.h>
//SensorMLX90614 mlx90614(node);

//#include <sensors/SensorBME280.h>
//SensorBME280 bme280(node);

//#include <sensors/SensorBMP085.h>
//SensorBMP085 bmp085(node);

//#include <sensors/SensorBMP180.h>
//SensorBMP180 bmp180(node);

//#include <sensors/SensorBMP280.h>
//SensorBMP280 bmp280(node);

//#include <sensors/SensorSonoff.h>
//SensorSonoff sonoff(node);

//#include <sensors/SensorHCSR04.h>
//SensorHCSR04 hcsr04(node,6,7);

//#include <sensors/SensorMCP9808.h>
//SensorMCP9808 mcp9808(node);

//#include <sensors/SensorMQ.h>
//SensorMQ mq(node,A0);

//#include <sensors/SensorMHZ19.h>
//SensorMHZ19 mhz19(node,6,7);

//#include <sensors/SensorAM2320.h>
//SensorAM2320 am2320(node);

//#include <sensors/SensorTSL2561.h>
//SensorTSL2561 tsl2561(node);

//#include <sensors/SensorPT100.h>
//SensorPT100 pt100(node,6);

//#include <sensors/SensorDimmer.h>
//SensorDimmer dimmer(node,3);

//#include <sensors/SensorRainGauge.h>
//SensorRainGauge rainGauge(node,3);

//#include <sensors/SensorPowerMeter.h>
//SensorPowerMeter powerMeter(node,3);

//#include <sensors/SensorWaterMeter.h>
//SensorWaterMeter waterMeter(node,3);

//#include <sensors/SensorPlantowerPMS.h>
//SensorPlantowerPMS pms(node,6,7);

//#include <sensors/SensorVL53L0X.h>
//SensorVL53L0X vl53l0x(node,3);

//#include <sensors/DisplaySSD1306.h>
//DisplaySSD1306 ssd1306(node);

//#include <sensors/SensorSHT31.h>
//SensorSHT31 sht31(node);

//#include <sensors/SensorSI7021.h>
//SensorSI7021 si7021(node);

//#include <sensors/SensorChirp.h>
//SensorChirp chirp(node);

//#include <sensors/DisplayHD44780.h>
//DisplayHD44780 hd44780(node);

//#include <sensors/SensorTTP.h>
//SensorTTP ttp(node);

//#include <sensors/SensorServo.h>
//SensorServo servo(node,6);

//#include <sensors/SensorAPDS9960.h>
//SensorAPDS9960 apds9960(node,3);

//#include <sensors/SensorNeopixel.h>
//SensorNeopixel neopixel(node,6);

//#include <sensors/SensorSDS011.h>
//SensorSDS011 sds011(node,6,7);

//#include <sensors/SensorFPM10A.h>
//SensorFPM10A fpm10a(node,4,5);

/***********************************
 * Main Sketch
 */

// before
void before() {
	
  /***********************************
   * Configure your sensors
   */
  // report measures of every attached sensors every 10 seconds
  //node.setReportIntervalSeconds(10);
  // report measures of every attached sensors every 10 minutes
  //node.setReportIntervalMinutes(10);
  // set the node to sleep in 30 seconds cycles
  //node.setSleepSeconds(30);
  // set the node to sleep in 5 minutes cycles
  //node.setSleepMinutes(5);
  // report battery level every 10 minutes
  //battery.setReportIntervalMinutes(10);
  // set an offset to -1 to a thermistor sensor
  //thermistor.setOffset(-1);
  // change the id of a the first child of a sht21 sensor
  //sht21.children.get(1)->setChildId(5);
  // report only when the analog value is above 40%
  //analog.children.get(1)->setMinThreshold(40);
  // power all the nodes through dedicated pins
  //node.setPowerManager(power);

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

#if NODEMANAGER_RECEIVE == ON
// receive
void receive(const MyMessage &message) {
  // call NodeManager receive routine
  node.receive(message);
}
#endif

#if NODEMANAGER_TIME == ON
// receiveTime
void receiveTime(unsigned long ts) {
  // call NodeManager receiveTime routine
  node.receiveTime(ts);
}
#endif
