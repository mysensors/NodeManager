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

NodeManager is intended to take care on your behalf of all those common tasks a MySensors node has to accomplish, 
speeding up the development cycle of your projects.
Documentation available on: https://github.com/mysensors/NodeManager
NodeManager provides built-in implementation of a number of sensors through ad-hoc classes. 

To use a buil-in sensor:
* Install the required library if any
* Enable the corresponding module (uncomment it) in the main sketch
* Declare the sensor (uncomment it) in the main sketch

Once created, the sensor will automatically present one or more child to the gateway and controller.
A list of buil-in sensors, module to enable, required dependencies and the number of child automatically created is presented below:

Sensor Name         |#Child | Module to enable   | Description                                                                                       | Dependencies
--------------------|-------|--------------------|---------------------------------------------------------------------------------------------------|----------------------------------------------------------
SensorBattery       | 1     | -                  | Built-in sensor for automatic battery reporting                                                   | - 
SensorSignal        | 1     | -                  | Built-in sensor for automatic signal level reporting                                              | -
SensorConfiguration | 1     | -                  | Built-in sensor for OTA remote configuration of any registered sensor                             | -
SensorAnalogInput   | 1     | USE_ANALOG_INPUT   | Generic analog sensor, return a pin's analog value or its percentage                              | -
SensorLDR           | 1     | USE_ANALOG_INPUT   | LDR sensor, return the light level of an attached light resistor in percentage                    | -
SensorRain          | 1     | USE_ANALOG_INPUT   | Rain sensor, return the percentage of rain from an attached analog sensor                         | -
SensorSoilMoisture  | 1     | USE_ANALOG_INPUT   | Soil moisture sensor, return the percentage of moisture from an attached analog sensor            | -
SensorThermistor    | 1     | USE_THERMISTOR     | Thermistor sensor, return the temperature based on the attached thermistor                        | -
SensorML8511        | 1     | USE_ML8511         | ML8511 sensor, return UV intensity                                                                | -
SensorACS712        | 1     | USE_ACS712         | ACS712 sensor, measure the current going through the attached module                              | -
SensorDigitalInput  | 1     | USE_DIGITAL_INPUT  | Generic digital sensor, return a pin's digital value                                              | -
SensorDigitalOutput | 1     | USE_DIGITAL_OUTPUT | Generic digital output sensor, allows setting the digital output of a pin to the requested value  | -
SensorRelay         | 1     | USE_DIGITAL_OUTPUT | Relay sensor, allows activating the relay                                                         | -
SensorLatchingRelay | 1     | USE_DIGITAL_OUTPUT | Latching Relay sensor, allows activating the relay with a pulse                                   | -
SensorDHT11         | 2     | USE_DHT            | DHT11 sensor, return temperature/humidity based on the attached DHT sensor                        | https://github.com/mysensors/MySensorsArduinoExamples/tree/master/libraries/DHT
SensorDHT22         | 2     | USE_DHT            | DHT22 sensor, return temperature/humidity based on the attached DHT sensor                        | https://github.com/mysensors/MySensorsArduinoExamples/tree/master/libraries/DHT
SensorSHT21         | 2     | USE_SHT21          | SHT21 sensor, return temperature/humidity based on the attached SHT21 sensor                      | https://github.com/SodaqMoja/Sodaq_SHT2x
SensorHTU21D        | 2     | USE_SHT21          | HTU21D sensor, return temperature/humidity based on the attached HTU21D sensor                    | https://github.com/SodaqMoja/Sodaq_SHT2x
SensorSwitch        | 1     | USE_SWITCH         | Generic switch, wake up the board when a pin changes status                                       | -
SensorDoor          | 1     | USE_SWITCH         | Door sensor, wake up the board and report when an attached magnetic sensor has been opened/closed | -
SensorMotion        | 1     | USE_SWITCH         | Motion sensor, wake up the board and report when an attached PIR has triggered                    | -
SensorDs18b20       | 1+    | USE_DS18B20        | DS18B20 sensor, return the temperature based on the attached sensor                               | https://github.com/milesburton/Arduino-Temperature-Control-Library
SensorBH1750        | 1     | USE_BH1750         | BH1750 sensor, return light level in lux                                                          | https://github.com/claws/BH1750
SensorMLX90614      | 2     | USE_MLX90614       | MLX90614 contactless temperature sensor, return ambient and object temperature                    | https://github.com/adafruit/Adafruit-MLX90614-Library
SensorBME280        | 4     | USE_BME280         | BME280 sensor, return temperature/humidity/pressure based on the attached BME280 sensor           | https://github.com/adafruit/Adafruit_BME280_Library
SensorBMP085        | 3     | USE_BMP085         | BMP085/BMP180 sensor, return temperature and pressure                                             | https://github.com/adafruit/Adafruit-BMP085-Library
SensorBMP280        | 3     | USE_BMP280         | BMP280 sensor, return temperature/pressure based on the attached BMP280 sensor                    | https://github.com/adafruit/Adafruit_BMP280_Library
SensorSonoff        | 1     | USE_SONOFF         | Sonoff wireless smart switch                                                                      | https://github.com/thomasfredericks/Bounce2
SensorHCSR04        | 1     | USE_HCSR04         | HC-SR04 sensor, return the distance between the sensor and an object                              | https://github.com/mysensors/MySensorsArduinoExamples/tree/master/libraries/NewPing
SensorMCP9808       | 1     | USE_MCP9808        | MCP9808 sensor, measure the temperature through the attached module                               | https://github.com/adafruit/Adafruit_MCP9808_Library
SensorMQ            | 1     | USE_MQ             | MQ sensor, return ppm of the target gas                                                           | -
SensorMHZ19         | 1     | USE_MHZ19          | MH-Z19 CO2 sensor via UART (SoftwareSerial, default on pins 6(Rx) and 7(Tx)                       | -
SensorAM2320        | 2     | USE_AM2320         | AM2320 sensors, return temperature/humidity based on the attached AM2320 sensor                   | https://github.com/thakshak/AM2320
SensorTSL2561       | 1     | USE_TSL2561        | TSL2561 sensor, return light in lux                                                               | https://github.com/adafruit/TSL2561-Arduino-Library
SensorPT100         | 1     | USE_PT100          | DFRobot Driver high temperature sensor, return the temperature from the attached PT100 sensor     | -
SensorDimmer        | 1     | USE_DIMMER         | Generic dimmer sensor used to drive a pwm output                                                  | -
SensorRainGauge     | 1     | USE_PULSE_METER    | Rain gauge sensor                                                                                 | -
SensorPowerMeter    | 1     | USE_PULSE_METER    | Power meter pulse sensor                                                                          | -
SensorWaterMeter    | 1     | USE_PULSE_METER    | Water meter pulse sensor                                                                          | -
SensorPlantowerPMS  | 3     | USE_PMS            | Plantower PMS particulate matter sensors (reporting PM<=1.0, PM<=2.5 and PM<=10.0 in µg/m³)       | https://github.com/fu-hsi/pms
SensorVL53L0X       | 1     | USE_VL53L0X        | VL53L0X laser time-of-flight distance sensor via I²C, sleep pin supported (optional)              | https://github.com/pololu/vl53l0x-arduino
DisplaySSD1306      | 1     | USE_SSD1306        | SSD1306 128x64 OLED display (I²C); By default displays values of all sensors and children         | https://github.com/greiman/SSD1306Ascii.git
SensorSHT31         | 2     | USE_SHT31          | SHT31 sensor, return temperature/humidity based on the attached SHT31 sensor                      | https://github.com/adafruit/Adafruit_SHT31

/**********************************
 * MySensors node configuration
 */

// General settings
#define SKETCH_NAME "NodeManager"
#define SKETCH_VERSION "1.0"
//#define MY_DEBUG
#define MY_NODE_ID 99

// NRF24 radio settings
#define MY_RADIO_NRF24
//#define MY_RF24_ENABLE_ENCRYPTION
//#define MY_RF24_CHANNEL 125
//#define MY_RF24_PA_LEVEL RF24_PA_HIGH
//#define MY_DEBUG_VERBOSE_RF24
//#define MY_RF24_DATARATE RF24_250KBPS

// RFM69 radio settings
//#define MY_RADIO_RFM69
//#define MY_RFM69_FREQUENCY RF69_868MHZ
//#define MY_RFM69_FREQUENCY RFM69_868MHZ
//#define MY_IS_RFM69HW
//#define MY_RFM69_NEW_DRIVER
//#define MY_RFM69_ENABLE_ENCRYPTION
//#define MY_RFM69_NETWORKID 100
//#define MY_DEBUG_VERBOSE_RFM69
//#define MY_RF69_IRQ_PIN D1
//#define MY_RF69_IRQ_NUM MY_RF69_IRQ_PIN
//#define MY_RF69_SPI_CS D2
//#define MY_RFM69_ATC_MODE_DISABLED

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

// Advanced settings
#define MY_BAUD_RATE 9600
//#define MY_SMART_SLEEP_WAIT_DURATION_MS 500
#define MY_SPLASH_SCREEN_DISABLED
//#define MY_DISABLE_RAM_ROUTING_TABLE_FEATURE
//#define MY_DISABLE_SIGNAL_REPORT

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
 * NodeManager modules
 */

//#define USE_ANALOG_INPUT
//#define USE_THERMISTOR
//#define USE_ML8511
//#define USE_ACS712
//#define USE_DIGITAL_INPUT
//#define USE_DIGITAL_OUTPUT
//#define USE_DHT
//#define USE_SHT21
//#define USE_SWITCH
//#define USE_DS18B20
//#define USE_BH1750
//#define USE_MLX90614
//#define USE_BME280
//#define USE_BMP085
//#define USE_BMP280
//#define USE_SONOFF
//#define USE_HCSR04
//#define USE_MCP9808
//#define USE_MQ
//#define USE_MHZ19
//#define USE_AM2320
//#define USE_TSL2561
//#define USE_PT100
//#define USE_DIMMER
//#define USE_PULSE_METER
//#define USE_PMS
//#define USE_VL53L0X
//#define USE_SSD1306
//#define USE_SHT31

/***********************************
 * NodeManager advanced settings
 */

#define NODEMANAGER_DEBUG

//#define DISABLE_POWER_MANAGER
//#define DISABLE_INTERRUPTS
//#define DISABLE_TRACK_LAST_VALUE
//#define DISABLE_EEPROM
//#define DISABLE_SLEEP
#define ENABLE_TIME

/***********************************
 * Load NodeManager Library
 */

#include "NodeManagerLibrary.h"
NodeManager node;

/***********************************
 * Add your sensors below
 */

// built-in sensors
//SensorBattery battery(node);
//SensorConfiguration configuration(node);
//SensorSignal signal(node);
//PowerManager power(5,6);

// Attached sensors
//SensorAnalogInput analog(node,A0);
//SensorLDR ldr(node,A0);
//SensorRain rain(node,A0);
//SensorSoilMoisture soil(node,A0);
//SensorThermistor thermistor(node,A0);
//SensorML8511 ml8511(node,A0);
//SensorACS712 acs712(node,A0);
//SensorDigitalInput digitalIn(node,6);
//SensorDigitalOutput digitalOut(node,6);
//SensorRelay relay(node,6);
//SensorLatchingRelay latching(node,6);
//SensorDHT11 dht11(node,6);
//SensorDHT22 dht22(node,6);
//SensorSHT21 sht21(node);
//SensorHTU21D htu21(node);
//SensorSwitch sensorSwitch(node,3);
//SensorDoor door(node,3);
//SensorMotion motion(node,3);
//SensorDs18b20 ds18b20(node,6);
//SensorBH1750 bh1750(node);
//SensorMLX90614 mlx90614(node);
//SensorBME280 bme280(node);
//SensorBMP085 bmp085(node);
//SensorBMP280 bmp280(node);
//SensorSonoff sonoff(node);
//SensorHCSR04 hcsr04(node,6);
//SensorMCP9808 mcp9808(node);
//SensorMQ mq(node,A0);
//SensorMHZ19 mhz19(node,6,7);
//SensorAM2320 am2320(node);
//SensorTSL2561 tsl2561(node);
//SensorPT100 pt100(node,6);
//SensorDimmer dimmer(node,A0);
//SensorRainGauge rainGauge(node,3);
//SensorPowerMeter powerMeter(node,3);
//SensorWaterMeter waterMeter(node,3);
//SensorPlantowerPMS pms(node,6,7);
//SensorVL53L0X vl53l0x(node, /*XSHUT_PIN=*/2);
//SensorSHT31 sht31(node);

// Other devices
//DisplaySSD1306 ssd1306(node);

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
  //node.setReportIntervalSeconds(10);
  //node.setReportIntervalMinutes(5);
  //node.setSleepMinutes(5);
  //node.setSleepSeconds(30);
  //setTime(1516543198);
/*
  RTC.set(1516543198);
  //setSyncProvider(RTC.get);
  setSyncInterval(0);
  */
  //node.setPowerManager(power);
  //battery.setReportIntervalMinutes(30);
  //sht21.children.get(1)->child_id = 5;
  
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
  /*
    tmElements_t tm;
  RTC.read(tm);
  Serial.print(tm.Day);
  Serial.print("-");
  Serial.print(tm.Month);
  Serial.print("-");
  Serial.print(tmYearToCalendar(tm.Year)-2000);
  Serial.print(" ");
  Serial.print(tm.Hour);
  Serial.print(":");
  Serial.print(tm.Minute);
  Serial.print(":");
  Serial.print(tm.Second);
  Serial.println("");
  
   Serial.println(now());
  Serial.println(millis());
    // digital clock display of the time
    Serial.print(hour());
    Serial.print(' ');
    Serial.print(minute());
    Serial.print(' ');
    Serial.print(second());
    Serial.print(' ');
    Serial.print(day());
    Serial.print(' ');
    Serial.print(month());
    Serial.print(' ');
    Serial.print(year()); 
    Serial.println(); 
    Serial.println(); 
*/
    
}

// receive
void receive(MyMessage &message) {
  // call NodeManager receive routine
  node.receive(message);
}

// receiveTime
void receiveTime(unsigned long ts) {
  // call NodeManager receiveTime routine
  node.receiveTime(ts);
}
