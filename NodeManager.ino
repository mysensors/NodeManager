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
 *
 * DESCRIPTION
 *
 * NodeManager is intended to take care on your behalf of all those common tasks a MySensors node has to accomplish, 
 * speeding up the development cycle of your projects.
 * Documentation available on: https://github.com/mysensors/NodeManager
*/
 
#define SKETCH_NAME "NodeManager"
#define SKETCH_VERSION "1.0"


/**********************************
 * MySensors node configuration
 */

// General settings
#define MY_BAUD_RATE 9600
//#define MY_DEBUG
#define MY_NODE_ID 99
//#define MY_SMART_SLEEP_WAIT_DURATION_MS 500
#define MY_SPLASH_SCREEN_DISABLED

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

// if enabled, enable debug messages on serial port
#define DEBUG 1

// Enable this module to use one of the following sensors: SensorAnalogInput, SensorLDR, SensorRain, SensorSoilMoisture
#define MODULE_ANALOG_INPUT 0
// Enable this module to use one of the following sensors: SensorThermistor
#define MODULE_THERMISTOR 0
// Enable this module to use one of the following sensors: SensorML8511
#define MODULE_ML8511 0
// Enable this module to use one of the following sensors: SensorACS712
#define MODULE_ACS712 0
// Enable this module to use one of the following sensors: SensorDigitalInput
#define MODULE_DIGITAL_INPUT 0
// Enable this module to use one of the following sensors: SensorDigitalOutput, SensorRelay, SensorLatchingRelay
#define MODULE_DIGITAL_OUTPUT 0
// Enable this module to use one of the following sensors: SensorDHT11, SensorDHT22
#define MODULE_DHT 0
// Enable this module to use one of the following sensors: SensorSHT21, SensorHTU21D
#define MODULE_SHT21 0
// Enable this module to use one of the following sensors: SensorSwitch, SensorDoor, SensorMotion
#define MODULE_SWITCH 0
// Enable this module to use one of the following sensors: SensorDs18b20
#define MODULE_DS18B20 0
// Enable this module to use one of the following sensors: SensorBH1750
#define MODULE_BH1750 0
// Enable this module to use one of the following sensors: SensorMLX90614
#define MODULE_MLX90614 0
// Enable this module to use one of the following sensors: SensorBME280
#define MODULE_BME280 0
// Enable this module to use one of the following sensors: SensorBMP085
#define MODULE_BMP085 0
// Enable this module to use one of the following sensors: SensorBMP280
#define MODULE_BMP280 0
// Enable this module to use one of the following sensors: SensorSonoff
#define MODULE_SONOFF 0
// Enable this module to use one of the following sensors: SensorHCSR04
#define MODULE_HCSR04 0
// Enable this module to use one of the following sensors: SensorMCP9808
#define MODULE_MCP9808 0
// Enable this module to use one of the following sensors: SensorMQ
#define MODULE_MQ 0
// Enable this module to use one of the following sensors: SensorMHZ19
#define MODULE_MHZ19 0
// Enable this module to use one of the following sensors: SensorAM2320    
#define MODULE_AM2320 0
// Enable this module to use one of the following sensors: SensorTSL2561    
#define MODULE_TSL2561 0
// Enable this module to use one of the following sensors: SensorPT100
#define MODULE_PT100 0
// Enable this module to use one of the following sensors: SensorDimmer
#define MODULE_DIMMER 0
// Enable this module to use one of the following sensors: SensorRainGauge, SensorPowerMeter, SensorWaterMeter
#define MODULE_PULSE_METER 1

/***********************************
 * Load NodeManager Library
 */
 
#include "NodeManagerLibrary.h"
NodeManager node;

/***********************************
 * Add your sensors below
 */

//SensorBattery battery(node);
SensorConfiguration configuration(node);
//SensorSignal signal(node);
//PowerManager power(5,6);

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
//SensorDs18b20 ds18b20(node,4);
//SensorBH1750 bh1750(node);
//SensorMLX90614 mlx90614(node);
//SensorBME280 bme280(node);
//SensorBMP085 bmp085(node);
//SensorBMP280 bmp280(node);
//SensorSonoff sonoff(node);
//SensorHCSR04 hcsr04(node,4);
//SensorMCP9808 mcp9808(node);
//SensorMQ mq(node,A0);
//SensorMHZ19 mhz19(node,6);
//SensorAM2320 am2320(node);
//SensorTSL2561 tsl2561(node);
//SensorPT100 pt100(node,4);
//SensorDimmer dimmer(node,A0);
//SensorRainGauge rainGauge(node,3);
//SensorPowerMeter powerMeter(node,3);
SensorWaterMeter waterMeter(node,3);

/***********************************
 * Main Sketch
 */

// before
void before() {
  // setup the serial port baud rate
  Serial.begin(MY_BAUD_RATE);

  /***********************************
 * Configure your sensors below
 */

  node.setReportIntervalSeconds(20);
  //battery.setReportIntervalSeconds(10);
  //node.setSleepSeconds(20);
  
  //node.setPowerManager(power);
  //battery.setReportIntervalSeconds(10);
  //sht.children.get(1)->child_id = 5;

  
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

// receive
void receive(const MyMessage &message) {
  // call NodeManager receive routine
  node.receive(message);
}

// receiveTime
void receiveTime(unsigned long ts) {
  // call NodeManager receiveTime routine
  node.receiveTime(ts);
}
