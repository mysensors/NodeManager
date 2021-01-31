/*
* The MySensors Arduino library handles the wireless radio link and protocol
* between your home built sensors/actuators and HA controller of choice.
* The sensors forms a self healing radio network with optional repeaters. Each
* repeater and gateway builds a routing tables in EEPROM which keeps track of the
* network topology allowing messages to be routed to nodes.
*
* Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
* Copyright (C) 2013-2017 Sensnology AB
* Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
*
* Documentation: http://www.mysensors.org
* Support Forum: http://forum.mysensors.org
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*/

/**************************
Template

This sketch can be used as a template since containing the most relevant MySensors library configuration settings,
NodeManager's settings, all its the supported sensors commented out and a sketch structure fully functional to operate with
NodeManager. Just uncomment the settings you need and the sensors you want to add and configure the sensors in before()
*/

/**********************************
 * MySensors node configuration
 */

// General settings
#define SKETCH_NAME "NodeManager"
#define SKETCH_VERSION "1.0"
//#define MY_DEBUG
//#define MY_NODE_ID 99

// NRF24 radio settings
#define MY_RADIO_RF24
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

// Optimizations when running on 2032 Coin Cell. Also set nodeManager.setSleepBetweenSend(500) and run the board at 1Mhz
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
//#define MY_WIFI_SSID ""
//#define MY_WIFI_PASSWORD ""

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
#define NODEMANAGER_INTERRUPTS ON
#define NODEMANAGER_SLEEP ON
#define NODEMANAGER_RECEIVE ON
#define NODEMANAGER_DEBUG_VERBOSE OFF
#define NODEMANAGER_POWER_MANAGER OFF
#define NODEMANAGER_CONDITIONAL_REPORT OFF
#define NODEMANAGER_EEPROM OFF
#define NODEMANAGER_TIME OFF
#define NODEMANAGER_RTC OFF
#define NODEMANAGER_SD OFF
#define NODEMANAGER_HOOKING OFF
#define NODEMANAGER_OTA_CONFIGURATION OFF
#define NODEMANAGER_SERIAL_INPUT OFF

// import NodeManager library (a nodeManager object will be then made available)
#include <MySensors_NodeManager.h>

/***********************************
 * Add your sensors
 */

//PowerManager power(5,6);

//#include <sensors/SensorBattery.h>
//SensorBattery battery;

//#include <sensors/SensorConfiguration.h>
//SensorConfiguration configuration;

//#include <sensors/SensorSignal.h>
//SensorSignal signal;

//#include <sensors/SensorAnalogInput.h>
//SensorAnalogInput analog(A0);

//#include <sensors/SensorLDR.h>
//SensorLDR ldr(A0);

//#include <sensors/SensorRain.h>
//SensorRain rain(A0);

//#include <sensors/SensorSoilMoisture.h>
//SensorSoilMoisture soil(A0);

//#include <sensors/SensorThermistor.h>
//SensorThermistor thermistor(A0);

//#include <sensors/SensorTMP102.h>
//SensorTMP102 tmp102;

//#include <sensors/SensorML8511.h>
//SensorML8511 ml8511(A0);

//#include <sensors/SensorACS712.h>
//SensorACS712 acs712(A0);

//#include <sensors/SensorDigitalInput.h>
//SensorDigitalInput digitalIn(6);

//#include <sensors/SensorDigitalOutput.h>
//SensorDigitalOutput digitalOut(6);

//#include <sensors/SensorRelay.h>
//SensorRelay relay(6);

//#include <sensors/SensorLatchingRelay1Pin.h>
//SensorLatchingRelay1Pin latching1pin(6);

//#include <sensors/SensorLatchingRelay2Pins.h>
//SensorLatchingRelay2Pins latching2pins(6,7);

//#include <sensors/SensorDHT11.h>
//SensorDHT11 dht11(6);

//#include <sensors/SensorDHT22.h>
//SensorDHT22 dht22(6);

//#include <sensors/SensorSHT21.h>
//SensorSHT21 sht21;

//#include <sensors/SensorHTU21D.h>
//SensorHTU21D htu21;

//#include <sensors/SensorInterrupt.h>
//SensorInterrupt interrupt(3);

//#include <sensors/SensorDoor.h>
//SensorDoor door(3);

//#include <sensors/SensorMotion.h>
//SensorMotion motion(3);

//#include <sensors/SensorDs18b20.h>
//SensorDs18b20 ds18b20(6);

//#include <sensors/SensorBH1750.h>
//SensorBH1750 bh1750;

//#include <sensors/SensorMLX90614.h>
//SensorMLX90614 mlx90614;

//#define NODEMANAGER_SENSOR_BOSCH_LITE
//#include <sensors/SensorBME280.h>
//SensorBME280 bme280;

//#include <sensors/SensorBMP085.h>
//SensorBMP085 bmp085;

//#include <sensors/SensorBMP180.h>
//SensorBMP180 bmp180;

//#include <sensors/SensorBMP280.h>
//SensorBMP280 bmp280;

//#include <sensors/SensorSonoff.h>
//SensorSonoff sonoff;

//#include <sensors/SensorHCSR04.h>
//SensorHCSR04 hcsr04(6,7);

//#include <sensors/SensorMCP9808.h>
//SensorMCP9808 mcp9808;

//#include <sensors/SensorMQ.h>
//SensorMQ mq(A0);

//#include <sensors/SensorMHZ19.h>
//SensorMHZ19 mhz19(6,7);

//#include <sensors/SensorAM2320.h>
//SensorAM2320 am2320;

//#include <sensors/SensorTSL2561.h>
//SensorTSL2561 tsl2561;

//#include <sensors/SensorPT100.h>
//SensorPT100 pt100(6);

//#include <sensors/SensorDimmer.h>
//SensorDimmer dimmer(3);

//#include <sensors/SensorRainGauge.h>
//SensorRainGauge rainGauge(3);

//#include <sensors/SensorPowerMeter.h>
//SensorPowerMeter powerMeter(3);

//#include <sensors/SensorWaterMeter.h>
//SensorWaterMeter waterMeter(3);

//#include <sensors/SensorPlantowerPMS.h>
//SensorPlantowerPMS pms(6,7);

//#include <sensors/SensorVL53L0X.h>
//SensorVL53L0X vl53l0x(3);

//#include <sensors/DisplaySSD1306.h>
//DisplaySSD1306 ssd1306;

//#include <sensors/SensorSHT31.h>
//SensorSHT31 sht31;

//#include <sensors/SensorSI7021.h>
//SensorSI7021 si7021;

//#include <sensors/SensorChirp.h>
//SensorChirp chirp;

//#include <sensors/DisplayHD44780.h>
//DisplayHD44780 hd44780;

//#include <sensors/SensorTTP.h>
//SensorTTP ttp;

//#include <sensors/SensorServo.h>
//SensorServo servo(6);

//#include <sensors/SensorAPDS9960.h>
//SensorAPDS9960 apds9960(3);

//#include <sensors/SensorNeopixel.h>
//SensorNeopixel neopixel(6);

//#include <sensors/SensorSDS011.h>
//SensorSDS011 sds011(6,7);

//#include <sensors/SensorFPM10A.h>
//SensorFPM10A fpm10a(4,5);

//#include <sensors/SensorPH.h>
//SensorPH ph(A0);

//#include <sensors/SensorPca9685W.h>
//SensorPca9685W pca9685W;

//#include <sensors/SensorPca9685Rgb.h>
//SensorPca9685Rgb pca9685Rgb;

//#include <sensors/SensorPca9685Rgbw.h>
//SensorPca9685Rgbw pca9685Rgbw;

//#include <sensors/SensorDSM501A.h>
//SensorDSM501A DSM501A;

//#include <sensors/SensorPN532.h>
//SensorPN532 pn532;

//#include <sensors/SensorCCS811.h>
//SensorCCS811 ccs811;

//#include <sensors/SensorGSM.h>
//SensorGSM gsm(6,7);


////Air conditioning (HVAC)
//#include <sensors/SensorHVAC.h>
////declare heatpump
//MitsubishiFDHeatpumpIR heatpumpIR;
//SensorHVAC hvac(MYSX_D4, &heatpumpIR);

//#include <sensors/SensorWaterLeak.h>
//SensorWaterLeak waterLeak(3);

//#include <sensors/SensorIRremote.h>
//unsigned int irSignal[59] = {8250,4150, 550,1600, 500,550, 500,600, 500,550, 500,1650, 500,550, 500,550, 550,550, 500,550, 500,600, 500,550, 500,550, 550,550, 500,550, 500,600, 500,550, 500,1650, 500,550, 550,550, 500,550, 500,550, 500,600, 500,550, 500,600, 500,1600, 500,600, 500,550, 500,550, 550}; 
//SensorIRremote remote(irSignal);

/***********************************
 * Main Sketch
 */

// before
void before() {

  /***********************************
   * Configure your sensors
   */

  // EXAMPLES:
  // report measures of every attached sensors every 10 seconds
  //nodeManager.setReportIntervalSeconds(10);
  // report measures of every attached sensors every 10 minutes
  //nodeManager.setReportIntervalMinutes(10);
  // set the node to sleep in 30 seconds cycles
  //nodeManager.setSleepSeconds(30);
  // set the node to sleep in 5 minutes cycles
  //nodeManager.setSleepMinutes(5);
  // report battery level every 10 minutes
  //battery.setReportIntervalMinutes(10);
  // set an offset to -1 to a thermistor sensor
  //thermistor.setOffset(-1);
  // change the id of a the first child of a sht21 sensor
  //sht21.children.get(1)->setChildId(5);
  // report only when the analog value is above 40%
  //analog.children.get(1)->setMinThreshold(40);
  // power all the nodes through dedicated pins
  //nodeManager.setPowerManager(power);

  // call NodeManager before routine
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

#if NODEMANAGER_RECEIVE == ON
// receive
void receive(const MyMessage &message) {
  // call NodeManager receive routine
  nodeManager.receive(message);
}
#endif

#if NODEMANAGER_TIME == ON
// receiveTime
void receiveTime(unsigned long ts) {
  // call NodeManager receiveTime routine
  nodeManager.receiveTime(ts);
}
#endif
