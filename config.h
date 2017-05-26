#ifndef config_h
#define config_h

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

//#define MY_REPEATER_FEATURE
//#define MY_TRANSPORT_WAIT_READY_MS (5*1000ul)
//#define MY_TRANSPORT_TIMEOUT_EXT_FAILURE_STATE (15*60*1000ul)
//#define MY_TRANSPORT_SANITY_CHECK
// already set as default
//#define MY_TRANSPORT_SANITY_CHECK_INTERVAL_MS (15*60*1000ul)
//#define MY_TRANSPORT_STATE_RETRIES 2

// Enable to support OTA for this node (needs DualOptiBoot boot-loader to fully work)
//#define MY_OTA_FIRMWARE_FEATURE
// How many milli seconds should we wait for OTA?
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
// if enabled, send a SLEEPING and AWAKE service messages just before entering and just after leaving a sleep cycle and STARTED when starting/rebooting
#define SERVICE_MESSAGES 0

// Enable this module to use one of the following sensors: SENSOR_ANALOG_INPUT, SENSOR_LDR, SENSOR_THERMISTOR, SENSOR_MQ, SENSOR_ML8511, SENSOR_ACS712, SENSOR_RAIN_GAUGE
#define MODULE_ANALOG_INPUT 1
// Enable this module to use one of the following sensors: SENSOR_DIGITAL_INPUT
#define MODULE_DIGITAL_INPUT 1
// Enable this module to use one of the following sensors: SENSOR_DIGITAL_OUTPUT, SENSOR_RELAY, SENSOR_LATCHING_RELAY
#define MODULE_DIGITAL_OUTPUT 1
// Enable this module to use one of the following sensors: SENSOR_DHT11, SENSOR_DHT22
#define MODULE_DHT 0
// Enable this module to use one of the following sensors: SENSOR_AM2320
#define MODULE_AM2320 0
// Enable this module to use one of the following sensors: SENSOR_SHT21
#define MODULE_SHT21 0
// Enable this module to use one of the following sensors: SENSOR_SWITCH, SENSOR_DOOR, SENSOR_MOTION
#define MODULE_SWITCH 0
// Enable this module to use one of the following sensors: SENSOR_DS18B20
#define MODULE_DS18B20 0
// Enable this module to use one of the following sensors: SENSOR_BH1750
#define MODULE_BH1750 0
// Enable this module to use one of the following sensors: SENSOR_TSL2561
#define MODULE_TSL2561 0
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
#endif

