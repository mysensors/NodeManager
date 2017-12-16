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
NodeManager node;
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
SensorSonoff sonoff(node);





//PowerManager power(5,6);
//SensorBattery battery(node);
SensorConfiguration configuration(node);
//SensorSignal signal(node);

// before
void before() {
  // setup the serial port baud rate
  Serial.begin(MY_BAUD_RATE);
//  battery.setReportIntervalSeconds(10);
  node.setReportIntervalSeconds(20);
  //node.setSleepSeconds(20);
  
  //node.setPowerManager(power);
  //battery.setReportIntervalSeconds(10);
//sht.children.get(1)->child_id = 5;
//node.sensors.get(0)->setPin(5);
//node.sensors.get(0)->children.get(0).child_id = 5;



  /*
   * Register below your sensors
  */
  /*
      pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);
    pinMode(5, OUTPUT);
    digitalWrite(5, LOW);
    */
  //node.setReportIntervalSeconds(20);
  
  
  
  /*
   * Register above your sensors
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

