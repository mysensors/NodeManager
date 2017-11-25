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
//SensorSHT21 sht(nodeManager);
SensorBattery battery(nodeManager);
//SensorSignal signal(nodeManager);

// before
void before() {
  // setup the serial port baud rate
  Serial.begin(MY_BAUD_RATE);  
  battery.setReportIntervalSeconds(10);

//nodeManager.sensors.get(0)->setPin(5);
//nodeManager.sensors.get(0)->children.get(0).child_id = 5;



  /*
   * Register below your sensors
  */
  /*
      pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);
    pinMode(5, OUTPUT);
    digitalWrite(5, LOW);
    */
  //nodeManager.setReportIntervalSeconds(20);
  
  
  
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
