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
SensorSHT21 sht(node);
//SensorBattery battery(node);
//SensorSignal signal(node);

// before
void before() {
  // setup the serial port baud rate
  Serial.begin(MY_BAUD_RATE);  
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
