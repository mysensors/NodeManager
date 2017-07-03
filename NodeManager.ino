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

// before
void before() {
  // setup the serial port baud rate
  Serial.begin(MY_BAUD_RATE);  

  /*
   * Register below your device specific config
  */

///////// BATTERY MANAGER
//   nodeManager.setBatteryMin(2.5);
//   nodeManager.setBatteryMax(3.3);
//   nodeManager.setBatteryInternalVcc(false);
//   nodeManager.setBatteryPin(A0);
//   nodeManager.setBatteryVoltsPerBit(0.003363075);

///////// POWER MANAGER
//  nodeManager.setRebootPin(8);
//  nodeManager.setPowerPins(4,5,1000);
//  nodeManager.setPowerPins(6,7,1000);
//  nodeManager.setSleepInterruptPin(false);
//  nodeManager.setInterrupt(2,CHANGE);
//  nodeManager.setInterrupt(3,CHANGE);
//  nodeManager.powerOn();

//////// SLEEP MANAGER
//  nodeManager.setSleep(SLEEP,10,MINUTES);
//  nodeManager.setSleepMode(SLEEP); 
//  nodeManager.setSleepTime(5);
//  nodeManager.setSleepUnit(SECONDS);
//  nodeManager.setSleepBetweenSend(10);

  /*
   * Register below your sensors
  */


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
