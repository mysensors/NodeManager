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
   * Register below your sensors
  */
  

   nodeManager.setRebootPin(A0);
//   nodeManager.setBatteryMin(2.5);
//   nodeManager.setBatteryMax(3.3);
//   nodeManager.setBatteryInternalVcc(true);
//   nodeManager.setBatteryPin(0);
//   setBatteryVoltsPerBit(0.003363075);
  nodeManager.setPowerPins(5,4,1000);
  nodeManager.setPowerPins(8,7,1000);
//  nodeManager.powerOn();
//  nodeManager.setSleep(SLEEP,30,SECONDS);
  nodeManager.setSleep(SLEEP,5,SECONDS);
//  nodeManager.setSleepMode(SLEEP);
//  nodeManager.setSleepTime(0);
//  nodeManager.setSleepBetweenSend(10);

//  nodeManager.setSleepInterruptPin(false);
//  int liquidLevel1 = nodeManager.registerSensor(SENSOR_SWITCH,1,3);
//  SensorSwitch* liquidLevelSensor1 = ((SensorSwitch*)nodeManager.getSensor(liquidLevel1));
// liquidLevelSensor1->setPowerPins(5,4,100);
//  int liquidLevel2 = nodeManager.registerSensor(SENSOR_SWITCH,2,6);
//  SensorSwitch* liquidLevelSensor2 = ((SensorSwitch*)nodeManager.getSensor(liquidLevel2));
//  liquidLevelSensor2->setPowerPins(8,7,100);
//  liquidLevelSensor2->setRetries(6);
  
//   nodeManager.registerSensor(SENSOR_DIGITAL_OUTPUT);
//   nodeManager.registerSensor(SENSOR_DHT11,A4);
//   nodeManager.registerSensor(SENSOR_DHT21,A5);
//   nodeManager.registerSensor(SENSOR_AM2320);

   nodeManager.registerSensor(SENSOR_SWITCH,3,1);
   nodeManager.registerSensor(SENSOR_SWITCH,6,2);

// or
//   int liquidLevel = nodeManager.registerSensor(SENSOR_SWITCH,3,1);
//   SensorSwitch* liquidLevelSensor = ((SensorSwitch*)nodeManager.getSensor(liquidLevel));
//   liquidLevelSensor->setPowerPins(5,4,100);
//   liquidLevelSensor->setMode(CHANGE);
//   liquidLevelSensor->setInitial(HIGH);
//   liquidLevelSensor->setTriggerTime(10);
//   liquidLevelSensor->setDebounce(10);

//    nodeManager.registerSensor(SENSOR_BH1750, 2);
// or
//   int Light = nodeManager.registerSensor(SENSOR_BH1750, 1);
//   SensorBH1750* LightSensor = ((SensorBH1750*)nodeManager.getSensor(Light));
//   LightSensor->setPowerPins(7,8,200);
//   LightSensor->setRetries(2);

  
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


