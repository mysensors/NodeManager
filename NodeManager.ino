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

///////// BATTERY MANAGER
   nodeManager.setBatteryMin(2.5);
   nodeManager.setBatteryMax(3.3);
   nodeManager.setBatteryInternalVcc(false);
   nodeManager.setBatteryPin(A0);
   nodeManager.setBatteryVoltsPerBit(0.003363075);

///////// POWER MANAGER
//   nodeManager.setRebootPin(A0);
//  nodeManager.setPowerPins(5,4,1000);
//  nodeManager.setPowerPins(8,7,1000);
//  nodeManager.powerOn();

//////// SLEEP MANAGER
  nodeManager.setSleep(SLEEP,5,MINUTES);
//  nodeManager.setSleepMode(WAIT); 
//  nodeManager.setSleepTime(5);
//  nodeManager.setSleepUnit(SECONDS);
//  nodeManager.setSleepBetweenSend(10);

//////// TEMPERATURE AND HUMIDITY SENSORS
//   nodeManager.registerSensor(SENSOR_DHT11,A4);
//   nodeManager.registerSensor(SENSOR_DHT21,A5);
   nodeManager.registerSensor(SENSOR_AM2320,1);
//   nodeManager.registerSensor(SENSOR_SHT21,1);

//////// DETECTION SENSORS
//   nodeManager.setSleepInterruptPin(false);
//   nodeManager.registerSensor(SENSOR_SWITCH,2,1);
//   nodeManager.registerSensor(SENSOR_SWITCH,3,2);
// or
//  
//  int liquidLevel1 = nodeManager.registerSensor(SENSOR_SWITCH,2,1);
//  SensorSwitch* liquidLevelSensor1 = ((SensorSwitch*)nodeManager.getSensor(liquidLevel1));
//  liquidLevelSensor1->setPowerPins(5,4,1000);
//  int liquidLevel2 = nodeManager.registerSensor(SENSOR_SWITCH,3,2);
//  SensorSwitch* liquidLevelSensor2 = ((SensorSwitch*)nodeManager.getSensor(liquidLevel2));
//  liquidLevelSensor2->setPowerPins(8,7,1000);
//  liquidLevelSensor2->setRetries(6);
//  liquidLevelSensor2->setMode(CHANGE);
//  liquidLevelSensor2>setInitial(HIGH);
//  liquidLevelSensor2->setTriggerTime(10);
//  liquidLevelSensor2->setDebounce(10);

//////// LIGHT SENSORS
//   nodeManager.registerSensor(SENSOR_BH1750,2);
// or
//   int Light = nodeManager.registerSensor(SENSOR_BH1750,2);
//   SensorBH1750* LightSensor = ((SensorBH1750*)nodeManager.getSensor(Light));
//   LightSensor->setPowerPins(7,8,200);
//   LightSensor->setRetries(2);

//   nodeManager.registerSensor(SENSOR_TSL2561,1);
   int Light = nodeManager.registerSensor(SENSOR_TSL2561,2);
   SensorTSL2561* LightSensor = ((SensorTSL2561*)nodeManager.getSensor(Light));
   LightSensor->setGain(0);
   LightSensor->setTiming(0);
   LightSensor->setSpectrum(0);

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


