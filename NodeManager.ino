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
//  nodeManager.setSleep(SLEEP,5,MINUTES);
  nodeManager.setSleepMode(SLEEP); 
  nodeManager.setSleepTime(5);
  nodeManager.setSleepUnit(SECONDS);
//  nodeManager.setSleepBetweenSend(10);

  /*
   * Register below your sensors
  */
//////// RELAY 
//  int relayState1 = nodeManager.registerSensor(SENSOR_RELAY,4,1);
//  SensorRelay* relay1 = ((SensorRelay*)nodeManager.getSensor(relayState1));
//  relay1->setInitialValue(LOW);
//  int relayState2 = nodeManager.registerSensor(SENSOR_RELAY,5,2);
//  SensorRelay* relay2 = ((SensorRelay*)nodeManager.getSensor(relayState2));
//  relay2->setInitialValue(HIGH);

//////// TEMPERATURE AND HUMIDITY SENSORS
//   nodeManager.registerSensor(SENSOR_DHT21,A5);
//   nodeManager.registerSensor(SENSOR_DHT11,A4);
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
//  liquidLevelSensor2->setPowerPins(7,6,1000);
//  liquidLevelSensor2->setRetries(6);
//  liquidLevelSensor2->setMode(CHANGE);
//  liquidLevelSensor2>setInitial(HIGH);
//  liquidLevelSensor2->setTriggerTime(10);
//  liquidLevelSensor2->setDebounce(10);

//////// LIGHT SENSORS
//     nodeManager.registerSensor(SENSOR_BH1750,2);
//     nodeManager.registerSensor(SENSOR_TSL2561,2);

// or
//   int Light = nodeManager.registerSensor(SENSOR_BH1750,2);
//   SensorBH1750* LightSensor = ((SensorBH1750*)nodeManager.getSensor(Light));
//   LightSensor->setPowerPins(7,8,200);
//   LightSensor->setRetries(2);

   int Light = nodeManager.registerSensor(SENSOR_TSL2561,2);
   SensorTSL2561* LightSensor = ((SensorTSL2561*)nodeManager.getSensor(Light));
// Maybe putting this in README ?
// You can change the gain on the fly, to adapt to brighter/dimmer light situations     
   LightSensor->setGain(SensorTSL2561::GAIN_0X); // set no gain (for bright situtations)
//   LightSensor->setGain(SensorTSL2561::GAIN_16X); // set 16x gain (for dim situations)
// Changing the integration time gives you a longer time over which to sense light
// longer timelines are slower, but are good in very low light situtations!
//   LightSensor->setTiming(SensorTSL2561::INTEGRATIONTIME_13MS); // shortest integration time (bright light)
   LightSensor->setTiming(SensorTSL2561::INTEGRATIONTIME_101MS); // medium integration time (medium light)
//   LightSensor->setTiming(SensorTSL2561::INTEGRATIONTIME_402MS); // longest integration time (dim light)
//   LightSensor->setSpectrum(SensorTSL2561::VISIBLE);
//   LightSensor->setSpectrum(SensorTSL2561::FULLSPECTRUM);
//   LightSensor->setSpectrum(SensorTSL2561::INFRARED); 
   LightSensor->setSpectrum(SensorTSL2561::FULL); // return LUX, IR, FULL and VISIBLE

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


