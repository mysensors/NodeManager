/*
NodeManager is intended to take care on your behalf of all those common tasks a MySensors node has to accomplish, speeding up the development cycle of your projects.

NodeManager includes the following main components:
- Sleep manager: allows managing automatically the complexity behind battery-powered sensors spending most of their time sleeping
- Power manager: allows powering on your sensors only while the node is awake
- Battery manager: provides common functionalities to read and report the battery level
- Remote configuration: allows configuring remotely the node without the need to have physical access to it
- Built-in personalities: for the most common sensors, provide embedded code so to allow their configuration with a single line 

Documentation available on: https://mynodemanager.sourceforge.io 
 */

 
// load user settings
#include "config.h"
// load MySensors library
#include <MySensors.h>
// load NodeManager library
#include "NodeManager.h"

// create a NodeManager instance
NodeManager nodeManager;

// before
void before() {
  // setup the serial port baud rate
  Serial.begin(9600);  
  // connect pin 4 to RST to enable rebooting the board with a message
  nodeManager.setRebootPin(4);
  // set battery minimum voltage. This will be used to calculate the level percentage
  //nodeManager.setBatteryMin(1.8);
  // instruct the board to sleep for 10 minutes for each cycle
  //nodeManager.setSleep(SLEEP,10,MINUTES);
  // When pin 3 is connected to ground, the board will stop sleeping
  //nodeManager.setSleepInterruptPin(3)
  // all the sensors' vcc and ground are connected to pin 6 (vcc) and 7 (ground). NodeManager will enable the vcc pin every time just before loop() and wait for 100ms for the sensors to settle
  //nodeManager.setPowerPins(6,7,100);
  // register a thermistor sensor attached to pin A2
  //nodeManager.registerSensor(SENSOR_THERMISTOR,A2);
  // register a LDR sensor attached to pin A1 and average 3 samples
  //int sensor_ldr = nodeManager.registerSensor(SENSOR_LDR,A1);
  //((SensorLDR*)nodeManager.get(sensor_ldr))->setSamples(3);
  
  nodeManager.before();
}

// presentation
void presentation() {
  // Send the sketch version information to the gateway and Controller
	sendSketchInfo("NodeManager", "1.0");
  // call NodeManager presentation routine
  nodeManager.presentation();

}

// setup
void setup() {
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


