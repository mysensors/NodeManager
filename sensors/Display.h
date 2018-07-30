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
#ifndef Display_h
#define Display_h

/*
* Display class
*/

class Display: public Sensor {
protected:
	const char* _caption = "";
	
public:
	Display(NodeManager& node_manager, int child_id = -255) {
		_name = "";
		// We don't need any sensors, but we need a child, otherwise the loop will never be executed
		children.allocateBlocks(1);
		new ChildString(this, _node->getAvailableChildId(child_id), S_INFO, V_TEXT,_name);
		// prevent reporting to the gateway at each display update
		setReportTimerMode(DO_NOT_REPORT);
	};

	// set a static caption text on header of the display
	void setCaption(const char* value) {
		_caption = value;
	};
	
	// print a static caption on top of the screen
	virtual void printCaption(const char* value) {};
	// print the given string
	virtual void print(const char* value) {};
	// print the given string and goes to the next line
	virtual void println(const char* value) {};
	// print the value of the given child
	virtual void printChild(Child* child) {};
	// clear the display
	virtual void clear() {};
	// set the cursor to the given position
	virtual void setCursor(int col,int row) {};

	// define what to do during loop
	void onLoop(Child* child) {
		// clear the screen
		clear();  
		// print caption
		printCaption(_caption);
		// cycle through all the sensors and children
		for (List<Sensor*>::iterator itr = _node->sensors.begin(); itr != _node->sensors.end(); ++itr) {
			Sensor* sensor = *itr;
			// skip this display sensor
			if (sensor == this) continue;
			// Loop through all children and show name, value (and type)
			for (List<Child*>::iterator chitr = sensor->children.begin(); chitr != sensor->children.end(); ++chitr) {
				Child* ch = *chitr;
				// print description if any
				if (strlen(ch->getDescription()) > 0) {
					print(ch->getDescription());
					print(": ");
				}
				// print value
				printChild(ch);
				// print type
				if (ch->getType() == V_TEMP) {
					if (_node->getIsMetric()) print("C");
					else print("F");
				}
				else if (ch->getType() == V_HUM || ch->getType() == V_PERCENTAGE) print("%");
				else if (ch->getType() == V_PRESSURE) print("Pa");
				else if (ch->getType() == V_WIND || ch->getType() == V_GUST) print("Km/h");
				else if (ch->getType() == V_VOLTAGE) print("V");
				else if (ch->getType() == V_CURRENT) print("A");
				else if (ch->getType() == V_LEVEL && ch->getPresentation() == S_SOUND) print("dB");
				else if (ch->getType() == V_LIGHT_LEVEL && ch->getPresentation() == S_LIGHT_LEVEL) print("%");
				else if (ch->getType() == V_RAINRATE) print("%");
				else if (ch->getType() == V_LEVEL && ch->getPresentation() == S_MOISTURE) print("%");
				println(nullptr);
			}
		}
	};

	// what to do as the main task when receiving a message
	void onReceive(MyMessage* message) {
		Child* child = getChild(message->sensor);
		if (child == nullptr) return;
		if (message->getCommand() == C_SET && message->type == child->getType()) {
			int text_start = 0;
			// if the string contains a "," at the second position, it means the first char is the row number
			if (strncmp(message->getString()+1,",",1) == 0) {
				setCursor(0,atoi(message->getString()));
				// text starts at position 2
				text_start = 2;
			}
			// print the received text
			print(message->getString()+text_start);
		}
	};
};
#endif