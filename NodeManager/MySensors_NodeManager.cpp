/*
* NodeManager Library
*/

// include dependencies
#include <Arduino.h>
#include <MySensors_NodeManager.h>





/******************************************
	Sensors
*/







#ifdef USE_CONFIGURATION


// contructor
SensorConfiguration::SensorConfiguration(NodeManager& node_manager): Sensor(node_manager) {
	_name = "CONFIG";
	children.allocateBlocks(1);
	new ChildInt(this,CONFIGURATION_CHILD_ID,S_CUSTOM,V_CUSTOM,_name);
}

// what to do during setup
void SensorConfiguration::onSetup() {
}

// what to do during loop
void SensorConfiguration::onLoop(Child* child) {
}

// what to do as the main task when receiving a message
void SensorConfiguration::onReceive(MyMessage* message) {
	// expect a REQ, V_CUSTOM message
	if (message->getCommand() != C_REQ && message->type != V_CUSTOM) return;
	// parse the request
	ConfigurationRequest request = ConfigurationRequest(message->sensor,message->getString());
	int function = request.getFunction();
	int child_id = request.getChildId();
	// if the message is for the board itself
	if (child_id == 0) {
		switch(function) {
		case 1: _node->hello(); break;
#if FEATURE_SLEEP == ON
		case 3: _node->setSleepSeconds(request.getValueInt()); break;
		case 4: _node->setSleepMinutes(request.getValueInt()); break;
		case 5: _node->setSleepHours(request.getValueInt()); break;
		case 29: _node->setSleepDays(request.getValueInt()); break;
		case 20: _node->setSleepBetweenSend(request.getValueInt()); break;
		case 9: _node->wakeup(); break;
#endif
#ifdef CHIP_AVR
		case 6: _node->reboot(); return;
#endif
#if FEATURE_EEPROM == ON
		case 7: _node->clearEeprom(); break;
		case 27: _node->saveToMemory(0,request.getValueInt()); break;
		case 40: _node->setSaveSleepSettings(request.getValueInt()); break;
#endif
		case 8: _node->sendMessage(CONFIGURATION_CHILD_ID,V_CUSTOM,VERSION); return;
		case 10: _node->setRetries(request.getValueInt()); break;
#if FEATURE_INTERRUPTS == ON
		case 19: _node->setSleepInterruptPin(request.getValueInt()); break;
		case 28: _node->setInterruptDebounce(request.getValueInt()); break;
#endif
		case 21: _node->setAck(request.getValueInt()); break;
		case 22: _node->setIsMetric(request.getValueInt()); break;
#if FEATURE_POWER_MANAGER == ON
		case 24: _node->powerOn(); break;
		case 25: _node->powerOff(); break;
#endif
		case 30: _node->setSleepOrWait(request.getValueInt()); break;
		case 31: _node->setRebootPin(request.getValueInt()); break;
		case 32: _node->setADCOff(); break;
		case 36: _node->setReportIntervalSeconds(request.getValueInt()); break;
		case 37: _node->setReportIntervalMinutes(request.getValueInt()); break;
		case 38: _node->setReportIntervalHours(request.getValueInt()); break;
		case 39: _node->setReportIntervalDays(request.getValueInt()); break;
#if FEATURE_TIME == ON
		case 41: _node->syncTime(); break;
		case 42: _node->sendMessage(CONFIGURATION_CHILD_ID,V_CUSTOM,(int)_node->getTime()); return;
#endif
		default: return; 
		}
		// the request is for a sensor
	} else {
		// retrieve the sensor the child is belonging to
		Sensor* sensor = _node->getSensorWithChild(child_id);
		if (sensor == nullptr) return;
		// if the message is for a function common to all the sensors
		if (request.getFunction() < 100) {
			switch(function) {
			case 1: sensor->setPin(request.getValueInt()); break;
			case 5: sensor->setSamples(request.getValueInt()); break;
			case 6: sensor->setSamplesInterval(request.getValueInt()); break;
#if FEATURE_POWER_MANAGER == ON
			case 13: sensor->powerOn(); break;
			case 14: sensor->powerOff(); break;
#endif
			case 24: sensor->setReportTimerMode((timer_mode)request.getValueInt()); break;
			case 25: sensor->setReportTimerValue(request.getValueInt()); break;
			case 26: sensor->setMeasureTimerMode((timer_mode)request.getValueInt()); break;
			case 27: sensor->setMeasureTimerValue(request.getValueInt()); break;
			case 16: sensor->setReportIntervalMinutes(request.getValueInt()); break;
			case 17: sensor->setReportIntervalSeconds(request.getValueInt()); break;
			case 19: sensor->setReportIntervalHours(request.getValueInt()); break;
			case 20: sensor->setReportIntervalDays(request.getValueInt()); break;
#if FEATURE_INTERRUPTS == ON			
			case 22: sensor->setInterruptMode(request.getValueInt()); break;
			case 23: sensor->setWaitAfterInterrupt(request.getValueInt()); break;
#endif
			default: return;
			}
		} else {
			sensor->onConfiguration(&request);
		}
	}
	_node->sendMessage(CONFIGURATION_CHILD_ID,V_CUSTOM,function);
}
			/*
			// the message is for a function specific to a sensor
			#ifdef USE_BATTERY
			if (strcmp(sensor->getName(),"BATTERY") == 0) {
				SensorBattery* custom_sensor = (SensorBattery*)sensor;
				switch(function) {
				case 102: custom_sensor->setMinVoltage(request.getValueFloat()); break;
				case 103: custom_sensor->setMaxVoltage(request.getValueFloat()); break;
				case 104: custom_sensor->setBatteryInternalVcc(request.getValueInt()); break;
				case 105: custom_sensor->setBatteryPin(request.getValueInt()); break;
				case 106: custom_sensor->setBatteryVoltsPerBit(request.getValueFloat()); break;
				case 107: custom_sensor->setBatteryCalibrationFactor(request.getValueFloat()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_SIGNAL
			if (strcmp(sensor->getName(),"SIGNAL") == 0) {
				SensorSignal* custom_sensor = (SensorSignal*)sensor;
				switch(function) {
				case 101: custom_sensor->setSignalCommand(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_ANALOG_INPUT
			if (strcmp(sensor->getName(),"ANALOG_I") == 0 || strcmp(sensor->getName(),"LDR") == 0 || strcmp(sensor->getName(),"RAIN") == 0 || strcmp(sensor->getName(),"SOIL") == 0) {
				SensorAnalogInput* custom_sensor = (SensorAnalogInput*)sensor;
				switch(function) {
				case 101: custom_sensor->setReference(request.getValueInt()); break;
				case 102: custom_sensor->setReverse(request.getValueInt()); break;
				case 103: custom_sensor->setOutputPercentage(request.getValueInt()); break;
				case 104: custom_sensor->setRangeMin(request.getValueInt()); break;
				case 105: custom_sensor->setRangeMax(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_THERMISTOR
			if (strcmp(sensor->getName(),"THERMISTOR") == 0) {
				SensorThermistor* custom_sensor = (SensorThermistor*)sensor;
				switch(function) {
				case 101: custom_sensor->setNominalResistor((long)request.getValueInt()); break;
				case 102: custom_sensor->setNominalTemperature(request.getValueInt()); break;
				case 103: custom_sensor->setBCoefficient(request.getValueInt()); break;
				case 104: custom_sensor->setSeriesResistor((long)request.getValueInt()); break;
				case 105: custom_sensor->setOffset(request.getValueFloat()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_ACS712
			if (strcmp(sensor->getName(),"ACS712") == 0) {
				SensorACS712* custom_sensor = (SensorACS712*)sensor;
				switch(function) {
				case 100: custom_sensor->setmVPerAmp(request.getValueInt()); break;
				case 102: custom_sensor->setOffset(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_DIGITAL_OUTPUT
			if (strcmp(sensor->getName(),"DIGITAL_O") == 0 || strcmp(sensor->getName(),"RELAY") == 0 || strcmp(sensor->getName(),"LATCHING1PIN") == 0 || strcmp(sensor->getName(),"LATCHING2PINS") == 0) {
				SensorDigitalOutput* custom_sensor = (SensorDigitalOutput*)sensor;
				switch(function) {
				case 104: custom_sensor->setLegacyMode(request.getValueInt()); break;
				case 105: custom_sensor->setSafeguard(request.getValueInt()); break;
				case 106: custom_sensor->setInputIsElapsed(request.getValueInt()); break;
				case 107: custom_sensor->setWaitAfterSet(request.getValueInt()); break;
				case 108: custom_sensor->setPulseWidth(request.getValueInt()); break;
				case 109: custom_sensor->setInvertValueToWrite(request.getValueInt()); break;
				case 110: custom_sensor->setPinOff(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_INTERRUPT
			if (strcmp(sensor->getName(),"INTERRUPT") == 0 || strcmp(sensor->getName(),"DOOR") == 0 || strcmp(sensor->getName(),"MOTION") == 0) {
				SensorInterrupt* custom_sensor = (SensorInterrupt*)sensor;
				switch(function) {
				case 105: custom_sensor->setInvertValueToReport(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_DS18B20
			if (strcmp(sensor->getName(),"DS18B20") == 0) {
				SensorDs18b20* custom_sensor = (SensorDs18b20*)sensor;
				switch(function) {
				case 101: custom_sensor->setResolution(request.getValueInt()); break;
				case 102: custom_sensor->setSleepDuringConversion(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_BH1750
			if (strcmp(sensor->getName(),"BH1750") == 0) {
				SensorBH1750* custom_sensor = (SensorBH1750*)sensor;
				switch(function) {
				case 101: custom_sensor->setMode(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#if defined(USE_BME280) || defined(USE_BMP085_180) || defined(USE_BMP280)
			if (strcmp(sensor->getName(),"BMP085") == 0 || strcmp(sensor->getName(),"BMP180") == 0 || strcmp(sensor->getName(),"BME280") == 0 || strcmp(sensor->getName(),"BMP280") == 0) {
				SensorBosch* custom_sensor = (SensorBosch*)sensor;
				switch(function) {
				case 101: custom_sensor->setForecastSamplesCount(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_SONOFF
			if (strcmp(sensor->getName(),"SONOFF") == 0) {
				SensorSonoff* custom_sensor = (SensorSonoff*)sensor;
				switch(function) {
				case 101: custom_sensor->setButtonPin(request.getValueInt()); break;
				case 102: custom_sensor->setRelayPin(request.getValueInt()); break;
				case 103: custom_sensor->setLedPin(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_HCSR04
			if (strcmp(sensor->getName(),"HCSR04") == 0) {
				SensorHCSR04* custom_sensor = (SensorHCSR04*)sensor;
				switch(function) {
				case 103: custom_sensor->setMaxDistance(request.getValueInt()); break;
				case 104: custom_sensor->setReportIfInvalid(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_MQ
			if (strcmp(sensor->getName(),"MQ") == 0) {
				SensorMQ* custom_sensor = (SensorMQ*)sensor;
				switch(function) {
				case 102: custom_sensor->setRlValue(request.getValueFloat()); break;
				case 103: custom_sensor->setRoValue(request.getValueFloat()); break;
				case 104: custom_sensor->setKnownPpm(request.getValueInt()); break;
				case 105: custom_sensor->setCalibrationSamples(request.getValueInt()); break;
				case 106: custom_sensor->setCalibrationSampleInterval(request.getValueInt()); break;
				case 107: custom_sensor->setSamples(request.getValueInt()); break;
				case 108: custom_sensor->setSampleInterval(request.getValueInt()); break;
				case 109: custom_sensor->setPoint1Ppm(request.getValueFloat()); break;
				case 110: custom_sensor->setPoint1Ratio(request.getValueFloat()); break;
				case 111: custom_sensor->setPoint2Ppm(request.getValueFloat()); break;
				case 112: custom_sensor->setPoint2Ratio(request.getValueFloat()); break;
				case 113: custom_sensor->setCurveScalingFactor(request.getValueFloat()); break;
				case 114: custom_sensor->setCurveExponent(request.getValueFloat()); break; 
				default: return;
				}
			}
			#endif
			#ifdef USE_TSL2561
			if (strcmp(sensor->getName(),"TSL2561") == 0) {
				SensorTSL2561* custom_sensor = (SensorTSL2561*)sensor;
				switch(function) {
				case 101: custom_sensor->setGain(request.getValueInt()); break;
				case 102: custom_sensor->setTiming(request.getValueInt()); break;
				case 103: custom_sensor->setSpectrum(request.getValueInt()); break;
				case 104: custom_sensor->setAddress(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_PT100
			if (strcmp(sensor->getName(),"PT100") == 0) {
				SensorPT100* custom_sensor = (SensorPT100*)sensor;
				switch(function) {
				case 101: custom_sensor->setVoltageRef(request.getValueFloat()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_DIMMER
			if (strcmp(sensor->getName(),"DIMMER") == 0) {
				SensorDimmer* custom_sensor = (SensorDimmer*)sensor;
				switch(function) {
				case 101: custom_sensor->setEasing(request.getValueInt()); break;
				case 102: custom_sensor->setDuration(request.getValueInt()); break;
				case 103: custom_sensor->setStepDuration(request.getValueInt()); break;
				case 104: custom_sensor->setReverse(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_PULSE_METER
			if (strcmp(sensor->getName(),"RAIN_GAUGE") == 0 || strcmp(sensor->getName(),"POWER") == 0 || strcmp(sensor->getName(),"WATER") == 0) {
				SensorPulseMeter* custom_sensor = (SensorPulseMeter*)sensor;
				switch(function) {
				case 102: custom_sensor->setPulseFactor(request.getValueFloat()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_SSD1306
			if (strcmp(sensor->getName(),"SSD1306") == 0) {
				DisplaySSD1306* display_SSD1306 = (DisplaySSD1306*)sensor;
				switch(function) {
				case 102: display_SSD1306->setContrast((uint8_t)request.getValueInt()); break;
				case 104: display_SSD1306->rotateDisplay((bool)request.getValueInt()); break;
				case 105: display_SSD1306->setFontSize(request.getValueInt()); break;
				case 106: display_SSD1306->setCaptionFontSize(request.getValueInt()); break;
				case 107: display_SSD1306->invertDisplay((bool)request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_CHIRP
			if (strcmp(sensor->getName(),"CHIRP") == 0) {
				SensorChirp* custom_sensor = (SensorChirp*)sensor;
				switch(function) {
				case 101: custom_sensor->setMoistureOffset(request.getValueInt()); break;
				case 102: custom_sensor->setMoistureRange(request.getValueInt()); break;
				case 103: custom_sensor->setReturnMoistureNormalized(request.getValueInt()); break;
				case 104: custom_sensor->setReturnLightReversed(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
			#ifdef USE_FPM10A
			if (strcmp(sensor->getName(),"FPM10A") == 0) {
				SensorFPM10A* custom_sensor = (SensorFPM10A*)sensor;
				switch(function) {
				case 101: custom_sensor->setMinConfidence(request.getValueInt()); break;
				case 102: custom_sensor->setWaitFingerForSeconds(request.getValueInt()); break;
				default: return;
				}
			}
			#endif
		}
	}
	_node->sendMessage(CONFIGURATION_CHILD_ID,V_CUSTOM,function);
}
*/

/******************************************
	ConfigurationRequest
*/

// contructor, tokenize a configuration request in the format "child_id,function,value"
ConfigurationRequest::ConfigurationRequest(int recipient_child_id, const char* string) {
	_recipient_child_id = recipient_child_id;
	char* ptr;
	// tokenize the string and get child id
	_child_id = atoi(strtok_r(const_cast<char*>(string), ",", &ptr));
	// tokenize the string and get function id
	_function = atoi(strtok_r(NULL, ",", &ptr));
	// tokenize the string and get the value
	_value = atof(strtok_r(NULL, ",", &ptr));
	debug(PSTR(LOG_OTA "REQ f=%d v=%d\n"),_function,_value);
}

// return the child id
int ConfigurationRequest::getRecipientChildId() {
	return _recipient_child_id;
}

// return the child id
int ConfigurationRequest::getChildId() {
	return _child_id;
}

// return the parsed function
int ConfigurationRequest::getFunction() {
	return _function;
}

// return the value as an int
int ConfigurationRequest::getValueInt() {
	return (int)_value;

}

// return the value as a float
float ConfigurationRequest::getValueFloat() {
	return _value;
}
#endif

