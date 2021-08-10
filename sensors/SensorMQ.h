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
#ifndef SensorMQ_h
#define SensorMQ_h

/*
	SensorMQ
*/

class SensorMQ: public Sensor {
protected:
	long _rl = 1000;
	long _ro = 0;
	int _known_ppm = 411;
	int _calibration_samples = 50;
	int _calibration_sample_interval = 500;
	int _sample_interval = 50;
	int _samples = 5;
	unsigned long _warmup_minutes = 0;
	float _point1_ppm = 200;
	float _point1_ratio = 5;
	float _point2_ppm = 10000;
	float _point2_ratio = 1.2;
	float _curve_scaling_factor = 0;
	float _curve_exponent = 0;
	
public:
	SensorMQ(int8_t pin, uint8_t child_id = 0): Sensor(pin) {
		_name = "MQ";
		children.allocateBlocks(1);
		new Child(this,INT,nodeManager.getAvailableChildId(child_id),S_AIR_QUALITY,V_LEVEL,_name);
	};

	// [102] set the load resistance on the board, in ohms (default: 1000);
	void setRlValue(float value) {
		_rl = value;
	};
	// [103] set the Ro resistance in ohms. By default will be calculated at startup during the calibration phase using the known ppm provided
	void setRoValue(float value) {
		_ro = value;
	};
	// [104] set the ppm used during the calibration (default: 411);
	void setKnownPpm(float value) {
		_known_ppm = value;
	};
	// [105] define how many samples we are going to take in the calibration phase (default: 50);
	void setCalibrationSamples(int value) {
		_calibration_samples = value;
	};
	// [106] define the time (in milisecond) between each sample in the cablibration phase (default: 500);
	void setCalibrationSampleInterval(int value) {
		_calibration_sample_interval = value;
	};
	// [107] define how many samples you are going to take in normal operation (default: 50);
	void setSamples(int value) {
		_samples = value;
	};
	// [108] define the time (in milisecond) between each sample in the normal operations (default: 5);
	void setSampleInterval(int value) {
		_sample_interval = value;
	};
	// [109] set the ppm (x) of a random point on the gas curve (default: 200)
	void setPoint1Ppm(float value) {
		_point1_ppm = value;
	}; 
	// [110] set the Rs/Ro ratio (y) of the same random point on the gas curve (default: 5)
	void setPoint1Ratio(float value) {
		_point1_ratio = value;
	};
	// [111] set the ppm (x) of another random point on the gas curve (default: 10000)
	void setPoint2Ppm(float value) {
		_point2_ppm = value;
	};
	// [112] set the Rs/Ro ratio (y) of the same random point on the gas curve (default: 1.2)
	void setPoint2Ratio(float value) {
		_point2_ratio = value;
	};
	// [113] with ppm = scaling_factor*x^exponent set the value manually, otherwise will be calculated automatically based on the two points provided
	void setCurveScalingFactor(float value) {
		_curve_scaling_factor = value;
	}; 
	// [114] with ppm = scaling_factor*x^exponent set the value manually, otherwise will be calculated automatically based on the two points provided
	void setCurveExponent(float value) {
		_curve_exponent = value;
	}; 
	// do not report for the given number of minutes, waiting for the sensor to warm up (default: 0);
	void setWarmupMinutes(int value) {
		_warmup_minutes = value;
	};

	// define what to do during setup
	void onSetup() {
		// prepare the pin for input
		pinMode(_pin, INPUT);
		// The curve function is ppm = scaling_factor*ratio^exponent. Since we know two points (ppm1,ratio1) and (ppm2,ratio2) we can calculate scaling_factor and exponent approximating a power regression
		if (_curve_exponent == 0) _curve_exponent = log(_point2_ppm/_point1_ppm)/log(_point2_ratio/_point1_ratio);
		if (_curve_scaling_factor == 0) _curve_scaling_factor = exp((log(_point1_ratio)*log(_point2_ppm)-log(_point2_ratio)*log(_point1_ppm))/(_point1_ratio-_point2_ratio));
		int rs = 0;
		if (_ro == 0) {
			// calibrate the sensor (the Ro resistance) if requested. Since ppm = scaling_factor*(rs/ro)^exponent, we need Rs to calculate Ro for the given ppm
			debug(PSTR(LOG_SENSOR "%s:CAL\n"),_name);
			rs = _getRsValue(_calibration_samples,_calibration_sample_interval);
			_ro = (long)(rs * exp( log(_curve_scaling_factor/_known_ppm) / _curve_exponent ));
		}
		debug(PSTR(LOG_SENSOR "%s:CAL OK Rs=%d Ro=%ld Rl=%ld F=%d.%02d x^=%d.%02d\n"),_name,rs,_ro,_rl,(int)_curve_scaling_factor,(int)((_curve_scaling_factor-(int)_curve_scaling_factor)*100)*(_curve_scaling_factor<0.0f?-1:1),(int)_curve_exponent,(int)((_curve_exponent-(int)_curve_exponent)*100)*(_curve_exponent<0.0f?-1:1));
	};

	// define what to do during loop
	void onLoop(Child* child) {
		// ppm = _curve_scaling_factor * (rs/ro) ^ _curve_exponent so we need Rs 
		float rs = _getRsValue(_samples,_sample_interval);
		// calculate the Rs / Ro ratio
		float rs_ro_ratio = rs / _ro;
		// calculate ppm 
		int ppm = _curve_scaling_factor * pow(rs_ro_ratio, _curve_exponent);
		debug(PSTR(LOG_SENSOR "%s(%d):READ Rs=%d Rs/Ro=%d.%02d ppm=%d\n"),_name,child->getChildId(),(int)rs,(int)rs_ro_ratio,(int)((rs_ro_ratio-(int)rs_ro_ratio)*100)*(rs_ro_ratio<0.0f?-1:1),ppm);
		// ppm cannot be negative
		if (ppm < 0) ppm = 0;
		// if warmup is configured, do not send the value back if within the warmup period
		if (_warmup_minutes > 0 && millis() < _warmup_minutes*60*1000) return;
		// store the value
		child->setValue(ppm);
	};
	
#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 102: setRlValue(request->getValueFloat()); break;
		case 103: setRoValue(request->getValueFloat()); break;
		case 104: setKnownPpm(request->getValueInt()); break;
		case 105: setCalibrationSamples(request->getValueInt()); break;
		case 106: setCalibrationSampleInterval(request->getValueInt()); break;
		case 107: setSamples(request->getValueInt()); break;
		case 108: setSampleInterval(request->getValueInt()); break;
		case 109: setPoint1Ppm(request->getValueFloat()); break;
		case 110: setPoint1Ratio(request->getValueFloat()); break;
		case 111: setPoint2Ppm(request->getValueFloat()); break;
		case 112: setPoint2Ratio(request->getValueFloat()); break;
		case 113: setCurveScalingFactor(request->getValueFloat()); break;
		case 114: setCurveExponent(request->getValueFloat()); break; 
		default: return;
		}
	};
#endif
protected:
	// get the rs value by sampling the resistance multiple times
	float _getRsValue(int samples, int sample_interval) {
		float total = 0;
		for (int i = 0; i < samples; i++) {
			int adc = analogRead(_pin);
			float rs = ( ((float)_rl*(1023-adc)/adc));
			total += rs;
			wait(sample_interval);
		}
		return total/(float)samples;
	};
};
#endif
