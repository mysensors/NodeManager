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
#ifndef SensorPca9685Led_h
#define SensorPca9685Led_h

/*
SensorPca9685Led
*/

#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class SensorPca9685Led {
protected:
    Adafruit_PWMServoDriver* _pca9685;
    bool _ownPca9685 = false;
    uint8_t _i2c_addr = 0xFF;
    int _pwm_ch = 0;
    int _target_color = 0;
    int _start_color = 0;
    int _cur_color = 0;
    unsigned long _start_time = 0;
    int _status = OFF;
    int _easing = EASE_LINEAR;
    int _duration = 1000;
    int _step_duration = 100;
public:
    enum _easing_list {
      EASE_LINEAR = 0,
      EASE_INSINE = 1,
      EASE_OUTSINE = 2,
      EASE_INOUTSINE = 3
    };
	
	SensorPca9685Led(int channel = 0,uint8_t i2c_addr = 0x40, Adafruit_PWMServoDriver* pca9685 = NULL) {
		_i2c_addr = i2c_addr;
		_pwm_ch = channel;
		// only create new ServoDriver if no one exists
		if (pca9685 == NULL) {
			_pca9685 = new Adafruit_PWMServoDriver(_i2c_addr);
			_ownPca9685 = true;  
		} else {
			_pca9685 = pca9685;
			_ownPca9685 = false;
		}
	};

	// setter/getter
	void setEasing(int value) {
		_easing = value;
	};
	void setDuration(int value) {
		_duration = value;
	};
	void setStepDuration(int value) {
		_step_duration = value;
	};

	// define what to do during setup
	void onSetup() {
		//only initialize PCA9685, if its ours
		if (_ownPca9685) {
			_pca9685->reset();
			_pca9685->begin();
			_pca9685->setPWMFreq(1600);  // This is the maximum PWM frequency
			_pca9685->setPWM(_pwm_ch, 0, 0); //set LED OFF; TODO: set all LED off
		}
	};
	
	// set the LED value
	void setVal(int value) {
	  // load fader with target colors 12bit (4095)   
	  _target_color = value;
	  // load fader with start colors;
	  _start_color = _cur_color;
	  // load fader with start time
	  unsigned long current_time = millis();
	  _start_time = current_time;
	  debug(PSTR(LOG_SENSOR "_pwm_ch=%d _target_color=%d _start_color=%d\n"),_pwm_ch,_target_color,_start_color);
	};

	// set the LED value as HEX-String
	void setValHex(String hexstring) {
	  //scale from 8bit to 12bit and call setVal
	  this->setVal((strtoul( hexstring.c_str(), NULL, 16) * 4095./255.));
	};

	// set the LED value as Percentage
	void setValPercentage(int percentage) {
	  //scale from 100% to 12bit and call setVal
	  this->setVal(percentage * 4095./100.);
	};

	// get the LED value
	int getVal() {
	  return _target_color;
	};
	
	// get the LED value as Percentage
	int getValPercentage() {
	  return (_target_color * 100./4095.);
	};
	
	// get the LED value as Hex-String (8-bit)
	String getValHex() {
	  return String((_target_color * 255./4095.),HEX);
	};

	void faderInc() {
	  int delta = 0;
	  unsigned long current_step = 0;
	  unsigned long current_time = millis();
	  //do nothing if start_time is smaller than current time OR if color already set
	  //TODO: Handle current_time resp millis() overflow
	  if ((current_time < _start_time) || (_target_color == _cur_color) ) return;
	  //fade within _duration
	  if((_start_time + _duration) > current_time) {
		// calculate the delta between the target value and the current
		delta = _target_color - _start_color;

		//calculate current timestep for this fader
		current_step = current_time - _start_time;
	  
		// calculate the smooth transition
		_cur_color = (int)(_getEasing(current_step,_start_color,delta,_duration));
	  } else {
		_cur_color = _target_color;
	  }
	  // write to the PWM output
	  _pca9685->setPWM(_pwm_ch, 0, _cur_color);
	  //makes fading very slow; only uncomment if necessary
	  // debug(PSTR(LOG_SENSOR "_pwm_ch=%d _target_color=%d _start_color=%d _start_time=%d  current_time=%d  current_step=%d  _duration=%d  value=%d\n"),_pwm_ch,_target_color,_start_color,_start_time,current_time,current_step,_duration,_cur_color);
	};

	// for smooth transitions. t: current time, b: beginning value, c: change in value, d: duration
	float _getEasing(float t, float b, float c, float d) {
	  if (_easing == EASE_INSINE) return -c * cos(t/d * (M_PI/2)) + c + b;
	  else if (_easing == EASE_OUTSINE) return c * sin(t/d * (M_PI/2)) + b;
	  else if (_easing == EASE_INOUTSINE) return -c/2 * (cos(M_PI*t/d) - 1) + b;
	  else return c*t/d + b;
	};
};
#endif