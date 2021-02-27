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
#ifndef SensorACS712_AC_h
#define SensorACS712_AC_h

/*
	SensorACS712_AC

	Hardware notes: Measuring AC current with ACS712 is a dumb idea, 
	but if you ended up doing so in hardware, here's a sensor for you. 
	- Make sure to use 1nF cap on the filtering pin to max out bandwidth
	- 0.1-1.0uF decoupling on AREF should do just fine

	This sensor uses so-called 'graphical' method
	How it works: 
	 - Get as N samples with fastest ADC rate possible
	 - Add up their squares and divide by N
	 - Take the square root
	 - Divide by sqrt(2) to get RMS value
	 - Divide by 1024.0 and multiply by 5000.0 to get volts
	 - Divide by mv_per_amp value to get amps
	 - PROFIT!

	Since some ACS712 clones from shady chineese vendors have different mV
	per amp values, the best option would be determining the value by using 
	a known load. Old light bulbs work just great!

	Usage:
		//instantiate the sensor;
		SensorACS712_AC ac(int pin);
		// Call ac.calibrateZero() when there is NO LOAD. 
		Until calibrated the sensor will always report zero
*/

#include <sensors/SensorACS712.h>
class SensorACS712_AC: public SensorACS712 {

protected:
	float _current_offset = NAN;
	uint16_t _midpoint = 512;

public:
	SensorACS712_AC(int pin, int child_id = -255): SensorACS712(pin, child_id) {
		children.get()->setDescription("ACS712_AC");
	};

	// define what to do during setup
	void onSetup() {
		// set the pin as input
		pinMode(_pin, INPUT);
		analogReference(DEFAULT);
		#if defined(__AVR__)
			// Make sure we're using the fastest adc speed possible
			ADCSRA &= ~ ((1<<0) | (1<<1) | (1<<2));
		#endif
	};

	uint16_t measure_avg(uint32_t tm)
	{
		int count = 0;
		uint32_t now = millis();
		uint32_t avg = 0;

		while (millis() - now < tm) 
		{
			avg += analogRead(_pin);
			count++;
		}
		avg = avg / count;
		return (uint16_t) avg;
	}

	double measure_once(uint32_t tm)
	{
		int count = 0;
		uint32_t now = millis();
		double rms = 0;

		analogRead(_pin); /* AnalogRead buffers values, so throw last one away */
		while (millis() - now < tm) 
		{
			int32_t value = analogRead(_pin) - _midpoint;
			rms += value * value;
			count++;
		}
		Serial.println(count);
		rms = sqrt( ((double) rms) / count);
		/* Get RMS voltage */
		double voltage = (rms / 1024.0) * 5000.0; 
		/* Get RMS current */
		double amps = voltage / _mv_per_amp;
		return amps;
	}

	void calibrateZero() {
		/* Calibrate actual midpoint */
		if (_current_offset != NAN) {
			_midpoint = measure_avg(500);
			_current_offset = measure_once(500);
		}
	}

	float measure() {
		/* AC current is 50/60Hz depending on the country, 
		*  That's 16-20 ms per cycle, we have to measure at least 3-4 cycles 
		*/
		return (float) measure_once(80) - _current_offset;		
	}

};
#endif