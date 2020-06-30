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
#ifndef SensorBosch_h
#define SensorBosch_h

/*
* SensorBosch
*/

class SensorBosch: public Sensor {
protected:
#if !defined(NODEMANAGER_SENSOR_BOSCH_LITE)
	const char* _weather[6] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
	int _forecast_samples_count = 5;
	float* _forecast_samples;
	int _minute_count = 0;
	float _pressure_avg;
	float _pressure_avg2;
	float _dP_dt;
	bool _first_round = true;
#endif

public:
	SensorBosch(uint8_t child_id = 0): Sensor(-1) {
		_name = "BOSCH";
#if !defined(NODEMANAGER_SENSOR_BOSCH_LITE)
		// initialize the forecast samples array
		_forecast_samples = new float[_forecast_samples_count];
#endif
	};
	
#if !defined(NODEMANAGER_SENSOR_BOSCH_LITE)
	// [101] define how many pressure samples to keep track of for calculating the forecast (default: 5)
	void setForecastSamplesCount(int value) {
		_forecast_samples_count = value;
	};
#endif
	
	// search for a given chip on i2c bus (emulating Adafruit's init() function)
	uint8_t detectI2CAddress(uint8_t chip_id) {
		// define the i2c addresses to test  
		uint8_t addresses[] = {0x77, 0x76};
		// define the register's address of the chip id (e.g. BMxxx_REGISTER_CHIPID)
		uint8_t register_address = 0xD0;
		// initialize wire
		Wire.begin();
		// for each i2c address to test
		for (uint8_t i = 0; i < sizeof(addresses); i++) { 
			uint8_t i2c_address = addresses[i];
			// read the value from the register (e.g. read8())
			Wire.beginTransmission((uint8_t)i2c_address);
			Wire.write((uint8_t)register_address);
			Wire.endTransmission();
			Wire.requestFrom((uint8_t)i2c_address, (byte)1);
			uint8_t value = Wire.read();
			// found the expected chip id, this is the correct i2c address to use
			if (value == chip_id) {
				debug(PSTR(LOG_SENSOR "%s:I2C a=0x%x\n"),_name,i2c_address);
				return i2c_address;
			}
		}
		debug(PSTR("!" LOG_SENSOR "%s:I2C ADDR\n"),_name);
		return addresses[0]; 
	};
	
#if NODEMANAGER_OTA_CONFIGURATION == ON
	// define what to do when receiving an OTA configuration request
	void onOTAConfiguration(ConfigurationRequest* request) {
		switch(request->getFunction()) {
		case 101: setForecastSamplesCount(request->getValueInt()); break;
		default: return;
		}
	};
#endif
	
protected:
#if !defined(NODEMANAGER_SENSOR_BOSCH_LITE)
	// returns the average of the latest pressure samples
	float _getLastPressureSamplesAverage() {
		float avg = 0;
		for (int i = 0; i < _forecast_samples_count; i++) avg += _forecast_samples[i];
		avg /= _forecast_samples_count;
		return avg;
	};
	
	// calculate and send the forecast back
	const char* _forecast(float pressure) {
		if (isnan(pressure)) return "";
		// Calculate the average of the last n minutes.
		int index = _minute_count % _forecast_samples_count;
		_forecast_samples[index] = pressure;
		_minute_count++;
		if (_minute_count > 185) _minute_count = 6;
		if (_minute_count == 5) _pressure_avg = _getLastPressureSamplesAverage();
		else if (_minute_count == 35) {
			float last_pressure_avg = _getLastPressureSamplesAverage();
			float change = (last_pressure_avg - _pressure_avg) * 0.1;
			// first time initial 3 hour
			if (_first_round) _dP_dt = change * 2; // note this is for t = 0.5hour
			else _dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
		}
		else if (_minute_count == 65) {
			float last_pressure_avg = _getLastPressureSamplesAverage();
			float change = (last_pressure_avg - _pressure_avg) * 0.1;
			//first time initial 3 hour
			if (_first_round) _dP_dt = change; //note this is for t = 1 hour
			else _dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
		}
		else if (_minute_count == 95) {
			float last_pressure_avg = _getLastPressureSamplesAverage();
			float change = (last_pressure_avg - _pressure_avg) * 0.1;
			// first time initial 3 hour
			if (_first_round)_dP_dt = change / 1.5; // note this is for t = 1.5 hour
			else _dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
		}
		else if (_minute_count == 125) {
			float last_pressure_avg = _getLastPressureSamplesAverage();
			// store for later use.
			_pressure_avg2 = last_pressure_avg; 
			float change = (last_pressure_avg - _pressure_avg) * 0.1;
			if (_first_round) _dP_dt = change / 2; // note this is for t = 2 hour
			else _dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
		}
		else if (_minute_count == 155) {
			float last_pressure_avg = _getLastPressureSamplesAverage();
			float change = (last_pressure_avg - _pressure_avg) * 0.1;
			if (_first_round) _dP_dt = change / 2.5; // note this is for t = 2.5 hour
			else _dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
		}
		else if (_minute_count == 185) {
			float last_pressure_avg = _getLastPressureSamplesAverage();
			float change = (last_pressure_avg - _pressure_avg) * 0.1;
			if (_first_round) _dP_dt = change / 3; // note this is for t = 3 hour
			else _dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
		}
		// Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
		_pressure_avg = _pressure_avg2; 
		// flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
		_first_round = false; 
		// calculate the forecast (STABLE = 0, SUNNY = 1, CLOUDY = 2, UNSTABLE = 3, THUNDERSTORM = 4, UNKNOWN = 5)
		int forecast = 5;
		//if time is less than 35 min on the first 3 hour interval.
		if (_minute_count < 35 && _first_round) forecast = 5;
		else if (_dP_dt < (-0.25)) forecast = 5;
		else if (_dP_dt > 0.25) forecast = 4;
		else if ((_dP_dt > (-0.25)) && (_dP_dt < (-0.05))) forecast = 2;
		else if ((_dP_dt > 0.05) && (_dP_dt < 0.25)) forecast = 1;
		else if ((_dP_dt >(-0.05)) && (_dP_dt < 0.05)) forecast = 0;
		else forecast = 5;
		return _weather[forecast];
	};
#endif
};
#endif