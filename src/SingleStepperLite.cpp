// Copyright 2025 Gun Deniz Akkoc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// https://github.com/gunakkoc/MultiStepperLite

#include <Arduino.h>
#include "SingleStepperLite.h"

#if TIME_AUTOCORRECT_SUPPORT
SingleStepperLite::SingleStepperLite() :  _min_pulse_width(DEF_MIN_PULSE_WIDTH), _time_autocorrect_enabled(false){}
#else
SingleStepperLite::SingleStepperLite() :  _min_pulse_width(DEF_MIN_PULSE_WIDTH){}
#endif

#if TIME_AUTOCORRECT_SUPPORT
// Here, current_time is used to calculate the time elapsed since the last motor step.
// If this delta time is equal to greater than the given step interval, the motor is stepped.
// Furthermore, if the delta time is greater than the step interval, the exact timing is missed.
// The lag is calculated as lag = delta_time - step_interval
// This lag is accumulated and corrected for the next step by moving the last step time back.
// This correction is limited by minimum step interval (the maximum speed) a stepper can achieve, provided by the user.
// The maximum correction is calculated as max_correctable_lag = target_step_interval - min_step_interval.
// Hence, max_correctable_lag is calculated when motor is started via start_finite or start_continuous.
// This strategy allows dynamically adopting to disruptions of frequent calling of the task function.

#ifndef current_time
inline void SingleStepperLite::_do_task_autocorrect(uint32_t current_time) {
#else
inline void SingleStepperLite::_do_task_autocorrect() {
#endif
	if (_m.running){
		return; //if not running, move on to the next motor
	}
	if (_m.steps){ //if remaning step count > 0
		if (_m.last_pin_state){ //if the last pin state is HIGH
#if SLOW_PROCESSOR 
			//for slow 8 bit processors, no need for minimum pulse width checking
			digitalWrite(_m.step_pin, LOW);
			_m.last_pin_state = false;
			return; //HIGH to LOW is done, move on to the next motor
#else
			//for 32-bit faster processors, minimum pulse width is enforced
			//thanksfully, these extra 32-bit operations are not costly in this case.
			if ((current_time - _m.last_high_time) >= _min_pulse_width){ //if minimum time has passed since the last HIGH
				digitalWrite(_m.step_pin, LOW);
				_m.last_low_time = current_time;
				_m.last_pin_state = false;
				return; //HIGH to LOW is done, move on to the next motor
			}
			return; //else move on to the next motor (until _min_pulse_width elapses)
#endif
		}
		_motor_delta_time = current_time - _m.last_corrected_high_time; //the time elapsed since the last (corrected) stepping time
		if (_motor_delta_time >= _m.step_interval){ //if step interval time has passed since the last stepping
#if SLOW_PROCESSOR == 0
			//need to check for minimum pulse width for faster processors
			if ((current_time - _m.last_low_time) < _min_pulse_width){
				return;
			} //if minimum time has not passed since the last LOW
#endif
			digitalWrite(_m.step_pin, HIGH); //take a step by pulling from LOW to HIGH
			_m.last_high_time = current_time;
			_m.last_corrected_high_time = current_time;
			_m.last_pin_state = true;
			if (_m.finite_mode) { //if finite mode, then decrease by 1, continuous mode doesn't decrease
				--_m.steps;
			}
			_m.total_lag += _motor_delta_time - _m.step_interval;
			if (_m.total_lag) {
				if (_m.total_lag > _m.max_correctable_lag){
					_m.last_corrected_high_time -= _m.max_correctable_lag; //correct maximum correctable time
					_m.total_lag -= _m.max_correctable_lag;
				} else {
					_m.last_corrected_high_time -= _m.total_lag; //correct all
					_m.total_lag = 0;
				}
			} else {
				_m.last_corrected_high_time = _m.last_high_time; //no correction needed
			}
			return; //stepping took place, move on to the next motor
		}
		return; //else move on to the next motor (until step_interval elapses)
	}
	//No remaining steps; ensure the motor is stopped gracefully.
	//Need to ensure: the last pin state is LOW and and enough time passed for it to be registered
#if SLOW_PROCESSOR
	digitalWrite(_m.step_pin, LOW);
	_m.last_pin_state = false;
	_m.running = false;
	return; //HIGH to LOW is done, move on to the next motor
#else
	if (_m.last_pin_state){ //if the last pin state is HIGH
		if ((current_time - _m.last_high_time) >= _min_pulse_width){ //if enough time has passed for HIGH to be registered by the motor drive
			digitalWrite(_m.step_pin, LOW); //pull LOW
			_m.last_pin_state = false;
			_m.last_low_time = current_time;
			return; //HIGH to LOW is done, move on to the next motor
		}
		return; //else move on to the next motor (until _min_pulse_width elapses)
	}
	if ((current_time - _m.last_low_time) >= _min_pulse_width){ //if enough time has passed for LOW to be registered by the motor driver
		_m.running = false; //now we can signal the motor is finished.
		return; //move on to the next motor
	}
	return; //else LOW to be registered by the motor driver will be waited.
#endif
}

void SingleStepperLite::set_min_step_interval(uint32_t min_interval){
	if (min_interval < (_min_pulse_width * 2)){return;}
	_m.min_step_interval = min_interval;
	if (_time_autocorrect_enabled && _m.running && (_m.step_interval >= min_interval)){
		_m.max_correctable_lag = _m.step_interval - min_interval;
	}
}

void SingleStepperLite::set_autocorrect(bool autocorrect){
	//if switching from disabled to enabled, handle last_corrected_high_time for running motors
	if (autocorrect && !_time_autocorrect_enabled){
		if (_m.running){
			_m.last_corrected_high_time = _m.last_high_time;
		}
	}
	_time_autocorrect_enabled = autocorrect;
}
#endif

void SingleStepperLite::set_step_interval(uint32_t step_interval){
	if (step_interval < (_min_pulse_width * 2)){return;}
#if TIME_AUTOCORRECT_SUPPORT
	_m.max_correctable_lag = step_interval - _m.min_step_interval;
#endif
	_m.step_interval = step_interval;
}

// Here, current_time is used to calculate the time elapsed since the last motor step.
// If this delta time is equal to greater than the given step interval, the motor is stepped.
// Each stepping is triggered by pulling the step pin from LOW to HIGH.
// The minimum pulse width is enforced to ensure the stepper driver registers the change.
// Moreover, the task aims to pull the step pin back to LOW after the minimum pulse width.
// Motor always finishes with LOW on the step pin and enough time passed for it to be registered.
#ifndef current_time
inline void SingleStepperLite::_do_task(uint32_t current_time) {
#else
inline void SingleStepperLite::_do_task() {
#endif
#if TIME_AUTOCORRECT_SUPPORT
	if (_time_autocorrect_enabled){
#ifndef current_time
		_do_task_autocorrect(current_time); //route to the autocorrect version
#else
		_do_task_autocorrect() //route to the autocorrect version
#endif
		return;
	}
#endif
	if (!_m.running){
		return; //if not running, move on to the next motor
	}
	if (_m.steps){ //if remaning step count > 0
		if (_m.last_pin_state){ //if the last pin state is HIGH
#if SLOW_PROCESSOR 
			//for slow 8 bit processors, no need for minimum pulse width checking
			digitalWrite(_m.step_pin, LOW);
			_m.last_pin_state = false;
			return; //HIGH to LOW is done, move on to the next motor
#else
			//for 32-bit faster processors, minimum pulse width is enforced
			//thanksfully, these extra 32-bit operations are not costly in this case.
			if ((current_time - _m.last_high_time) >= _min_pulse_width){ //if minimum time has passed since the last HIGH
				digitalWrite(_m.step_pin, LOW);
				_m.last_low_time = current_time;
				_m.last_pin_state = false;
				return; //HIGH to LOW is done, move on to the next motor
			}
			return; //else move on to the next motor (until _min_pulse_width elapses)
#endif
		}
		if ((current_time - _m.last_high_time) >= _m.step_interval){ //if step interval time has passed since the last HIGH
#if SLOW_PROCESSOR == 0
			//need to check for minimum pulse width for faster processors
			if ((current_time - _m.last_low_time) < _min_pulse_width){
				return;
			} //if minimum time has not passed since the last LOW
#endif
			digitalWrite(_m.step_pin, HIGH); //take a step by pulling from LOW to HIGH
			_m.last_high_time = current_time;
			_m.last_pin_state = true;
			if (_m.finite_mode) { //if finite mode, then decrease by 1, continuous mode doesn't decrease
				--_m.steps;
			}
			return; //stepping took place, move on to the next motor
		}
		return; //else move on to the next motor (until step_interval elapses)
	}
	//If no remaining steps then ensure the motor is stopped gracefully.
	//So that the last pin state is LOW and and enough time passed for it to be registered.
#if SLOW_PROCESSOR
	digitalWrite(_m.step_pin, LOW);
	_m.last_pin_state = false;
	_m.running = false;
	return; //HIGH to LOW is done, move on to the next motor
#else
	if (_m.last_pin_state){ //if the last pin state is HIGH
		if ((current_time - _m.last_high_time) >= _min_pulse_width){ //if minimum time has passed for stepper driver to register a change 
			digitalWrite(_m.step_pin, LOW);
			_m.last_low_time = current_time;
			_m.last_pin_state = false;
			return; //HIGH to LOW is done, move on to the next motor
		}
		return; //else move on to the next motor (until _min_pulse_width elapses)
	}
	if ((current_time - _m.last_low_time) >= _min_pulse_width){ //if enough time has passed for LOW to be registered by the motor driver
		_m.running = false; //now we can signal the motor is finished.
		return; //move on to the next motor
	}
	return; //else LOW to be registered by the motor driver will be waited.
#endif
}

#ifndef current_time
void SingleStepperLite::do_task(uint32_t current_time){
	_do_task(current_time);
}
#endif

void SingleStepperLite::do_task(){
#ifndef current_time
	_do_task(micros());
#else
	_do_task();	
#endif
}

void SingleStepperLite::set_min_pulse_width(uint32_t min_pulse_width){
	_min_pulse_width = min_pulse_width;
}

void SingleStepperLite::init_stepper(int step_pin){
	pinMode(step_pin, OUTPUT);
	digitalWrite(step_pin, LOW);
	_m.step_pin = step_pin;
	_m.last_pin_state = false;
	_m.steps = 0;
	_m.running = false;
#if TIME_AUTOCORRECT_SUPPORT
	_m.min_step_interval = DEF_MIN_STEP_INTERVAL;
	// _m.total_lag = 0;
#endif
// 	_m.last_high_time = 0;
// #if SLOW_PROCESSOR == 0
// 	_m.last_low_time = 0;
// #endif
// #if TIME_AUTOCORRECT_SUPPORT
// 	_m.last_corrected_high_time = 0;
// 	_m.max_correctable_lag = 0;
// #endif
}

#ifndef current_time
bool SingleStepperLite::_start_motor(uint32_t step_interval, uint32_t step_count, uint8_t finite_mode, uint32_t current_time){
#else
bool SingleStepperLite::_start_motor(uint32_t step_interval, uint32_t step_count, uint8_t finite_mode){
#endif
	if (step_interval < (_min_pulse_width * 2)){return false;}
#if TIME_AUTOCORRECT_SUPPORT
	if (_time_autocorrect_enabled && (step_interval < _m.min_step_interval) ){return false;}
	_m.total_lag = 0;
	_m.max_correctable_lag = step_interval - _m.min_step_interval;
	_m.last_corrected_high_time = current_time;
#endif
	digitalWrite(_m.step_pin, LOW);
	_m.last_high_time = current_time;
#if SLOW_PROCESSOR == 0
	_m.last_low_time = current_time;
#endif
	_m.last_pin_state = false;
	_m.steps = step_count;
	_m.step_interval = step_interval;
	_m.finite_mode = finite_mode; //1 for finite mode, 0 for continuous mode
	_m.running = true;
	return true;
}

#ifndef current_time
bool SingleStepperLite::start_finite(uint32_t step_interval, uint32_t step_count, uint32_t current_time){
	return _start_motor(step_interval, step_count, 1, current_time);
}
#endif

bool SingleStepperLite::start_finite(uint32_t step_interval, uint32_t step_count){
#ifndef current_time
	return _start_motor(step_interval, step_count, 1, micros());
#else
	return _start_motor(step_interval, step_count, 1);
#endif
}

#ifndef current_time
bool SingleStepperLite::start_continuous(uint32_t step_interval, uint32_t current_time){
	return _start_motor(step_interval, 1, 0, current_time);
}
#endif

bool SingleStepperLite::start_continuous(uint32_t step_interval){
#ifndef current_time
	return _start_motor(step_interval, 1, 0, micros());
#else
	return _start_motor(step_interval, 1, 0);
#endif
}

void SingleStepperLite::stop(){ //this will stop the motor immediately
	_m.steps = 0;
	//the _m.running must be checked for graceful finish
}

void SingleStepperLite::pause(){ //this will pause the motor immediately
	_m.running = false;
}

#ifndef current_time
void SingleStepperLite::resume(){
	resume(micros());
}

void SingleStepperLite::resume(uint32_t current_time){
	_m.last_high_time = current_time;
#if TIME_AUTOCORRECT_SUPPORT
	_m.last_corrected_high_time = current_time;
	_m.total_lag = 0;
#endif
#if SLOW_PROCESSOR == 0
	_m.last_low_time = current_time;
#endif
	_m.running = true;
}
#else
void SingleStepperLite::resume(){
	_m.last_high_time = current_time;
#if TIME_AUTOCORRECT_SUPPORT
	_m.last_corrected_high_time = current_time;
	_m.total_lag = 0;
#endif
#if SLOW_PROCESSOR == 0
	_m.last_low_time = current_time;
#endif
	_m.running = true;
}
#endif

bool SingleStepperLite::is_running(){
	return _m.running;
}

bool SingleStepperLite::is_finished(){
	return ((!_m.running) && (_m.steps == 0));
}

bool SingleStepperLite::is_paused(){
	return ((!_m.running) && (_m.steps > 0));
}

uint32_t SingleStepperLite::get_remaining_steps(){
	return _m.steps;
}