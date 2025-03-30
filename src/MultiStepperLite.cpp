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

#include "MultiStepperLite.h"
#ifdef ARDUINO
	#include <Arduino.h>
#endif
//TODO digitalWrite, LOW, HIGH macros for non-Arduino frameworks

namespace {
	typedef struct { //17 bytes
		int step_pin;
		bool running;
		bool last_pin_state;
		uint8_t finite_mode; //1 for finite mode, 0 for continuous mode
		uint32_t step_interval;
		uint32_t steps; //keeps track of remaining number of steps (if finite mode)
		uint32_t last_high_time;
	#if SLOW_PROCESSOR == 0 //+4 bytes
		uint32_t last_low_time; 
	#endif
	#if TIME_AUTOCORRECT_SUPPORT //+16 bytes
		uint32_t last_corrected_high_time;
		uint32_t total_lag;
		uint32_t min_step_interval;
		uint32_t max_correctable_lag;
	#endif
	} StepperMotor_t;
	StepperMotor_t _stepper_motors[MAX_MOTOR_COUNT];
}

#if TIME_AUTOCORRECT_SUPPORT
MultiStepperLite::MultiStepperLite(uint8_t count) : _stepper_count(count), _min_pulse_width(DEF_MIN_PULSE_WIDTH), _time_autocorrect_enabled(false){}
#else
MultiStepperLite::MultiStepperLite(uint8_t count) : _stepper_count(count), _min_pulse_width(DEF_MIN_PULSE_WIDTH){}
#endif

#if TIME_AUTOCORRECT_SUPPORT
// Here, current_motor_time is used to calculate the time elapsed since the last motor step.
// If this delta time is equal to greater than the given step interval, the motor is stepped.
// Furthermore, if the delta time is greater than the step interval, the exact timing is missed.
// The lag is calculated as lag = delta_time - step_interval
// This lag is accumulated and corrected for the next step by moving the last step time back.
// This correction is limited by minimum step interval (the maximum speed) a stepper can achieve, provided by the user.
// The maximum correction is calculated as max_correctable_lag = target_step_interval - min_step_interval.
// Hence, max_correctable_lag is calculated when motor is started via start_finite or start_continuous.
// This strategy allows dynamically adopting to disruptions of frequent calling of the task function.
#ifndef current_motor_time
inline void MultiStepperLite::_do_tasks_autocorrect(uint32_t current_motor_time) {
#else
inline void MultiStepperLite::_do_tasks_autocorrect() {
#endif
	StepperMotor_t *m = _stepper_motors;
	uint8_t i = _stepper_count;
	while (i--){
		if (!m->running){
			++m;
			continue; //if not running, move on to the next motor
		}
		if (m->steps){ //if remaning step count > 0
			if (m->last_pin_state){ //if the last pin state is HIGH
#if SLOW_PROCESSOR 
				//for slow 8 bit processors, no need for minimum pulse width checking
				digitalWrite(m->step_pin, LOW);
				m->last_pin_state = false;
				++m;
				continue; //HIGH to LOW is done, move on to the next motor
#else
				//for 32-bit faster processors, minimum pulse width is enforced
				//thanksfully, these extra 32-bit operations are not costly in this case.
				if ((current_motor_time - m->last_high_time) >= _min_pulse_width){ //if minimum time has passed since the last HIGH
					digitalWrite(m->step_pin, LOW);
					m->last_low_time = current_motor_time;
					m->last_pin_state = false;
					++m;
					continue; //HIGH to LOW is done, move on to the next motor
				}
				++m;
				continue; //else move on to the next motor (until _min_pulse_width elapses)
#endif
			}
			_motor_delta_time = current_motor_time - m->last_corrected_high_time; //the time elapsed since the last (corrected) stepping time
			if (_motor_delta_time >= m->step_interval){ //if step interval time has passed since the last stepping
#if SLOW_PROCESSOR == 0
				//need to check for minimum pulse width for faster processors
				if ((current_motor_time - m->last_low_time) < _min_pulse_width){
					++m;
					continue;
				} //if minimum time has not passed since the last LOW
#endif
				digitalWrite(m->step_pin, HIGH); //take a step by pulling from LOW to HIGH
				m->last_high_time = current_motor_time;
				m->last_corrected_high_time = m->last_high_time;
				m->last_pin_state = true;
				if (m->finite_mode) { //if finite mode, then decrease by 1, continuous mode doesn't decrease
					--m->steps;
				}
				m->total_lag += _motor_delta_time - m->step_interval;
				if (m->total_lag) {
					if (m->total_lag > m->max_correctable_lag){
						m->last_corrected_high_time -= m->max_correctable_lag; //correct maximum correctable time
						m->total_lag -= m->max_correctable_lag;
					} else {
						m->last_corrected_high_time -= m->total_lag; //correct all
						m->total_lag = 0;
					}
				}
				++m;
				continue; //stepping took place, move on to the next motor
			}
			++m;
			continue; //else move on to the next motor (until step_interval elapses)
		}
		//No remaining steps; ensure the motor is stopped gracefully.
		//Need to ensure: the last pin state is LOW and and enough time passed for it to be registered
#if SLOW_PROCESSOR
		digitalWrite(m->step_pin, LOW);
		m->last_pin_state = false;
		m->running = false;
		++m;
		continue; //HIGH to LOW is done, move on to the next motor
#else
		if (m->last_pin_state){ //if the last pin state is HIGH
			if ((current_motor_time - m->last_high_time) >= _min_pulse_width){ //if enough time has passed for HIGH to be registered by the motor drive
				digitalWrite(m->step_pin, LOW); //pull LOW
				m->last_pin_state = false;
				m->last_low_time = current_motor_time;
				++m;
				continue; //HIGH to LOW is done, move on to the next motor
			}
			++m;
			continue; //else move on to the next motor (until _min_pulse_width elapses)
		}
		if ((current_motor_time - m->last_low_time) >= _min_pulse_width){ //if enough time has passed for LOW to be registered by the motor driver
			m->running = false; //now we can signal the motor is finished.
			++m;
			continue; //move on to the next motor
		}
		++m;
		continue; //else LOW to be registered by the motor driver will be waited.
#endif
	}
}

void MultiStepperLite::set_min_step_interval(uint8_t motor_index, uint32_t min_interval){
	if (motor_index >= _stepper_count){return;}
	if (min_interval < (_min_pulse_width * 2)){return;}
	StepperMotor_t *m = &_stepper_motors[motor_index];
	m->min_step_interval = min_interval;
	if (_time_autocorrect_enabled && m->running && (m->step_interval >= min_interval)){
		m->max_correctable_lag = m->step_interval - min_interval;
	}
}

void MultiStepperLite::set_autocorrect(bool autocorrect){
	//if switching from disabled to enabled, handle last_corrected_high_time for running motors
	if (autocorrect && !_time_autocorrect_enabled){
		uint8_t i = _stepper_count;
		StepperMotor_t *m = _stepper_motors;
		while(i--){
			if (m->running){
				m->last_corrected_high_time = m->last_high_time;
			}
			++m;
		}
	}
	_time_autocorrect_enabled = autocorrect;
}
#endif

void MultiStepperLite::set_step_interval(uint8_t motor_index, uint32_t step_interval){
	if (motor_index >= _stepper_count){return;}
	if (step_interval < (_min_pulse_width * 2)){return;}
#if TIME_AUTOCORRECT_SUPPORT
	StepperMotor_t *m = &_stepper_motors[motor_index];
	m->max_correctable_lag = step_interval - m->min_step_interval;
	m->step_interval = step_interval;
#else
	_stepper_motors[motor_index].step_interval = step_interval;
#endif
}

// Here, current_motor_time is used to calculate the time elapsed since the last motor step.
// If this delta time is equal to greater than the given step interval, the motor is stepped.
// Each stepping is triggered by pulling the step pin from LOW to HIGH.
// The minimum pulse width is enforced to ensure the stepper driver registers the change.
// Moreover, the task aims to pull the step pin back to LOW after the minimum pulse width.
// Motor always finishes with LOW on the step pin and enough time passed for it to be registered.
#ifndef current_motor_time
inline void MultiStepperLite::_do_tasks(uint32_t current_motor_time) {
#else
inline void MultiStepperLite::_do_tasks() {
#endif
#if TIME_AUTOCORRECT_SUPPORT
	if (_time_autocorrect_enabled){
#ifndef current_motor_time
		_do_tasks_autocorrect(current_motor_time); //route to the autocorrect version
#else
		_do_tasks_autocorrect(); //route to the autocorrect version
#endif
		return;
	}
#endif
	uint8_t i = _stepper_count;
	StepperMotor_t *m = _stepper_motors;
	while(i--){
		if (!m->running){
			++m;
			continue; //if not running, move on to the next motor
		}
		if (m->steps){ //if remaning step count > 0
			if (m->last_pin_state){ //if the last pin state is HIGH
#if SLOW_PROCESSOR 
				//for slow 8 bit processors, no need for minimum pulse width checking
				digitalWrite(m->step_pin, LOW);
				m->last_pin_state = false;
				++m;
				continue; //HIGH to LOW is done, move on to the next motor
#else
				//for 32-bit faster processors, minimum pulse width is enforced
				//thanksfully, these extra 32-bit operations are not costly in this case.
				if ((current_motor_time - m->last_high_time) >= _min_pulse_width){ //if minimum time has passed since the last HIGH
					digitalWrite(m->step_pin, LOW);
					m->last_low_time = current_motor_time;
					m->last_pin_state = false;
					++m;
					continue; //HIGH to LOW is done, move on to the next motor
				}
				++m;
				continue; //else move on to the next motor (until _min_pulse_width elapses)
#endif
			}
			if ((current_motor_time - m->last_high_time) >= m->step_interval){ //if step interval time has passed since the last HIGH
#if SLOW_PROCESSOR == 0
				//need to check for minimum pulse width for faster processors
				if ((current_motor_time - m->last_low_time) < _min_pulse_width){
					++m;
					continue;
				} //if minimum time has not passed since the last LOW
#endif
				digitalWrite(m->step_pin, HIGH); //take a step by pulling from LOW to HIGH
				m->last_high_time = current_motor_time;
				m->last_pin_state = true;
				if (m->finite_mode) { //if finite mode, then decrease by 1, continuous mode doesn't decrease
					--m->steps;
				}
				++m;
				continue; //stepping took place, move on to the next motor
			}
			++m;
			continue; //else move on to the next motor (until step_interval elapses)
		}
		//If no remaining steps then ensure the motor is stopped gracefully.
		//So that the last pin state is LOW and and enough time passed for it to be registered.
#if SLOW_PROCESSOR
		digitalWrite(m->step_pin, LOW);
		m->last_pin_state = false;
		m->running = false;
		++m;
		continue; //HIGH to LOW is done, move on to the next motor
#else
		if (m->last_pin_state){ //if the last pin state is HIGH
			if ((current_motor_time - m->last_high_time) >= _min_pulse_width){ //if minimum time has passed for stepper driver to register a change 
				digitalWrite(m->step_pin, LOW);
				m->last_low_time = current_motor_time;
				m->last_pin_state = false;
				++m;
				continue; //HIGH to LOW is done, move on to the next motor
			}
			++m;
			continue; //else move on to the next motor (until _min_pulse_width elapses)
		}
		if ((current_motor_time - m->last_low_time) >= _min_pulse_width){ //if enough time has passed for LOW to be registered by the motor driver
			m->running = false; //now we can signal the motor is finished.
			++m;
			continue; //move on to the next motor
		}
		++m;
		continue; //else LOW to be registered by the motor driver will be waited.
#endif
	}
}

#ifndef current_motor_time
void MultiStepperLite::do_tasks(uint32_t current_motor_time){
	_do_tasks(current_motor_time);
}
#endif

void MultiStepperLite::do_tasks(){
#ifndef current_motor_time
	_do_tasks(micros());
#else
	_do_tasks();
#endif
}

void MultiStepperLite::set_stepper_count(uint8_t motor_count){
	if (motor_count > MAX_MOTOR_COUNT){return;}
	_stepper_count = motor_count;
}

void MultiStepperLite::set_min_pulse_width(uint32_t min_pulse_width){
	_min_pulse_width = min_pulse_width;
}

void MultiStepperLite::init_stepper(uint8_t motor_index, int step_pin){
	if (motor_index >= _stepper_count){return;}
	pinMode(step_pin, OUTPUT);
	digitalWrite(step_pin, LOW);
	StepperMotor_t *m = &_stepper_motors[motor_index];
	m->step_pin = step_pin;
	m->last_pin_state = false;
	m->steps = 0;
	m->running = false;
#if TIME_AUTOCORRECT_SUPPORT
	m->min_step_interval = DEF_MIN_STEP_INTERVAL;
	// m->total_lag = 0;
#endif
// 	m->last_high_time = 0;
// #if SLOW_PROCESSOR == 0
// 	m->last_low_time = 0;
// #endif
// #if TIME_AUTOCORRECT_SUPPORT
// 	m->last_corrected_high_time = 0;
// 	m->max_correctable_lag = 0;
// #endif
}

#ifndef current_motor_time
bool MultiStepperLite::_start_motor(uint8_t motor_index, uint32_t step_interval, uint32_t step_count, uint8_t finite_mode, uint32_t current_motor_time){
#else
bool MultiStepperLite::_start_motor(uint8_t motor_index, uint32_t step_interval, uint32_t step_count, uint8_t finite_mode){
#endif
	if (motor_index >= _stepper_count){return false;}
	StepperMotor_t *m = &_stepper_motors[motor_index];
	if (step_interval < (_min_pulse_width * 2)){return false;}
#if TIME_AUTOCORRECT_SUPPORT
	if (_time_autocorrect_enabled && (step_interval < m->min_step_interval) ){return false;}
	m->total_lag = 0;
	m->max_correctable_lag = step_interval - m->min_step_interval;
	m->last_corrected_high_time = current_motor_time;
#endif
	digitalWrite(m->step_pin, LOW);
	m->last_high_time = current_motor_time;
#if SLOW_PROCESSOR == 0
	m->last_low_time = current_motor_time;
#endif
	m->last_pin_state = false;
	m->steps = step_count;
	m->step_interval = step_interval;
	m->finite_mode = finite_mode; //1 for finite mode, 0 for continuous mode
	m->running = true;
	return true;
}

#ifndef current_motor_time
bool MultiStepperLite::start_finite(uint8_t motor_index, uint32_t step_interval, uint32_t step_count, uint32_t current_motor_time){
	return _start_motor(motor_index, step_interval, step_count, 1, current_motor_time);
}
#endif

bool MultiStepperLite::start_finite(uint8_t motor_index, uint32_t step_interval, uint32_t step_count){
#ifndef current_motor_time
	return _start_motor(motor_index, step_interval, step_count, 1, micros());
#else
	return _start_motor(motor_index, step_interval, step_count, 1);
#endif
}

#ifndef current_motor_time
bool MultiStepperLite::start_continuous(uint8_t motor_index, uint32_t step_interval, uint32_t current_motor_time){
	return _start_motor(motor_index, step_interval, 1, 0, current_motor_time);
}
#endif

bool MultiStepperLite::start_continuous(uint8_t motor_index, uint32_t step_interval){
#ifndef current_motor_time
	return _start_motor(motor_index, step_interval, 1, 0, micros());
#else
	return _start_motor(motor_index, step_interval, 1, 0);
#endif
}

void MultiStepperLite::stop(uint8_t motor_index){ //this will stop the motor immediately
	if (motor_index >= _stepper_count){return;}
	_stepper_motors[motor_index].steps = 0;
	//the _stepper_motors[n].running must be checked for graceful finish
}

void MultiStepperLite::pause(uint8_t motor_index){ //this will pause the motor immediately
	if (motor_index >= _stepper_count){return;}
	_stepper_motors[motor_index].running = false;
}

#ifndef current_motor_time
void MultiStepperLite::resume(uint8_t motor_index){
	resume(motor_index, micros());
}

void MultiStepperLite::resume(uint8_t motor_index, uint32_t current_motor_time){
	if (motor_index >= _stepper_count){return;}
	StepperMotor_t *m = &_stepper_motors[motor_index];
	m->last_high_time = current_motor_time;
#if TIME_AUTOCORRECT_SUPPORT
	m->last_corrected_high_time = current_motor_time;
	m->total_lag = 0;
#endif
#if SLOW_PROCESSOR == 0
	m->last_low_time = current_motor_time;
#endif
	m->running = true;
}
#else
void MultiStepperLite::resume(uint8_t motor_index){
	if (motor_index >= _stepper_count){return;}
	StepperMotor_t *m = &_stepper_motors[motor_index];
	m->last_high_time = current_motor_time;
#if TIME_AUTOCORRECT_SUPPORT
	m->last_corrected_high_time = current_motor_time;
	m->total_lag = 0;
#endif
#if SLOW_PROCESSOR == 0
	m->last_low_time = current_motor_time;
#endif
	m->running = true;
}
#endif

bool MultiStepperLite::is_running(uint8_t motor_index){
	if (motor_index >= _stepper_count){return false;}
	return _stepper_motors[motor_index].running;
}

bool MultiStepperLite::is_finished(uint8_t motor_index){
	if (motor_index >= _stepper_count){return false;}
	return ((!_stepper_motors[motor_index].running) && (_stepper_motors[motor_index].steps == 0));
}

bool MultiStepperLite::is_paused(uint8_t motor_index){
	if (motor_index >= _stepper_count){return false;}
	return ((!_stepper_motors[motor_index].running) && (_stepper_motors[motor_index].steps > 0));
}

uint32_t MultiStepperLite::get_remaining_steps(uint8_t motor_index){
	if (motor_index >= _stepper_count){return 0;}
	return _stepper_motors[motor_index].steps;
}