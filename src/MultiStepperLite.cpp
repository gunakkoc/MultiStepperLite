// Author Gun Deniz Akkoc (2025) | github.com/gunakkoc/MultiStepperLite
// License: Apache License 2.0

#include <Arduino.h>
#include "MultiStepperLite.h"

#if TIME_AUTOCORRECT_SUPPORT
MultiStepperLite::MultiStepperLite(uint8_t count) : _stepper_count(count), _min_pulse_width(DEF_MIN_PULSE_WIDTH), _time_autocorrect_enabled(false){}
#else
MultiStepperLite::MultiStepperLite(uint8_t count) : _stepper_count(count), _min_pulse_width(DEF_MIN_PULSE_WIDTH){}
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
void MultiStepperLite::_do_tasks_autocorrect(uint32_t current_time) {
	for (uint8_t i=0; i<_stepper_count; i++){
		StepperMotor_t *m = &_stepper_motors[i];
		if (!m->running){
			continue; //if not running, move on to the next motor
		}
		if (m->steps){ //if remaning step count > 0
			if (m->last_pin_state){ //if the last pin state is HIGH
#if SLOW_PROCESSOR 
				//for slow 8 bit processors, no need for minimum pulse width checking
				digitalWrite(m->step_pin, LOW);
				m->last_pin_state = false;
				continue; //HIGH to LOW is done, move on to the next motor
#else
				//for 32-bit faster processors, minimum pulse width is enforced
				//thanksfully, these extra 32-bit operations are not costly in this case.
				if ((current_time - m->last_high_time) >= _min_pulse_width){ //if minimum time has passed since the last HIGH
					digitalWrite(m->step_pin, LOW);
					m->last_low_time = current_time;
					m->last_pin_state = false;
					continue; //HIGH to LOW is done, move on to the next motor
				}
				continue; //else move on to the next motor (until _min_pulse_width elapses)
#endif
			}
			_motor_delta_time = current_time - m->last_corrected_high_time; //the time elapsed since the last (corrected) stepping time
			if (_motor_delta_time >= m->step_interval){ //if step interval time has passed since the last stepping
#if SLOW_PROCESSOR == 0
				//need to check for minimum pulse width for faster processors
				if ((current_time - m->last_low_time) < _min_pulse_width){continue;} //if minimum time has not passed since the last LOW
#endif
				digitalWrite(m->step_pin, HIGH); //take a step by pulling from LOW to HIGH
				m->last_high_time = current_time;
				m->last_corrected_high_time = current_time;
				m->last_pin_state = true;
				m->steps -= m->finite_mode; //if finite mode, then decrease by 1, continuous mode doesn't decrease
				m->total_lag += _motor_delta_time - m->step_interval;
				if (m->total_lag) {
					if (m->total_lag > m->max_correctable_lag){
						m->last_corrected_high_time -= m->max_correctable_lag; //correct maximum correctable time
						m->total_lag -= m->max_correctable_lag;
					} else {
						m->last_corrected_high_time -= m->total_lag; //correct all
						m->total_lag = 0;
					}
				} else {
					m->last_corrected_high_time = m->last_high_time; //no correction needed
				}
				continue; //stepping took place, move on to the next motor
			}
			continue; //else move on to the next motor (until step_interval elapses)
		}
		//No remaining steps; ensure the motor is stopped gracefully.
		//Need to ensure: the last pin state is LOW and and enough time passed for it to be registered
#if SLOW_PROCESSOR
		digitalWrite(m->step_pin, LOW);
		m->last_pin_state = false;
		m->running = false;
#else
		if (m->last_pin_state){ //if the last pin state is HIGH
			if ((current_time - m->last_high_time) >= _min_pulse_width){ //if enough time has passed for HIGH to be registered by the motor drive
				digitalWrite(m->step_pin, LOW); //pull LOW
				m->last_pin_state = false;
				m->last_low_time = current_time;
				continue; //HIGH to LOW is done, move on to the next motor
			}
			continue; //else move on to the next motor (until _min_pulse_width elapses)
		}
		if ((current_time - m->last_low_time) >= _min_pulse_width){ //if enough time has passed for LOW to be registered by the motor driver
			m->running = false; //now we can signal the motor is finished.
			continue; //move on to the next motor
		}
		//else LOW to be registered by the motor driver will be waited.
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
		for (uint8_t i=0; i<_stepper_count; i++){
			StepperMotor_t *m = &_stepper_motors[i];
			if (m->running){
				m->last_corrected_high_time = m->last_high_time;
			}
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

// Here, current_time is used to calculate the time elapsed since the last motor step.
// If this delta time is equal to greater than the given step interval, the motor is stepped.
// Each stepping is triggered by pulling the step pin from LOW to HIGH.
// The minimum pulse width is enforced to ensure the stepper driver registers the change.
// Moreover, the task aims to pull the step pin back to LOW after the minimum pulse width.
// Motor always finishes with LOW on the step pin and enough time passed for it to be registered.
void MultiStepperLite::do_tasks(uint32_t current_time) {
#if TIME_AUTOCORRECT_SUPPORT
	if (_time_autocorrect_enabled){
		_do_tasks_autocorrect(current_time); //route to the autocorrect version
		return;
	}
#endif
	for (uint8_t i=0; i<_stepper_count; i++){
		StepperMotor_t *m = &_stepper_motors[i];
		if (!m->running){
			continue; //if not running, move on to the next motor
		}
		if (m->steps){ //if remaning step count > 0
			if (m->last_pin_state){ //if the last pin state is HIGH
#if SLOW_PROCESSOR 
				//for slow 8 bit processors, no need for minimum pulse width checking
				digitalWrite(m->step_pin, LOW);
				m->last_pin_state = false;
				continue; //HIGH to LOW is done, move on to the next motor
#else
				//for 32-bit faster processors, minimum pulse width is enforced
				//thanksfully, these extra 32-bit operations are not costly in this case.
				if ((current_time - m->last_high_time) >= _min_pulse_width){ //if minimum time has passed since the last HIGH
					digitalWrite(m->step_pin, LOW);
					m->last_low_time = current_time;
					m->last_pin_state = false;
					continue; //HIGH to LOW is done, move on to the next motor
				}
				continue; //else move on to the next motor (until _min_pulse_width elapses)
#endif
			}
			if ((current_time - m->last_high_time) >= m->step_interval){ //if step interval time has passed since the last HIGH
#if SLOW_PROCESSOR == 0
				//need to check for minimum pulse width for faster processors
				if ((current_time - m->last_low_time) < _min_pulse_width){continue;} //if minimum time has not passed since the last LOW
#endif
				digitalWrite(m->step_pin, HIGH); //take a step by pulling from LOW to HIGH
				m->last_high_time = current_time;
				m->last_pin_state = true;
				m->steps -= m->finite_mode; //if finite mode, then decrease by 1, continuous mode doesn't decrease
				continue; //stepping took place, move on to the next motor
			}
			continue; //else move on to the next motor (until step_interval elapses)
		}
		//If no remaining steps then ensure the motor is stopped gracefully.
		//So that the last pin state is LOW and and enough time passed for it to be registered.
#if SLOW_PROCESSOR
		digitalWrite(m->step_pin, LOW);
		m->last_pin_state = false;
		m->running = false;
		continue; //HIGH to LOW is done, move on to the next motor
#else
		if (m->last_pin_state){ //if the last pin state is HIGH
			if ((current_time - m->last_high_time) >= _min_pulse_width){ //if minimum time has passed for stepper driver to register a change 
				digitalWrite(m->step_pin, LOW);
				m->last_low_time = current_time;
				m->last_pin_state = false;
				continue; //HIGH to LOW is done, move on to the next motor
			}
			continue; //else move on to the next motor (until _min_pulse_width elapses)
		}
		if ((current_time - m->last_low_time) >= _min_pulse_width){ //if enough time has passed for LOW to be registered by the motor driver
			m->running = false; //now we can signal the motor is finished.
			continue; //move on to the next motor
		}
		// continue; //else LOW to be registered by the motor driver will be waited.
#endif
	}
}

void MultiStepperLite::do_tasks(){
	do_tasks(micros());
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

bool MultiStepperLite::_start_motor(uint8_t motor_index, uint32_t step_interval, uint32_t step_count, uint8_t finite_mode, uint32_t current_time){
	if (motor_index >= _stepper_count){return false;}
	StepperMotor_t *m = &_stepper_motors[motor_index];
	if (step_interval < (_min_pulse_width * 2)){return false;}
#if TIME_AUTOCORRECT_SUPPORT
	if (_time_autocorrect_enabled && (step_interval < m->min_step_interval) ){return false;}
	m->total_lag = 0;
	m->max_correctable_lag = step_interval - m->min_step_interval;
	m->last_corrected_high_time = current_time;
#endif
	digitalWrite(m->step_pin, LOW);
	m->last_high_time = current_time;
#if SLOW_PROCESSOR == 0
	m->last_low_time = current_time;
#endif
	m->last_pin_state = false;
	m->steps = step_count;
	m->step_interval = step_interval;
	m->finite_mode = finite_mode; //1 for finite mode, 0 for continuous mode
	m->running = true;
	return true;
}

bool MultiStepperLite::start_finite(uint8_t motor_index, uint32_t step_interval, uint32_t step_count, uint32_t current_time){
	return _start_motor(motor_index, step_interval, step_count, 1, current_time);
}

bool MultiStepperLite::start_finite(uint8_t motor_index, uint32_t step_interval, uint32_t step_count){
	return _start_motor(motor_index, step_interval, step_count, 1, micros());
}

bool MultiStepperLite::start_continuous(uint8_t motor_index, uint32_t step_interval, uint32_t current_time){
	return _start_motor(motor_index, step_interval, 1, 0, current_time);
}

bool MultiStepperLite::start_continuous(uint8_t motor_index, uint32_t step_interval){
	return _start_motor(motor_index, step_interval, 1, 0, micros());
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

void MultiStepperLite::resume(uint8_t motor_index){
	resume(motor_index, micros());
}

void MultiStepperLite::resume(uint8_t motor_index, uint32_t current_time){
	if (motor_index >= _stepper_count){return;}
	StepperMotor_t *m = &_stepper_motors[motor_index];
	m->last_high_time = current_time;
#if TIME_AUTOCORRECT_SUPPORT
	m->last_corrected_high_time = current_time;
	m->total_lag = 0;
#endif
#if SLOW_PROCESSOR == 0
	m->last_low_time = current_time;
#endif
	m->running = true;
}

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