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

#ifndef MULTISTEPPERLITE_H
#define MULTISTEPPERLITE_H

#include <Arduino.h>

// Can define current motor time with an atomic 32-bit counter of a timer
// such as TIM1->CNT on STM32
// EX:
// #define current_motor_time TIM1->CNT

#ifndef SLOW_PROCESSOR
#ifdef ARDUINO_ARCH_AVR
#define SLOW_PROCESSOR 1 //set to 1 for slow 8 bit processors (e.g. ATmega328P) //saves 4 bytes per motor and many 32-bit operations
#else
#define SLOW_PROCESSOR 0 //set to 0 for faster processors (e.g. ESP32, RP2040, STM32)
#endif
#endif

#ifndef DEF_MIN_PULSE_WIDTH
#define DEF_MIN_PULSE_WIDTH 2 //typically maximum 2 microseconds (for DRV8825, and shorter for others)
#endif

//If you are sure the calls between do_tasks() will be longer than MIN_PULSE_WIDTH, set SLOW_PROCESSOR to 0
//Typically, just the micros() call on AVR (slow) processors is > 3 microseconds, enough for nearly all stepper drivers.

#ifndef TIME_AUTOCORRECT_SUPPORT
#define TIME_AUTOCORRECT_SUPPORT 1 //set to 0 to disable time correction support, saves 16 bytes per motor
#ifndef DEF_MIN_STEP_INTERVAL
#define DEF_MIN_STEP_INTERVAL 1000 //default minimum step interval in microseconds
#endif
#endif

#ifndef MAX_MOTOR_COUNT
#define MAX_MOTOR_COUNT 6 //maximum number of motors; total footprint ~= [17 (up to 37) bytes x motor count] + ~16bytes
#endif

class MultiStepperLite {

public:
	MultiStepperLite(uint8_t count);
	void do_tasks(); //this function must be called frequently to keep the motors running, ideally in the main loop
	void set_stepper_count(uint8_t motor_count);
	void set_min_pulse_width(uint32_t min_pulse_width); //generally needed when providing your own 32-bit timekeeping variable (e.g. a timer with 2us resolution)
	void set_step_interval(uint8_t motor_index, uint32_t step_interval); //change the step interval of a running motor
	void init_stepper(uint8_t motor_index, int step_pin);
	bool start_finite(uint8_t motor_index, uint32_t step_interval, uint32_t step_count);
	bool start_continuous(uint8_t motor_index, uint32_t step_interval);
#ifndef current_motor_time
	void do_tasks(uint32_t current_motor_time); //only to be used when providing your own 32-bit timekeeping variable
	void resume(uint8_t motor_index, uint32_t current_motor_time); //only to be used when providing your own 32-bit timekeeping variable
	bool start_continuous(uint8_t motor_index, uint32_t step_interval, uint32_t current_motor_time); //only to be used when providing your own 32-bit timekeeping variable
	bool start_finite(uint8_t motor_index, uint32_t step_interval, uint32_t step_count, uint32_t current_motor_time); //only to be used when providing your own 32-bit timekeeping variable
#endif
	void stop(uint8_t motor_index);
	void pause(uint8_t motor_index);
	void resume(uint8_t motor_index);
	bool is_running(uint8_t motor_index);
	bool is_finished(uint8_t motor_index); //returns true if the motor is not running and has no remaining steps
	bool is_paused(uint8_t motor_index); //returns true if the motor is not running but has remaining steps
	uint32_t get_remaining_steps(uint8_t motor_index);
#if TIME_AUTOCORRECT_SUPPORT
	void set_min_step_interval(uint8_t motor_index, uint32_t min_interval); //required when enabling time autocorrect
	void set_autocorrect(bool autocorrect); //enable or disable time autocorrection
#endif

private:
#ifndef current_motor_time
	inline void _do_tasks(uint32_t current_motor_time); //internal function to handle motor stepping
	bool _start_motor(uint8_t motor_index, uint32_t step_interval, uint32_t step_count, uint8_t finite_mode, uint32_t current_motor_time);
#else
	inline void _do_tasks(); //internal function to handle motor stepping
	bool _start_motor(uint8_t motor_index, uint32_t step_interval, uint32_t step_count, uint8_t finite_mode);
#endif
	uint8_t _stepper_count;
	uint32_t _min_pulse_width;
#if TIME_AUTOCORRECT_SUPPORT
	uint32_t _motor_delta_time;
	bool _time_autocorrect_enabled;
#ifndef current_motor_time
	void _do_tasks_autocorrect(uint32_t current_motor_time);
#else
	void _do_tasks_autocorrect();
#endif
#endif
};

#endif // MULTISTEPPERLITE_H