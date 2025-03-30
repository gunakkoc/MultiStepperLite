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

#ifndef SINGLESTEPPERLITE_H
#define SINGLESTEPPERLITE_H

// --- DEFINITIONS GUIDE ---

// SLOW_PROCESSOR set to 1 for slow 8 bit processors (e.g. ATmega328P) //saves 4 bytes per motor and many 32-bit operations
// SLOW_PROCESSOR set to 0 for faster processors (e.g. ESP32, RP2040, STM32)
// If you are sure the calls between do_tasks() will be longer than MIN_PULSE_WIDTH, set SLOW_PROCESSOR to 0
// Typically, just the micros() call on AVR (slow) processors is > 3 microseconds, enough for virtually all stepper drivers.

// If #define current_motor_time is used, then all timings will be evaluated on every access to a timekeeping variable
// Example: #define current_motor_time micros() for Arduino
// Example: #define current_motor_time time_us_32() for Rasberry Pi Pico series
// Example: #define current_motor_time TIM1->CNT for STM32
// if not defined, then every time the do_tasks() function is called, the current time will be evaluated only once
// this saves a lot of time on slow processors such as ATmega328P where calling micros() takes >3microseconds

// TIME_AUTOCORRECT_SUPPORT set to 1 to enable time autocorrection support
// TIME_AUTOCORRECT_SUPPORT set to 0 to disable time autocorrection support, saves 16 bytes per motor

// DEF_MIN_PULSE_WIDTH is the default minimum pulse width (default 2 in microseconds)
// DEF_MIN_STEP_INTERVAL is the default minimum step interval (default 1000 in microseconds)

// NOTE: if non-microsecond timer is used, then the minimum pulse width and minimum step interval must be set accordingly

// --- END OF DEFINITIONS GUIDE ---

#ifdef ARDUINO //if Arduino framework is used

	#include <Arduino.h>
	#ifdef ARDUINO_ARCH_AVR
		#ifndef SLOW_PROCESSOR
			#define SLOW_PROCESSOR 1 //set to 1 for slow 8 bit processors (e.g. ATmega328P) //saves 4 bytes per motor and many 32-bit operations
		#endif //SLOW_PROCESSOR
	#else //ARDUINO_ARCH_AVR
		#ifndef SLOW_PROCESSOR
			#define SLOW_PROCESSOR 0 //set to 0 for faster processors (e.g. ESP32, RP2040, STM32)
		#endif //SLOW_PROCESSOR
	#endif //ARDUINO_ARCH_AVR

#else //for non-Arduino frameworks 

	#ifdef PICO_BOARD
		#ifndef SLOW_PROCESSOR
			#define SLOW_PROCESSOR 0 //set to 0 for faster processors (e.g. ESP32, RP2040, STM32)
		#endif //SLOW_PROCESSOR
		#ifndef current_motor_time
			#include "pico/time.h"
			#define current_motor_time time_us_32() //use the time_us_32() function for Pico, it is atomic hence no cost.
		#endif //current_motor_time
	#endif //PICO_BOARD

	#ifdef STM32
		#ifndef SLOW_PROCESSOR
			#define SLOW_PROCESSOR 0 //set to 0 for faster processors (e.g. ESP32, RP2040, STM32)
		#endif //SLOW_PROCESSOR

		//#define current_motor_time TIM5->CNT //use the TIM2->CNT for STM32, it is atomic hence no cost.
		#ifndef current_motor_time
			#error "current_motor_time needs to be defined for STM32. Please see the header file for details."
		#endif //current_motor_time
		// EXAMPLE TIM5 as 32-bit microsecond timer
		// void MX_TIM5_Init(void) {
		//     // Enable TIM5 clock
		//     __HAL_RCC_TIM5_CLK_ENABLE();

		//     htim5.Instance = TIM5;
		//     htim5.Init.Prescaler = (SystemCoreClock / 1000000) - 1;  // 84MHz → PSC=83 → 1MHz tick
		//     htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
		//     htim5.Init.Period = 0xFFFFFFFF;  // Max 32-bit value (free-running counter)
		//     htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		//     htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  // No reload needed

		//     if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		//         Error_Handler();  // Handle error
		//     }

		//     // Start the timer
		//     HAL_TIM_Base_Start(&htim5);
		// }
	#endif //STM32

	#ifndef SLOW_PROCESSOR //if not defined MCU, assume fast
		#define SLOW_PROCESSOR 0
	#endif //SLOW_PROCESSOR

#endif //ARDUINO

#ifndef TIME_AUTOCORRECT_SUPPORT
	#define TIME_AUTOCORRECT_SUPPORT 1 //set to 0 to disable time correction support, saves 16 bytes per motor
	#ifndef DEF_MIN_STEP_INTERVAL
		#define DEF_MIN_STEP_INTERVAL 1000 //default minimum step interval in microseconds
	#endif
#endif

#ifndef DEF_MIN_PULSE_WIDTH
	#define DEF_MIN_PULSE_WIDTH 2 //typically maximum 2 microseconds (for DRV8825, and shorter for others)
#endif

class SingleStepperLite {

	public:
		SingleStepperLite();
		void do_tasks(); //this function must be called frequently to keep the motors running, ideally in the main loop
		void set_min_pulse_width(uint32_t min_pulse_width); //generally needed when providing your own 32-bit timekeeping variable (e.g. a timer with 2us resolution)
		void set_step_interval(uint32_t step_interval); //change the step interval of a running motor
		void init_stepper(int step_pin);
		bool start_finite(uint32_t step_interval, uint32_t step_count);
		bool start_continuous(uint32_t step_interval);
#ifndef current_motor_time
		void do_tasks(uint32_t current_motor_time); //only to be used when providing your own 32-bit timekeeping variable
		void resume(uint32_t current_motor_time); //only to be used when providing your own 32-bit timekeeping variable
		bool start_continuous(uint32_t step_interval, uint32_t current_motor_time); //only to be used when providing your own 32-bit timekeeping variable
		bool start_finite(uint32_t step_interval, uint32_t step_count, uint32_t current_motor_time); //only to be used when providing your own 32-bit timekeeping variable
#endif
		void stop();
		void pause();
		void resume();
		bool is_running();
		bool is_finished(); //returns true if the motor is not running and has no remaining steps
		bool is_paused(); //returns true if the motor is not running but has remaining steps
		uint32_t get_remaining_steps();
	#if TIME_AUTOCORRECT_SUPPORT
		void set_min_step_interval(uint32_t min_interval); //required when enabling time autocorrect
		void set_autocorrect(bool autocorrect); //enable or disable time autocorrection
	#endif
	
	private:
#ifndef current_motor_time
		inline void _do_tasks(uint32_t current_motor_time); //internal function to handle motor stepping
		bool _start_motor(uint32_t step_interval, uint32_t step_count, uint8_t finite_mode, uint32_t current_motor_time);
#else
		inline void _do_tasks(); //internal function to handle motor stepping
		bool _start_motor(uint32_t step_interval, uint32_t step_count, uint8_t finite_mode);
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

#endif // SINGLESTEPPERLITE_H