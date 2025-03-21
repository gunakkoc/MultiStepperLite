// Author Gun Deniz Akkoc (2025) | github.com/gunakkoc/MultiStepperLite
// License: Apache 2.0

#ifndef MULTISTEPPERLITE_H
#define MULTISTEPPERLITE_H

#include <Arduino.h>

#ifndef SLOW_PROCESSOR
#ifdef ARDUINO_ARCH_AVR
#define SLOW_PROCESSOR 1 //set to 1 for slow 8 bit processors (e.g. ATmega328P) //saves 4 bytes per motor and many 32-bit operations
#else
#define SLOW_PROCESSOR 0 //set to 0 for faster processors (e.g. ESP32, RP2040, STM32)
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
	void do_tasks(uint32_t current_time); //only to be used when providing your own 32-bit timekeeping variable
	void set_stepper_count(uint8_t motor_count);
	void set_min_pulse_width(uint32_t min_pulse_width); //generally needed when providing your own 32-bit timekeeping variable (e.g. a timer with 2us resolution)
	void init_stepper(uint8_t motor_index, int step_pin);
	bool start_finite(uint8_t motor_index, uint32_t step_interval, uint32_t step_count);
	bool start_finite(uint8_t motor_index, uint32_t step_interval, uint32_t step_count, uint32_t current_time); //only to be used when providing your own 32-bit timekeeping variable
	bool start_continuous(uint8_t motor_index, uint32_t step_interval);
	bool start_continuous(uint8_t motor_index, uint32_t step_interval, uint32_t current_time); //only to be used when providing your own 32-bit timekeeping variable
	void stop(uint8_t motor_index);
	void pause(uint8_t motor_index);
	void resume(uint8_t motor_index);
	void resume(uint8_t motor_index, uint32_t current_time); //only to be used when providing your own 32-bit timekeeping variable
	bool is_running(uint8_t motor_index);
	bool is_finished(uint8_t motor_index); //returns true if the motor is not running and has no remaining steps
	bool is_paused(uint8_t motor_index); //returns true if the motor is not running but has remaining steps
	uint32_t get_remaining_steps(uint8_t motor_index);
#if TIME_AUTOCORRECT_SUPPORT
	void set_min_step_interval(uint8_t motor_index, uint32_t min_interval); //required when enabling time autocorrect
	void set_autocorrect(bool autocorrect); //enable or disable time autocorrection
#endif

private:
	void _start_motor(uint8_t motor_index, uint32_t step_interval, uint32_t step_count, uint8_t finite_mode, uint32_t current_time);
	uint8_t _stepper_count;
	uint32_t _min_pulse_width;
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
#if TIME_AUTOCORRECT_SUPPORT
	uint32_t _motor_delta_time;
	bool _time_autocorrect_enabled;
	void _do_tasks_autocorrect(uint32_t current_time);
#endif
};

#endif // MULTISTEPPERLITE_H