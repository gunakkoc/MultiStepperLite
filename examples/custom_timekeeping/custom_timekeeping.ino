// Author Gun Deniz Akkoc (2025) | github.com/gunakkoc/MultiStepperLite
// License: Apache 2.0

// An example of using MultiStepperLite with custom timekeeping.

// Without the extra arguments, MultiStepperLite uses micros() for timekeeping by default.
// If a custom timer is used, for instance with higher resolution, then a timekeeping variable can be passed as an argument.
// - The timekeeping variable has to be 32bits and must roll over after reaching 2^32.
// - The related functions where the extra current_time argument can be provided are:
// start_finite(uint8_t motor_index, uint32_t step_interval, uint32_t step_count, uint32_t current_time);
// start_continuous(uint8_t motor_index, uint32_t step_interval, uint32_t current_time);
// resume(uint8_t motor_index, uint32_t current_time);
// do_tasks(uint32_t current_time);
// - Please note, for this case, minimum pulse width must be adjusted accordingly:
// set_min_pulse_width(uint32_t min_pulse_width);
// - When autocorrecting time, the minimum step interval must be set for each motor using the custom time units:
// set_min_step_interval(uint8_t motor_index, uint32_t min_interval);

// In this example, millis() is used as the custom timekeeping variable.

#include "MultiStepperLite.h"
#define motor0_enabledPin 8
#define motor0_stepPin 2
#define motor0_dirPin 5
#define motor0_endstopPin 9
#define motor1_enabledPin 7
#define motor1_stepPin 3
#define motor1_dirPin 6
#define motor1_endstopPin 10

bool led_last_state;
unsigned long led_last_blink_time;

steppers = MultiStepperLite(2); //initialize for 2 motors

void setup(){
	pinMode(motor0_enabledPin, OUTPUT);
	pinMode(motor0_stepPin, OUTPUT);
	pinMode(motor0_dirPin, OUTPUT);
	pinMode(motor0_endstopPin, INPUT);
	digitalWrite(motor0_stepPin, LOW);
	digitalWrite(motor0_dirPin, LOW);
	digitalWrite(motor0_enabledPin, LOW);

	pinMode(motor1_enabledPin, OUTPUT);
	pinMode(motor1_stepPin, OUTPUT);
	pinMode(motor1_dirPin, OUTPUT);
	pinMode(motor1_endstopPin, INPUT);
	digitalWrite(motor1_stepPin, LOW);
	digitalWrite(motor1_dirPin, LOW);
	digitalWrite(motor1_enabledPin, LOW);


	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);
	led_last_state = LOW;
	led_last_blink_time = millis();


    //Since the custom timekeeping variable is millis(), all time definitions will be in milliseconds

    steppers.set_min_pulse_width(1); //set minimum pulse width to 1 millisecond
	//initialize each of 2 motors with their step pin
	steppers.init_stepper(0, motor0_stepPin);
	steppers.init_stepper(1, motor1_stepPin);

    steppers.set_min_step_interval(0, 1); //set minimum step interval for motor 0 to 1 millisecond
    steppers.set_min_step_interval(1, 2); //set minimum step interval for motor 1 to 2 milliseconds

    steppers.set_autocorrect(true); //enable time autocorrection
	
	//start motor 0, with 6 milliseconds delay between steps and with finite steps of 4294967295 (max number of steps)
	steppers.start_finite(0, 10, 4294967295);
	
	//start motor 1, with 8 milliseconds delay between steps and with finite steps of 4294967295 (max number of steps)
	steppers.start_finite(1, 8, 4294967295);
	

}

void loop(){

    uint32_t current_time_ms = millis(); //current milliseconds
	
	if (digitalRead(motor0_endstopPin) == LOW){ //if end stop of motor 0 is triggered
		//you can access remaining steps here via nsteps = steppers.get_remaining_steps(0);
		steppers.stop(0); //stop motor 0, also resets remaining steps to 0
	}
	
	if (digitalRead(motor1_endstopPin) == LOW){ //if end stop of motor 1 is triggered
	    //you can access remaining steps here via nsteps = steppers.get_remaining_steps(1);
		steppers.stop(1); //stop motor 1, also resets remaining steps to 0
	}
	

    //do the tasks for the steppers, using time keeping variable in milliseconds
	stepper.do_tasks(current_time_ms);
	
	//LED blinking task
	if (steppers.is_finished(0) && steppers.is_finished(1)){ //if both motor 0 and 1 are finished
		digitalWrite(LED_BUILTIN, HIGH);
	} else {
		if ((current_time_ms - led_last_blink_time) > 1000){ //else if 1 second has passed since the last change of LED_BUILTIN
			led_last_state = !led_last_state;
			digitalWrite(LED_BUILTIN, led_last_state);
			led_last_blink_time = current_time_ms; //save the last time the state of LED has changed
		}
	}
}