// Author Gun Deniz Akkoc (2025) | github.com/gunakkoc/MultiStepperLite
// License: Apache 2.0

// This example demostrates the automatic time correction capability of the MultiStepperLite.
// Even with blocking heavy workloads in the main loop, the motors will try to catch up.
// The motors will never exceed maximum motor speed, dictated via set_min_step_interval() function.
// At the end, the motors should finish their tasks in time without the delays accumulating.

#include "MultiStepperLite.h"
#define motor0_enabledPin 8
#define motor0_stepPin 2
#define motor0_dirPin 5
#define motor1_enabledPin 7
#define motor1_stepPin 3
#define motor1_dirPin 6

bool led_last_state;
unsigned long led_last_blink_time;

MultiStepperLite steppers(2); //initialize for 2 motors

void setup(){
	pinMode(motor0_enabledPin, OUTPUT);
	pinMode(motor0_stepPin, OUTPUT);
	pinMode(motor0_dirPin, OUTPUT);
	digitalWrite(motor0_stepPin, LOW);
	digitalWrite(motor0_dirPin, LOW);
	digitalWrite(motor0_enabledPin, LOW);

	pinMode(motor1_enabledPin, OUTPUT);
	pinMode(motor1_stepPin, OUTPUT);
	pinMode(motor1_dirPin, OUTPUT);
	digitalWrite(motor1_stepPin, LOW);
	digitalWrite(motor1_dirPin, LOW);
	digitalWrite(motor1_enabledPin, LOW);


	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);
	led_last_state = LOW;
	led_last_blink_time = micros();
	
	//initialize each of 2 motors with their step pin
	steppers.init_stepper(0, motor0_stepPin);
	steppers.init_stepper(1, motor1_stepPin);
	
	//set minimum step interval microseconds the motors can handle. With no microstepping and free rotation, typically 1000 microseconds for Nema17
	//it is recommended to test the minimum value on your setup to ensure no step skipping occurs.

	//for instance, test motor 0 with 1000 microseconds step interval
	//and increase the value until the motor doesn't skip steps
	//steppers.start_continous(0, 1000);

	//this will be used for correcting time deviation between motor steps
	//this setting is required for enabling time autocorrection
	steppers.set_min_step_interval(0, 1000);
	steppers.set_min_step_interval(1, 1000);
	
	//enable automatic time correction, so when a motor step is delayed, the next step will happen faster.
	//this ensures the motor task ends on time even steppers.do_tasks() is called with irregular delays.
	//but this costs some computation and trades-off uniform intervals between steps.
	steppers.set_autocorrect(true);
	
	//start motor 1, with 4000 microseconds delay between steps and with finite steps of 2500, should take 10 seconds
	steppers.start_finite(1, 4000, 2500);
	
	//start motor 1, with 2000 microseconds delay between steps and with finite steps of 5000, should take 10 seconds
	steppers.start_finite(1, 2000, 5000);
	
}

void loop(){
	
	//with stepper_motor_set_autocorrect(true); both motors should complete at the same time that is after 10 seconds
	//even with random blocking workloads in between.
	steppers.do_tasks();
	//alternatively, call steppers.do_tasks(now_us); //if you have your own timekeeping variable
	//this can be useful if micros() is already called for other purposes, as micros() is rather costly to call
	//without an argument, the function calls micros() internally
	
	delayMicroseconds(random(1000)); //simulate a randomly long blocking task to disrupt the timing
	
	//LED blinking task
	if (steppers.is_finished(0) && steppers.is_finished(1)){ //if both motor 0 and 1 are finished
		digitalWrite(LED_BUILTIN, HIGH);
	} else {
		unsigned long now_us = micros();
		if ((now_us - led_last_blink_time) > 1000000){ //else if 1 second has passed since the last change of LED_BUILTIN
			led_last_state = !led_last_state;
			digitalWrite(LED_BUILTIN, led_last_state);
			led_last_blink_time = now_us; //save the last time the state of LED has changed
		}
	}
}