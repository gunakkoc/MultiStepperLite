// Author Gun Deniz Akkoc (2025) | github.com/gunakkoc/MultiStepperLite
// License: Apache 2.0

//	A minimal demostration of running 2 motors indepedently and simultaneously.
//	While blinking the LED until the motors finish their tasks or until the end stops are triggered.
//	No interrupts or delay functions are used. So that the user can prioritize tasks freely.


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

MultiStepperLite steppers(2); //initialize for 2 motors

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
	led_last_blink_time = micros();
	
	//initialize each of 2 motors with their step pin
	steppers.init_stepper(0, motor0_stepPin);
	steppers.init_stepper(1, motor1_stepPin);
	
	//start motor 0, with 4000 microseconds delay between steps and with finite steps of 4294967295 (max number of steps)
	steppers.start_finite(0, 4000, 4294967295);
	
	//start motor 1 to run indefinitely, with 2000 microseconds delay between steps
	steppers.start_continuous(1, 2000);
	
}

void loop(){
	
	if (digitalRead(motor0_endstopPin) == LOW){ //if end stop of motor 0 is triggered
		//you can access remaining steps here via nsteps = steppers.get_remaining_steps(0);
		steppers.stop(0); //stop motor 0, also resets remaining steps to 0
	}
	
	if (digitalRead(motor1_endstopPin) == LOW){ //if end stop of motor 1 is triggered
		steppers.stop(1); //stop motor 1
	}
	
	steppers.do_tasks();

	//alternatively, call steppers.do_tasks(now_us); //if you have your own microsecond timekeeping variable
	//this can be useful if micros() is already called for other purposes, as micros() is rather costly to call
	//without an argument, the function calls micros() internally
	
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