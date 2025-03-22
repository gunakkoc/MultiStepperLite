// A minimal demostration of running 3 motors indepedently and simultaneously using a generic CNC shield.
// While blinking the LED until the motors finish their tasks.
// No interrupts or delay functions are used. So that the user can prioritize tasks freely.

#include "MultiStepperLite.h"
#define motor_enabledPin 8
#define motor0_stepPin 2
#define motor0_dirPin 5
#define motor1_stepPin 3
#define motor1_dirPin 6
#define motor2_stepPin 4
#define motor2_dirPin 7

bool led_last_state;
uint32_t led_last_blink_time;
MultiStepperLite steppers(3); //initialize for 3 motors

void setup(){
	//X on the CNC shield
	pinMode(motor0_stepPin, OUTPUT);
	pinMode(motor0_dirPin, OUTPUT);
	digitalWrite(motor0_stepPin, LOW);
	digitalWrite(motor0_dirPin, LOW);

	//Y on the CNC shield
	pinMode(motor1_stepPin, OUTPUT);
	pinMode(motor1_dirPin, OUTPUT);
	digitalWrite(motor1_stepPin, LOW);
	digitalWrite(motor1_dirPin, LOW);


	//Z on the CNC shield
	pinMode(motor2_stepPin, OUTPUT);
	pinMode(motor2_dirPin, OUTPUT);
	digitalWrite(motor2_stepPin, LOW);
	digitalWrite(motor2_dirPin, LOW);

	pinMode(motor_enabledPin, OUTPUT); //on the generic CNC shield, a single pin controls all motors' enabled pins
	digitalWrite(motor_enabledPin, LOW);

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);
	led_last_state = LOW;
	led_last_blink_time = micros();
	
	//initialize each of 3 motors with their step pin
	steppers.init_stepper(0, motor0_stepPin);
	steppers.init_stepper(1, motor1_stepPin);
	steppers.init_stepper(2, motor2_stepPin);

	steppers.start_continuous(0, 2000); //start motor 0, in continuous mode, with 2000 microseconds delay between steps
	steppers.start_finite(1, 2000, 4000); //start motor 1, with 2000 microseconds delay between steps and with finite steps of 4000
	steppers.start_finite(2, 3000, 5000); //start motor 2, with 3000 microseconds delay between steps and with finite steps of 5000
}

void loop(){
	//This blinks the LED every second until motor 1 and 2 are running.
	//When these motors are done, also stop motor 0, which is set to run indefinitely.
	
	uint32_t now_us = micros(); //current microseconds
	
	steppers.do_tasks();
	//alternatively, call steppers.do_tasks(now_us) if you have your own timekeeping variable
	//this can be useful if micros() is already called for other purposes, as micros() is rather costly to call
	//without an argument, the function calls micros() internally.
	
	if (steppers.is_finished(1) && steppers.is_finished(2)){ //if both motor 1 and 2 are finished
		steppers.stop(0); //also stop motor 0
		digitalWrite(LED_BUILTIN, HIGH);
	} else if ((now_us - led_last_blink_time) > 1000000){ //else if 1 second has passed since the last change of LED_BUILTIN
		led_last_state = !led_last_state;
		digitalWrite(LED_BUILTIN, led_last_state);
		led_last_blink_time = now_us; //save the last time the state of LED has changed
	}
}