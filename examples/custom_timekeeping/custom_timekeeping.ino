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

	uint32_t now_ms = millis();
	led_last_blink_time = now_ms;
	//Since the custom timekeeping variable is millis(), all time definitions will be in milliseconds

	steppers.set_min_pulse_width(1); //set minimum pulse width to 1 millisecond
	//initialize each of 2 motors with their step pin
	steppers.init_stepper(0, motor0_stepPin);
	steppers.init_stepper(1, motor1_stepPin);

	//min step interval must not be smaller than (min_pulse_width * 2)
	steppers.set_min_step_interval(0, 3); //set minimum step interval for motor 0 to 3 milliseconds
	steppers.set_min_step_interval(1, 3); //set minimum step interval for motor 1 to 3 milliseconds

	steppers.set_autocorrect(true); //enable time autocorrection

	//start motor 0, with 6 milliseconds delay between steps and with finite steps of 1000.
	steppers.start_finite(0, 10, 1000, now_ms);

	//start motor 1, with 8 milliseconds delay between steps and with finite steps of 2000.
	steppers.start_finite(1, 8, 2000, now_ms);
	

}

void loop(){

	uint32_t current_time_ms = millis(); //current milliseconds

	//do the tasks for the steppers, using time keeping variable in milliseconds
	steppers.do_tasks(current_time_ms);

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