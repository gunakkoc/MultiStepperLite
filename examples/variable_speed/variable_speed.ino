// Change the speed of a running stepper motor on-the-fly.

#include "MultiStepperLite.h"
#define motor0_enabledPin 8
#define motor0_stepPin 2
#define motor0_dirPin 5

#define motor1_enabledPin 7
#define motor1_stepPin 3
#define motor1_dirPin 6

MultiStepperLite steppers(2); //initialize for 2 motors

bool led_last_state;
uint32_t led_last_blink_time;
uint32_t last_speed_change_time;

void setup(){

    //enable motor's and set directions
    pinMode(motor0_enabledPin, OUTPUT);
    pinMode(motor0_dirPin, OUTPUT);
    digitalWrite(motor0_stepPin, LOW);
    digitalWrite(motor0_enabledPin, LOW);

    pinMode(motor1_enabledPin, OUTPUT);
    pinMode(motor1_dirPin, OUTPUT);
    digitalWrite(motor1_dirPin, LOW);
    digitalWrite(motor1_enabledPin, LOW);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    led_last_state = LOW;
    led_last_blink_time = micros();
    last_speed_change_time = micros();

    //initialize each of 2 motors with their index and their step pin
    steppers.init_stepper(0, motor0_stepPin);
    steppers.init_stepper(1, motor1_stepPin);

    //start motor 0, with 4000 microseconds delay between steps and with finite steps of 2500
    steppers.start_finite(0, 4000, 2500);

    //start motor 1 to run indefinitely, with 2000 microseconds delay between steps
    steppers.start_continuous(1, 2000);

}

void loop(){

    steppers.do_tasks();
    //alternatively, define uint32_t now_us = micros() and call steppers.do_tasks(now_us)
    //this can be useful if micros() is already called for other purposes, as micros() is rather costly to call
    //without an argument, the function calls micros() internally

    uint32_t now_us = micros(); //current microseconds

    //Blink the LED until motor 0 is finished
    if (steppers.is_finished(0)){
        digitalWrite(LED_BUILTIN, HIGH);
    } else {
        if ((now_us - led_last_blink_time) > 1000000){ //else if 1 second has passed since the last change of LED_BUILTIN
            led_last_state = !led_last_state;
            digitalWrite(LED_BUILTIN, led_last_state);
            led_last_blink_time = now_us; //save the last time the state of LED has changed
        }
    }

    //Every 3 seconds, change the speed of motor 1.
    if ((now_us - last_speed_change_time) > 3000000){
        last_speed_change_time = now_us;
        if (steppers.is_running(1)){
            steppers.set_step_interval(1, 2000); //change the step interval to 2000 microseconds
        } else {
            steppers.set_step_interval(1, 4000); //change the step interval to 4000 microseconds
        }
    }

    //do other tasks
}