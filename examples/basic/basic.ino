// Run 2 motors concurrently with MultiStepperLite

#include "MultiStepperLite.h"
#define motor0_enabledPin 8
#define motor0_stepPin 2
#define motor0_dirPin 5

#define motor1_enabledPin 7
#define motor1_stepPin 3
#define motor1_dirPin 6

MultiStepperLite steppers(2); //initialize for 2 motors

bool motor0_finish_signalled = false;

void setup(){
    Serial.begin(9600);

    //enable both motors and set directions
    pinMode(motor0_enabledPin, OUTPUT);
    pinMode(motor0_dirPin, OUTPUT);
    digitalWrite(motor0_stepPin, LOW);
    digitalWrite(motor0_enabledPin, LOW);

    pinMode(motor1_enabledPin, OUTPUT);
    pinMode(motor1_dirPin, OUTPUT);
    digitalWrite(motor1_dirPin, LOW);
    digitalWrite(motor1_enabledPin, LOW);

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

    if (steppers.is_finished(0)){ //if motor 0 completed all the steps
        if (!motor0_finish_signalled) { //if end of motor task is not signalled already
            Serial.println("Motor 0 is finished.");
            motor0_finish_signalled = true;
        }
    }
    //do other tasks
}
