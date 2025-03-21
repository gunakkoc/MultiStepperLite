[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# MultiStepperLite

MultiStepperLite is a lightweight C++ library designed for independently controlling multiple stepper motors with a single microcontroller in a non-blocking manner. 

With this task based operation, the MCU can also carry out other heavy workloads, such as communication and reading sensors. Time autocorrection functionallity is also implemented so that even random delays between motor task calls are accounted for and the steppers can finish their steps in time without these delays accumulating.

This library is the stand-alone motor control library. For a full implementation example (a multichannel peristaltic pump) including serial communication, Python and SiLa2 interfaces see [HiPeristaltic](https://github.com/gunakkoc/HiPeristaltic)

## Features

- Control multiple stepper motors simultaneously and independently.
- No delay functions or interrupts are used.
- Lightweight and efficient for embedded systems.
- Compatible with many microcontroller platforms (Arduino, Raspberry Pi Pico, STM32, ...)
- Compatible with the most used stepper drivers (A4988, DRV8825, TMC2208, TB6600, ...)
- Easy-to-use API for quick integration.
- Supports custom time keeping (i.e., 32-bit timers with prescalers for sub-microsecond precision)

## Getting Started

1. Clone or download this repository.
2. Copy the `MultiStepperLite` folder into your project's library directory.
3. Include the library in your code:
    ```cpp
    #include "MultiStepperLite.h"
    ```

## Minimal Example

```cpp
#include "MultiStepperLite.h"
#define motor0_enabledPin 8
#define motor0_stepPin 2
#define motor0_dirPin 5

#define motor1_enabledPin 7
#define motor1_stepPin 3
#define motor1_dirPin 6

steppers = MultiStepperLite(2); //initialize for 2 motors

bool motor0_finish_signalled = false;

void setup(){
    Serial.begin(9600);

    //enable motor's and set directions
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
    steppers.start_finite(0, 4000, 4294967295);
    
    //start motor 1 to run indefinitely, with 2000 microseconds delay between steps
    steppers.start_continuous(1, 2000);

}

void loop(){

    stepper.do_tasks();
    //alternatively, define uint32_t now_us = micros() and call steppers.do_tasks(now_us)
    //this can be useful if micros() is already called for other purposes, as micros() is rather costly to call
    //without an argument, the function calls micros() internally

    if (steppers.if_finished(0)){ //if motor 0 completed all the steps
        if (!motor0_finish_signalled) { //if end of motor task is not signalled already
            Serial.println("Motor 0 is finished.");
            motor0_finish_signalled = true;
        }
    } 

    //do other tasks
}
```

## Documentation

For advanced features refer to the examples provided in the `examples` folder.

- A basic example with 3 motors using a generic CNC shield: `examples/CNC_shield/`
- Operation with endstops example: `examples/endstops/`
- Time autocorrection example: `examples/time_autocorrection/`
- Custom timekeeping example: `examples/custom_timekeeping/`

## Notes

### Timekeeping

By default, `MultiStepperLite` uses micros() and therefore microseconds for timekeeping. All defaults such as minimum pulse width for motor driver to register a change (default 2us), and minimum step interval of a motor (default 1000us) are defined in microseconds. Please see `examples/custom_timekeeping/` if you want to use a different timer.

### Time Autocorrection

Time autocorrection requires defining minimum step interval (max. speed) the motor can handle. This value needs to be provided by the user:

```cpp
steppers.set_min_step_interval(MOTOR_INDEX, MIN_MOTOR_STEP_INTERVAL);
```

Please make sure the step interval is long enough that the motor doesn't skip steps. A typical Nema17 stepper without a load has approximately minimum ~1000us step interval (also the default).

Autocorrection can be enabled with:
```cpp
steppers.set_autocorrect(true);
```

With autocorrection turned on (default is off), the delays in calling `do_tasks()` are compensated by running the next step(s) with shorter frequencies but not shorter than `min_step_interval`. With this, the motor will try to finish the task in `step_count x step_interval` time. In severe cases the motor might take a few fast steps. Hence, if the maximum consistency between the steppings is targeted, then keep this feature disabled.

Please refer to the full example in `examples/time_autocorrection/`.

## License

This project is licensed under the Apache 2.0 License. See the [LICENSE](LICENSE) file for details.
