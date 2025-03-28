#define SLOW_PROCESSOR 1
#define TIME_AUTOCORRECT_SUPPORT 1
// #define current_motor_time micros()

#include "SingleStepperLite.h"
#include "MultiStepperLite.h"

SingleStepperLite s;
MultiStepperLite m(2);
uint32_t s_startus;
uint32_t m0_startus;
uint32_t m1_startus;

uint32_t s_endus = 0;
uint32_t m0_endus = 0;
uint32_t m1_endus = 0;
void setup() {
  Serial.begin(115200);
  s.init_stepper(2);
  s.set_min_pulse_width(1);
  s.set_min_step_interval(3);
  s.set_autocorrect(false);

  m.init_stepper(0,3);
  m.init_stepper(1,4);
  m.set_min_pulse_width(1);
  m.set_min_step_interval(0,3);
  m.set_min_step_interval(1,3);
  m.set_autocorrect(false);

  bool result = true;
  s_startus = micros();
  result = result & s.start_finite(5000,2000);

  m0_startus = micros();
  result = result & m.start_finite(0,5000,2000);

  m1_startus = micros();
  result = result & m.start_finite(1,6000,1000);

  if (result){
      Serial.println("Started.");
  } else {
      Serial.println("Failed.");
  }
}

void loop() {
    s.do_task();
    m.do_tasks();
    //delayMicroseconds(random(1000));
    if (!s.is_running() && (s_endus==0)){
      s_endus = micros();
    }
    if (!m.is_running(0) && (m0_endus==0)){
      m0_endus = micros();
    }
    if (!m.is_running(1) && (m1_endus==0)){
      m1_endus = micros();
    }
    if (!s.is_running() && !m.is_running(0) && !m.is_running(1)){
        uint32_t now_us = micros();
        Serial.println("s runtime us (expected 10sec): ");
        Serial.println(s_endus - s_startus);
        Serial.println("m0 runtime us (expected 10sec): ");
        Serial.println(m0_endus - m0_startus);
        Serial.println("m1 runtime us (expected 6 sec): ");
        Serial.println(m1_endus - m1_startus);
        while (1){}
    }
}