#pragma once

/*
 * Code was inspired by https://github.com/xkam1x/Arduino-PWM-Reader/blob/master/PWM.cpp
 * - I modified the library code because it allocated too much memory (the state for 20 pins, I need just 2)
 */

#define STEERING 0
#define THROTTLE 1

unsigned long rc_input_time[2];
unsigned int rc_input[2];
bool rc_input_state[2];

int pins[2];

void rc_input_changed(int input) {
  unsigned long now = micros();
  bool state_now = digitalRead(pins[input]);
  if (state_now != rc_input_state[input]) {
    if (state_now == HIGH) {
      // start the stopwatch when the voltage is HIGH
      rc_input_time[input] = now;
    } else {
      // when the voltage drops to LOW, measure how long it was HIGH
      // => that's the width of the PWM pulse
      rc_input[input] = (unsigned int)(now - rc_input_time[input]);
    }

    rc_input_state[input] = state_now;
  }
}

void rc_input_steering_changed() {
  rc_input_changed(STEERING);
}

void rc_input_throttle_changed() {
  rc_input_changed(THROTTLE);
}

#define PIN_RC_INPUT_THROTTLE 3
#define PIN_RC_INPUT_STEERING 2

void attach_rc_input_interrupts() {
  pins[STEERING] = PIN_RC_INPUT_STEERING;
  pins[THROTTLE] = PIN_RC_INPUT_THROTTLE;

  pinMode(PIN_RC_INPUT_STEERING, INPUT);
  pinMode(PIN_RC_INPUT_THROTTLE, INPUT);

  // read the initial state
  rc_input_state[STEERING] = digitalRead(pins[STEERING]);
  rc_input_state[THROTTLE] = digitalRead(pins[THROTTLE]);

  rc_input[STEERING] = STEERING_CENTER_PWM;
  rc_input[THROTTLE] = THROTTLE_NONE_PWM;

  attachInterrupt(digitalPinToInterrupt(PIN_RC_INPUT_STEERING), rc_input_steering_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_INPUT_THROTTLE), rc_input_throttle_changed, CHANGE);
}

void detach_rc_input_interrupts() {
  detachInterrupt(digitalPinToInterrupt(PIN_RC_INPUT_STEERING));
  detachInterrupt(digitalPinToInterrupt(PIN_RC_INPUT_THROTTLE));
}
