#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdint.h>
#include "stepmotor.h"

#if STEPMOTOR_Q == 1
#define PORT_INTERRUPT_FLAG (1 << COM1A0)
#else
#define PORT_INTERRUPT_FLAG ((1 << COM1A0) | (1 << COM1B0))
#endif

#ifndef ENA_PIN
// 8th pin on arduino
#define ENA_PIN (1 << 0)
#endif

#ifndef ACCELERATION_STEP
#define ACCELERATION_STEP 1
#endif // ACCELERATION_STEP

uint16_t motor_intervals[STEPMOTOR_Q];
uint32_t motor_constants[STEPMOTOR_Q];
volatile uint32_t motor_steps[STEPMOTOR_Q] = {0};
uint32_t motor_accel_steps[STEPMOTOR_Q] = {0};
static uint16_t current_speed = 0;
uint16_t target_speed;

void motors_init() {
  memset(motor_intervals, UINT8_MAX, sizeof(motor_intervals));
  uint8_t sreg = SREG;
  cli();
#if STEPMOTOR_Q == 1
  DDRB |= 0x03; // enable PB1 and PB2 as output pins
#else
  DDRB |= 0x07; // enable PB1, PB2 and PB3 as output pins
#endif
  OCR1A = 0;
  TCCR1A = 0;
  TCNT1 = 0;
  TCCR1B = 1 << CS10;
  TIMSK1 = (1 << OCIE1A);

  SREG = sreg;
}

#define speed(a, x) ((a)/(x))

uint8_t accelerate() {
#if STEPMOTOR_Q == 1
  if (motor_steps[0] == 0) { // All motors stopped
#else
  if (motor_steps[0] == 0 && motor_steps[1] == 0) { // All motors stopped
#endif
    PORTB |= ENA_PIN;
    current_speed = 0;
    return 0;
  }
  uint16_t local_intervals[STEPMOTOR_Q];
  if (current_speed == 0) {
    PORTB &= ~ENA_PIN; // Restart motors
    current_speed = MINSPEED;
    local_intervals[0] = speed(motor_constants[0], MINSPEED);
    motor_accel_steps[0] = motor_steps[0] / 2; // Setting upper bound to prevent overacceleration
#if STEPMOTOR_Q == 2
    local_intervals[1] = speed(motor_constants[1], MINSPEED);
    motor_accel_steps[1] = motor_steps[1] / 2;
#endif // STEPMOTOR_Q == 2
    uint8_t sreg = SREG;
    cli();
    memcpy(motor_intervals, local_intervals, sizeof(motor_intervals));
    TCCR1A |= PORT_INTERRUPT_FLAG;
    OCR1A = motor_intervals[0] + TCNT1;
#if STEPMOTOR_Q == 2
    OCR1B = motor_intervals[1] + TCNT1;
#endif
    SREG = sreg;
    return 1;
  }
#if STEPMOTOR_Q == 1
  if (motor_steps[0] <= motor_accel_steps[0]) {
#else
  if (motor_steps[0] <= motor_accel_steps[0] || motor_steps[1] <= motor_accel_steps[1]) {
#endif
    if (current_speed == MINSPEED)
      return;
    current_speed -= ACCELERATION_STEP;
  } else {
    if (current_speed >= target_speed) {
      if (current_speed == target_speed) {
        // By one errors are not critical here
        motor_accel_steps[0] = (motor_accel_steps[0] * 2) - motor_steps[0];
#if STEPMOTOR_Q == 2
        motor_accel_steps[1] = (motor_accel_steps[1] * 2) - motor_steps[1];
#endif // STEPMOTOR_Q == 2
        ++current_speed; // FIXME: using current speed as marker of accel_steps processed
      }
      return 1;
    }
    current_speed += ACCELERATION_STEP;
  }
  local_intervals[0] = speed(motor_constants[0], current_speed);
#if STEPMOTOR_Q == 2
  local_intervals[1] = speed(motor_constants[1], current_speed);
#endif // STEPMOTOR_Q == 2
  uint8_t sreg = SREG;
  cli();
  memcpy(motor_intervals, local_intervals, sizeof(motor_intervals));
  SREG = sreg;
  return 1;
}

void emergency_stop () {
  TCCR1A &= ~((1 << COM1A0) | (1 << COM1B0));
  memset(motor_steps, 0, sizeof(motor_steps));
  memset(motor_intervals, UINT8_MAX, sizeof(motor_intervals));
}

ISR(TIMER1_COMPA_vect) {
  OCR1A += motor_intervals[0];
  if (motor_steps[0] > 0) {
    --motor_steps[0];
    return;
  }
  TCCR1A &= ~(1 << COM1A0);
  motor_intervals[0] = UINT16_MAX;
}

#if STEPMOTOR_Q == 2
ISR(TIMER1_COMPB_vect) {
  OCR1B += motor_intervals[1];
  if (motor_steps[1] > 0) {
    --motor_steps[1];
    return;
  }
  TCCR1A &= ~(1 << COM1B0);
  motor_intervals[1] = UINT16_MAX;
}
#endif
