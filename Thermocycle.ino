#include <EEPROM.h>
#include <GyverTM1637.h>
#include "stepmotor.h"
#include "NumberButtons.hpp"

#define ESTOP_PIN 3
#define ENDBUTTON_PIN 2
#define SWITCH_PIN 12
#define DIR_PIN 13
#define RELAY_PIN1 6
#define RELAY_PIN2 7
#define DISP_CLK_PIN 4
#define DISP_DIO_PIN 5
#define POSADD_PIN A0
#define POSSUB_PIN A1
#define CYCLEADD_PIN A2
#define CYCLESUB_PIN A3

/*------------------- Значение всех изменяемых величин -------------------*/
#define PRESCALER 1
#define MICROSTEP 3200
#define C1 1

#define CYCLE_TIME_MS 5000
// 128000 == 81 mm
#define CYCLE_LENGTH_STEPS (uint32_t)268642
#define NUMBER_CYCLES 10

#define ACCELERATION_STEP_TIME (F_CPU / 1000) // Every 1 ms

#define MOVE_SPEED 300
#define CALIBRATION_SPEED 50

GyverTM1637 cycles_disp(DISP_CLK_PIN, DISP_DIO_PIN);
NumberButtons<uint16_t> cycles_max(CYCLEADD_PIN, CYCLESUB_PIN, 9999, 10);

uint16_t cycle_start;
bool system_enabled = false;

void move_blocking() {
  uint8_t sreg = SREG;
  cli();
  uint16_t timer = TCNT1;
  SREG = sreg;
  while (accelerate()) {
    for (;;) {
      delayMicroseconds(4);
      cli();
      uint16_t timer2 = TCNT1;
      SREG = sreg;
      if (timer2 - timer >= ACCELERATION_STEP_TIME) {
        timer += ACCELERATION_STEP_TIME;
        break;
      }
    }
  }
}

volatile uint32_t last_steps = 0;

uint32_t move_blocking_condition(bool (*condition)()) {
  uint32_t diff = 0;
  uint8_t sreg = SREG;
  cli();
  memset(motor_steps, UINT8_MAX, sizeof(motor_steps));
  uint16_t timer = TCNT1;
  SREG = sreg;
  while (accelerate()) {
    for (;;) {
      if (!condition() && !diff) {
        cli();
        diff = UINT32_MAX - motor_steps[0];
        memset(motor_steps, 0, sizeof(motor_steps));
      }
      cli();
      uint16_t timer2 = TCNT1;
      SREG = sreg;
      if (timer2 - timer >= ACCELERATION_STEP_TIME) {
        timer += ACCELERATION_STEP_TIME;
        break;
      }
    }
  }
  if (last_steps > 0) {
    diff = UINT32_MAX - last_steps;
    last_steps = 0;
  }
  return diff;
}

bool read_end_button() {
  return digitalRead(ENDBUTTON_PIN);
}

uint16_t current_cycle = UINT16_MAX;
uint32_t start_position = 0;

void disable_system_isr() {
  if (motor_steps[0] > 0)
    last_steps = motor_steps[0];
  emergency_stop();
  current_cycle = -2;
  digitalWrite(RELAY_PIN1, 0);
  digitalWrite(RELAY_PIN2, 0);
}

void calibrate_position () {
  digitalWrite(DIR_PIN, 1);
  cycles_disp.displayByte(_empty, _P, _O, _S);
  target_speed = CALIBRATION_SPEED;
  move_blocking_condition(read_end_button);
  cli();
  if (last_steps > 0) {
    exit(-1);
  }
  digitalWrite(DIR_PIN, 0);
  if (!digitalRead(SWITCH_PIN)) {
    // Calibration mode
    motor_steps[0] = UINT32_MAX;
    start_position = UINT32_MAX;
  } else {
    motor_steps[0] = start_position;
    target_speed = MOVE_SPEED;
  }
  sei();
  cycles_disp.displayByte(_N, _U, _L, _L);
  move_blocking();
  target_speed = MOVE_SPEED;
  start_position -= last_steps; /* If we stop motor preemptively, it will remember its new position */
  if (last_steps > 0) {
    for (uint8_t i = 0; i < sizeof(start_position); ++i) {
      EEPROM.write(i, start_position >> (i * 8));
    }
  }
  last_steps = 0;
}

void setup() {
  // Serial.begin(9600); // Подключаем монитор
  // Serial.setTimeout(5);
// Вычисление коэффициентов
  motor_constants[0] = (F_CPU * 60.0) / (PRESCALER * MICROSTEP * 2 * C1);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(ENDBUTTON_PIN, INPUT_PULLUP);
  pinMode(ESTOP_PIN, INPUT_PULLUP); // Using external pullup resistor
  pinMode(POSADD_PIN, INPUT_PULLUP);
  pinMode(POSSUB_PIN, INPUT_PULLUP);
  pinMode(CYCLEADD_PIN, INPUT_PULLUP);
  pinMode(CYCLESUB_PIN, INPUT_PULLUP);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(RELAY_PIN1, OUTPUT);
  pinMode(RELAY_PIN2, OUTPUT);
  pinMode(DISP_CLK_PIN, OUTPUT);
  pinMode(DISP_DIO_PIN, OUTPUT);
  target_speed = MOVE_SPEED;
  delayMicroseconds(4);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), disable_system_isr, FALLING);
  motors_init();
  cycles_disp.clear();
  cycles_disp.brightness(7);
  cycle_start = millis();
  for (uint8_t i = 0; i < sizeof(start_position); ++i) {
    start_position |= (uint32_t)EEPROM.read(i) << (i * 8);
  }
  /*start_position = 60000;
  for (uint8_t i = 0; i < sizeof(start_position); ++i) {
    EEPROM.write(i, start_position >> (i * 8));
  }*/
  calibrate_position();
}

bool direction = 1;

bool position_buttons_pressed() {
  return digitalRead(POSADD_PIN) != digitalRead(POSSUB_PIN);
}

uint8_t update_position(uint16_t time) {
  static uint16_t next_check_time = 0;
  if (time - next_check_time >= 0x8000)
    return 0;
  next_check_time = time;
  bool sub = !digitalRead(POSSUB_PIN);
  if (!digitalRead(POSADD_PIN) == sub)
    return 0;
  // blocking!
  digitalWrite(DIR_PIN, sub);
  target_speed = CALIBRATION_SPEED;
  uint32_t diff = move_blocking_condition(position_buttons_pressed);
  target_speed = MOVE_SPEED;
  start_position += (sub) ? -diff : diff;
  for (uint8_t i = 0; i < sizeof(start_position); ++i) {
    EEPROM.write(i, start_position >> (i * 8));
  }
  return 1;
}

void do_cycle(uint16_t time) {
  if (!digitalRead(SWITCH_PIN)) {
    if (!system_enabled) {
      if (last_steps > 0) {
        cli();
        motor_steps[0] = last_steps;
        last_steps = 0;
        sei();
        move_blocking();
        if (last_steps) {
          return;
        }
      }
      digitalWrite(RELAY_PIN1, direction);
      digitalWrite(RELAY_PIN2, !direction);
      cycle_start = time;
    }
    system_enabled = true;
  } else {
    current_cycle = 0;
    cycles_disp.displayInt(cycles_max.value);
    system_enabled = false;
  }
  if (time - cycle_start < CYCLE_TIME_MS) {
    return;
  }
  digitalWrite(RELAY_PIN1, 0);
  digitalWrite(RELAY_PIN2, 0);
  if (!system_enabled || current_cycle >= cycles_max.value) {
    return;
  }
  direction = !direction;
  digitalWrite(DIR_PIN, direction);
  cli();
  motor_steps[0] = CYCLE_LENGTH_STEPS;
  sei();
  move_blocking();
  ++current_cycle;
  cycles_disp.displayInt(current_cycle);
  if (last_steps > 0) {
    // Setting it so we will return back when repositioning
    direction = !direction;
    digitalWrite(DIR_PIN, direction);
    last_steps = CYCLE_LENGTH_STEPS - last_steps;
    return;
  }
  digitalWrite(RELAY_PIN1, direction);
  digitalWrite(RELAY_PIN2, !direction);
  cycle_start = millis();
}

// Throwaway code down here. Should do it as a prototype
void loop() {
  uint16_t time = millis();
  switch (cycles_max.tick(time)) {
    case NUMBER_CHANGED:
      cycles_disp.displayInt(cycles_max.value);
      break;
    case NUMBER_READY:
      if (system_enabled)
        cycles_disp.displayInt(current_cycle);
      break;
  }
  if (update_position(time))
    return;
  do_cycle(time);
}