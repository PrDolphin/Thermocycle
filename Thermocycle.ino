#include <GyverTM1637.h>
#include "stepmotor.h"

#define ESTOP_PIN 3
#define ENDBUTTON_PIN 2
#define SWITCH_PIN 12
#define DIR_PIN 13
#define RELAY_PIN1 6
#define RELAY_PIN2 7
#define DISP_CLK_PIN 4
#define DISP_DIO_PIN 5

/*------------------- Значение всех изменяемых величин -------------------*/
#define PRESCALER 1
#define MICROSTEP  3200
#define C1 1

#define CYCLE_TIME_MS 5000
// 128000 == 81 mm
#define CYCLE_LENGTH_STEPS (uint32_t)268642
#define ENDBUTTON_OFFSET (uint32_t)10000
#define NUMBER_CYCLES 10

#define ACCELERATION_STEP_TIME (F_CPU / 1000) // Every 1 ms

GyverTM1637 cycles_disp(DISP_CLK_PIN, DISP_DIO_PIN);

uint32_t cycle_start;
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

uint8_t current_cycle = UINT8_MAX;

void disable_system_isr() {
  emergency_stop();
  current_cycle = UINT8_MAX - 1;
  digitalWrite(RELAY_PIN1, 0);
  digitalWrite(RELAY_PIN2, 0);
}

void calibrate_position () {
  digitalWrite(DIR_PIN, 1);
  cli();
  memset(motor_steps, UINT8_MAX, sizeof(motor_steps));
  sei();
  accelerate();
  while (digitalRead(ENDBUTTON_PIN)) {
    if (!digitalRead(ESTOP_PIN))
      return;
  }
  cli();
  memset(motor_steps, 0, sizeof(motor_steps));
  sei();
  while(accelerate());
  cli();
  if (!digitalRead(ESTOP_PIN)) {
    sei();
    return;
  }
  motor_steps[0] = ENDBUTTON_OFFSET;
  sei();
  digitalWrite(DIR_PIN, 0);
  move_blocking();
}

void setup() {
  // Serial.begin(9600); // Подключаем монитор
  // Serial.setTimeout(5);
// Вычисление коэффициентов
  motor_constants[0] = (F_CPU * 60.0) / (PRESCALER * MICROSTEP * 2 * C1);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(ENDBUTTON_PIN, INPUT_PULLUP);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(RELAY_PIN1, OUTPUT);
  pinMode(RELAY_PIN2, OUTPUT);
  pinMode(DISP_CLK_PIN, OUTPUT);
  pinMode(DISP_DIO_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), disable_system_isr, FALLING);
  motors_init();
  cycles_disp.displayInt(0);
  cycle_start = millis();
  calibrate_position();
  //Serial.println("Ready");
}

bool direction = 1;

// Throwaway code down here. Should do it as a prototype
void loop() {
  if (!digitalRead(SWITCH_PIN)) {
    if (!system_enabled) {
      digitalWrite(RELAY_PIN1, direction);
      digitalWrite(RELAY_PIN2, !direction);
      cycle_start = millis();
    }
    system_enabled = true;
  } else {
    current_cycle = 0;
    cycles_disp.displayInt(0);
    system_enabled = false;
    digitalWrite(RELAY_PIN1, 0);
    digitalWrite(RELAY_PIN2, 0);
    //Serial.println("STOP");
    return;
  }
  if (millis() - cycle_start < CYCLE_TIME_MS) {
    return;
  }
  digitalWrite(RELAY_PIN1, 0);
  digitalWrite(RELAY_PIN2, 0);
  if (current_cycle >= NUMBER_CYCLES) {
    //Serial.println("Max CYCLE");
    return;
  }
  //Serial.println("Start");
  direction = !direction;
  digitalWrite(DIR_PIN, direction);
  delayMicroseconds(4);
  cli();
  motor_steps[0] = CYCLE_LENGTH_STEPS;
  sei();
  move_blocking();
  if (!digitalRead(ESTOP_PIN))
    return;
  ++current_cycle;
  cycles_disp.displayInt(current_cycle);
  digitalWrite(RELAY_PIN1, direction);
  digitalWrite(RELAY_PIN2, !direction);
  cycle_start = millis();
  //Serial.print("Cycle number: "); Serial.println(current_cycle);

}