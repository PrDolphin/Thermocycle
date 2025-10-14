#include "stepmotor.h"

#define SWITCH_PIN 12
#define DIR_PIN 13
#define RELAY_PIN1 6
#define RELAY_PIN2 7

/*------------------- Значение всех изменяемых величин -------------------*/
#define PRESCALER 1
#define MICROSTEP  3200
#define C1 1

#define CYCLE_TIME_MS 30000
// 128000 == 81 mm
#define CYCLE_LENGTH_STEPS (uint32_t)268642
#define NUMBER_CYCLES 10


#define ACCELERATION_STEP_TIME (F_CPU / 1000) // Every 1 ms

uint32_t cycle_start;
uint8_t current_cycle = NUMBER_CYCLES;
void setup() {
  //Serial.begin(9600); // Подключаем монитор
  //Serial.setTimeout(5);
// Вычисление коэффициентов
  motor_constants[0] = (F_CPU * 60.0) / (PRESCALER * MICROSTEP * 2 * C1);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(RELAY_PIN1, OUTPUT);
  pinMode(RELAY_PIN2, OUTPUT);
  motors_init();
  cycle_start = millis();
  if (digitalRead(SWITCH_PIN) == 0)
    current_cycle = NUMBER_CYCLES;
  accelerate();
  //Serial.println("Ready");
}

bool direction = 1;


extern uint16_t motor_intervals[STEPMOTOR_Q];
// Throwaway code down here. Should do it as a prototype
void loop() {  
  //delay(100);
  if ((digitalRead(SWITCH_PIN) != 0) ) {
    cycle_start = millis();
    current_cycle = 0;
    //Serial.println("STOP");
    return;
  }
  if (current_cycle == NUMBER_CYCLES) {
    //Serial.println("Max CYCLE");
    return;
  }
  //Serial.println("Start");
  if ((millis() - cycle_start < CYCLE_TIME_MS)) {
    return;
  }
  direction = !direction;
  digitalWrite(DIR_PIN, direction);
  digitalWrite(RELAY_PIN1, 0);
  digitalWrite(RELAY_PIN2, 0);
  delayMicroseconds(4);
  cli();
  motor_steps[0] = CYCLE_LENGTH_STEPS;
  uint16_t timer = TCNT1;
  sei();
  while (accelerate()) {
    for (;;) {
      delayMicroseconds(4);
      cli();
      uint16_t timer2 = TCNT1;
      sei();
      if (timer2 - timer >= ACCELERATION_STEP_TIME) {
        timer += ACCELERATION_STEP_TIME;
        break;
      }
    }
  }
  digitalWrite(RELAY_PIN1, direction);
  digitalWrite(RELAY_PIN2, !direction);
  ++current_cycle;
  //Serial.print("Cycle number: "); Serial.println(current_cycle);
  cycle_start = millis();

}