#ifndef STEPMOTOR_H
#define STEPMOTOR_H

#ifndef STEPMOTOR_Q
#define STEPMOTOR_Q 1
#endif // STEPMOTOR_Q

#ifndef MINSPEED
#define MINSPEED 10
#endif // MINSPEED

#if STEPMOTOR_Q < 0 || STEPMOTOR_Q > 2
#error "Disallowed motors count"
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t motor_constants[STEPMOTOR_Q];
extern volatile uint32_t motor_steps[STEPMOTOR_Q];
extern uint16_t target_speed;
void motors_init();
uint8_t accelerate();
void emergency_stop();

#ifdef __cplusplus
}
#endif

#endif // STEPMOTOR_H