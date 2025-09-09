/**
  ******************************************************************************
  * @file           : motors.h
  * @brief          : Header file for DC Gear Motor and micro Servo Motor
  ******************************************************************************
  * @attention
  *
  *
  *
  *
  *
  *
  *
  *
  ******************************************************************************
  */

#ifndef MOTORS_H_
#define MOTORS_H_

#include "stm32g4xx_hal.h"

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ DC Gear Motor --------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#define RAMP_STEP 10        // Step size for ramping
#define RAMP_DELAY_MS 5     // Delay between steps
#define DIR_CHANGE_DELAY 50 // Delay when changing direction

// --- Tuning knobs ---
#define SPEED_STRAIGHT  60   // base speed when centered
#define SPEED_SOFT      50   // mild correction inner wheel
#define SPEED_STRONG    35   // stronger correction inner wheel
#define SPEED_OUTER_HI  75   // outer wheel when correcting

#define CONTROL_PERIOD_MS 10 // 100 Hz loop

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart3;

// Motor direction enum
typedef enum {
   MOTOR_DIR_COAST = 0,
   MOTOR_DIR_FORWARD,
   MOTOR_DIR_REVERSE
   //MOTOR_DIR_BRAKE
} MotorDirection;

// Motor configuration struct
typedef struct {
   TIM_HandleTypeDef *htim;
   uint32_t channel_in1;
   uint32_t channel_in2;
   GPIO_TypeDef *ena_port;
   uint16_t ena_pin;
   uint32_t last_pulse;     // For ramping
   MotorDirection last_dir; // Track last directio
} MotorHandle;

typedef struct {
    uint8_t leftDuty;
    uint8_t rightDuty;
} MotorCommand;

void Motors_Init(MotorHandle *motor_right, MotorHandle *motor_left);
int8_t clamp_int(int8_t value, int8_t min, int8_t max);
void move_robot(int8_t left_speed, int8_t right_speed);
void stop_robot(void);


//void Motor_SetCompare(MotorHandle *m, MotorDirection dir, uint32_t pulse);
//void Motor_SetPWM(MotorHandle *m, MotorDirection dir, uint8_t duty, uint8_t ramp_step);


//void moveForward(uint8_t leftDuty, uint8_t rightDuty);
//void stop(void);

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Micro Servo SG90 9g --------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

extern TIM_HandleTypeDef htim16;
void servo_set_angle(uint8_t angle);


#endif /* MOTORS_H_ */
