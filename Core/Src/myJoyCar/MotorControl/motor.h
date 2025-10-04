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
#include <stdbool.h>
#include <stdint.h>

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ DC Gear Motor --------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#define MAX_PWM 100u
#define MIN_PWM 0
#define MIN_CCR_L  70u
#define MIN_CCR_R  70u



// Motor configuration struct
typedef struct {
   TIM_HandleTypeDef *htim;
   uint32_t channel_in1;
   uint32_t channel_in2;
   GPIO_TypeDef *ena_port;
   uint16_t ena_pin;
   //uint32_t last_pulse;     // For ramping
   //MotorDirection last_dir; // Track last directio
} MotorHandle;

extern MotorHandle Mot_right;
extern MotorHandle Mot_left;

void motors_init(MotorHandle *motor_right, MotorHandle *motor_left);
void move_robot(int8_t left_speed, int8_t right_speed);
void stop_robot(void);


/*


int8_t ForwardSpeed_R = 56;
int8_t ForwardSpeed_L = 56;

int8_t ForwardCurveSpeed = 56;
int8_t stop_curve = 0;
int8_t stop = 0;

int8_t rightSpeed = 56;
int8_t leftSpeed = 56;

uint16_t CRR_MAX = 500;
uint16_t CCR_56Per = 311;
uint16_t CCR_random = 302;
uint16_t CCR_52Per = 294;

uint16_t R_CCR_52Per = 252;
uint16_t L_CCR_52Per = 250;
uint16_t R_CCR_52Per_Curve = 290;
uint16_t L_CCR_52Per_Curve = 289;

#define RAMP_STEP 10        // Step size for ramping
#define RAMP_DELAY_MS 5     // Delay between steps
#define DIR_CHANGE_DELAY 50 // Delay when changing direction

// --- Tuning knobs ---
#define SPEED_STRAIGHT  60   // base speed when centered
#define SPEED_SOFT      50   // mild correction inner wheel
#define SPEED_STRONG    35   // stronger correction inner wheel
#define SPEED_OUTER_HI  75   // outer wheel when correcting

// Motor direction enum
typedef enum {
   MOTOR_DIR_COAST = 0,
   MOTOR_DIR_FORWARD,
   MOTOR_DIR_REVERSE
   //MOTOR_DIR_BRAKE
} MotorDirection;

typedef struct {
    uint8_t leftDuty;
    uint8_t rightDuty;
} MotorCommand;


*/
//void Motor_SetCompare(MotorHandle *m, MotorDirection dir, uint32_t pulse);
//void Motor_SetPWM(MotorHandle *m, MotorDirection dir, uint8_t duty, uint8_t ramp_step);


//void moveForward(uint8_t leftDuty, uint8_t rightDuty);
//void stop(void);





// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Encoders -------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------


/*
typedef struct {
    TIM_HandleTypeDef *htim;   // Timer configured in Encoder Mode
    bool inverted;             // true to flip direction
    uint32_t cpr;              // counts per *wheel* revolution (PPR * 4 * gear_ratio)
    float wheel_circum_m;      // wheel circumference in meters
} EncoderConfig;

typedef struct {
    EncoderConfig cfg;
    uint16_t last_timer;       // last raw 16-bit (or lower 16 of 32-bit) count
    int32_t pos_counts;        // accumulated 32-bit position
    float vel_cps;             // counts per second (low-pass optional)
} EncoderHandle;

typedef struct {
    int32_t  L_counts;
    float    L_rpm;
    float    L_mps;
    int32_t  R_counts;
    float    R_rpm;
    float    R_mps;
} EncoderData_t;

void Encoder_Init(EncoderHandle *e, const EncoderConfig *cfg);
void Encoder_Zero(EncoderHandle *e, int32_t to_counts);
void Encoder_Update(EncoderHandle *e, float dt_s); // call at a fixed rate

// Queries
int32_t Encoder_GetCounts(const EncoderHandle *e);
float   Encoder_GetRevolutions(const EncoderHandle *e);    // wheel revs
float   Encoder_GetRPS(const EncoderHandle *e);            // wheel rev/s
float   Encoder_GetRPM(const EncoderHandle *e);            // wheel rpm
float   Encoder_GetLinearSpeed(const EncoderHandle *e);    // m/s (at wheel)


void Encoders_InitAll(void);
void Encoders_UpdateAll(float dt_s);

*/



// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Micro Servo SG90 9g --------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#define FAR_LEFT_ANGLE  180u
#define CENTER_ANGLE 	92u
#define FAR_RIGHT_ANGLE 0

#define MAX_SERVO_PULSE 240u
#define CENTER_SERVO_PULSE 147u
#define MIN_SERVO_PULSE 50u

extern TIM_HandleTypeDef htim16;

// Motor configuration struct
typedef struct {
   TIM_HandleTypeDef *htim;
   uint32_t channel;
} ServoHandle;

void servo_init(ServoHandle *servo);
void servo_set_angle(uint8_t angle);


#endif /* MOTORS_H_ */
