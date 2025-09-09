/**
  ******************************************************************************
  * @file           : motors.c
  * @brief          : Source file for DC Gear Motor and micro Servo Motor
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

#include "motor.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ DC Gear Motor --------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

extern MotorHandle Mot_right;
extern MotorHandle Mot_left;


/*
static const MotorCommand kCommandTable[8] = {
     {  0,  0 },                        // handled specially (search) 				NO_LINE      000
     {OUTER_HI, SPEED_STRONG},          // steer right: left faster, right slower 	RIGHT        001
     {SPEED_STRAIGHT, SPEED_STRAIGHT},  // go straight								MIDDLE       010
     {OUTER_HI, SPEED_SOFT},            // gentle right								RIGHT_CURVE  011
     {SPEED_STRONG, OUTER_HI},          // steer left: left slower, right faster	LEFT         100
     {  0,  0 },                        // not used here							U_TURN       101
     {SPEED_SOFT,  OUTER_HI},           // gentle left								LEFT_CURVE   110
     {SPEED_STRAIGHT, SPEED_STRAIGHT},  // treat like straight for this use-case	JUNCTION     111
};
*/

void Motors_Init(MotorHandle *motor_right, MotorHandle *motor_left)
{
  	HAL_GPIO_WritePin(motor_right->ena_port, motor_right->ena_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor_left->ena_port, motor_left->ena_pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(motor_right->htim, motor_right->channel_in1);
	HAL_TIM_PWM_Start(motor_right->htim, motor_right->channel_in2);
	HAL_TIM_PWM_Start(motor_left->htim, motor_left->channel_in1);
	HAL_TIM_PWM_Start(motor_left->htim, motor_left->channel_in2);
}


int8_t clamp_int(int8_t value, int8_t min, int8_t max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}


void move_robot(int8_t left_speed, int8_t right_speed)
{
	int8_t leftMot_v  = clamp_int(left_speed , -100, 100);
	int8_t rightMot_v = clamp_int(right_speed, -100, 100);

	if(leftMot_v >= 0){
		uint32_t target_pulse_left = (uint32_t)(499 * leftMot_v / 100.0f);
		__HAL_TIM_SET_COMPARE(Mot_left.htim, Mot_left.channel_in2, 0);
		__HAL_TIM_SET_COMPARE(Mot_left.htim, Mot_left.channel_in1, target_pulse_left);
	} else {
		leftMot_v = leftMot_v * -1;
		uint32_t target_pulse_left = (uint32_t)(499 * leftMot_v / 100.0f);

		__HAL_TIM_SET_COMPARE(Mot_left.htim, Mot_left.channel_in1, 0);
		__HAL_TIM_SET_COMPARE(Mot_left.htim, Mot_left.channel_in2, target_pulse_left);
	}


	if(rightMot_v >= 0){
		uint32_t target_pulse_right = (uint32_t)(499 * rightMot_v / 100.0f);
		__HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in1, 0);
		__HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in2, target_pulse_right);
	} else {
		rightMot_v = rightMot_v * -1;
		uint32_t target_pulse_right = (uint32_t)(499 * rightMot_v / 100.0f);

		__HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in2, 0);
		__HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in1, target_pulse_right);
	}

}

void stop_robot(void)
{

	__HAL_TIM_SET_COMPARE(Mot_left.htim, Mot_left.channel_in2, 0);
	__HAL_TIM_SET_COMPARE(Mot_left.htim, Mot_left.channel_in1, 0);

	__HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in1, 0);
	__HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in2, 0);

}



/*

void Motor_SetCompare(MotorHandle *m, MotorDirection dir, uint32_t pulse)
{
	switch (dir){
	case MOTOR_DIR_FORWARD: __HAL_TIM_SET_COMPARE(m->htim, m->channel_in2, 0);
    						__HAL_TIM_SET_COMPARE(m->htim, m->channel_in1, pulse);
    						break;
	case MOTOR_DIR_REVERSE: __HAL_TIM_SET_COMPARE(m->htim, m->channel_in1, 0);
    						__HAL_TIM_SET_COMPARE(m->htim, m->channel_in2, pulse);
    						break;
	default:				__HAL_TIM_SET_COMPARE(m->htim, m->channel_in1, 0);		// COAST
						    __HAL_TIM_SET_COMPARE(m->htim, m->channel_in2, 0);
	}
}


void Motor_SetPWM(MotorHandle *m, MotorDirection dir, uint8_t duty, uint8_t ramp_step)
{
	const TickType_t xPeriod = pdMS_TO_TICKS(1);

   uint32_t target_pulse = (uint32_t)(m->htim->Init.Period * duty / 100.0f);

   // If changing direction, ramp down and stop first to avoid shoot-through
   if (dir != m->last_dir) {
       for (uint32_t p = m->last_pulse; p > 0; p -= ramp_step) {
           Motor_SetCompare(m, m->last_dir, p);
           vTaskDelay(xPeriod);
       }
       // Fully stop
       Motor_SetCompare(m, MOTOR_DIR_FORWARD, 0);
       Motor_SetCompare(m, MOTOR_DIR_REVERSE, 0);
       vTaskDelay(xPeriod); // delay before reversing
   }
   // Ramp speed smoothly
   if (target_pulse > m->last_pulse) {
       for (uint32_t p = m->last_pulse; p <= target_pulse; p += ramp_step) {
           Motor_SetCompare(m, dir, p);
           vTaskDelay(xPeriod); // ramp delay step
       }
   } else {
       for (uint32_t p = m->last_pulse; p >= target_pulse; p -= ramp_step) {
           Motor_SetCompare(m, dir, p);
           vTaskDelay(xPeriod); // ramp delay step
       }
   }
   m->last_pulse = target_pulse;
   m->last_dir = dir;
}



void Motor_SetPWM(MotorHandle *m, MotorDirection dir, uint8_t duty, uint8_t ramp_step)
{

   uint32_t target_pulse = (uint32_t)(m->htim->Init.Period * duty / 100.0f);

   // Ramp speed smoothly
   if (target_pulse > m->last_pulse) {
           Motor_SetCompare(m, dir, p);
   } else {
           Motor_SetCompare(m, dir, p);
   }
   m->last_pulse = target_pulse;
   m->last_dir = dir;
}



int8_t ticks_to_rpm(uint32_t period_ticks,
                                 uint32_t timer_clk_hz,
                                 uint32_t prescaler_plus_1)
{
    if (period_ticks == 0) return 0.0f;
    float tick_hz = (float)timer_clk_hz / (float)prescaler_plus_1;
    float signal_hz = tick_hz / (float)period_ticks;
    return (signal_hz * 60.0f) / (float)PULSES_PER_REV;
}


*/






// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Micro Servo SG90 9g --------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------
void servo_set_angle(uint8_t angle) {
   if (angle > 180) angle = 180;
   if (angle < 0) angle = 0;
   // Pulse range: 1ms (100) to 2ms (200) at 10µs resolution (100 kHz)
   uint16_t pulse = 100 + (angle * 100) / 180;
   __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pulse);
}



