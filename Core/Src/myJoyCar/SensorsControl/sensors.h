/**
  ******************************************************************************
  * @file           : sensors.h
  * @brief          : Header file for multiple sensor
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

#ifndef SENSORS_H_
#define SENSORS_H_


#include <stdbool.h>
#include "stm32g4xx_hal.h"

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Ultrasonic Sensor HC-SR04 --------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#define NO_OBSTACLE 44444

void USS_init(TIM_HandleTypeDef *trig_tim, uint32_t trig_channel, TIM_HandleTypeDef *echo_tim, uint32_t echo_channel);
uint32_t USS_get_value(uint32_t pulse_ticks);
void printDistance(uint32_t dist, uint32_t pulse);

// -----------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Line Tracker Sensor SEN-KY033LT --------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------

void LTS_init(TIM_HandleTypeDef *tim, uint32_t channel);

#define SENSOR_LEFT   (1 << 2)
#define SENSOR_MIDDLE (1 << 1)
#define SENSOR_RIGHT  (1 << 0)

typedef struct {
   GPIO_TypeDef *leftPort;   uint16_t leftPin;
   GPIO_TypeDef *middlePort; uint16_t middlePin;
   GPIO_TypeDef *rightPort;  uint16_t rightPin;
} LTS_Config;


typedef enum {
	NO_LINE 	= 0,    // 000
	RIGHT 		= 1,   	// 001
	MIDDLE 		= 2,    // 010
	RIGHT_CURVE = 3, 	// 011
	LEFT 		= 4,    // 100
	U_TURN 		= 5,	// 101
	LEFT_CURVE 	= 6,    // 110
	JUNCTION 	= 7  	// 111
} LineState;

typedef struct {
   bool leftLine_detected;
   bool middleLine_detected;
   bool rightLine_detected;
   LineState current_state;
} LTSensor;

/*
const LineState lineActionLUT[8] = {
	NO_LINE,        // 000
	RIGHT,   		// 001
	MIDDLE,      	// 010
	RIGHT_CURVE, 	// 011
	LEFT,    		// 100
	U_TURN,		 	// 101
	LEFT_CURVE,     // 110
	JUNCTION  		// 111
};
*/

//extern UART_HandleTypeDef huart3;

LTSensor get_LTS_value(const LTS_Config *cfg);
void printLinePos(LTSensor current_data);



/*
 typedef enum {
    STATE_IDLE,
    STATE_FOLLOW_LINE,
    STATE_ADJUST_LEFT,
    STATE_ADJUST_RIGHT,
    STATE_LOST_LINE,
    STATE_LINE_END,
    STATE_COUNT
} LineFollowerState;

typedef LineFollowerState (*StateTransitionFunc)(void);
typedef void (*StateActionFunc)(void);

 typedef struct {
    const char* name;
    StateActionFunc action;
    StateTransitionFunc next_state;
} State;

int S_L, S_C, S_R; // global or passed-in

void read_line_sensors(void) {
        S_L  = HAL_GPIO_ReadPin(cfg->leftPort, cfg->leftPin)   == GPIO_PIN_SET,
        S_C  = HAL_GPIO_ReadPin(cfg->middlePort, cfg->middlePin) == GPIO_PIN_SET,
        S_R  = HAL_GPIO_ReadPin(cfg->rightPort, cfg->rightPin) == GPIO_PIN_SET
}

void action_idle(void) {
    stop_motors();
}

void action_follow_line(void) {
    set_motor_speed(BASE_SPEED, BASE_SPEED);
}

void action_adjust_left(void) {
    set_motor_speed(BASE_SPEED - 10, BASE_SPEED + 10);
}

void action_adjust_right(void) {
    set_motor_speed(BASE_SPEED + 10, BASE_SPEED - 10);
}

void action_lost_line(void) {
    // Could slow down, scan, or do nothing
    set_motor_speed(BASE_SPEED / 2, BASE_SPEED / 2);
}

void action_line_end(void) {
    stop_motors();
}

LineFollowerState transition_idle(void) {
    return S_C ? STATE_FOLLOW_LINE : STATE_IDLE;
}

LineFollowerState transition_follow_line(void) {
    if (S_L == 1 && S_C == 0) return STATE_ADJUST_LEFT;
    if (S_R == 1 && S_C == 0) return STATE_ADJUST_RIGHT;
    if (S_L == 0 && S_C == 0 && S_R == 0) return STATE_LOST_LINE;
    return STATE_FOLLOW_LINE;
}

LineFollowerState transition_adjust_left(void) {
    if (S_C == 1) return STATE_FOLLOW_LINE;
    if (S_L == 0 && S_C == 0 && S_R == 0) return STATE_LOST_LINE;
    return STATE_ADJUST_LEFT;
}

LineFollowerState transition_adjust_right(void) {
    if (S_C == 1) return STATE_FOLLOW_LINE;
    if (S_L == 0 && S_C == 0 && S_R == 0) return STATE_LOST_LINE;
    return STATE_ADJUST_RIGHT;
}

LineFollowerState transition_lost_line(void) {
    static uint32_t lost_time = 0;
    if (S_C || S_L || S_R) {
        lost_time = 0;
        return STATE_FOLLOW_LINE;
    }
    lost_time += 10; // called every 10ms
    if (lost_time > 300) return STATE_LINE_END;
    return STATE_LOST_LINE;
}

LineFollowerState transition_line_end(void) {
    return STATE_LINE_END; // Terminal state
}

State fsm[STATE_COUNT] = {
    [STATE_IDLE] =        {"IDLE",        action_idle,        transition_idle},
    [STATE_FOLLOW_LINE] = {"FOLLOW_LINE", action_follow_line, transition_follow_line},
    [STATE_ADJUST_LEFT] = {"ADJUST_LEFT", action_adjust_left, transition_adjust_left},
    [STATE_ADJUST_RIGHT] ={"ADJUST_RIGHT",action_adjust_right,transition_adjust_right},
    [STATE_LOST_LINE] =   {"LOST_LINE",   action_lost_line,   transition_lost_line},
    [STATE_LINE_END] =    {"LINE_END",    action_line_end,    transition_line_end},
};

void LineFollowerTask(void *pvParameters) {
    LineFollowerState current_state = STATE_IDLE;

    for(;;) {
        read_line_sensors();

        fsm[current_state].action();

        LineFollowerState next = fsm[current_state].next_state();

        current_state = next;

        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz update
    }
}
 */

// -----------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Infrared Proximity Sensor SEN-KY032IR --------------------------------
// -----------------------------------------------------------------------------------------------------------------------------

#define LOW 0
#define HIGH 1
#define OBSTACLE_LEFT   (1 << 1)
#define OBSTACLE_RIGHT  (1 << 0)

typedef struct {
   GPIO_TypeDef *leftPort;  uint16_t leftPin;
   GPIO_TypeDef *rightPort; uint16_t rightPin;
} IPS_Config;

typedef struct {
   bool leftObstacle_detected;
   bool rightObstacle_detected;
} IPSensor;

/*
typedef enum {
	OBSTACLE_BOTH_SIDES = 0,    // 00
	OBSTACLE_RIGHT_SIDE	= 1,   	// 01
	OBSTACLE_LEFT_SIDE	= 2,    // 10
	NO_OBSTACLES 		= 3 	// 11
} ObstacleState;
*/

IPSensor get_IPS_value(const IPS_Config *cfg);
void printObstaclePos(IPSensor current_data);

// -----------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ RGB Lights WS2812B  --------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------

#define MAX_LED 8
#define USE_BRIGHTNESS 1
#define PI 3.14159265

extern TIM_HandleTypeDef htim4;

void Set_LED (int LEDnum, int Red, int Green, int Blue);
void Set_Brightness (int brightness);
void WS2812_Send (void);

// -----------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Speed Sensor LM393  --------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------

#ifndef SPEED_SENSOR_MAX
#define SPEED_SENSOR_MAX 4
#endif

typedef struct {
   /* Timer used for input capture (the one your ISR is using) */
   TIM_HandleTypeDef *htim_ic;

   /* Which channels are used (e.g. TIM_CHANNEL_1 for right, _2 for left) */
   uint32_t ch_right;
   uint32_t ch_left;

   /* These MUST point to the variables your ISR updates (period in timer ticks) */
   volatile uint32_t *period_right_ptr;
   volatile uint32_t *period_left_ptr;

   /* Timing configuration */
   uint32_t timer_clk_hz;   /* timer input clock in Hz (after APB multiplier) */
   uint32_t prescaler;      /* PSC register value (i.e., TIMx->PSC), not PSC+1 */
   /* Encoder resolution: pulses/edges per mechanical revolution (after x2/x4) */
   uint32_t pulses_per_rev;

} Speed_Config;

typedef struct {
   float rpm_right;
   float rpm_left;
} Speed_Data;

void Speed_Init(const Speed_Config *cfg);
uint32_t GetTIMClockHz(TIM_HandleTypeDef *htim);
Speed_Data Speed_Get(const Speed_Config *cfg);
//void printMotorSpeed(const Speed_Config *cfg, UART_HandleTypeDef *huart);


#endif /* SENSORS_H_ */
