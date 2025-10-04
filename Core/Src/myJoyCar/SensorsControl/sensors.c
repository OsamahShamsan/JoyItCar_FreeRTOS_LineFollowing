/**
  ******************************************************************************
  * @file           : sensors.c
  * @brief          : Source file for multiple sensor
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


#include <sensors.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>


// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Ultrasonic Sensor HC-SR04 --------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void USS_init(TIM_HandleTypeDef *trig_tim, uint32_t trig_channel,
              TIM_HandleTypeDef *echo_tim, uint32_t echo_channel)
{
    HAL_TIM_PWM_Start(trig_tim, trig_channel);
    HAL_TIM_IC_Start_IT(echo_tim, echo_channel);
}

uint32_t USS_get_value(uint32_t pulse_ticks){
	if	(pulse_ticks >= 38000) return NO_OBSTACLE;	// NO_OBSTACLE = 0
	return (uint32_t)pulse_ticks / 580;
}

void printDistance(uint32_t dist, uint32_t pulse){
	char msg[64];

	if(dist == 0){
		snprintf(msg, sizeof(msg), "Out of range\r\n");
	} else {
		snprintf(msg, sizeof(msg), "dt: %lu ms   | distance %lu cm\r\n", pulse, dist);
	}
	//HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

// -----------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Line Tracker Sensor SEN-KY033LT --------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------

void LTS_init(TIM_HandleTypeDef *tim, uint32_t channel){
	HAL_TIM_PWM_Start(tim, channel);
}

LTSensor get_LTS_value(const LTS_Config *cfg)
{

	LTSensor val = {
        .leftLine_detected   = HAL_GPIO_ReadPin(cfg->leftPort,   cfg->leftPin)   == GPIO_PIN_SET,
        .middleLine_detected = HAL_GPIO_ReadPin(cfg->middlePort, cfg->middlePin) == GPIO_PIN_SET,
        .rightLine_detected  = HAL_GPIO_ReadPin(cfg->rightPort,  cfg->rightPin)  == GPIO_PIN_SET
    };
     val.current_state =
          (val.leftLine_detected   ? SENSOR_LEFT   : 0)
        | (val.middleLine_detected ? SENSOR_MIDDLE : 0)
        | (val.rightLine_detected  ? SENSOR_RIGHT  : 0);
     return val;

}


static inline uint8_t read_bits(const LTS_Config *cfg)
{
    // read each port once (fast, consistent)
    uint32_t idrL = cfg->leftPort->IDR;
    uint32_t idrC = cfg->middlePort->IDR;
    uint32_t idrR = cfg->rightPort->IDR;

    uint8_t bL = (idrL & cfg->leftPin)   ? 1 : 0;
    uint8_t bC = (idrC & cfg->middlePin) ? 1 : 0;
    uint8_t bR = (idrR & cfg->rightPin)  ? 1 : 0;

    return (bL<<2) | (bC<<1) | (bR<<0);      // 0..7
}

/*
uint8_t bits = read_bits(cfg);

LTSensor val = {
   .leftLine_detected   = (bits & SENSOR_LEFT)   != 0,
   .middleLine_detected = (bits & SENSOR_MIDDLE) != 0,
   .rightLine_detected  = (bits & SENSOR_RIGHT)  != 0,
   .current_state       = (LineState)bits
};
return val;
*/

/*
void handleLineAction(LineAction action) {
    switch (action) {
        case ACTION_FORWARD:
            goStraight();
            break;
        case ACTION_TURN_LEFT:
            turnSharpLeft();
            break;
        case ACTION_TURN_RIGHT:
            turnSharpRight();
            break;
        case ACTION_SLIGHT_LEFT:
            turnSlightLeft();
            break;
        case ACTION_SLIGHT_RIGHT:
            turnSlightRight();
            break;
        case ACTION_STOP:
            stopMotors();
            break;
        case ACTION_HANDLE_JUNCTION:
            handleJunction();
            break;
    }
}
*/

void printLinePos(LTSensor current_data){
	char msg[64];
	uint8_t lineBits = 0;

	if (current_data.leftLine_detected)   lineBits |= SENSOR_LEFT;   // bit 2
	if (current_data.middleLine_detected) lineBits |= SENSOR_MIDDLE; // bit 1
	if (current_data.rightLine_detected)  lineBits |= SENSOR_RIGHT;  // bit 0

	if(lineBits == 0){
		snprintf(msg, sizeof(msg), "No lines detected\r\n");
	} else {
		snprintf(msg, sizeof(msg), "Lines position: %u\r\n", lineBits);
	}
	//HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

// -----------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Infrared Proximity Sensor SEN-KY032IR --------------------------------
// -----------------------------------------------------------------------------------------------------------------------------

IPSensor get_IPS_value(const IPS_Config *cfg)
{
    IPSensor val = {
    		.leftObstacle_detected = HAL_GPIO_ReadPin(cfg->leftPort, cfg->leftPin) == LOW,
			.rightObstacle_detected = HAL_GPIO_ReadPin(cfg->rightPort, cfg->rightPin) == LOW
    };

    return val;
}

void printObstaclePos(IPSensor current_data) {
    char msg[64];
    uint8_t obstacleBits = 0;

    // Each bit = 1 means "no obstacle", 0 means "obstacle"
    if (current_data.leftObstacle_detected)  obstacleBits |= OBSTACLE_LEFT;   // bit 1
    if (current_data.rightObstacle_detected) obstacleBits |= OBSTACLE_RIGHT;  // bit 0

    if (obstacleBits == 0) {
        snprintf(msg, sizeof(msg), "No obstacles detected\r\n");
    } else {
        snprintf(msg, sizeof(msg), "Obstacles position: %u\r\n", obstacleBits);
    }
    //HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

// -----------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ RGB Lights WS2812B  --------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------

uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];  // for brightness
uint16_t pwmData[(24*MAX_LED)+50];

volatile int datasentflag;

void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}

void Set_Brightness (int brightness)  // 0-45
{
#if USE_BRIGHTNESS

	if (brightness > 45) brightness = 45;
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j=1; j<4; j++)
		{
			float angle = 90-brightness;  // in degrees
			angle = angle*PI / 180;  // in rad
			LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
		}
	}
#endif
}

void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;


	for (int i= 0; i<MAX_LED; i++)
	{
#if USE_BRIGHTNESS
		color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
#else
		color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));
#endif

		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				pwmData[indx] = 141;  // 2/3 of the Period
			}

			else pwmData[indx] = 70;  // 1/3 of the Period

			indx++;
		}

	}

	for (int i=0; i<50; i++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_4, (uint32_t *)pwmData, indx);
	while (!datasentflag){};
	datasentflag = 0;
}

// -----------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Speed Sensor LM393  --------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------

void Speed_Init(const Speed_Config *cfg)
{
    /* Start IC with interrupts on both channels. The user’s ISR will run. */
    if (cfg->htim_ic != NULL) {

    	HAL_TIM_Base_Start_IT(cfg->htim_ic);

        if (cfg->ch_right != 0U) {
            HAL_TIM_IC_Start_IT(cfg->htim_ic, cfg->ch_right);
        }
        if (cfg->ch_left != 0U) {
            HAL_TIM_IC_Start_IT(cfg->htim_ic, cfg->ch_left);
        }
    }
}

uint32_t GetTIMClockHz(TIM_HandleTypeDef *htim)
{
    uint32_t pclk, hclk, mul = 1U;
	pclk = HAL_RCC_GetPCLK2Freq();
	hclk = HAL_RCC_GetHCLKFreq();
	if ((hclk / pclk) != 1U) mul = 2U;   // APB2 prescaler != 1 → timer x2

    return pclk * mul;
}

/* Local helper: convert a period (timer ticks) into RPM */
float period_to_rpm(uint32_t period_ticks,
                           uint32_t timer_clk_hz,
                           uint32_t prescaler,
                           uint32_t pulses_per_rev)
{
    if (period_ticks == 0U || pulses_per_rev == 0U) {
        return 0.0f;
    }

    /* Timer tick frequency after prescaler */
    const float tick_hz = (float)timer_clk_hz / (float)(prescaler + 1U);

    /* Input signal frequency = tick_hz / period_ticks */
    const float sig_hz = tick_hz / (float)period_ticks;

    /* RPM = (sig_hz * 60) / pulses_per_rev */
    return (sig_hz * 60.0f) / (float)pulses_per_rev;
}

Speed_Data Speed_Get(const Speed_Config *cfg)
{
    Speed_Data out = {0};

    /* Snapshot periods once (32-bit aligned volatile reads are atomic on Cortex-M) */
    const uint32_t pr = (cfg->period_right_ptr) ? *(cfg->period_right_ptr) : 0U;
    const uint32_t pl = (cfg->period_left_ptr)  ? *(cfg->period_left_ptr)  : 0U;

    out.rpm_right = period_to_rpm(pr, cfg->timer_clk_hz, cfg->prescaler, cfg->pulses_per_rev);
    out.rpm_left  = period_to_rpm(pl, cfg->timer_clk_hz, cfg->prescaler, cfg->pulses_per_rev);

    return out;
}

/*
void printMotorSpeed(const Speed_Config *cfg, UART_HandleTypeDef *huart)
{
    Speed_Data d = Speed_Get(cfg);

    char buf[64];
    int n = snprintf(buf, sizeof(buf), "RPM R: %.2f, L: %.2f\r\n", d.rpm_right, d.rpm_left);
    if (n < 0) return;

    HAL_UART_Transmit(huart, (uint8_t*)buf, (uint16_t)strlen(buf), HAL_MAX_DELAY);
}
*/


