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


extern TIM_HandleTypeDef htim1;

static inline int8_t clamp_int(int8_t v, int8_t lo, int8_t hi){
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ DC Gear Motor --------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

//extern TIM_HandleTypeDef htim2;
//extern TIM_HandleTypeDef htim3;
//extern UART_HandleTypeDef huart3;

void motors_init(MotorHandle *motor_right, MotorHandle *motor_left)
{
  	HAL_GPIO_WritePin(motor_right->ena_port, motor_right->ena_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor_left->ena_port, motor_left->ena_pin, GPIO_PIN_SET);

	HAL_TIM_PWM_Start(motor_right->htim, motor_right->channel_in1);
	HAL_TIM_PWM_Start(motor_right->htim, motor_right->channel_in2);
	HAL_TIM_PWM_Start(motor_left->htim, motor_left->channel_in1);
	HAL_TIM_PWM_Start(motor_left->htim, motor_left->channel_in2);

	__HAL_TIM_SET_COMPARE(Mot_left.htim, Mot_left.channel_in2, 0);
	__HAL_TIM_SET_COMPARE(Mot_left.htim, Mot_left.channel_in1, 0);

	__HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in1, 0);
	__HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in2, 0);
}


static inline uint32_t timer_period(TIM_HandleTypeDef *htim) {
    return __HAL_TIM_GET_AUTORELOAD(htim); // current ARR
}

static inline uint32_t speed_to_ccr_motor(TIM_HandleTypeDef *htim,  uint32_t min_ccr, int8_t v)
{
    uint32_t period = timer_period(htim);
    uint8_t duty_cycle = (uint8_t)(v < 0 ? -v : v);
    if (duty_cycle == 0) return 0;
    uint16_t slope = (period - min_ccr)  / (MAX_PWM - MIN_PWM);
    return ((uint32_t)slope * (uint32_t)duty_cycle) +  min_ccr; // y = m * x + b  =>  x -> duty cycle, y -> CCR
}

void move_robot(int8_t left_speed, int8_t right_speed)
{
	int8_t L = clamp_int(left_speed,  -100, 100);
	int8_t R = clamp_int(right_speed, -100, 100);

	uint32_t ccrL = speed_to_ccr_motor(Mot_left.htim,  MIN_CCR_L, L);
	uint32_t ccrR = speed_to_ccr_motor(Mot_right.htim, MIN_CCR_R, R);

	if (L >= 0) {
		__HAL_TIM_SET_COMPARE(Mot_left.htim,  Mot_left.channel_in2, 0);
		__HAL_TIM_SET_COMPARE(Mot_left.htim,  Mot_left.channel_in1, ccrL);
	} else {
		__HAL_TIM_SET_COMPARE(Mot_left.htim,  Mot_left.channel_in1, 0);
		__HAL_TIM_SET_COMPARE(Mot_left.htim,  Mot_left.channel_in2, ccrL);
	}

	if (R >= 0) {
		__HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in1, 0);
		__HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in2, ccrR);
	} else {
		__HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in2, 0);
		__HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in1, ccrR);
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

/*

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


/*
	  s_brake = (v*v / 2*a) + min_dist;
	  if (d > 5)   set_speed = k*(d - 5);     // approach
	  else         set_speed = 0;             // hold
*/

/*
	  sensor = get_LTS_value(&cfg);
	  ips = get_IPS_value(&ips_cfg);
*/

/*
	  //move_forward
	  Motor_SetPWM(&Mot_right, MOTOR_DIR_FORWARD, 50.0, ramping_step);
	  Motor_SetPWM(&Mot_left, MOTOR_DIR_FORWARD, 50.0, ramping_step);
	  vTaskDelayUntil(&xLastWakeTime, xPeriod);


	  Motor_SetPWM(&Mot_right, MOTOR_DIR_REVERSE, 50.0, ramping_step);
	  Motor_SetPWM(&Mot_left, MOTOR_DIR_REVERSE, 50.0, ramping_step);
	  vTaskDelayUntil(&xLastWakeTime, xPeriod);

	  Motor_SetPWM(&Mot_right, MOTOR_DIR_COAST, 0.0, ramping_step);
	  Motor_SetPWM(&Mot_left, MOTOR_DIR_COAST, 0.0, ramping_step);
	  vTaskDelayUntil(&xLastWakeTime, xPeriod);
*/


// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Encoders -------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------


/*
//extern EncoderHandle leftEnc;
//extern EncoderHandle rightEnc;

static inline int16_t signed_delta_u16(uint16_t now, uint16_t last) {
    return (int16_t)(now - last); // handles wrap by design
}

void Encoder_Init(EncoderHandle *e, const EncoderConfig *cfg)
{
    e->cfg = *cfg;
    e->pos_counts = 0;
    e->vel_cps = 0.0f;

    // Start the timer in encoder mode (CubeMX created htim init)
    HAL_TIM_Encoder_Start(e->cfg.htim, TIM_CHANNEL_ALL);

    // Optional: center to reduce immediate wrap
    __HAL_TIM_SET_COUNTER(e->cfg.htim, 0);
    e->last_timer = (uint16_t)__HAL_TIM_GET_COUNTER(e->cfg.htim);
}

void Encoder_Zero(EncoderHandle *e, int32_t to_counts)
{
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    e->pos_counts = to_counts;
    __set_PRIMASK(prim);
}

void Encoder_Update(EncoderHandle *e, float dt_s)
{
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(e->cfg.htim);
    int16_t  d   = signed_delta_u16(now, e->last_timer);
    e->last_timer = now;

    int32_t inc = (int32_t)d;
    if (e->cfg.inverted) inc = -inc;

    // Position (32-bit)
    e->pos_counts += inc;

    // Velocity in counts/sec (simple IIR low-pass optional)
    if (dt_s > 0.0f) {
        float cps = (float)inc / dt_s;
        // simple smoothing; set alpha=1.0f for no filtering
        const float alpha = 0.3f;
        e->vel_cps = alpha * cps + (1.0f - alpha) * e->vel_cps;
    }
}

int32_t Encoder_GetCounts(const EncoderHandle *e) { return e->pos_counts; }

float Encoder_GetRevolutions(const EncoderHandle *e)
{
    return (e->cfg.cpr > 0) ? ((float)e->pos_counts / (float)e->cfg.cpr) : 0.0f;
}

float Encoder_GetRPS(const EncoderHandle *e)
{
    return (e->cfg.cpr > 0) ? (e->vel_cps / (float)e->cfg.cpr) : 0.0f;
}

float Encoder_GetRPM(const EncoderHandle *e) { return 60.0f * Encoder_GetRPS(e); }

float Encoder_GetLinearSpeed(const EncoderHandle *e) // m/s at the wheel rim
{
    return Encoder_GetRPS(e) * e->cfg.wheel_circum_m;
}



void Encoders_InitAll(void)
{
    EncoderConfig L = {
        .htim = &htim1,              // whatever TIM is left encoder
        .inverted = false,
        .cpr = 3840u,                // example: 8 PPR × 4 × 120
        .wheel_circum_m = 0.210f     // example: wheel circumference in meters
    };


    EncoderConfig R = {
        .htim = &htim2,              // whatever TIM is right encoder
        .inverted = true,            // flip direction if needed
        .cpr = 3840u,
        .wheel_circum_m = 0.210f
    };

    Encoder_Init(&leftEnc,  &L);
    //Encoder_Init(&rightEnc, &R);
}

void Encoders_UpdateAll(float dt_s)
{
    Encoder_Update(&leftEnc,  dt_s);
    //Encoder_Update(&rightEnc, dt_s);
}



typedef struct{
	int16_t velocity;
	int64_t position;
	uint32_t last_counter_value;
}encoder_instance;

volatile encoder_instance enc_instance_mot;

volatile uint32_t timer_counter = 0;
volatile uint32_t encoder_position = 1;
volatile uint32_t encoder_velocity = 2;

void update_encoder(volatile encoder_instance *encoder_value, TIM_HandleTypeDef *htim)
 {
	uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim);
	static uint8_t first_time = 0;
	if(!first_time) {
	   encoder_value ->velocity = 0;
	   first_time = 1;

	} else {

	  if(temp_counter == encoder_value ->last_counter_value) {
		encoder_value ->velocity = 0;

	  } else if (temp_counter > encoder_value ->last_counter_value)  {
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
				encoder_value ->velocity = -encoder_value ->last_counter_value - (__HAL_TIM_GET_AUTORELOAD(htim)-temp_counter);

			} else {
				encoder_value ->velocity = temp_counter -  encoder_value ->last_counter_value;
			}
	  } else {
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
				encoder_value ->velocity = temp_counter - encoder_value ->last_counter_value;

			} else {
			encoder_value ->velocity = temp_counter + (__HAL_TIM_GET_AUTORELOAD(htim) - encoder_value ->last_counter_value);
			}
	   }
	}
	encoder_value ->position += encoder_value ->velocity;
	encoder_value ->last_counter_value = temp_counter;
}


typedef struct {
    TIM_HandleTypeDef *htim;   // Timer in Encoder mode
    bool inverted;             // flip direction if needed
    uint32_t cpr;              // counts per wheel revolution
    float wheel_circum_m;      // circumference in meters
} EncoderConfig;

typedef struct {
    EncoderConfig cfg;
    uint16_t last_timer;       // last raw timer count
    int32_t pos_counts;        // 32-bit position
    float vel_cps;             // counts/sec (filtered)
} EncoderHandle;

static inline int16_t signed_delta_u16(uint16_t now, uint16_t last) {
    return (int16_t)(now - last); // handles wrap
}


void Encoder_Init(EncoderHandle *e, EncoderConfig *cfg) {
    e->cfg = *cfg;
    e->pos_counts = 0;
    e->vel_cps = 0.0f;

    HAL_TIM_Encoder_Start(e->cfg.htim, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(e->cfg.htim, 0);
    e->last_timer = (uint16_t)__HAL_TIM_GET_COUNTER(e->cfg.htim);
}

void Encoder_Update(EncoderHandle *e, float dt_s) {
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(e->cfg.htim);
    int16_t d = signed_delta_u16(now, e->last_timer);
    e->last_timer = now;

    int32_t inc = (int32_t)d;
    if (e->cfg.inverted) inc = -inc;

    e->pos_counts += inc;

    if (dt_s > 0.0f) {
        float cps = (float)inc / dt_s;
        const float alpha = 0.3f; // smoothing
        e->vel_cps = alpha * cps + (1.0f - alpha) * e->vel_cps;
    }
}

int32_t Encoder_GetCounts(EncoderHandle *e) {
    return e->pos_counts;
}

float Encoder_GetRPM(EncoderHandle *e) {
    return (e->cfg.cpr > 0) ? (60.0f * e->vel_cps / e->cfg.cpr) : 0.0f;
}

float Encoder_GetLinearSpeed(EncoderHandle *e) {
    return (e->cfg.cpr > 0) ? (e->vel_cps * e->cfg.wheel_circum_m / e->cfg.cpr) : 0.0f;
}


EncoderHandle enc1;   // later you can add enc2, enc3, enc4

static volatile uint8_t uart3_tx_busy = 0;

*/

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Micro Servo SG90 9g --------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------
extern ServoHandle servo;

void servo_init(ServoHandle *servo)
{
	HAL_TIM_PWM_Start(servo->htim, servo->channel);
}

void servo_set_angle(uint8_t angle) {
   if (angle < FAR_RIGHT_ANGLE) angle = FAR_RIGHT_ANGLE;
   if (angle > FAR_LEFT_ANGLE)  angle = FAR_LEFT_ANGLE;

   uint8_t slope = (MAX_SERVO_PULSE - MIN_SERVO_PULSE) / (FAR_LEFT_ANGLE - FAR_RIGHT_ANGLE);

   uint16_t pulse = slope * angle + MIN_SERVO_PULSE; // Pulse range: 1ms (100) to 2ms (200) at 10µs resolution (100 kHz)
   __HAL_TIM_SET_COMPARE(servo.htim, servo.channel, pulse);
}






// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------  -------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

/*

// ---- UART3 busy flag (optional safety) ----

//static uint32_t dropCount = 0;   // counts how many messages we skipped


//static EncoderHandle leftEnc;
//static EncoderHandle rightEnc;

//static EncoderData_t gEncData;           // shared


//SemaphoreHandle_t gEncMtx = NULL;       // mutex protecting shared encoder data

int __io_putchar(int ch) { ITM_SendChar(ch); return ch; }  // routes printf to SWO

// optional: unbuffer stdout so prints appear immediately
void printf_init_unbuffered(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
}

static void swo_unbuffered(void){ setvbuf(stdout, NULL, _IONBF, 0); }

// Minimal retarget: printf → ITM (SWO)
int _write(int file, char *ptr, int len){
    (void)file;
    for(int i=0;i<len;i++) ITM_SendChar((uint32_t)*ptr++);
    return len;
}



void app_init_encoders(void)
{
    EncoderConfig leftCfg = {
        .htim = &htim1,            // the TIM you put in Encoder Mode
        .inverted = false,         // set true if direction is reversed
        .cpr = 2048u * 4u * 30u,   // example: 2048 PPR encoder, x4, 30:1 gearbox
        .wheel_circum_m = 0.314f,  // e.g., 100 mm diameter => pi*0.10 = 0.314 m
    };
    Encoder_Init(&leftEnc, &leftCfg);
}

// Call at fixed rate (e.g., 1 kHz)
void app_encoder_tick_1kHz(void)
{
    Encoder_Update(&leftEnc, 0.001f);
}



LTSensor sensor = {0,0,0,0};

 extern volatile int datasentflag;
 volatile uint32_t period_left = 0;
volatile uint32_t period_right = 0;
LineState lines_state = 0;
IPSensor ips  = {0,0};
LTS_Config cfg = {GPIOC, LTS_left_GPI_Pin, GPIOC, LTS_middle_GPI_Pin, GPIOC, LTS_right_GPI_Pin};

ServoHandle servo = {
    .htim = &htim16,
    .channel = TIM_CHANNEL_1,   // PB4 (example, your mapping)
};

const uint8_t ramping_step = 5;

volatile uint8_t angle = 75;

#define PULSES_PER_REV 20

// Shared outputs (read in your task)
volatile uint32_t period_right_ticks = 0;  // TIM15 ticks between edges (right)
volatile uint32_t period_left_ticks  = 0;  // TIM15 ticks between edges (left)
volatile uint32_t last_right_ms = 0;
volatile uint32_t last_left_ms  = 0;


// Internals for overflow-safe measurement
volatile uint32_t tim15_overflow_count = 0;

uint32_t rpmR = 0;
uint32_t rpmL = 0;

uint8_t rxByte;              // one byte received
char rxLine[16];             // buffer for ASCII number
uint8_t rxIndex = 0;

volatile uint8_t g_lts_bits;
volatile uint8_t line_bits = 0;



void uart_receive_line(char *buf, uint16_t maxlen) {
    uint16_t i = 0;
    uint8_t c;
    while (i < maxlen - 1) {
        if (HAL_UART_Receive(&huart3, &c, 1, 5000) == HAL_OK) { // 5s timeout
            if (c == '\r' || c == '\n') break;
            buf[i++] = c;
        }
    }
    buf[i] = '\0';
}
*/


/*

 uint32_t distance_cm = 0;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{

	HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_4);
	datasentflag=1;
}


// For Bluetooth recieving

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        char c = (char)rxByte;

        if (c == '\r') {
            // ignore CR
        }
        else if (c == '\n') {
            // end of line → convert string to number
            rxLine[rxIndex] = '\0';
            int v = atoi(rxLine);
            if (v < 0) v = 0;
            if (v > 180) v = 180;
            angle = (uint8_t)v;

            rxIndex = 0; // reset for next number
        }
        else {
            if (rxIndex < sizeof(rxLine) - 1) {
                rxLine[rxIndex++] = c;
            } else {
                rxIndex = 0; // overflow → reset
            }
        }

        HAL_UART_Receive_DMA(&huart3, &rxByte, 1);
    }
}


servo_init(&servo);

LTS_Config cfg = {GPIOC, LTS_left_GPI_Pin,
				  GPIOC, LTS_middle_GPI_Pin,
				  GPIOC, LTS_right_GPI_Pin
				 };

IPS_Config ips_cfg = {GPIOC, IPS_left_GPI_Pin,  GPIOC, IPS_right_GPI_Pin};

servo_set_angle(CENTER_ANGLE);

const TickType_t LINE_LOSS_TIMEOUT  = pdMS_TO_TICKS(200);
TickType_t last_on_line  = xTaskGetTickCount();

LTS_init(&htim16, TIM_CHANNEL_1);


uint8_t raw = g_lts_bits;
LineState lines_state = (LineState)(raw & 0x07);


TickType_t now = xTaskGetTickCount();

if (s != NO_LINE) {
  last_on_line = now;
}

if (s == NO_LINE && (int32_t)(now - last_on_line) >= (int32_t)LINE_LOSS_TIMEOUT ) {
  stop_robot();
  continue;
}


switch(sensor.current_state){
case NO_LINE: 	move_robot(ForwardSpeed, ForwardSpeed); break;
case MIDDLE:		move_robot(ForwardSpeed, ForwardSpeed); break;

case RIGHT:		 move_robot(ForwardCurveSpeed,stop); break;
case RIGHT_CURVE:	 move_robot(ForwardCurveSpeed,stop); break;

case LEFT:		 move_robot(stop, ForwardCurveSpeed); break;
case LEFT_CURVE:	 move_robot(stop,ForwardCurveSpeed); break;

case U_TURN:		stop_robot(); 	break;
case JUNCTION:	stop_robot(); 	break;
}

TickType_t xLastWakeTime = xTaskGetTickCount();
TickType_t noLineStartTime = 0;

LTS_Config cfg = {GPIOC, LTS_left_GPI_Pin, GPIOC, LTS_middle_GPI_Pin, GPIOC, LTS_right_GPI_Pin};

bool noLineActive = false;
vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(1000));

  sensor = get_LTS_value(&cfg);

  if (sensor.current_state == NO_LINE ) {
	  if (!noLineActive){
		  noLineStartTime = xTaskGetTickCount();
		  noLineActive = true;
	  } else {
		  if((xTaskGetTickCount() - noLineStartTime) >= pdMS_TO_TICKS(3000)){
			  stop_robot();
			  vTaskDelay(pdMS_TO_TICKS(10));
			  continue;
		  }
	  }
  } else {
	  noLineActive = false;
  }


		//vTaskDelayUntil(&xLastWake, period);

		// --- Update encoders at fixed dt ---
		//Encoder_Update(&leftEnc,  dt);
		//Encoder_Update(&rightEnc, dt);
		//Encoders_UpdateAll(dt);

		// --- Read ---
		EncoderData_t tmp;
		tmp.L_counts = Encoder_GetCounts(&leftEnc);
		tmp.L_rpm    = Encoder_GetRPM(&leftEnc);
		tmp.L_mps    = Encoder_GetLinearSpeed(&leftEnc);

		tmp.R_counts = Encoder_GetCounts(&rightEnc);
		tmp.R_rpm    = Encoder_GetRPM(&rightEnc);
		tmp.R_mps    = Encoder_GetLinearSpeed(&rightEnc);

		if (gEncMtx) {
			xSemaphoreTake(gEncMtx, portMAX_DELAY);
			gEncData = tmp;
			xSemaphoreGive(gEncMtx);
		} else {
			gEncData = tmp;
		}

		//GPIO_PinState L = HAL_GPIO_ReadPin(GPIOC, LTS_left_GPI_Pin);
		//GPIO_PinState R = HAL_GPIO_ReadPin(GPIOC, LTS_right_GPI_Pin);


  switch(g_lts_bits){
	  case NO_LINE: 	move_robot(ForwardSpeed, ForwardSpeed); break;
	  case MIDDLE:		move_robot(ForwardSpeed, ForwardSpeed); break;

	  case RIGHT:		move_robot(ForwardCurveSpeed,stop); 		break;
	  case RIGHT_CURVE:	move_robot(ForwardCurveSpeed, stop_curve);  break;

	  case LEFT:		move_robot(stop, ForwardCurveSpeed); 		break;
	  case LEFT_CURVE:	move_robot(stop_curve, ForwardCurveSpeed);  break;

	  case U_TURN:		stop_robot(); 	break;
	  case JUNCTION:	stop_robot(); 	break;
  }


		if (L == GPIO_PIN_SET && R == GPIO_PIN_SET){
			__HAL_TIM_SET_COMPARE(Mot_left.htim, Mot_left.channel_in2, 0);
			__HAL_TIM_SET_COMPARE(Mot_left.htim, Mot_left.channel_in1, 0);

			__HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in1, 0);
			__HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in2, 0);
		}

		else if(HAL_GPIO_ReadPin(GPIOC, LTS_left_GPI_Pin) == GPIO_PIN_SET){
			  //move_robot(stop, ForwardCurveSpeed);
			  __HAL_TIM_SET_COMPARE(Mot_left.htim, Mot_left.channel_in2, 0);
			  __HAL_TIM_SET_COMPARE(Mot_left.htim, Mot_left.channel_in1, 0);

			  __HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in1, 0);
			  __HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in2, 250);
		  }

		  else if (HAL_GPIO_ReadPin(GPIOC, LTS_right_GPI_Pin) == GPIO_PIN_SET) {
			  //move_robot(ForwardCurveSpeed,stop);
			  __HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in1, 0);
			  __HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in2, 0);

			  __HAL_TIM_SET_COMPARE(Mot_left.htim,  Mot_left.channel_in2, 0);
			  __HAL_TIM_SET_COMPARE(Mot_left.htim,  Mot_left.channel_in1, 250);
		  }

		  else {
			  //move_robot(ForwardSpeed, ForwardSpeed);
			  __HAL_TIM_SET_COMPARE(Mot_left.htim,  Mot_left.channel_in2, 0);
			  __HAL_TIM_SET_COMPARE(Mot_left.htim,  Mot_left.channel_in1, 315);

			  __HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in1, 0);
			  __HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in2, 318);
		  }

		//vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(10));

		//uint16_t R_crr_current = R_CCR_52Per;
	//uint16_t L_crr_current = L_CCR_52Per;

	//const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz update
	//const float dt = 0.010f;
	//TickType_t xLastWake = xTaskGetTickCount();


	TickType_t xLastWakeTime = xTaskGetTickCount();
	vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(4000));
	taskENTER_CRITICAL();
				__HAL_TIM_SET_COMPARE(Mot_left.htim,  Mot_left.channel_in2, 0);
				__HAL_TIM_SET_COMPARE(Mot_left.htim,  Mot_left.channel_in1, 500);
				__HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in1, 0);
				__HAL_TIM_SET_COMPARE(Mot_right.htim, Mot_right.channel_in2, 500);

				__HAL_TIM_SET_COUNTER(&htim6, 0);
				__HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
				HAL_TIM_Base_Start_IT(&htim6);
	taskEXIT_CRITICAL();

*/

/*
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        uart3_tx_busy = 0;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(StatsPrint_TaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void to_binary(char *dest, uint8_t val, uint8_t bits)
{
    for (int i = bits - 1; i >= 0; --i) {
        *dest++ = (val & (1 << i)) ? '1' : '0';
    }
    *dest = '\0';
}
*/









