#ifndef _DEF_H_
#define _DEF_H_

#define VERBOSE_OUTPUT 1
#define CAM_USE_MULTICLIENT 1
#define USE_ESP32_PWM_API 0

#if USE_ESP32_PWM_API == 1
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_TIMER LEDC_TIMER_0
#define LEFT_MOTOR_PWM_CHANNEL (LEDC_CHANNEL_0)
#define RIGHT_MOTOR_PWM_CHANNEL (LEDC_CHANNEL_1)
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY          (50) // Frequency in Hertz. Set frequency at 4 kHz
#endif

#endif