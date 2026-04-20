#include "Ultrasonic.h"

void Ultrasonic_Init(Ultrasonic_HandleTypeDef *hus, TIM_HandleTypeDef *h_trig,
		uint32_t ch_trig, TIM_HandleTypeDef *h_echo, uint32_t ch_echo) {
	hus->htim_trigger = h_trig;
	hus->trigger_channel = ch_trig;
	hus->htim_echo = h_echo;
	hus->echo_channel = ch_echo;
	hus->diff = 0;
	hus->distance_cm = 0.0f;
}

void Ultrasonic_StartAutonomous(Ultrasonic_HandleTypeDef *hus) {
	// 1. Start Autonomous Trigger Generation (60ms cycle) [cite: 262, 456]
	HAL_TIM_Base_Start(hus->htim_trigger);
	HAL_TIM_PWM_Start(hus->htim_trigger, hus->trigger_channel);

	// 2. Start Echo Capture in Slave Mode Reset [cite: 815, 816, 996]
	HAL_TIM_Base_Start(hus->htim_echo);
	HAL_TIM_IC_Start_IT(hus->htim_echo, TIM_CHANNEL_1); // Rising Edge Reset
	HAL_TIM_IC_Start_IT(hus->htim_echo, TIM_CHANNEL_2); // Falling Edge Capture
}

void Ultrasonic_CaptureCallback(Ultrasonic_HandleTypeDef *hus,
		TIM_HandleTypeDef *htim) {
	if (htim->Instance == hus->htim_echo->Instance) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			// CCR2 = t_high in microseconds [cite: 1006, 1110]
			hus->diff = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

			if (hus->diff > 0) {
				// Conversion factor: 1/58 cm/us
				hus->distance_cm = (float) hus->diff / 58.0f;
			}
		}
	}
}
