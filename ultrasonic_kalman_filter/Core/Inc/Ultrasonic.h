#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

#include "main.h"

typedef struct {
	// Hardware Handles
	TIM_HandleTypeDef *htim_trigger;
	uint32_t trigger_channel;

	TIM_HandleTypeDef *htim_echo;
	uint32_t echo_channel;

	// Measurement Data
	volatile uint32_t diff;         // Pulse duration in microseconds
	volatile float distance_m;
	volatile float distance_cm;
	volatile float distance_mm;
} Ultrasonic_HandleTypeDef;

// Functional Prototypes
void Ultrasonic_Init(Ultrasonic_HandleTypeDef *hus, TIM_HandleTypeDef *h_trig,
		uint32_t ch_trig, TIM_HandleTypeDef *h_echo, uint32_t ch_echo);
void Ultrasonic_StartAutonomous(Ultrasonic_HandleTypeDef *hus);
void Ultrasonic_CaptureCallback(Ultrasonic_HandleTypeDef *hus,
		TIM_HandleTypeDef *htim);

#endif
