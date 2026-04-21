#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include "main.h"

/**
 * @brief Kalman Filter Model Selection
 */
typedef enum {
	KF_MODEL_ZERO_ORDER,   // Constant Position
	KF_MODEL_FIRST_ORDER,  // Constant Velocity
	KF_MODEL_MSD           // Mass-Spring-Damper Dynamics
} KF_Model_t;

/**
 * @brief Kalman Filter Handle Structure
 * Designed for 2nd order systems (max 2 states: [pos, vel])
 */
typedef struct {
	KF_Model_t model;
	float dt;               // Sampling period (e.g., 0.001f)

	// State Vector [pos, vel]^T
	float x[2];

	// Covariance Matrices (Stored as 1D arrays for performance)
	float P[4];             // Estimate Error Covariance
	float F[4];             // State Transition Matrix
	float Q[4];             // Process Noise Covariance
	float H[2];             // Observation Matrix
	float R;                // Measurement Noise Covariance

	// MSD Specific Parameters
	float m, c, k;
} Kalman_HandleTypeDef;

// Functional Prototypes
void Kalman_InitZeroOrder(Kalman_HandleTypeDef *hkf, float p_init, float q_p,
		float r_meas);
void Kalman_InitFirstOrder(Kalman_HandleTypeDef *hkf, float p_init, float dt,
		float q_p, float q_v, float r_meas);
void Kalman_InitMSD(Kalman_HandleTypeDef *hkf, float p_init, float dt, float m,
		float c, float k, float q_p, float q_v, float r_meas);

/**
 * @brief Predict and Update step
 * @param f_ext External force (only used in MSD model)
 * @param measurement Raw distance from ultrasonic sensor
 */
float Kalman_Update(Kalman_HandleTypeDef *hkf, float f_ext, float measurement);

#endif
