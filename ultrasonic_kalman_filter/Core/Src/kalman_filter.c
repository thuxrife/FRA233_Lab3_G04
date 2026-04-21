#include "kalman_filter.h"

/**
 * @brief Manual 2x2 Matrix Identity and Initialization
 */
static void _Kalman_ResetMatrices(Kalman_HandleTypeDef *hkf) {
	for (int i = 0; i < 4; i++) {
		hkf->P[i] = 0.0f;
		hkf->F[i] = 0.0f;
		hkf->Q[i] = 0.0f;
	}
	hkf->P[0] = 1.0f;
	hkf->P[3] = 1.0f; // Initial uncertainty
}

void Kalman_InitZeroOrder(Kalman_HandleTypeDef *hkf, float p_init, float q_p,
		float r_meas) {
	_Kalman_ResetMatrices(hkf);
	hkf->model = KF_MODEL_ZERO_ORDER;
	hkf->x[0] = p_init;
	hkf->x[1] = 0.0f;
	hkf->F[0] = 1.0f; // p_k = p_{k-1}
	hkf->H[0] = 1.0f;
	hkf->H[1] = 0.0f;
	hkf->Q[0] = q_p;
	hkf->R = r_meas;
}

void Kalman_InitFirstOrder(Kalman_HandleTypeDef *hkf, float p_init, float dt,
		float q_p, float q_v, float r_meas) {
	_Kalman_ResetMatrices(hkf);
	hkf->model = KF_MODEL_FIRST_ORDER;
	hkf->dt = dt;
	hkf->x[0] = p_init;
	hkf->F[0] = 1.0f;
	hkf->F[1] = dt;
	hkf->F[3] = 1.0f;
	hkf->H[0] = 1.0f;
	hkf->Q[0] = q_p;
	hkf->Q[3] = q_v;
	hkf->R = r_meas;
}

void Kalman_InitMSD(Kalman_HandleTypeDef *hkf, float p_init, float dt, float m,
		float c, float k, float q_p, float q_v, float r_meas) {
	_Kalman_ResetMatrices(hkf);
	hkf->model = KF_MODEL_MSD;
	hkf->dt = dt;
	hkf->m = m;
	hkf->c = c;
	hkf->k = k;
	hkf->x[0] = p_init;

	// Euler Discretization of MSD Dynamics
	hkf->F[0] = 1.0f;
	hkf->F[1] = dt;
	hkf->F[2] = -(k / m) * dt;
	hkf->F[3] = 1.0f - (c / m) * dt;

	hkf->H[0] = 1.0f;
	hkf->Q[0] = q_p;
	hkf->Q[3] = q_v;
	hkf->R = r_meas;
}

float Kalman_Update(Kalman_HandleTypeDef *hkf, float f_ext, float measurement) {
	// 1. Prediction Step: x = F*x + B*u
	float x_next[2];
	if (hkf->model == KF_MODEL_MSD) {
		float accel_ext = (f_ext / hkf->m) * hkf->dt;
		x_next[0] = hkf->F[0] * hkf->x[0] + hkf->F[1] * hkf->x[1];
		x_next[1] = hkf->F[2] * hkf->x[0] + hkf->F[3] * hkf->x[1] + accel_ext;
	} else {
		x_next[0] = hkf->F[0] * hkf->x[0] + hkf->F[1] * hkf->x[1];
		x_next[1] = hkf->F[2] * hkf->x[0] + hkf->F[3] * hkf->x[1];
	}

	// 2. Predict Covariance: P = F*P*F' + Q
	float p_new[4];
	p_new[0] = hkf->F[0] * (hkf->P[0] * hkf->F[0] + hkf->P[1] * hkf->F[1])
			+ hkf->F[1] * (hkf->P[2] * hkf->F[0] + hkf->P[3] * hkf->F[1])
			+ hkf->Q[0];
	p_new[1] = hkf->F[0] * (hkf->P[0] * hkf->F[2] + hkf->P[1] * hkf->F[3])
			+ hkf->F[1] * (hkf->P[2] * hkf->F[2] + hkf->P[3] * hkf->F[3])
			+ hkf->Q[1];
	p_new[2] = hkf->F[2] * (hkf->P[0] * hkf->F[0] + hkf->P[1] * hkf->F[1])
			+ hkf->F[3] * (hkf->P[2] * hkf->F[0] + hkf->P[3] * hkf->F[1])
			+ hkf->Q[2];
	p_new[3] = hkf->F[2] * (hkf->P[0] * hkf->F[2] + hkf->P[1] * hkf->F[3])
			+ hkf->F[3] * (hkf->P[2] * hkf->F[2] + hkf->P[3] * hkf->F[3])
			+ hkf->Q[3];

	// 3. Kalman Gain: K = P*H' / (H*P*H' + R)
	float S = p_new[0] + hkf->R;
	float K[2] = { p_new[0] / S, p_new[2] / S };

	// 4. Update State: x = x + K*(z - H*x)
	float y = measurement - x_next[0];
	hkf->x[0] = x_next[0] + K[0] * y;
	hkf->x[1] = x_next[1] + K[1] * y;

	// 5. Update Covariance: P = (I - K*H)*P
	hkf->P[0] = (1.0f - K[0]) * p_new[0];
	hkf->P[1] = (1.0f - K[0]) * p_new[1];
	hkf->P[2] = -K[1] * p_new[0] + p_new[2];
	hkf->P[3] = -K[1] * p_new[1] + p_new[3];

	return hkf->x[0];
}
