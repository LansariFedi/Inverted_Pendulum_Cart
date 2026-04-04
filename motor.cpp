#include "motor.h"

#include <math.h>

static const int MOTOR_L_HIGH_PIN = 14;
static const int MOTOR_L_LOW_PIN = 12;
static const int MOTOR_R_HIGH_PIN = 27;
static const int MOTOR_R_LOW_PIN = 26;

static const int PWM_FREQ = 5000;
static const int PWM_RESOLUTION = 8;
static const int PWM_MAX_DUTY = (1 << PWM_RESOLUTION) - 1;

static float clampf(float value, float minValue, float maxValue) {
	if (value < minValue) {
		return minValue;
	}
	if (value > maxValue) {
		return maxValue;
	}
	return value;
}

bool motorBegin() {
	pinMode(MOTOR_L_HIGH_PIN, OUTPUT);
	pinMode(MOTOR_L_LOW_PIN, OUTPUT);
	pinMode(MOTOR_R_HIGH_PIN, OUTPUT);
	pinMode(MOTOR_R_LOW_PIN, OUTPUT);

	bool lHighOk = ledcAttach(MOTOR_L_HIGH_PIN, PWM_FREQ, PWM_RESOLUTION);
	bool lLowOk = ledcAttach(MOTOR_L_LOW_PIN, PWM_FREQ, PWM_RESOLUTION);
	bool rHighOk = ledcAttach(MOTOR_R_HIGH_PIN, PWM_FREQ, PWM_RESOLUTION);
	bool rLowOk = ledcAttach(MOTOR_R_LOW_PIN, PWM_FREQ, PWM_RESOLUTION);

	motorStop();
	return lHighOk && lLowOk && rHighOk && rLowOk;
}

void motorSetNormalized(float command) {
	float saturated = clampf(command, -1.0f, 1.0f);
	int duty = (int)(fabsf(saturated) * (float)PWM_MAX_DUTY);
	if (duty > PWM_MAX_DUTY) {
		duty = PWM_MAX_DUTY;
	}

	if (saturated >= 0.0f) {
		ledcWrite(MOTOR_L_HIGH_PIN, duty);
		ledcWrite(MOTOR_L_LOW_PIN, 0);
		ledcWrite(MOTOR_R_HIGH_PIN, duty);
		ledcWrite(MOTOR_R_LOW_PIN, 0);
	} else {
		ledcWrite(MOTOR_L_HIGH_PIN, 0);
		ledcWrite(MOTOR_L_LOW_PIN, duty);
		ledcWrite(MOTOR_R_HIGH_PIN, 0);
		ledcWrite(MOTOR_R_LOW_PIN, duty);
	}
}

void motorStop() {
	ledcWrite(MOTOR_L_HIGH_PIN, 0);
	ledcWrite(MOTOR_L_LOW_PIN, 0);
	ledcWrite(MOTOR_R_HIGH_PIN, 0);
	ledcWrite(MOTOR_R_LOW_PIN, 0);
}