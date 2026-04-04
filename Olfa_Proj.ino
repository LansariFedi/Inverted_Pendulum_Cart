#include "motor.h"
#include "mpu.h"
#include "pid.h"

static PidBlock sBalancePid;
static const float TARGET_ANGLE_DEG = 0.0f;
static const float MOTOR_POLARITY = -1.0f;
static uint32_t sLastControlMicros = 0;

void setup() {
  Serial.begin(115200);

  bool imuOk = pendulumAngleBegin();
  bool motorOk = motorBegin();

  pidInit(sBalancePid, 0.08f, 0.6f, 0.002f, -1.0f, 1.0f);
  pidSetIntegratorLimits(sBalancePid, -0.4f, 0.4f);
  sLastControlMicros = micros();

  if (!imuOk) {
    Serial.println("MPU init failed");
  }
  if (!motorOk) {
    Serial.println("Motor init failed");
  }
}

void loop() {
  float thetaDeg = 0.0f;
  if (!pendulumAngleWork(thetaDeg)) {
    motorStop();
    delay(5);
    return;
  }

  uint32_t now = micros();
  float dt = (now - sLastControlMicros) * 1e-6f;
  sLastControlMicros = now;
  if (dt <= 0.0f || dt > 0.1f) {
    dt = 0.01f;
  }

  float motorCmd = pidUpdate(sBalancePid, TARGET_ANGLE_DEG, thetaDeg, dt);
  float motorDriveCmd = MOTOR_POLARITY * motorCmd;
  motorSetNormalized(motorDriveCmd);

  static uint32_t sLastPrintMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - sLastPrintMs >= 100) {
    sLastPrintMs = nowMs;
    Serial.print("theta=");
    Serial.print(thetaDeg, 2);
    Serial.print(" cmd=");
    Serial.println(motorDriveCmd, 3);
  }

  delay(5);
}