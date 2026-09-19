#include "ServoControl.h"
#include <stdlib.h>
#include <cmath> // Hỗ trợ fmod

#define ARR 5000U
#define MAX_PWM (ARR - 1)

Servo::Servo()
    : enc(nullptr), servoControl(nullptr), speed(0), targetPosition(0),
      currentPosition(0), direction(true), Kp(1.0f), Ki(0.0f), Kd(0.0f), lastTick(0),
      minAngleLimit(SERVO_DEFAULT_MIN_ANGLE), maxAngleLimit(SERVO_DEFAULT_MAX_ANGLE) {}

Servo::Servo(Encoder_t* enc, ServoControl_t* servoControl)
    : enc(enc), servoControl(servoControl), speed(0), targetPosition(0),
      currentPosition(0), direction(true), Kp(1.0f), Ki(0.0f), Kd(0.0f), lastTick(0),
      minAngleLimit(SERVO_DEFAULT_MIN_ANGLE), maxAngleLimit(SERVO_DEFAULT_MAX_ANGLE) {}

void Servo::setupServo(Encoder_t* enc, ServoControl_t* servoControl) {
    this->enc = enc;
    this->servoControl = servoControl;
    this->currentPosition = 0;
    this->targetPosition = 0;
    this->minAngleLimit = SERVO_DEFAULT_MIN_ANGLE;
    this->maxAngleLimit = SERVO_DEFAULT_MAX_ANGLE;
    this->lastTick = HAL_GetTick();

    if (this->enc != nullptr && this->enc->htim != nullptr) {
        HAL_TIM_Encoder_Start(this->enc->htim, TIM_CHANNEL_ALL);
        __HAL_TIM_SET_COUNTER(this->enc->htim, 0);
    }
}

void Servo::setMechanicalConfig(float minAngle, float maxAngle) {
    this->minAngleLimit = minAngle;
    this->maxAngleLimit = maxAngle;
}

void Servo::setPID(float kp, float ki, float kd) {
    this->Kp = kp;
    this->Ki = ki;
    this->Kd = kd;
}

void Servo::ReadEncoder() {
    if (enc == nullptr || enc->htim == nullptr) return;

    bool isCountingUp = ((enc->htim->Instance->CR1 & TIM_CR1_DIR) == 0);
    this->direction = isCountingUp;

    uint16_t temp_counter = (uint16_t)__HAL_TIM_GET_COUNTER(enc->htim);
    __HAL_TIM_SET_COUNTER(enc->htim, 0);

    if (this->direction) {
        this->currentPosition += temp_counter;
    } else {
        if (temp_counter > 0) {
            uint16_t steps_backwards = (uint16_t)(65536U - temp_counter);
            this->currentPosition -= steps_backwards;
        }
    }
}

void Servo::updateEncoderPosition() {
    ReadEncoder();
}

int32_t Servo::getCurrentPosition() const {
    return currentPosition;
}

void Servo::home() {
    this->currentPosition = 0;
    this->targetPosition = 0;
    applyPWM(0);
}

void Servo::setDirection(bool isForward) {
    this->direction = isForward;
}

void Servo::setSpeed(float stepFrequency) {
    float calculatedSpeed = calculateSpeed(stepFrequency);
    this->speed = (calculatedSpeed > 0) ? calculatedSpeed : 0;
}

void Servo::applyPWM(int32_t pwmValue) {
    if (servoControl == nullptr) return;

    if (pwmValue > (int32_t)MAX_PWM) pwmValue = MAX_PWM;
    if (pwmValue < -(int32_t)MAX_PWM) pwmValue = -MAX_PWM;

    if (pwmValue > 0) {
        __HAL_TIM_SET_COMPARE(servoControl->htim1, servoControl->channel1, pwmValue);
        __HAL_TIM_SET_COMPARE(servoControl->htim2, servoControl->channel2, 0);
    } else if (pwmValue < 0) {
        __HAL_TIM_SET_COMPARE(servoControl->htim1, servoControl->channel1, 0);
        __HAL_TIM_SET_COMPARE(servoControl->htim2, servoControl->channel2, -pwmValue);
    } else {
        __HAL_TIM_SET_COMPARE(servoControl->htim1, servoControl->channel1, 0);
        __HAL_TIM_SET_COMPARE(servoControl->htim2, servoControl->channel2, 0);
    }
}

void Servo::setOnOff(bool isEnabled) {
    if (servoControl == nullptr) return;

    if (isEnabled) {
        HAL_TIM_PWM_Start(servoControl->htim1, servoControl->channel1);
        HAL_TIM_PWM_Start(servoControl->htim2, servoControl->channel2);
    } else {
        applyPWM(0);
        HAL_TIM_PWM_Stop(servoControl->htim1, servoControl->channel1);
        HAL_TIM_PWM_Stop(servoControl->htim2, servoControl->channel2);
    }
}

void Servo::moveTo(int32_t position) {
    this->targetPosition = position;
    setOnOff(true);
}

void Servo::setTargetPosition(int32_t position) {
    this->targetPosition = position;
}

float Servo::calculatePID(float dt) {
    float error = static_cast<float>(targetPosition - currentPosition);
    static float integral = 0.0f;
    static float previous_error = 0.0f;

    integral += error * dt;
    if (integral > (float)MAX_PWM) integral = (float)MAX_PWM;
    if (integral < -(float)MAX_PWM) integral = -(float)MAX_PWM;

    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - previous_error) / dt;
    }

    float output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    return output;
}

void Servo::updatePosition() {
    uint32_t currentTick = HAL_GetTick();
    float dt = (float)(currentTick - lastTick) / 1000.0f;

    if (dt > 0.0f) {
        ReadEncoder();
        float pidOutput = calculatePID(dt);
        applyPWM((int32_t)pidOutput);
        lastTick = currentTick;
    }
}

// --- THUẬT TOÁN CHỌN CHIỀU AN TOÀN CHO SERVO ---
void Servo::setTargetAngle(float degrees) {
    if (degrees < minAngleLimit) degrees = minAngleLimit;
    if (degrees > maxAngleLimit) degrees = maxAngleLimit;

    float currentAbsAngle = ((float)currentPosition / PULSE_PER_REV) * 360.0f;
    float currentMod = fmod(currentAbsAngle, 360.0f);
    if (currentMod < 0.0f) currentMod += 360.0f;

    float safeDiff = degrees - currentMod;
    float newAbsAngle = currentAbsAngle + safeDiff;

    this->targetPosition = (int32_t)((newAbsAngle / 360.0f) * PULSE_PER_REV);
}

float Servo::getCurrentRPM(float dt) {
    if (dt <= 0.0f) return 0.0f;

    static int32_t lastPosition = 0;
    int32_t deltaPosition = this->currentPosition - lastPosition;
    lastPosition = this->currentPosition;

    float rpm = ((float)deltaPosition / PULSE_PER_REV) * (60.0f / dt);
    return rpm;
}

float Servo::calculateSpeed(float rpmTarget) {
    return (rpmTarget * PULSE_PER_REV) / 60.0f;
}

bool Servo::isTargetReached(int32_t tolerance) {
    return (abs(targetPosition - currentPosition) <= tolerance);
}