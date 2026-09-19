#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "main.h"

// =========================================================================
// THÔNG SỐ CƠ KHÍ MẶC ĐỊNH CHO SERVO
// =========================================================================
#define SERVO_DEFAULT_MIN_ANGLE       0.0f      
#define SERVO_DEFAULT_MAX_ANGLE       270.0f    

#define ENCODER_BASE_PPR 11.0f    
#define GEAR_RATIO 100.0f         
#define GEAR_ADDITIONAL_RATIO 1.0f 
#define PULSE_PER_REV (ENCODER_BASE_PPR * 4.0f * GEAR_RATIO * GEAR_ADDITIONAL_RATIO) 

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t channel1;
    uint32_t channel2;
} Encoder_t;

typedef struct {
    TIM_HandleTypeDef* htim1;
    uint32_t channel1;

    TIM_HandleTypeDef* htim2;
    uint32_t channel2;
} ServoControl_t;

#ifdef __cplusplus
} 
#endif

#ifdef __cplusplus
class Servo {
private:
    Encoder_t* enc;
    ServoControl_t* servoControl;

    int32_t speed;
    volatile int32_t targetPosition;
    volatile int32_t currentPosition;
    bool direction; 

    float Kp, Ki, Kd;
    uint32_t lastTick;

    float minAngleLimit;
    float maxAngleLimit;

    void applyPWM(int32_t pwmValue);

public:
    Servo();
    Servo(Encoder_t* enc, ServoControl_t* servoControl);

    void setupServo(Encoder_t* enc, ServoControl_t* servoControl);
    void setMechanicalConfig(float minAngle, float maxAngle); 
    
    void setPID(float kp, float ki, float kd);

    bool isTargetReached(int32_t tolerance = 15);
    void ReadEncoder();
    void setTargetPosition(int32_t position);
    void setSpeed(float stepFrequency);
    void moveTo(int32_t position);
    void setOnOff(bool isEnabled);
    void setDirection(bool isForward);
    
    void updatePosition(); 
    
    int32_t getCurrentPosition() const;
    void home();

    float calculateSpeed(float stepFrequency);
    void updateEncoderPosition();
    float calculatePID(float dt);

    void setTargetAngle(float degrees); // Thuật toán chọn chiều cho Servo
    float getCurrentRPM(float dt);
};
#endif // __cplusplus

#endif // SERVO_CONTROL_H