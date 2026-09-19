#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include "main.h"

// =========================================================================
// THÔNG SỐ CƠ KHÍ MẶC ĐỊNH CHO STEPPER
// =========================================================================
#define DEFAULT_MIN_ANGLE       0.0f      // Góc giới hạn dư	ới (Limit)
#define DEFAULT_MAX_ANGLE       270.0f    // Góc giới hạn trên (Limit)
#define DEFAULT_PULSE_PER_REV   400.0f   // Số xung / 1 vòng quay của động cơ
#define DEFAULT_GEAR_RATIO      1.0f      // Tỷ số truyền (Gear ratio)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    GPIO_TypeDef* direction_port;
    uint16_t      direction_pin;
    
    GPIO_TypeDef* onoff_port;
    uint16_t      onoff_pin;
} MotorControl_t;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class Stepper
{
private:
    TIM_HandleTypeDef* htim;
    uint32_t channel;

    float speed;
    int count;

    MotorControl_t pins;

    volatile int32_t nextPosition;
    volatile int32_t currentPosition;

    float minAngleLimit;
    float maxAngleLimit;
    float pulsePerRev;
    float gearRatio;

public:
    Stepper();
    Stepper(TIM_HandleTypeDef* htim, uint32_t channel, MotorControl_t pins);

    void setupStepper(TIM_HandleTypeDef* htim, uint32_t channel, MotorControl_t pins);
    void setMechanicalConfig(float minAngle, float maxAngle, float ppr, float gear);
    
    void setOnOff(bool isEnabled);
    void setDirection(bool isForward);
    void changeSpeed(float stepFrequency);
    
    void step();
    int32_t distanceToGo();
    void stepCount();
    
    void move(int32_t position);          
    void moveToAngle(float targetAngle);  // Hàm di chuyển góc với thuật toán chống va chạm Limit
    
    void home();
    
    int32_t getCurrentPosition() const;
    void onTimerInterrupt();
};
#endif // __cplusplus

#endif
