#ifndef StepperCcontrol_H
#define StepperCcontrol_H
#include "stm32f4xx_hal.h"
#include <stdint.h> // Cho các kiểu dữ liệu uint32_t, uint16_t, v.v.

class Stepper
{
private:
    TIM_HandleTypeDef* htim;
    uint8_t channel;
    int speed;
public:
    Stepper();
    void step();
    void setSpeed(int speed);
    void setDirection(bool direction);


};