#include "StepperControl.h"
#include <cmath> // Thư viện toán học để tính fmod

#define ARR_VALUE_MAX 999U 
#define ARR_VALUE_MIN 199U    
#define PSC_VALUE 95U   
#define PulseWidth (ARR_VALUE_MIN / 2) 

#define SYSTEM_TIMER_CLOCK 16000000.0f

Stepper::Stepper() 
    : htim(nullptr), channel(0), speed(0), count(0),
      pins({nullptr, 0, nullptr, 0}),
      nextPosition(0), currentPosition(0),
      minAngleLimit(DEFAULT_MIN_ANGLE), maxAngleLimit(DEFAULT_MAX_ANGLE), 
      pulsePerRev(DEFAULT_PULSE_PER_REV), gearRatio(DEFAULT_GEAR_RATIO) {}

Stepper::Stepper(TIM_HandleTypeDef* htim, uint32_t channel, MotorControl_t pins)
    : htim(htim), channel(channel), speed(1), count(0),
      pins(pins),
      nextPosition(0), currentPosition(0),
      minAngleLimit(DEFAULT_MIN_ANGLE), maxAngleLimit(DEFAULT_MAX_ANGLE), 
      pulsePerRev(DEFAULT_PULSE_PER_REV), gearRatio(DEFAULT_GEAR_RATIO) {}

void Stepper::setupStepper(TIM_HandleTypeDef* htim, uint32_t channel, MotorControl_t pins) {
    this->htim = htim;
    this->channel = channel;
    this->pins = pins; 
    
    this->currentPosition = 0;
    this->nextPosition = 0;
    
    this->minAngleLimit = DEFAULT_MIN_ANGLE;
    this->maxAngleLimit = DEFAULT_MAX_ANGLE;
    this->pulsePerRev = DEFAULT_PULSE_PER_REV;
    this->gearRatio = DEFAULT_GEAR_RATIO;

    changeSpeed(1000.0f); 
    setOnOff(false);
}

void Stepper::setMechanicalConfig(float minAngle, float maxAngle, float ppr, float gear) {
    this->minAngleLimit = minAngle;
    this->maxAngleLimit = maxAngle;
    this->pulsePerRev = ppr;
    this->gearRatio = gear;
}

void Stepper::step(){
    if (currentPosition < nextPosition) {
        setDirection(true);  
        __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
        HAL_TIM_PWM_Start(htim, channel);    
        HAL_TIM_Base_Start_IT(htim);         
    }
    else if (currentPosition > nextPosition) {
        setDirection(false); 
        __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
        HAL_TIM_PWM_Start(htim, channel);
        HAL_TIM_Base_Start_IT(htim);
    }
    else {
        HAL_TIM_PWM_Stop(htim, channel);     
        HAL_TIM_Base_Stop_IT(htim);          
        setOnOff(false);                     
    }
}

void Stepper::setOnOff(bool isEnabled) {
    GPIO_PinState state = isEnabled ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(pins.onoff_port, pins.onoff_pin, state);
}

void Stepper::changeSpeed(float stepFrequency) {
    if (stepFrequency <= 0 || htim == nullptr) return;

    uint32_t arr_val = (uint32_t)(SYSTEM_TIMER_CLOCK / stepFrequency) - 1;
    if (arr_val > 65535) arr_val = 65535;
    else if (arr_val < 159) arr_val = 159; 

    __HAL_TIM_SET_AUTORELOAD(htim, arr_val);
    __HAL_TIM_SET_COMPARE(htim, channel, (arr_val + 1) / 2); 
}

void Stepper::stepCount() {
    if (currentPosition < nextPosition) {
        currentPosition++;
    } else if (currentPosition > nextPosition) {
        currentPosition--;
    }

    if (currentPosition == nextPosition) {
    	HAL_TIM_PWM_Stop(htim, channel);       
		HAL_TIM_Base_Stop_IT(htim);           
        setOnOff(false);
    }
}

void Stepper::move(int32_t position) {
    nextPosition = position;
    setOnOff(true);
    step();
}

// --- THUẬT TOÁN CHỌN CHIỀU AN TOÀN DỰA VÀO LIMIT ---
void Stepper::moveToAngle(float targetAngle) {
    // 1. Ép góc mục tiêu vào vùng an toàn để không đâm vào giới hạn cứng
    if (targetAngle < minAngleLimit) targetAngle = minAngleLimit;
    if (targetAngle > maxAngleLimit) targetAngle = maxAngleLimit;

    // 2. Tính góc tuyệt đối hiện hành (có thể vượt 360 nếu từng quay nhiều vòng)
    float currentAbsAngle = ((float)currentPosition / (pulsePerRev * gearRatio)) * 360.0f;
    
    // 3. Quy đổi góc hiện tại về hệ quy chiếu [0, 360)
    float currentMod = fmod(currentAbsAngle, 360.0f);
    if (currentMod < 0.0f) currentMod += 360.0f;

    // 4. Vì cả Target và Current đều ép chặt trong vùng [min, max] an toàn,
    // Nên khoảng cách đại số thuần túy sẽ LUÔN LUÔN là đường đi không cắt qua vùng cấm.
    // Ví dụ: Target=0, Current=180 -> SafeDiff = -180 (Bắt buộc phải đi lùi, không được tiến lên 270)
    float safeDiff = targetAngle - currentMod;

    // 5. Cập nhật góc tuyệt đối an toàn và xuất xung
    float newAbsAngle = currentAbsAngle + safeDiff;
    int32_t targetSteps = (int32_t)((newAbsAngle / 360.0f) * pulsePerRev * gearRatio);

    this->move(targetSteps);
}

int32_t Stepper::getCurrentPosition() const {
    return this->currentPosition;
}

void Stepper::onTimerInterrupt() {
    stepCount();
}

void Stepper::setDirection(bool isForward) {
    GPIO_PinState state = isForward ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(pins.direction_port, pins.direction_pin, state);
}

void Stepper::home() {
    setOnOff(true);
    move(-1000000); 
}

int32_t Stepper::distanceToGo() {
    return (nextPosition - currentPosition);
}