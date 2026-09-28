#include "a4988.hpp"

namespace a4988
{
const Fault A4988::Init()
{
    // Enable motor controller
    auto fault = Enable();
    if (fault != Fault::NO_FAULT) { return fault; }

    // Default to clockwise direction
    fault = SetDir(StepDir::CW);
    if (fault != Fault::NO_FAULT) { return fault; }

    // Default to full step
    fault = SetResolution(StepResolution::FULL);
    if (fault != Fault::NO_FAULT) { return fault; }

    // This driver assumes minimum step frequency of 1 Hz before it rounds down to zero
    // Set clock prescaler
    __HAL_TIM_SET_PRESCALER(&_periph.htim, _periph.pwmClockFrequency / (_minimumStepFrequency * UINT16_MAX));

    // Initialize step frequency to 0
    fault = SetFrequency(0.0f);
    if (fault != Fault::NO_FAULT) { return fault; }

    return fault;
}

const Fault A4988::SetDir(StepDir dir)
{
    auto fault = Fault::NO_FAULT;
    switch (dir)
    {
        case StepDir::CW:
        {
            HAL_GPIO_WritePin(&_periph.dirPinPort, _periph.dirPinNum, GPIO_PIN_SET);
            break;
        }
        case StepDir::CCW:
        {
            HAL_GPIO_WritePin(&_periph.dirPinPort, _periph.dirPinNum, GPIO_PIN_RESET);
            break;
        }
        default:
        {
            fault = Fault::INVALID_DIR;
            break;
        }
    }
    return fault;
}

const Fault A4988::SetResolution(StepResolution res)
{
    auto fault = Fault::NO_FAULT;
    switch (res)
    {
        case StepResolution::FULL:
        {
            HAL_GPIO_WritePin(&_periph.ms1PinPort, _periph.ms1PinNum, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(&_periph.ms2PinPort, _periph.ms2PinNum, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(&_periph.ms3PinPort, _periph.ms3PinNum, GPIO_PIN_RESET);
            break;
        }
        case StepResolution::HALF:
        {
            HAL_GPIO_WritePin(&_periph.ms1PinPort, _periph.ms1PinNum, GPIO_PIN_SET);
            HAL_GPIO_WritePin(&_periph.ms2PinPort, _periph.ms2PinNum, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(&_periph.ms3PinPort, _periph.ms3PinNum, GPIO_PIN_RESET);
            break;
        }
        case StepResolution::QUARTER:
        {
            HAL_GPIO_WritePin(&_periph.ms1PinPort, _periph.ms1PinNum, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(&_periph.ms2PinPort, _periph.ms2PinNum, GPIO_PIN_SET);
            HAL_GPIO_WritePin(&_periph.ms3PinPort, _periph.ms3PinNum, GPIO_PIN_RESET);
            break;
        }
        case StepResolution::EIGHTH:
        {
            HAL_GPIO_WritePin(&_periph.ms1PinPort, _periph.ms1PinNum, GPIO_PIN_SET);
            HAL_GPIO_WritePin(&_periph.ms2PinPort, _periph.ms2PinNum, GPIO_PIN_SET);
            HAL_GPIO_WritePin(&_periph.ms3PinPort, _periph.ms3PinNum, GPIO_PIN_RESET);
            break;
        }
        case StepResolution::SIXTEENTH:
        {
            HAL_GPIO_WritePin(&_periph.ms1PinPort, _periph.ms1PinNum, GPIO_PIN_SET);
            HAL_GPIO_WritePin(&_periph.ms2PinPort, _periph.ms2PinNum, GPIO_PIN_SET);
            HAL_GPIO_WritePin(&_periph.ms3PinPort, _periph.ms3PinNum, GPIO_PIN_SET);
            break;
        }
        default:
        {
            fault = Fault::INVALID_RESOLUTION;
            break;
        }
    }
    return fault;
}

const Fault A4988::SetFrequency(float freq)
{
    if (freq < 0) { return Fault::INVALID_FREQ; }
    else if (freq == 0)
    {
        // 0 is actually valid. We'll just disable pwm generation
        auto fault = Fault::NO_FAULT;
        if (HAL_TIM_PWM_Stop(&_periph.htim, _periph.htimCh) != HAL_OK) { fault = Fault::HAL_FAULT; }
        return fault;
    }

    // Enable PWM generation if it wasn't already
    if (HAL_TIM_PWM_Start(&_periph.htim, _periph.htimCh) != HAL_OK) { return Fault::HAL_FAULT; }

    // Compute reload register max value from frequency
    uint32_t arr = _periph.pwmClockFrequency / (freq * (_periph.htim.Instance->PSC));

    /// TODO: we can actually adjust the prescaler dynamically here instead of erroring
    if (arr > UINT16_MAX) { return Fault::FREQ_TOO_HIGH; }

    // Set new autoreload
    __HAL_TIM_SET_AUTORELOAD(&_periph.htim, arr);

    // Keep pwm signal at 50% duty cycle
    __HAL_TIM_SET_COMPARE(&_periph.htim, _periph.htimCh, arr / 2);

    return Fault::NO_FAULT;
}

const Fault A4988::Enable()
{
    HAL_GPIO_WritePin(&_periph.resetPinPort, _periph.resetPinNum, GPIO_PIN_SET);
    return Fault::NO_FAULT;
}

const Fault A4988::Disable()
{
    HAL_GPIO_WritePin(&_periph.resetPinPort, _periph.resetPinNum, GPIO_PIN_RESET);
    return Fault::NO_FAULT;
}
}  // namespace a4988
