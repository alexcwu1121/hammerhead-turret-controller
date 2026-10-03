#ifndef A4988_HPP_
#define A4988_HPP_

#include <cstdint>

#include "gpio.h"

namespace a4988
{
/// @brief A4988 stepper motor fault states
enum Fault : uint8_t
{
    NO_FAULT = 0u,
    INVALID_PIN,
    INVALID_RESOLUTION,
    INVALID_DIR,
    INVALID_FREQ,
    FREQ_TOO_LOW,
    HAL_FAULT,
    NUM_FAULTS
};

/// @brief Step resolutions
enum StepResolution : uint8_t
{
    FULL = 0u,
    HALF,
    QUARTER,
    EIGHTH,
    SIXTEENTH
};

/// @brief Step direction
enum StepDir : uint8_t
{
    CW,
    CCW
};

/// @brief A4988 peripheral assignments
struct A4988Peripherals
{
    /// @brief Step pin port
    GPIO_TypeDef& stepPinPort;
    /// @brief Step pin num
    uint16_t stepPinNum = 0u;

    /// @brief MS! pin port
    GPIO_TypeDef& ms1PinPort;
    /// @brief MS! pin num
    uint16_t ms1PinNum = 0u;

    /// @brief MS2 pin port
    GPIO_TypeDef& ms2PinPort;
    /// @brief MS2 pin num
    uint16_t ms2PinNum = 0u;

    /// @brief MS3 pin port
    GPIO_TypeDef& ms3PinPort;
    /// @brief MS3 pin num
    uint16_t ms3PinNum = 0u;

    /// @brief Dir pin port
    GPIO_TypeDef& dirPinPort;
    /// @brief Dir pin num
    uint16_t dirPinNum = 0u;

    /// @brief Reset pin port
    GPIO_TypeDef& resetPinPort;
    /// @brief Step pin num
    uint16_t resetPinNum = 0u;

    /// @brief PWM clock frequency
    uint32_t pwmClockFrequency = 0u;
    /// @brief Step pwm timer handle
    TIM_HandleTypeDef& htim;
    /// @brief Step pwm timer channel
    uint16_t htimCh;
};

/// @brief Simple A4988 stepper motor driver
class A4988
{
public:
    /// @brief A4988 ctor
    /// @param periph
    A4988(A4988Peripherals periph) : _periph(periph) {}

    ~A4988() = default;
    A4988(const A4988&) = delete;
    A4988& operator=(const A4988&) = delete;
    A4988(A4988&&) = delete;
    A4988& operator=(A4988&&) = delete;

    /// @brief Initialize a4988
    /// @return a4988::Fault
    [[nodiscard]] Fault Init() const;

    /// @brief Set step direction
    /// @param dir
    /// @return a4988::Fault
    [[nodiscard]] Fault SetDir(StepDir dir) const;

    /// @brief Set step resolution
    /// @param res
    /// @return a4988::Fault
    [[nodiscard]] Fault SetResolution(StepResolution res) const;

    /// @brief Set step frequency
    /// @param freq
    /// @return a4988::Fault
    [[nodiscard]] Fault SetFrequency(float freq) const;

    /// NOTE: Sleep and enable pins unused in this application
    /// NOTE: Enable and Disable below are slight misnomers. They control RESET to disable motor output stage only

    /// @brief Enable motor driver
    /// @return a4988::Fault
    [[nodiscard]] Fault Enable() const;

    /// @brief Disable motor driver
    /// @return a4988::Fault
    [[nodiscard]] Fault Disable() const;

private:
    /// @brief Minimum step frequency before round down to zero (Hz)
    static constexpr uint32_t _minimumStepFrequency {2u};

    /// @brief A4988 peripheral assignments
    A4988Peripherals _periph;
};
}  // namespace a4988

#endif
