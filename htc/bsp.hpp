#ifndef BSP_HPP_
#define BSP_HPP_

#include <cstdint>

#include "bsp.hpp"
#include "can.h"
#include "gpio.h"
#include "qpcpp.hpp"
#include "spi.h"
#include "stm32f3xx_it.h"
#include "tim.h"
#include "usart.h"

namespace bsp
{
/// @brief Number of ticks per second
constexpr std::uint32_t TICKS_PER_SEC {1000U};
/// @brief APB2 block frequency
constexpr std::uint32_t APB2_CLOCK_FREQUENCY {48000000U};

/// @brief Subsystem IDs
enum SubsystemID : uint8_t
{
    CLI_SUBSYSTEM = 0U,
    PAN_STEPPER_SUBSYSTEM,
    TILT_STEPPER_SUBSYSTEM,
    PAN_HOME_SUBSYSTEM,
    TILT_HOME_SUBSYSTEM,
    IMU_SUBSYSTEM,
    CONTROL_SUBSYSTEM,
    NUM_SUBSYSTEMS  // Keep this last
};

/// @brief Subsystem ID to string table
/// @param id
/// @return
constexpr const char* SubsystemIDToStr(SubsystemID id)
{
    switch (id)
    {
        case SubsystemID::CLI_SUBSYSTEM:
        {
            return "CLI_SUBSYSTEM";
        }
        case SubsystemID::PAN_STEPPER_SUBSYSTEM:
        {
            return "PAN_STEPPER_SUBSYSTEM";
        }
        case SubsystemID::TILT_STEPPER_SUBSYSTEM:
        {
            return "TILT_STEPPER_SUBSYSTEM";
        }
        case SubsystemID::IMU_SUBSYSTEM:
        {
            return "IMU_SUBSYSTEM";
        }
        case SubsystemID::CONTROL_SUBSYSTEM:
        {
            return "CONTROL_SUBSYSTEM";
        }
        default:
        {
            return "";
        }
    }
}

/// @brief Public QP signals
enum PublicSignals : QP::QSignal
{
    IMU_SIG = QP::Q_USER_SIG,
    FAULT_SIG,
    REQUEST_FAULT_SIG,
    STEPPER_MODE_CHANGED_SIG,
    STEPPER_ENC_STATE_SIG,
    MAX_PUB_SIG  // Keep this last
};

/// @brief Maximum number of faults each subsystem may implement
constexpr uint8_t MAX_SUBSYSTEM_FAULTS = 16U;

/// @brief Fault event
class FaultEvt : public QP::QEvt
{
public:
    /// @brief Originating subsystem
    SubsystemID id;
    /// @brief Fault code
    uint8_t fault;
    /// @brief Fault active/inactive
    bool active;
};

/// @brief Stepper controller mode changed event
class StepperModeChangedEvt : public QP::QEvt
{
public:
    /// @brief Originating subsystem
    SubsystemID id;
    /// @brief Mode
    uint8_t mode;
};

/// @brief Stepper encoder state event
class StepperEncStateEvt : public QP::QEvt
{
public:
    /// @brief Originating subsystem
    SubsystemID id;
    /// @brief Absolute position in rad
    float absPos;
    /// @brief Angular rate in rad/s
    float rate;
};

}  // namespace bsp

/// @brief System clock configuration
void SystemClock_Config(void);

#endif  // BSP_HPP_
