#ifndef BSP_HPP_
#define BSP_HPP_

#include <cstdint>

#include "bsp.hpp"
#include "can.h"
#include "gpio.h"
#include "i2c.h"
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
    TURRET_SUBSYSTEM,
    IMU_SUBSYSTEM,
    CONTROL_SUBSYSTEM,
    NUM_SUBSYSTEMS  // Keep this last
};

/// @brief Public QP signals
enum PublicSignals : QP::QSignal
{
    IMU_SIG = QP::Q_USER_SIG,
    FAULT_SIG,
    REQUEST_FAULT_SIG,
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

}  // namespace bsp

/// @brief System clock configuration
void SystemClock_Config(void);

#endif  // BSP_HPP_
