#ifndef BSP_HPP_
#define BSP_HPP_

#include <cstdint>

namespace bsp
{
/// @brief Number of ticks per second
constexpr std::uint32_t TICKS_PER_SEC{1000U};

/// @brief Set pan stepping 

/// @brief Set tilt stepping period

}  // namespace bsp

/// @brief System clock configuration
void SystemClock_Config(void);

#endif  // BSP_HPP_
