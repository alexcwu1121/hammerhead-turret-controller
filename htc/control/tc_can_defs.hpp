#ifndef TC_CAN_DEFS_HPP_
#define TC_CAN_DEFS_HPP_

#include <cstdint>

/// @brief Published CAN ID lowest index
static constexpr uint16_t PubCANIDIdx = 0x300;

/// @brief Published CAN message IDs>
enum PubCANID : uint16_t
{
    PUB_IMU_DATA_ACC_XY = PubCANIDIdx,
    PUB_IMU_DATA_ACC_Z_GYR_X,
    PUB_IMU_DATA_GYR_Y_GYR_Z,
    PUB_HEARTBEAT,
    PUB_PAN_ENC_STATE,        // [0:3] absolute position (rad, float), [4:7] angular velocity (rad/s, float)
    PUB_TILT_ENC_STATE,       // [0:3] absolute position (rad, float), [4:7] angular velocity (rad/s, float)
    PUB_FAULT_INDEX = 0x320,  // starting index for faults
    MAX_PUB_ID
};

/// @brief Susbcribed CAN ID lowest index
static constexpr uint16_t SubCANIDIdx = 0x400;

/// @brief Subscribed CAN message IDs
enum SubCANID : uint16_t
{
    SUB_HOME_PAN = SubCANIDIdx,
    SUB_HOME_TILT,
    SUB_ABORT_HOME_PAN,
    SUB_ABORT_HOME_TILT,
    SUB_SET_MODE_PAN,
    SUB_SET_MODE_TILT,
    SUB_SET_RATE_PAN,
    SUB_SET_RATE_TILT,
    SUB_SET_POS_PAN,
    SUB_SET_POS_TILT,
    SUB_SET_POS_KP_PAN,
    SUB_SET_POS_KI_PAN,
    SUB_SET_POS_KD_PAN,
    SUB_RATE_SLEW_PAN,
    SUB_SET_POS_KP_TILT,
    SUB_SET_POS_KI_TILT,
    SUB_SET_POS_KD_TILT,
    SUB_RATE_SLEW_TILT,
    SUB_ENABLE_TILT_INSTANT_STOP,
    SUB_ENABLE_PAN_INSTANT_STOP,
    SUB_DISABLE_TILT_INSTANT_STOP,
    SUB_DISABLE_PAN_INSTANT_STOP,
    SUB_ENABLE_TILT_STABILIZATION,
    SUB_ENABLE_PAN_STABILIZATION,
    SUB_DISABLE_TILT_STABILIZATION,
    SUB_DISABLE_PAN_STABILIZATION,
    SUB_WRITE_IMU_RESET,
    SUB_WRITE_IMU_COMP,
    SUB_POKE_WATCHDOG,
    SUB_ENABLE_WATCHDOG,
    SUB_DISABLE_WATCHDOG,
    SUB_ENABLE_PAN,
    SUB_ENABLE_TILT,
    SUB_DISABLE_PAN,
    SUB_DISABLE_TILT,
    MAX_SUB_ID
};

#endif
