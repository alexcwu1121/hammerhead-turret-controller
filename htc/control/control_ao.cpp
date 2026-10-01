#include "control_ao.hpp"

#include <cstdarg>
#include <cstring>

#include "bsp.hpp"
#include "can.h"
#include "cli_ao.hpp"
#include "gpio.h"
#include "imu_ao.hpp"
#include "stepper_ao.hpp"
#include "thirdparty/printf.h"

/// @brief Published CAN ID lowest index
static constexpr uint16_t PubCANIDIdx = 0x300;

/// @brief Published CAN message IDs
enum PubCANID : uint16_t
{
    PUB_IMU_DATA_ACC_XY = PubCANIDIdx,
    PUB_IMU_DATA_ACC_Z_GYR_X,
    PUB_IMU_DATA_GYR_Y_GYR_Z,
    MAX_PUB_ID
};

/// @brief Susbcribed CAN ID lowest index
static constexpr uint16_t SubCANIDIdx = 0x400;

/// @brief Subscribed CAN message IDs
enum SubCANID : uint16_t
{
    SUB_HOME_PAN = SubCANIDIdx,
    SUB_HOME_TILT,
    SUB_SET_RATE_PAN,
    SUB_SET_RATE_TILT,
    SUB_SET_POS_PAN,
    SUB_SET_POS_TILT,
    SUB_WRITE_IMU_RESET,
    SUB_WRITE_IMU_COMP,
    MAX_SUB_ID
};

/// @brief CAN receive fifo message callback
/// @param hcan
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    // Retrieve the pending message
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) == HAL_OK)
    {
        /// TODO: Only standard IDs supported
        if (header.IDE == CAN_ID_STD)
        {
            // Inject events
            switch (header.StdId)
            {
                case SubCANID::SUB_SET_RATE_PAN:
                {
                    if (header.DLC == 4)
                    {
                        float rate = 0.0f;
                        memcpy(&rate, data, sizeof(float));
                        stepper::StepperAO::PanInst().SetRateSetpoint(rate);
                    }
                    break;
                }
                case SubCANID::SUB_SET_RATE_TILT:
                {
                    if (header.DLC == 4)
                    {
                        float rate = 0.0f;
                        memcpy(&rate, data, sizeof(float));
                        stepper::StepperAO::TiltInst().SetRateSetpoint(rate);
                    }
                    break;
                }
                case SubCANID::SUB_WRITE_IMU_RESET:
                {
                    imu::IMUAO::Inst().Reset();
                    break;
                }
                case SubCANID::SUB_WRITE_IMU_COMP:
                {
                    imu::IMUAO::Inst().RunIMUCompensation();
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
    }
}

namespace control
{
ControlAO::ControlAO() :
    QP::QActive(&initial),
    _faultRecoveryTimer(this, PrivateSignals::RESET_SIG, 0U),
    _faultRequestTimer(this, PrivateSignals::SUBS_FAULT_REQUEST_SIG, 0U)
{}

void ControlAO::Start(const QP::QPrioSpec priority, bsp::SubsystemID id)
{
    _id = id;
    _isStarted = true;
    this->start(priority,      // QP prio. of the AO
                _queue,        // event queue storage
                _queueSize,    // queue size [events]
                nullptr, 0U);  // no stack storage
}

void ControlAO::SetFault(bsp::SubsystemID id, uint8_t fault, bool active)
{
    if (fault > bsp::MAX_SUBSYSTEM_FAULTS)
    {
        // Fault code can't exceed max faults
        cli::CLIAO::Inst().Printf("ERROR: Fault code out of range");
        return;
    }

    if (_faultStates[id][fault] != active)
    {
        // Update internal fault state
        _faultStates[id][fault] = active;
        if (active)
        {
            // Enable fault LED if fault becomes active
            HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET);
        }
        else
        {
            // Check for presence of any fault
            bool is_fault = false;
            for (uint8_t subsystem = 0; subsystem < bsp::SubsystemID::NUM_SUBSYSTEMS; subsystem++)
            {
                for (uint8_t fault = 0; fault < bsp::MAX_SUBSYSTEM_FAULTS; fault++)
                {
                    if (_faultStates[subsystem][fault] != 0)
                    {
                        is_fault = true;
                        break;
                    }
                }
            }
            if (is_fault) { HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET); }
            else { HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET); }
        }
    }
}

Q_STATE_DEF(ControlAO, initial)
{
    Q_UNUSED_PAR(e);
    subscribe(bsp::PublicSignals::FAULT_SIG);
    subscribe(bsp::PublicSignals::IMU_SIG);

    return tran(&initializing);
}

Q_STATE_DEF(ControlAO, root)
{
    QP::QState status_;
    switch (e->sig)
    {
        case PrivateSignals::RESET_SIG:
        {
            status_ = tran(&initializing);
            break;
        }
        case PrivateSignals::FAULT_SIG:
        {
            status_ = tran(&error);
            break;
        }
        case bsp::PublicSignals::FAULT_SIG:
        {
            // Update fault status
            SetFault(Q_EVT_CAST(bsp::FaultEvt)->id, Q_EVT_CAST(bsp::FaultEvt)->fault,
                     Q_EVT_CAST(bsp::FaultEvt)->active);
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::PRINT_FAULT_SIG:
        {
            static const char* fmt = "\t%s: %d\n\r";
            char buf[cli::CLIAO::cliPrintBufSize] = {0};

            // Print all control fault statuses
            static const char* control = "Control Subsystem\n\r";
            memset(buf, 0U, sizeof(buf));
            char* ptr = (char*)memcpy(buf, control, strlen(control));
            ptr += strlen(control);
            for (uint8_t fault = 0; fault < control::Fault::NUM_FAULTS; fault++)
            {
                bool state = _faultStates[bsp::SubsystemID::CONTROL_SUBSYSTEM][fault];

                ptr += snprintf(ptr, buf + cli::CLIAO::cliPrintBufSize - ptr, fmt,
                                control::FaultToStr((control::Fault)fault), state);
            }
            cli::CLIAO::Inst().Printf(buf);

            // Print all pan stepper fault statuses
            static const char* pan_stepper = "Pan Stepper Subsystem\n\r";
            memset(buf, 0U, sizeof(buf));
            ptr = (char*)memcpy(buf, pan_stepper, strlen(pan_stepper));
            ptr += strlen(pan_stepper);
            for (uint8_t fault = 0; fault < stepper::Fault::NUM_FAULTS; fault++)
            {
                bool state = _faultStates[bsp::SubsystemID::PAN_STEPPER_SUBSYSTEM][fault];
                ptr += snprintf(ptr, buf + cli::CLIAO::cliPrintBufSize - ptr, fmt,
                                stepper::FaultToStr((stepper::Fault)fault), state);
            }
            cli::CLIAO::Inst().Printf(buf);

            // Print all tilt stepper fault statuses
            static const char* tilt_stepper = "Tilt Stepper Subsystem\n\r";
            memset(buf, 0U, sizeof(buf));
            ptr = (char*)memcpy(buf, tilt_stepper, strlen(tilt_stepper));
            ptr += strlen(tilt_stepper);
            for (uint8_t fault = 0; fault < stepper::Fault::NUM_FAULTS; fault++)
            {
                bool state = _faultStates[bsp::SubsystemID::TILT_STEPPER_SUBSYSTEM][fault];
                ptr += snprintf(ptr, buf + cli::CLIAO::cliPrintBufSize - ptr, fmt,
                                stepper::FaultToStr((stepper::Fault)fault), state);
            }
            cli::CLIAO::Inst().Printf(buf);

            // Print all imu fault statuses
            static const char* imu = "IMU Subsystem\n\r";
            memset(buf, 0U, sizeof(buf));
            ptr = (char*)memcpy(buf, imu, strlen(imu));
            ptr += strlen(imu);
            for (uint8_t fault = 0; fault < imu::Fault::NUM_FAULTS; fault++)
            {
                bool state = _faultStates[bsp::SubsystemID::IMU_SUBSYSTEM][fault];
                ptr += snprintf(ptr, buf + cli::CLIAO::cliPrintBufSize - ptr, fmt, imu::FaultToStr((imu::Fault)fault),
                                state);
            }
            cli::CLIAO::Inst().Printf(buf);

            // Print all cli fault statuses
            static const char* cli = "CLI Subsystem\n\r";
            memset(buf, 0U, sizeof(buf));
            ptr = (char*)memcpy(buf, cli, strlen(cli));
            ptr += strlen(cli);
            for (uint8_t fault = 0; fault < cli::Fault::NUM_FAULTS; fault++)
            {
                bool state = _faultStates[bsp::SubsystemID::CLI_SUBSYSTEM][fault];
                ptr += snprintf(ptr, buf + cli::CLIAO::cliPrintBufSize - ptr, fmt, cli::FaultToStr((cli::Fault)fault),
                                state);
            }
            cli::CLIAO::Inst().Printf(buf);

            status_ = Q_RET_HANDLED;
            break;
        }
        case bsp::PublicSignals::IMU_SIG:
        {
            _canTxHeader.ExtId = 0x00;
            _canTxHeader.IDE = CAN_ID_STD;
            _canTxHeader.RTR = CAN_RTR_DATA;
            _canTxHeader.DLC = 8;
            _canTxHeader.TransmitGlobalTime = DISABLE;

            // Fragment IMU data into 8 byte chunks and send

            // Accelerometer x and y
            _canTxHeader.StdId = PubCANID::PUB_IMU_DATA_ACC_XY;
            memcpy(_canTxData, &Q_EVT_CAST(imu::IMUEvt)->data.acc[0], sizeof(float));
            memcpy(_canTxData + 4, &Q_EVT_CAST(imu::IMUEvt)->data.acc[1], sizeof(float));
            if (HAL_CAN_AddTxMessage(&hcan, &_canTxHeader, _canTxData, &_canTxMailbox) != HAL_OK)
            {
                SetFault(bsp::SubsystemID::CONTROL_SUBSYSTEM, Fault::CONTROL_CAN_TX_FAILED, true);
                status_ = Q_RET_HANDLED;
                break;
            }

            // Accelerometer z and gyro x
            _canTxHeader.StdId = PubCANID::PUB_IMU_DATA_ACC_Z_GYR_X;
            memcpy(_canTxData, &Q_EVT_CAST(imu::IMUEvt)->data.acc[2], sizeof(float));
            memcpy(_canTxData + 4, &Q_EVT_CAST(imu::IMUEvt)->data.gyr[0], sizeof(float));
            if (HAL_CAN_AddTxMessage(&hcan, &_canTxHeader, _canTxData, &_canTxMailbox) != HAL_OK)
            {
                SetFault(bsp::SubsystemID::CONTROL_SUBSYSTEM, Fault::CONTROL_CAN_TX_FAILED, true);
                status_ = Q_RET_HANDLED;
                break;
            }

            // Gyro y and z
            _canTxHeader.StdId = PubCANID::PUB_IMU_DATA_GYR_Y_GYR_Z;
            memcpy(_canTxData, &Q_EVT_CAST(imu::IMUEvt)->data.gyr[1], sizeof(float));
            memcpy(_canTxData + 4, &Q_EVT_CAST(imu::IMUEvt)->data.gyr[2], sizeof(float));
            if (HAL_CAN_AddTxMessage(&hcan, &_canTxHeader, _canTxData, &_canTxMailbox) != HAL_OK)
            {
                SetFault(bsp::SubsystemID::CONTROL_SUBSYSTEM, Fault::CONTROL_CAN_TX_FAILED, true);
                status_ = Q_RET_HANDLED;
                break;
            }

            // Automatically clear fault state on successful transmission
            SetFault(bsp::SubsystemID::CONTROL_SUBSYSTEM, Fault::CONTROL_CAN_TX_FAILED, false);

            status_ = Q_RET_HANDLED;
            break;
        }
        default:
        {
            status_ = super(&top);
            break;
        }
    }
    return status_;
}

Q_STATE_DEF(ControlAO, initializing)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // Set up CAN filters
            CAN_FilterTypeDef can_filter;
            can_filter.FilterBank = 0;
            can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
            can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
            can_filter.FilterIdHigh = (SubCANIDIdx << 5U) & 0xFFFF;
            can_filter.FilterIdLow = 0x0000;
            can_filter.FilterMaskIdHigh = SubCANIDIdx << 5U;
            can_filter.FilterMaskIdLow = 0x0000;
            can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
            can_filter.FilterActivation = ENABLE;

            if (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK)
            {
                ControlAO::SetFault(bsp::SubsystemID::CONTROL_SUBSYSTEM, Fault::CONTROL_INIT_FAILED, true);
                status_ = tran(&error);
                break;
            }

            // Start CAN peripheral
            if (HAL_CAN_Start(&hcan) != HAL_OK)
            {
                ControlAO::SetFault(bsp::SubsystemID::CONTROL_SUBSYSTEM, Fault::CONTROL_INIT_FAILED, true);
                status_ = tran(&error);
                break;
            }

            // Enable recieve interrupt
            if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
            {
                ControlAO::SetFault(bsp::SubsystemID::CONTROL_SUBSYSTEM, Fault::CONTROL_INIT_FAILED, true);
                status_ = tran(&error);
                break;
            }

            // Finish initialization
            static QP::QEvt evt(PrivateSignals::INITIALIZED_SIG);
            POST(&evt, this);

            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::INITIALIZED_SIG:
        {
            status_ = tran(&active);
            break;
        }
        default:
        {
            status_ = super(&root);
            break;
        }
    }
    return status_;
}

Q_STATE_DEF(ControlAO, active)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // Arm subsystem fault heartbeat timer
            _faultRequestTimer.armX(_faultRequestTimerInterval, _faultRequestTimerInterval);

            status_ = Q_RET_HANDLED;
            break;
        }
        case Q_EXIT_SIG:
        {
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::SUBS_FAULT_REQUEST_SIG:
        {
            QP::QEvt* evt = Q_NEW(QP::QEvt, bsp::PublicSignals::REQUEST_FAULT_SIG);
            PUBLISH(evt, this);
            status_ = Q_RET_HANDLED;
            break;
        }
        default:
        {
            status_ = super(&root);
            break;
        }
    }
    return status_;
}

Q_STATE_DEF(ControlAO, error)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // Start attempting fault recovery
            _faultRecoveryTimer.armX(_faultRecoveryTimerInterval, _faultRecoveryTimerInterval);
            status_ = Q_RET_HANDLED;
            break;
        }
        case Q_EXIT_SIG:
        {
            // Disable fault recovery
            _faultRecoveryTimer.disarm();

            // Clear all INTERNAL faults on exit
            for (uint8_t fault = 0U; fault < Fault::NUM_FAULTS; fault++) { SetFault(_id, fault, false); }

            status_ = Q_RET_HANDLED;
            break;
        }
        default:
        {
            status_ = super(&root);
            break;
        }
    }
    return status_;
}
}  // namespace control
