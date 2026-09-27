#include "imu_ao.hpp"

#include <stdarg.h>

#include "bsp.hpp"
#include "cli_ao.hpp"
#include "gpio.h"

namespace imu
{
IMUAO::IMUAO() :
    QP::QActive(&initial),
    _imu(SPI_CS_GPIO_Port, SPI_CS_Pin, &hspi3),
    _imuTimer(this, PrivateSignals::IMU_SERVICE_SIG, 0U),
    _imuStreamTimer(this, PrivateSignals::IMU_STREAM_SIG, 0U),
    _faultRecoveryTimer(this, PrivateSignals::RESET_SIG, 0U)
{}

void IMUAO::Start(const QP::QPrioSpec priority, bsp::SubsystemID id)
{
    _id = id;
    _isStarted = true;
    this->start(priority,      // QP prio. of the AO
                _queue,        // event queue storage
                _queueSize,    // queue size [events]
                nullptr, 0U);  // no stack storage
}

void IMUAO::SetFault(bsp::SubsystemID id, uint8_t fault, bool active)
{
    if (_faultStates[fault] != active)
    {
        // Update internal fault state
        _faultStates[fault] = active;
        // Publish fault update
        bsp::FaultEvt* evt = Q_NEW(bsp::FaultEvt, bsp::PublicSignals::FAULT_SIG);
        evt->id = _id;
        evt->fault = fault;
        evt->active = active;
        PUBLISH(evt, this);
    }
}

Q_STATE_DEF(IMUAO, initial)
{
    Q_UNUSED_PAR(e);
    return tran(&initializing);
}

Q_STATE_DEF(IMUAO, root)
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
        default:
        {
            status_ = super(&top);
            break;
        }
    }
    return status_;
}

Q_STATE_DEF(IMUAO, initializing)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // Verify IMU device ID and enable SPI peripheral
            imu::Fault fault = _imu.Initialize();

            // Set accelerometer ODR
            fault = _imu.SetAccODR(imu::BMI270::AccODR::ODR_800);

            // Set accelerometer range
            fault = _imu.SetAccRange(imu::BMI270::AccRange::G_4);

            // Set gyroscope ODR
            fault = _imu.SetGyrODR(imu::BMI270::GyrODR::ODR_1K6);

            // Set gyroscope range
            fault = _imu.SetGyrRange(imu::BMI270::GyrRange::DPS_500);

            // Initialize
            if (fault != imu::Fault::NO_FAULT)
            {
                // Update fault status
                SetFault(_id, fault, true);

                static QP::QEvt evt(PrivateSignals::FAULT_SIG);
                POST(&evt, this);
            }
            else
            {
                static QP::QEvt evt(PrivateSignals::INITIALIZED_SIG);
                POST(&evt, this);
            }

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

Q_STATE_DEF(IMUAO, active)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // Arm imu polling timer
            _imuTimer.armX(_imuTimerInterval, _imuTimerInterval);
            status_ = Q_RET_HANDLED;
            break;
        }
        case Q_EXIT_SIG:
        {
            // Disarm imu polling timer
            _imuTimer.disarm();
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::IMU_SERVICE_SIG:
        {
            imu::Fault fault = _imu.ReadData(_imuData);
            if (!fault)
            {
                // Publish IMU data
                IMUEvt* evt = Q_NEW(IMUEvt, bsp::PublicSignals::IMU_SIG);
                evt->data = _imuData;
                PUBLISH(evt, nullptr);
            }
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::RUN_IMU_COMPENSATION_SIG:
        {
            // Disable IMU polling timer during compensation
            _imuTimer.disarm();
            // Run compensation
            imu::Fault fault = _imu.RunCompensation();
            // Reenable IMU polling timer
            _imuTimer.armX(_imuTimerInterval, _imuTimerInterval);

            if (fault != imu::Fault::NO_FAULT)
            {
                cli::CLIAO::Inst().Printf("ERROR: Fault during IMU compensation (%hhu)", fault);
            }
            else { cli::CLIAO::Inst().Printf("INFO: IMU compensation finished"); }
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::START_IMU_STREAM:
        {
            // Arm imu stream timer
            _imuStreamTimer.armX(_imuStreamTimerInterval, _imuStreamTimerInterval);
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::STOP_IMU_STREAM:
        {
            // Disarm imu stream timer
            _imuStreamTimer.disarm();
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::IMU_STREAM_SIG:
        {
            // Print IMU data
            cli::CLIAO::Inst().Printf(
                ">>>>>>>>>>>>>>\n\r"
                "Acc X: %+7.2f g\n\r"
                "Acc Y: %+7.2f g\n\r"
                "Acc Z: %+7.2f g\n\r"
                "Gyr X: %+7.2f dps\n\r"
                "Gyr Y: %+7.2f dps\n\r"
                "Gyr Z: %+7.2f dps\n\r"
                ">>>>>>>>>>>>>>\n\r",
                _imuData.acc[0], _imuData.acc[1], _imuData.acc[2], _imuData.gyr[0], _imuData.gyr[1], _imuData.gyr[2]);
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

Q_STATE_DEF(IMUAO, error)
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

            // Clear all faults on exit
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
}  // namespace imu
