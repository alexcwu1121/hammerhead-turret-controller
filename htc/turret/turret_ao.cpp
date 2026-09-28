#include "turret_ao.hpp"

#include <cstdlib>

#include "bsp.hpp"
#include "cli_ao.hpp"
#include "gpio.h"
#include "qpcpp.hpp"
#include "tim.h"

namespace turret
{
TurretAO::TurretAO() :
    QP::QActive(&initial),
    _faultRecoveryTimer(this, PrivateSignals::RESET_SIG, 0U),
    _panDriver({.stepPinPort = *PAN_STEP_GPIO_Port,
                .stepPinNum = PAN_STEP_Pin,
                .ms1PinPort = *PAN_MS1_GPIO_Port,
                .ms1PinNum = PAN_MS1_Pin,
                .ms2PinPort = *PAN_MS2_GPIO_Port,
                .ms2PinNum = PAN_MS2_Pin,
                .ms3PinPort = *PAN_MS3_GPIO_Port,
                .ms3PinNum = PAN_MS3_Pin,
                .dirPinPort = *PAN_DIR_GPIO_Port,
                .dirPinNum = PAN_DIR_Pin,
                .resetPinPort = *PAN_RESET_GPIO_Port,
                .resetPinNum = PAN_RESET_Pin,
                .pwmClockFrequency = bsp::APB2_CLOCK_FREQUENCY,
                .htim = htim1,
                .htimCh = TIM_CHANNEL_1}),
    _tiltDriver({.stepPinPort = *TILT_STEP_GPIO_Port,
                 .stepPinNum = TILT_STEP_Pin,
                 .ms1PinPort = *TILT_MS1_GPIO_Port,
                 .ms1PinNum = TILT_MS1_Pin,
                 .ms2PinPort = *TILT_MS2_GPIO_Port,
                 .ms2PinNum = TILT_MS2_Pin,
                 .ms3PinPort = *TILT_MS3_GPIO_Port,
                 .ms3PinNum = TILT_MS3_Pin,
                 .dirPinPort = *TILT_DIR_GPIO_Port,
                 .dirPinNum = TILT_DIR_Pin,
                 .resetPinPort = *TILT_RESET_GPIO_Port,
                 .resetPinNum = TILT_RESET_Pin,
                 .pwmClockFrequency = bsp::APB2_CLOCK_FREQUENCY,
                 .htim = htim17,
                 .htimCh = TIM_CHANNEL_1})
{}

void TurretAO::Start(const QP::QPrioSpec priority, bsp::SubsystemID id)
{
    _id = id;
    _isStarted = true;
    this->start(priority,      // QP prio. of the AO
                _queue,        // event queue storage
                _queueSize,    // queue size [events]
                nullptr, 0U);  // no stack storage
}

void TurretAO::SetFault(bsp::SubsystemID id, uint8_t fault, bool active)
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

Q_STATE_DEF(TurretAO, initial)
{
    Q_UNUSED_PAR(e);
    return tran(&initializing);
}

Q_STATE_DEF(TurretAO, root)
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
        case PrivateSignals::DISABLE_PAN_SIG:
        {
            _panDriver.Disable();
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::DISABLE_TILT_SIG:
        {
            _tiltDriver.Disable();
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

Q_STATE_DEF(TurretAO, initializing)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // Initialize stepper drivers
            a4988::Fault fault;
            fault = _panDriver.Init();
            fault = _tiltDriver.Init();

            if (fault != a4988::Fault::NO_FAULT)
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

Q_STATE_DEF(TurretAO, active)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            status_ = Q_RET_HANDLED;
            break;
        }
        case Q_EXIT_SIG:
        {
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::SET_PAN_RATE_DIRECT_SIG:
        {
            float omega = Q_EVT_CAST(SetRateEvt)->omega;

            /// TODO: make a LUT to transition among microstepping resolutions at lower angular rates

            // compute direction
            a4988::StepDir dir = omega > 0 ? a4988::StepDir::CCW : a4988::StepDir::CW;

            // compute step frequency based on stepper resolution
            float freq = std::abs(omega) * _stepsPerRev / 6.28;

            // set direction and frequency
            _panDriver.SetDir(dir);
            _panDriver.SetFrequency(freq);

            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::SET_TILT_RATE_DIRECT_SIG:
        {
            float omega = Q_EVT_CAST(SetRateEvt)->omega;

            // compute direction
            a4988::StepDir dir = omega > 0 ? a4988::StepDir::CCW : a4988::StepDir::CW;

            // compute step frequency based on stepper resolution
            float freq = std::abs(omega) * _stepsPerRev / 6.28;

            // set direction and frequency
            _tiltDriver.SetDir(dir);
            _tiltDriver.SetFrequency(freq);

            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::ENABLE_PAN_SIG:
        {
            _panDriver.Enable();
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::ENABLE_TILT_SIG:
        {
            _tiltDriver.Enable();
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

Q_STATE_DEF(TurretAO, error)
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
}  // namespace turret
