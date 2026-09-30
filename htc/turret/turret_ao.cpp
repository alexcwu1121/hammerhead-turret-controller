#include "turret_ao.hpp"

#include <algorithm>
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
    _encoderTimer(this, PrivateSignals::POLL_ENCODER_SIG, 0U),
    _encoderStreamTimer(this, PrivateSignals::ENCODER_STREAM_SIG, 0U),
    _clTimer(this, PrivateSignals::CL_UPDATE_SIG, 0U),
    _steppers {a4988::A4988({.stepPinPort = *PAN_STEP_GPIO_Port,
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
               a4988::A4988({.stepPinPort = *TILT_STEP_GPIO_Port,
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
                             .htimCh = TIM_CHANNEL_1})}
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

void TurretAO::GetFreqResolutionForRate(float omega, float& freq, a4988::StepResolution& resolution)
{
    // compute full step frequency based on stepper resolution
    freq = _gearRatio * std::abs(omega) * _stepsPerRev / 6.28;

    /// TODO: For now, only use eighth microstepping
    resolution = a4988::StepResolution::EIGHTH;
    freq *= 8;  // NOLINT
}

Fault TurretAO::SetPWMFromRate(StepperID stepper, float omega)
{
    // compute direction
    a4988::StepDir dir = omega > 0 ? a4988::StepDir::CCW : a4988::StepDir::CW;

    // compute step frequency and step resolution
    float freq = 0.0f;
    a4988::StepResolution resolution = a4988::StepResolution::FULL;
    GetFreqResolutionForRate(omega, freq, resolution);

    // set direction and frequency
    a4988::Fault fault = _steppers[stepper].SetDir(dir);
    if (fault != a4988::Fault::NO_FAULT) { return Fault::SET_DIR_FAILED; }

    fault = _steppers[stepper].SetResolution(resolution);
    if (fault != a4988::Fault::NO_FAULT) { return Fault::SET_RES_FAILED; }

    fault = _steppers[stepper].SetFrequency(freq);
    if (fault != a4988::Fault::NO_FAULT) { return Fault::SET_FREQ_FAILED; }
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
        case PrivateSignals::DISABLE_SIG:
        {
            _steppers[Q_EVT_CAST(StepperControlEvt)->stepper].Disable();
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
            for (const auto& stepper : _steppers) { fault = stepper.Init(); }
            if (fault != a4988::Fault::NO_FAULT)
            {
                // Update fault status
                SetFault(_id, Fault::STEPPER_INIT_FAILED, true);

                // Queue up fault state transition
                static QP::QEvt evt(PrivateSignals::FAULT_SIG);
                POST(&evt, this);

                status_ = Q_RET_HANDLED;
                break;
            }

            // Initialize encoders
            HAL_StatusTypeDef pan_enc_fault = HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
            HAL_StatusTypeDef tilt_enc_fault = HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
            if (pan_enc_fault != HAL_OK || tilt_enc_fault != HAL_OK)
            {
                // Update fault status
                SetFault(_id, Fault::ENCODER_INIT_FAILED, true);

                // Queue up fault state transition
                static QP::QEvt evt(PrivateSignals::FAULT_SIG);
                POST(&evt, this);

                status_ = Q_RET_HANDLED;
                break;
            }

            // Queue up active state transition
            static QP::QEvt evt(PrivateSignals::INITIALIZED_SIG);
            POST(&evt, this);

            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::INITIALIZED_SIG:
        {
            status_ = tran(&active_cl);
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
            _encoderTimer.armX(_encoderTimerInterval, _encoderTimerInterval);

            // set initial encoder positions
            _lastPanEnc = __HAL_TIM_GET_COUNTER(&htim2);
            _lastTiltEnc = __HAL_TIM_GET_COUNTER(&htim3);

            status_ = Q_RET_HANDLED;
            break;
        }
        case Q_EXIT_SIG:
        {
            _encoderTimer.disarm();
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::SET_MODE_SIG:
        {
            if (Q_EVT_CAST(SetModeEvt)->mode == Mode::OPEN_LOOP) { status_ = tran(&active_ol); }
            else if (Q_EVT_CAST(SetModeEvt)->mode == Mode::CLOSED_LOOP) { status_ = tran(&active_cl); }
            else { status_ = Q_RET_HANDLED; }
            break;
        }
        case PrivateSignals::ENABLE_SIG:
        {
            _steppers[Q_EVT_CAST(StepperControlEvt)->stepper].Enable();
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::POLL_ENCODER_SIG:
        {
            uint16_t pan_enc = __HAL_TIM_GET_COUNTER(&htim2);
            uint16_t tilt_enc = __HAL_TIM_GET_COUNTER(&htim3);

            // compute delta in encoder positions since last sample
            int16_t pan_enc_delta = static_cast<int16_t>(pan_enc - _lastPanEnc);
            int16_t tilt_enc_delta = static_cast<int16_t>(tilt_enc - _lastTiltEnc);

            // IIR filter
            auto iir = [](float& prior, const float& obs) { prior += _iirAlpha * (obs - prior); };

            // convert to rad/s and iir filter
            iir(_lastPanRate,
                static_cast<float>(_encoderTimerFreq) * pan_enc_delta * 6.28f / (_encoderCPR * _gearRatio));
            iir(_lastTiltRate,
                static_cast<float>(_encoderTimerFreq) * tilt_enc_delta * 6.28f / (_encoderCPR * _gearRatio));

            // update last encoder values
            _lastPanEnc = pan_enc;
            _lastTiltEnc = tilt_enc;

            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::START_ENCODER_STREAM_SIG:
        {
            _encoderStreamTimer.armX(_encoderStreamTimerInterval, _encoderStreamTimerInterval);
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::STOP_ENCODER_STREAM_SIG:
        {
            _encoderStreamTimer.disarm();
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::ENCODER_STREAM_SIG:
        {
            // Print encoder position and rate data
            cli::CLIAO::Inst().Printf(
                ">>>>>>>>>>>>>>\n\r"
                "Pan Enc Rate: %+7.4f rad/s\n\r"
                "Tilt Enc Rate: %+7.4f rad/s\n\r"
                ">>>>>>>>>>>>>>\n\r",
                _lastPanRate, _lastTiltRate);
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

Q_STATE_DEF(TurretAO, active_ol)
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
        case PrivateSignals::SET_RATE_DIRECT_SIG:
        {
            // Set PWM waveform
            float omega_clamped = std::clamp(Q_EVT_CAST(SetRateEvt)->omega, -_maxRate, _maxRate);
            Fault fault = SetPWMFromRate(Q_EVT_CAST(SetRateEvt)->stepper, omega_clamped);

            if (fault != NO_FAULT)
            {
                // Update fault status
                SetFault(_id, fault, true);

                // Just indicate a fault in this case

                status_ = Q_RET_HANDLED;
                break;
            }

            status_ = Q_RET_HANDLED;
            break;
        }
        default:
        {
            status_ = super(&active);
            break;
        }
    }
    return status_;
}

Q_STATE_DEF(TurretAO, active_cl)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            _clTimer.armX(_clTimerInterval, _clTimerInterval);

            // initialize rate setpoints to current angular velocities
            _panRateSetpoint = _lastPanRate;
            _tiltRateSetpoint = _lastTiltRate;

            // initialize rate commands to current angular velocities
            // this assumes open loop angular velocity commands are close to reality, which is pretty much true
            _panRateCommand = _lastPanRate;
            _tiltRateCommand = _lastTiltRate;

            status_ = Q_RET_HANDLED;
            break;
        }
        case Q_EXIT_SIG:
            SetPWMFromRate(Q_EVT_CAST(SetRateEvt)->stepper, Q_EVT_CAST(SetRateEvt)->omega);
            {
                _clTimer.disarm();
                status_ = Q_RET_HANDLED;
                break;
            }
        case PrivateSignals::SET_RATE_SIG:
        {
            float omega_clamped = std::clamp(Q_EVT_CAST(SetRateEvt)->omega, -_maxRate, _maxRate);

            if (Q_EVT_CAST(SetRateEvt)->stepper == StepperID::PAN) { _panRateSetpoint = omega_clamped; }
            else if (Q_EVT_CAST(SetRateEvt)->stepper == StepperID::TILT) { _tiltRateSetpoint = omega_clamped; }

            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::CL_UPDATE_SIG:
        {
            float pan_rate_error = _panRateSetpoint - _lastPanRate;
            float pan_correction_mag = std::min(std::abs(pan_rate_error), _clSlewRate);
            if (pan_rate_error > 0) { _panRateCommand += pan_correction_mag; }
            else if (pan_rate_error < 0) { _panRateCommand -= pan_correction_mag; }

            float tilt_rate_error = _tiltRateSetpoint - _lastTiltRate;
            float tilt_correction_mag = std::min(std::abs(tilt_rate_error), _clSlewRate);
            if (tilt_rate_error > 0) { _tiltRateCommand += tilt_correction_mag; }
            else if (tilt_rate_error < 0) { _tiltRateCommand -= pan_correction_mag; }

            // Set pan PWM waveform
            Fault fault = SetPWMFromRate(StepperID::PAN, _panRateCommand);

            if (fault != NO_FAULT)
            {
                // Update fault status
                SetFault(_id, fault, true);

                // Just indicate a fault in this case

                status_ = Q_RET_HANDLED;
                break;
            }

            // Set tilt PWM waveform
            fault = SetPWMFromRate(StepperID::TILT, _tiltRateCommand);

            if (fault != NO_FAULT)
            {
                // Update fault status
                SetFault(_id, fault, true);

                // Just indicate a fault in this case

                status_ = Q_RET_HANDLED;
                break;
            }

            status_ = Q_RET_HANDLED;
            break;
        }
        default:
        {
            status_ = super(&active);
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
