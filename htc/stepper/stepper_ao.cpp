#include "stepper_ao.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "bsp.hpp"
#include "cli_ao.hpp"
#include "gpio.h"
#include "imu_ao.hpp"
#include "qpcpp.hpp"
#include "tim.h"

namespace stepper
{
/// @brief Pan stepper driver
static a4988::A4988 panDriver = a4988::A4988Peripherals {.stepPinPort = *TILT_STEP_GPIO_Port,
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
                                                         .htimCh = TIM_CHANNEL_1};

/// @brief Tilt stepper driver
static a4988::A4988 tiltDriver = a4988::A4988Peripherals {.stepPinPort = *PAN_STEP_GPIO_Port,
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
                                                          .htimCh = TIM_CHANNEL_1};

StepperAO::StepperAO(a4988::A4988& stepperDriver, TIM_HandleTypeDef& htim, Axis axis, Options options) :
    QP::QActive(&initial),
    _stepperDriver(stepperDriver),
    _encoderTim(htim),
    _axis(axis),
    _faultRecoveryTimer(this, PrivateSignals::RESET_SIG, 0U),
    _encoderPollTimer(this, PrivateSignals::POLL_ENCODER_SIG, 0U),
    _clTimer(this, PrivateSignals::CL_UPDATE_SIG, 0U),
    _encoderStreamTimer(this, PrivateSignals::ENCODER_STREAM_SIG, 0U),
    _options(options)
{}

StepperAO& StepperAO::PanInst()
{
    static StepperAO inst(
        panDriver, htim2, Axis::Z,
        {.posKp = 15.0f, .posKi = 0.0f, .posKd = 1.8f, .extGearRatio = 2.9888888f, .invertStabilization = true});
    return inst;
}

StepperAO& StepperAO::TiltInst()
{
    static StepperAO inst(tiltDriver, htim3, Axis::Y, {.posKp = 5.0f, .posKi = 0.0f, .posKd = 0.6f});
    return inst;
}

void StepperAO::Start(const QP::QPrioSpec priority, bsp::SubsystemID id)
{
    _id = id;
    _isStarted = true;
    this->start(priority,      // QP prio. of the AO
                _queue,        // event queue storage
                _queueSize,    // queue size [events]
                nullptr, 0U);  // no stack storage
}

void StepperAO::SetFault(bsp::SubsystemID id, uint8_t fault, bool active)
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

void StepperAO::GetFreqResolutionForRate(float omega, float& freq, a4988::StepResolution& resolution)
{
    // compute full step frequency based on stepper resolution
    freq = gearRatio * std::abs(omega) * stepsPerRev / (2 * pi);

    /// TODO: For now, only use eighth microstepping
    resolution = a4988::StepResolution::EIGHTH;
    freq *= 8;  // NOLINT
}

Fault StepperAO::SetPWMFromRate(float omega)
{
    // compute direction
    a4988::StepDir dir = omega > 0 ? a4988::StepDir::CCW : a4988::StepDir::CW;

    // compute step frequency and step resolution
    float freq = 0.0f;
    a4988::StepResolution resolution = a4988::StepResolution::FULL;
    GetFreqResolutionForRate(omega, freq, resolution);

    // set direction and frequency
    a4988::Fault fault = _stepperDriver.SetDir(dir);
    if (fault != a4988::Fault::NO_FAULT) { return Fault::SET_DIR_FAILED; }

    fault = _stepperDriver.SetResolution(resolution);
    if (fault != a4988::Fault::NO_FAULT) { return Fault::SET_RES_FAILED; }

    fault = _stepperDriver.SetFrequency(freq);
    if (fault != a4988::Fault::NO_FAULT) { return Fault::SET_FREQ_FAILED; }

    return Fault::NO_FAULT;
}

float StepperAO::GetEffectivePosition()
{
    float eff_pos = _absLastPos - _absHomePos;
    if (_options.imuStabilizationEnabled)
    {
        if (_options.invertStabilization) { eff_pos += _imuEstimatedPos; }
        else { eff_pos -= _imuEstimatedPos; }
    }
    return eff_pos;
}

Q_STATE_DEF(StepperAO, initial)
{
    Q_UNUSED_PAR(e);
    subscribe(bsp::PublicSignals::IMU_SIG);
    return tran(&initializing);
}

Q_STATE_DEF(StepperAO, root)
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
            auto fault = _stepperDriver.Disable();
            if (fault != a4988::Fault::NO_FAULT) { SetFault(_id, Fault::STEPPER_DISABLE_FAILED, true); }

            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::SET_HOME_SIG:
        {
            // Set home position
            _absHomePos = Q_EVT_CAST(SetpointEvt)->setpoint;

            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::ENABLE_STABILIZATION_SIG:
        {
            _options.imuStabilizationEnabled = true;
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::DISABLE_STABILIZATION_SIG:
        {
            _options.imuStabilizationEnabled = false;
            status_ = Q_RET_HANDLED;
            break;
        }
        case bsp::PublicSignals::IMU_SIG:
        {
            // perform axis mapping from imu to NED vehicle frame
            // Gyro X -> Vehicle Y
            // Gyro Y -> Vehicle Z
            // Gyro Z -> Vehicle X
            float imu_rate;
            switch (_axis)
            {
                case Axis::X:
                {
                    imu_rate = Q_EVT_CAST(imu::IMUEvt)->data.gyr[2] * deg2rad;
                    break;
                }
                case Axis::Y:
                {
                    imu_rate = Q_EVT_CAST(imu::IMUEvt)->data.gyr[0] * deg2rad;
                    break;
                }
                case Axis::Z:
                {
                    imu_rate = Q_EVT_CAST(imu::IMUEvt)->data.gyr[1] * deg2rad;
                    break;
                }
                default:
                {
                    imu_rate = 0.0f;
                    break;
                }
            }

            // integrate imu rate to estimate position
            _imuEstimatedPos += imu_rate * (1.0f / static_cast<float>(imu::IMUAO::imuTimerFreq));

            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::SET_PID_GAINS_SIG:
        {
            _options.posKp = Q_EVT_CAST(SetPIDGainsEvt)->kp;
            _options.posKi = Q_EVT_CAST(SetPIDGainsEvt)->ki;
            _options.posKd = Q_EVT_CAST(SetPIDGainsEvt)->kd;
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::SET_RATE_SLEW_SIG:
        {
            _options.clSlewRate = Q_EVT_CAST(SetpointEvt)->setpoint;
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

Q_STATE_DEF(StepperAO, initializing)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // Initialize stepper driver
            if (_stepperDriver.Init() != a4988::Fault::NO_FAULT)
            {
                // Update fault status
                SetFault(_id, Fault::STEPPER_INIT_FAILED, true);

                // Queue up fault state transition
                static QP::QEvt evt(PrivateSignals::FAULT_SIG);
                POST(&evt, this);

                status_ = Q_RET_HANDLED;
                break;
            }

            // Initialize encoder
            if (HAL_TIM_Encoder_Start(&_encoderTim, TIM_CHANNEL_ALL) != HAL_OK)
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
            // Start in closed loop position control mode
            status_ = tran(&active_cl_pos);
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

Q_STATE_DEF(StepperAO, active)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            _encoderPollTimer.armX(_encoderPollTimerInterval, _encoderPollTimerInterval);

            // set initial encoder position
            _lastEnc = __HAL_TIM_GET_COUNTER(&_encoderTim);

            // also set initial absolute encoder position
            _absLastPos = static_cast<float>(_lastEnc) * counts2Rad / _options.extGearRatio;

            status_ = Q_RET_HANDLED;
            break;
        }
        case Q_EXIT_SIG:
        {
            _encoderPollTimer.disarm();
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::SET_MODE_SIG:
        {
            switch (Q_EVT_CAST(SetModeEvt)->mode)
            {
                case Mode::OPEN_LOOP:
                {
                    status_ = tran(&active_ol);
                    break;
                }
                case Mode::CLOSED_LOOP:
                {
                    status_ = tran(&active_cl);
                    break;
                }
                case Mode::CLOSED_LOOP_POS:
                {
                    status_ = tran(&active_cl_pos);
                    break;
                }
                default:
                {
                    status_ = Q_RET_HANDLED;
                    break;
                }
            }
            break;
        }
        case PrivateSignals::ENABLE_SIG:
        {
            auto fault = _stepperDriver.Enable();
            if (fault != a4988::Fault::NO_FAULT) { SetFault(_id, Fault::STEPPER_ENABLE_FAILED, true); }

            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::POLL_ENCODER_SIG:
        {
            uint16_t enc = __HAL_TIM_GET_COUNTER(&_encoderTim);

            // compute delta in encoder position since last sample
            int16_t enc_delta = static_cast<int16_t>(enc - _lastEnc);

            // convert to rad/s and iir filter
            float obs = static_cast<float>(_encoderPollTimerFreq) * enc_delta * counts2Rad / _options.extGearRatio;
            _lastRate += _iirAlpha * (obs - _lastRate);

            // update last encoder value
            _lastEnc = enc;

            // also compute absolute angle
            _absLastPos += static_cast<float>(enc_delta) * counts2Rad / _options.extGearRatio;

            // publish encoder state
            bsp::StepperEncStateEvt* evt = Q_NEW(bsp::StepperEncStateEvt, bsp::PublicSignals::STEPPER_ENC_STATE_SIG);
            evt->id = _id;
            evt->absPos = _absLastPos;
            evt->rate = _lastRate;
            PUBLISH(evt, this);

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
                "%s\n\r"
                "Encoder Rate: %+7.4f rad/s\n\r"
                "Encoder Pos:  %+7.4f rad\n\r"
                "IMU Est Pos:  %+7.4f rad\n\r"
                ">>>>>>>>>>>>>>\n\r",
                bsp::SubsystemIDToStr(_id), _lastRate, _absLastPos, _imuEstimatedPos);
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

Q_STATE_DEF(StepperAO, active_ol)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // notify of mode change
            bsp::StepperModeChangedEvt* evt =
                Q_NEW(bsp::StepperModeChangedEvt, bsp::PublicSignals::STEPPER_MODE_CHANGED_SIG);
            evt->id = _id;
            evt->mode = Mode::OPEN_LOOP;
            PUBLISH(evt, this);

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
            float omega = std::clamp(Q_EVT_CAST(SetpointEvt)->setpoint, -maxRate, maxRate);
            Fault fault = SetPWMFromRate(omega);

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

Q_STATE_DEF(StepperAO, active_cl)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // start timer for control loop updates
            _clTimer.armX(_clTimerInterval, _clTimerInterval);

            // initialize rate setpoint to current angular velocities
            _rateSetpoint = _lastRate;

            // initialize rate command to current angular velocity
            // this assumes open loop angular velocity commands are close to reality, which is pretty much true
            _rateCommand = _lastRate;

            // notify of mode change
            bsp::StepperModeChangedEvt* evt =
                Q_NEW(bsp::StepperModeChangedEvt, bsp::PublicSignals::STEPPER_MODE_CHANGED_SIG);
            evt->id = _id;
            evt->mode = Mode::CLOSED_LOOP;
            PUBLISH(evt, this);

            status_ = Q_RET_HANDLED;
            break;
        }
        case Q_EXIT_SIG:
        {
            _clTimer.disarm();
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::SET_RATE_SIG:
        {
            _rateSetpoint = std::clamp(Q_EVT_CAST(SetpointEvt)->setpoint, -maxRate, maxRate);
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::CL_UPDATE_SIG:
        {
            float rate_error = _rateSetpoint - _lastRate;
            float correction_mag = std::min(std::abs(rate_error), _options.clSlewRate);

            // sometimes, the motor will stall against an elastic surface and kick back in the opposite direction.
            // in that event, if we set rate command to last rate, then the motor will travel in the opposite direction
            // and have to reverse thus, if the current observed rate is opposite the desired rate, we will immediately
            // zero the rate command
            if (_options.instantStop && std::signbit(rate_error) != std::signbit(_lastRate)) { _rateCommand = 0.0f; }
            else { _rateCommand = _lastRate; }

            if (rate_error > 0) { _rateCommand += correction_mag; }
            else if (rate_error < 0) { _rateCommand -= correction_mag; }

            // Set PWM waveform
            auto fault = SetPWMFromRate(_rateCommand);
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

Q_STATE_DEF(StepperAO, active_cl_pos)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // use the same update timer as rate control
            /// TODO: if we need rate decoupling, consider adding another timer

            // Initialize position setpoint to current effective position
            _posSetpoint = GetEffectivePosition();

            // refresh pid gains
            _posPID.SetGains(_options.posKp, _options.posKi, _options.posKd);

            // reset PID controller state
            _posPID.Reset();

            // notify of mode change
            bsp::StepperModeChangedEvt* evt =
                Q_NEW(bsp::StepperModeChangedEvt, bsp::PublicSignals::STEPPER_MODE_CHANGED_SIG);
            evt->id = _id;
            evt->mode = Mode::CLOSED_LOOP_POS;
            PUBLISH(evt, this);

            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::SET_POS_SIG:
        {
            _posSetpoint = Q_EVT_CAST(SetpointEvt)->setpoint - _absHomePos;
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::CL_UPDATE_SIG:
        {
            // Position control loop update
            _rateSetpoint =
                _posPID.Update(_posSetpoint, GetEffectivePosition(), 1.0f / static_cast<float>(_clTimerFreq));

            // Saturate rate setpoint
            _rateSetpoint = std::clamp(_rateSetpoint, -maxRate, maxRate);

            // Mark as unhandled so this event propagates up a level to the rate control loop
            status_ = Q_RET_UNHANDLED;
            break;
        }
        default:
        {
            status_ = super(&active_cl);
            break;
        }
    }
    return status_;
}

Q_STATE_DEF(StepperAO, error)
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
}  // namespace stepper
