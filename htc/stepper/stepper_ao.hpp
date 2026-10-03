#ifndef STEPPER_AO_HPP_
#define STEPPER_AO_HPP_

#include "a4988.hpp"
#include "bsp.hpp"
#include "pid.hpp"
#include "qpcpp.hpp"

namespace stepper
{
/// @brief Standard nema 8 steps per revolution
static constexpr uint16_t stepsPerRev = 200U;
/// @brief Gearbox reduction ratio
static constexpr float gearRatio = 10.0f;
/// @brief Encoder counts per revolution
static constexpr uint16_t encoderCPR = 4000U;
/// @brief Pi
static constexpr float pi = 3.14159265358979323846f;
/// @brief Encoder counts to shaft angle in radians conversion factor
static constexpr float counts2Rad = 2 * pi / (encoderCPR * gearRatio);
/// @brief Degree to radian conversion factor
static constexpr float deg2rad = pi / 180.0f;
/// @brief Maximum angular velocity setpoint. NEMA8 maximum angular velocity ~150 rad/s. Anything above and risk
/// stalling.
static constexpr float maxRate = 150.0f / gearRatio;

/// @brief Axis in NED frame
enum Axis : uint8_t
{
    X = 0U,
    Y,
    Z,
    NUM_AXES
};

/// @brief Fault codes
enum Fault : uint8_t
{
    NO_FAULT = 0U,
    STEPPER_INIT_FAILED,
    ENCODER_INIT_FAILED,
    STEPPER_ENABLE_FAILED,
    STEPPER_DISABLE_FAILED,
    SET_DIR_FAILED,
    SET_FREQ_FAILED,
    SET_RES_FAILED,
    NUM_FAULTS
};

/// @brief Fault code to string table
/// @param fault
/// @return
constexpr const char* FaultToStr(Fault fault)
{
    switch (fault)
    {
        case Fault::NO_FAULT:
        {
            return "NO_FAULT";
        }
        case Fault::STEPPER_INIT_FAILED:
        {
            return "STEPPER_INIT_FAILED";
        }
        case Fault::ENCODER_INIT_FAILED:
        {
            return "ENCODER_INIT_FAILED";
        }
        case Fault::STEPPER_ENABLE_FAILED:
        {
            return "STEPPER_ENABLE_FAILED";
        }
        case Fault::STEPPER_DISABLE_FAILED:
        {
            return "STEPPER_DISABLE_FAILED";
        }
        case Fault::SET_DIR_FAILED:
        {
            return "SET_DIR_FAILED";
        }
        case Fault::SET_FREQ_FAILED:
        {
            return "SET_FREQ_FAILED";
        }
        case Fault::SET_RES_FAILED:
        {
            return "SET_RES_FAILED";
        }
        default:
        {
            return "";
        }
    }
}

/// @brief Control modes
enum Mode : uint8_t
{
    OPEN_LOOP = 0U,
    CLOSED_LOOP,
    CLOSED_LOOP_POS,
    NUM_MODES
};

/// @brief Option configurations
struct Options
{
    /// @brief Closed loop rate control maximum slew rate (rad). Not time normalized.
    float clSlewRate = 0.50f;
    /// @brief Position PID proportional gain
    float posKp = 5.0f;
    /// @brief Position PID integral gain
    float posKi = 0.0f;
    /// @brief Position PID derivative gain
    float posKd = 0.6f;
    /// @brief Gear ratio external to stepper motor and 10:1 gearbox
    float extGearRatio = 1.0f;
    /// @brief Whether or not to invert stabilization direction
    bool invertStabilization = false;
    /// @brief Allow instantaneous stop of stepper motor when changing direction. Screws with position control, but can
    /// make rate control snappier
    bool instantStop = false;
    /// @brief If IMU stabilization is enabled
    bool imuStabilizationEnabled = true;
};

/// @brief Stepper AO
class StepperAO : public QP::QActive
{
public:
    /// @brief Constructor
    StepperAO(a4988::A4988& stepperDriver, TIM_HandleTypeDef& htim, Axis axis, Options options);
    StepperAO(const StepperAO&) = delete;
    StepperAO& operator=(const StepperAO&) = delete;
    StepperAO(StepperAO&&) = delete;
    StepperAO& operator=(StepperAO&&) = delete;

    /// @brief Get pan stepper instance
    /// @return StepperAO&
    static StepperAO& PanInst();

    /// @brief Get tilt stepper instance
    /// @return StepperAO&
    static StepperAO& TiltInst();

    /// @brief Start StepperAO
    /// @param priority
    /// @param id
    void Start(const QP::QPrioSpec priority, bsp::SubsystemID id);

    /// @brief Set open-loop or closed-loop control mode
    inline void SetMode(Mode mode);

    /// @brief Set a speed setpoint without damping (rad/s)
    inline void SetRateSetpointDirect(float omega);

    /// @brief Set a speed setpoint (rad/s)
    inline void SetRateSetpoint(float omega);

    /// @brief Set a position setpoint (rad)
    inline void SetPositionSetpoint(float pos);

    /// @brief Set rate control slew rate (rad)
    inline void SetRateSlew(float slew);

    /// @brief Set PID gains
    inline void SetPIDGains(float kp, float ki, float kd);  // NOLINT

    /// @brief Set absolute home position (rad)
    inline void SetHome(float home);

    /// @brief Enable IMU stabilization
    inline void EnableStabilization();

    /// @brief Disable IMU stabilization
    inline void DisableStabilization();

    /// @brief Enable motor
    inline void Enable();

    /// @brief Disable motor
    inline void Disable();

    /// @brief Start encoder stream
    inline void StartEncoderStream();

    /// @brief Stop encoder stream
    inline void StopEncoderStream();

    /// @brief Reset stepper controller AO
    inline void Reset();

private:
    /// @brief Subsystem ID
    bsp::SubsystemID _id;
    /// @brief Event queue size
    static constexpr uint16_t _queueSize = 128U;
    /// @brief Event queue storage
    QP::QEvtPtr _queue[_queueSize] = {0};
    /// @brief Flag indicating if AO has executed initial transition
    bool _isStarted = false;

    /// @brief Stepper driver
    a4988::A4988& _stepperDriver;
    /// @brief Encoder timer handle
    TIM_HandleTypeDef& _encoderTim;

    /// @brief Axis of rotation in NED frame. Only matters if IMU stabilization is enabled.
    Axis _axis;

    /// @brief Internal fault recovery timer
    QP::QTimeEvt _faultRecoveryTimer;
    /// @brief Internal fault recovery timer period in ticks
    uint32_t _faultRecoveryTimerInterval = bsp::TICKS_PER_SEC / 100U;
    /// @brief Fault states
    bool _faultStates[stepper::Fault::NUM_FAULTS] = {false};

    /// @brief Rate IIR filter learning rate
    static constexpr float _iirAlpha = 1.0f;

    /// @brief Encoder polling timer
    QP::QTimeEvt _encoderPollTimer;
    /// @brief Encoder polling timer frequency in Hz
    static constexpr uint32_t _encoderPollTimerFreq = 100U;
    /// @brief Encoder polling timer interval
    static constexpr uint32_t _encoderPollTimerInterval = bsp::TICKS_PER_SEC / _encoderPollTimerFreq;

    /// @brief Closed loop stepper control polling timer
    QP::QTimeEvt _clTimer;
    /// @brief Stepper closed loop control timer frequency in Hz
    static constexpr uint32_t _clTimerFreq = 50U;
    /// @brief Stepper closed loop control update interval
    static constexpr uint32_t _clTimerInterval = bsp::TICKS_PER_SEC / _clTimerFreq;

    /// @brief Encoder CLI streaming timer
    QP::QTimeEvt _encoderStreamTimer;
    /// @brief Encoder CLI streaming timer interval
    static constexpr uint32_t _encoderStreamTimerInterval = bsp::TICKS_PER_SEC / 5U;

    /// @brief Position PID controller
    csys::PID _posPID;

    /// @brief Stepper optional parameters
    Options _options;

private:  // NOLINT Dynamic internal states
    /// @brief Rate setpoint (rad/s)
    float _rateSetpoint = 0.0f;

    /// @brief Closed loop controller command (rad/s)
    float _rateCommand = 0.0f;

    /// @brief Last encoder value (counts)
    uint16_t _lastEnc = 0u;

    /// @brief Last absolute angular position value (rad)
    float _absLastPos = 0.0f;

    /// @brief Absolute home angular position value in (rad)
    float _absHomePos = 0.0f;

    /// @brief Last encoder measured angular rate (rad/s)
    float _lastRate = 0.0f;

    /// @brief Closed loop position setpoint (rad)
    float _posSetpoint = 0.0f;

    /// @brief IMU integrated orientation (rad)
    float _imuEstimatedPos = 0.0f;

private:
    /// @brief Private CLIAO signals
    enum PrivateSignals : QP::QSignal
    {
        INITIALIZED_SIG = bsp::PublicSignals::MAX_PUB_SIG,
        FAULT_SIG,
        SET_MODE_SIG,
        SET_RATE_DIRECT_SIG,
        SET_RATE_SIG,
        SET_POS_SIG,
        SET_PID_GAINS_SIG,
        SET_RATE_SLEW_SIG,
        SET_HOME_SIG,
        ENABLE_STABILIZATION_SIG,
        DISABLE_STABILIZATION_SIG,
        ENABLE_SIG,
        DISABLE_SIG,
        RESET_SIG,
        POLL_ENCODER_SIG,
        START_ENCODER_STREAM_SIG,
        STOP_ENCODER_STREAM_SIG,
        ENCODER_STREAM_SIG,
        CL_UPDATE_SIG,
        MAX_PRIV_SIG
    };

    /// @brief Setpoint modification evt
    class SetpointEvt : public QP::QEvt
    {
    public:
        SetpointEvt(QP::QSignal sig) : QP::QEvt(sig) {}
        float setpoint;
    };

    /// @brief Mode control event
    class SetModeEvt : public QP::QEvt
    {
    public:
        SetModeEvt(QP::QSignal sig) : QP::QEvt(sig) {}
        Mode mode;
    };

    /// @brief PID gain set event
    class SetPIDGainsEvt : public QP::QEvt
    {
    public:
        SetPIDGainsEvt(QP::QSignal sig) : QP::QEvt(sig) {}
        float kp;
        float ki;
        float kd;
    };

    /// @brief Set and publish fault
    void SetFault(bsp::SubsystemID subsystem, uint8_t fault, bool active);

    /// @brief Given a desired angular velocity, compute step frequency and step resolution to handle resonance when
    /// full stepping at low speeds
    /// @param[in] omega angular velocity
    /// @param[out] freq step frequency
    /// @param[out] resolution step resolution
    /// @return void
    static void GetFreqResolutionForRate(float omega, float& freq, a4988::StepResolution& resolution);

    /// @brief Given a desired angular velocity, configure stepper drive pwm waveform
    /// @param[in] omega angular velocity
    /// @return Fault
    Fault SetPWMFromRate(float omega);

    /// @brief Get effective position after static home offset and IMU compensation
    /// @return Fault
    float GetEffectivePosition();

    /// @brief Initial state
    Q_STATE_DECL(initial);
    /// @brief Root state
    Q_STATE_DECL(root);
    /// @brief Initialize
    Q_STATE_DECL(initializing);
    /// @brief Active
    Q_STATE_DECL(active);
    /// @brief Active Open Loop Rate Control
    Q_STATE_DECL(active_ol);
    /// @brief Active Closed Loop Rate Control
    Q_STATE_DECL(active_cl);
    /// @brief Active Closed Loop Position Control
    Q_STATE_DECL(active_cl_pos);
    /// @brief Fault
    Q_STATE_DECL(error);
};  // class StepperAO

inline void StepperAO::SetMode(Mode mode)
{
    if (_isStarted)
    {
        SetModeEvt* evt = Q_NEW(SetModeEvt, PrivateSignals::SET_MODE_SIG);
        evt->mode = mode;
        POST(evt, this);
    }
}

inline void StepperAO::SetRateSetpointDirect(float omega)
{
    if (_isStarted)
    {
        SetpointEvt* evt = Q_NEW(SetpointEvt, PrivateSignals::SET_RATE_DIRECT_SIG);
        evt->setpoint = omega;
        POST(evt, this);
    }
}

inline void StepperAO::SetRateSetpoint(float omega)
{
    if (_isStarted)
    {
        SetpointEvt* evt = Q_NEW(SetpointEvt, PrivateSignals::SET_RATE_SIG);
        evt->setpoint = omega;
        POST(evt, this);
    }
}

inline void StepperAO::SetPositionSetpoint(float pos)
{
    if (_isStarted)
    {
        SetpointEvt* evt = Q_NEW(SetpointEvt, PrivateSignals::SET_POS_SIG);
        evt->setpoint = pos;
        POST(evt, this);
    }
}

inline void StepperAO::SetPIDGains(float kp, float ki, float kd)  // NOLINT
{
    if (_isStarted)
    {
        SetPIDGainsEvt* evt = Q_NEW(SetPIDGainsEvt, PrivateSignals::SET_PID_GAINS_SIG);
        evt->kp = kp;
        evt->ki = ki;
        evt->kd = kd;
        POST(evt, this);
    }
}

inline void StepperAO::SetRateSlew(float slew)
{
    if (_isStarted)
    {
        SetpointEvt* evt = Q_NEW(SetpointEvt, PrivateSignals::SET_RATE_SLEW_SIG);
        evt->setpoint = slew;
        POST(evt, this);
    }
}

inline void StepperAO::SetHome(float home)
{
    if (_isStarted)
    {
        SetpointEvt* evt = Q_NEW(SetpointEvt, PrivateSignals::SET_HOME_SIG);
        evt->setpoint = home;
        POST(evt, this);
    }
}

inline void StepperAO::EnableStabilization()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::ENABLE_STABILIZATION_SIG);
        POST(&evt, this);
    }
}

inline void StepperAO::DisableStabilization()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::DISABLE_STABILIZATION_SIG);
        POST(&evt, this);
    }
}

inline void StepperAO::Enable()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::ENABLE_SIG);
        POST(&evt, this);
    }
}

inline void StepperAO::Disable()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::DISABLE_SIG);
        POST(&evt, this);
    }
}

inline void StepperAO::StartEncoderStream()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::START_ENCODER_STREAM_SIG);
        POST(&evt, this);
    }
}

inline void StepperAO::StopEncoderStream()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::STOP_ENCODER_STREAM_SIG);
        POST(&evt, this);
    }
}

inline void StepperAO::Reset()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::RESET_SIG);
        POST(&evt, this);
    }
}

}  // namespace stepper

#endif
