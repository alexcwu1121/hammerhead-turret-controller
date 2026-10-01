#ifndef STEPPER_AO_HPP_
#define STEPPER_AO_HPP_

#include "a4988.hpp"
#include "bsp.hpp"
#include "pid.hpp"
#include "qpcpp.hpp"

namespace stepper
{
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

/// @brief Stepper options struct
struct StepperOpt
{
    /// @brief Offset of home position relative to homing trigger position in radians
    float homeOffset;
};

/// @brief Stepper AO
class StepperAO : public QP::QActive
{
public:
    /// @brief Constructor
    StepperAO(a4988::A4988& stepperDriver, TIM_HandleTypeDef& htim, StepperOpt opt);
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

    /// @brief Start homing sequence
    inline void Home();

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
    /// TODO: absorb a lot of constants below into stepper options as needed
    /// @brief Stepper options
    StepperOpt _opt;

    /// @brief Internal fault recovery timer
    QP::QTimeEvt _faultRecoveryTimer;
    /// @brief Internal fault recovery timer period in ticks
    uint32_t _faultRecoveryTimerInterval = bsp::TICKS_PER_SEC / 100U;
    /// @brief Fault states
    bool _faultStates[stepper::Fault::NUM_FAULTS] = {false};

    /// @brief Standard nema 8 steps per revolution
    static constexpr uint16_t _stepsPerRev = 200U;
    /// @brief Gearbox reduction ratio
    static constexpr float _gearRatio = 10.0f;
    /// @brief Encoder counts per revolution
    static constexpr uint16_t _encoderCPR = 4000U;
    /// @brief Encoder counts to shaft angle in radians conversion factor
    static constexpr float _counts2Rad = 6.28f / (_encoderCPR * _gearRatio);
    /// @brief Rate IIR filter learning rate
    static constexpr float _iirAlpha = 1.0f;

    /// @brief Encoder polling timer
    QP::QTimeEvt _encoderPollTimer;
    /// @brief Encoder polling timer frequency in Hz
    static constexpr uint32_t _encoderPollTimerFreq = 50U;
    /// @brief Encoder polling timer interval
    static constexpr uint32_t _encoderPollTimerInterval = bsp::TICKS_PER_SEC / _encoderPollTimerFreq;

    /// @brief Closed loop stepper control polling timer
    QP::QTimeEvt _clTimer;
    /// @brief Stepper closed loop control timer frequency in Hz
    static constexpr uint32_t _clTimerFreq = 20U;
    /// @brief Stepper closed loop control update interval
    static constexpr uint32_t _clTimerInterval = bsp::TICKS_PER_SEC / _clTimerFreq;
    /// @brief Closed loop stepper slew rate per interval (rad/s)
    static constexpr float _clSlewRate = 1.00f;

    /// @brief Maximum angular velocity setpoint. NEMA8 maximum angular velocity ~150 rad/s. Anything above and risk
    /// stalling.
    static constexpr float _maxRate = 150.0f / _gearRatio;

    /// @brief Encoder CLI streaming timer
    QP::QTimeEvt _encoderStreamTimer;
    /// @brief Encoder CLI streaming timer interval
    static constexpr uint32_t _encoderStreamTimerInterval = bsp::TICKS_PER_SEC / 5U;

    /// @brief Tate setpoint (rad/s)
    float _rateSetpoint = 0.0f;

    /// @brief Closed loop controller command (rad/s)
    float _rateCommand = 0.0f;

    /// @brief Last encoder value
    uint16_t _lastEnc = 0u;

    /// @brief Last absolute angular position value in counts
    int _absLastPos = 0;

    /// @brief Position offset relative to home
    uint16_t _homeOffset = 0u;

    /// @brief Last encoder measured angular rate (rad/s)
    float _lastRate = 0.0f;

    /// @brief Closed loop position setpoint (rad)
    float _posSetpoint = 0.0f;

    /// @brief Position PID controller
    csys::PID _posPID;
    /// @brief Position PID proportional gain
    float _posKp = 5.0f;
    /// @brief Position PID integral gain
    float _posKi = 0.0f;
    /// @brief Position PID derivative gain
    float _posKd = 1.0f;

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
        HOME_SIG,
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

inline void StepperAO::Home()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::HOME_SIG);
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
