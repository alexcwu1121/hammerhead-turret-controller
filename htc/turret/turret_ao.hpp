#ifndef TURRET_AO_HPP_
#define TURRET_AO_HPP_

#include "a4988.hpp"
#include "bsp.hpp"
#include "qpcpp.hpp"

namespace turret
{
/// @brief Fault codes
enum Fault : uint8_t
{
    NO_FAULT = 0U,
    STEPPER_INIT_FAILED,
    ENCODER_INIT_FAILED,
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

/// @brief Stepper IDs
enum StepperID : uint8_t
{
    TILT = 0U,
    PAN,
    NUM_STEPPERS
};

/// @brief Control modes
enum Mode : uint8_t
{
    OPEN_LOOP = 0U,
    CLOSED_LOOP,
    NUM_MODES
};

/// @brief Turret AO
class TurretAO : public QP::QActive
{
public:
    /// @brief Constructor
    TurretAO();
    TurretAO(const TurretAO&) = delete;
    TurretAO& operator=(const TurretAO&) = delete;
    TurretAO(TurretAO&&) = delete;
    TurretAO& operator=(TurretAO&&) = delete;

    /// @brief Get instance
    /// @return TurretAO&
    static TurretAO& Inst()
    {
        static TurretAO inst;
        return inst;
    }

    /// @brief Start TurretAO
    /// @param priority
    /// @param id
    void Start(const QP::QPrioSpec priority, bsp::SubsystemID id);

    /// @brief Set open-loop or closed-loop control mode
    inline void SetMode(Mode mode);

    /// @brief Set a speed setpoint without damping (rad/s)
    inline void SetRateSetpointDirect(StepperID stepper, float omega);

    /// @brief Set a speed setpoint (rad/s)
    inline void SetRateSetpoint(StepperID stepper, float omega);

    /// @brief Start homing sequence
    inline void Home(StepperID stepper);

    /// @brief Enable motor
    inline void Enable(StepperID stepper);

    /// @brief Disable motor
    inline void Disable(StepperID stepper);

    /// @brief Start encoder stream
    inline void StartEncoderStream();

    /// @brief Stop encoder stream
    inline void StopEncoderStream();

    /// @brief Reset turret controller AO
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
    /// @brief Internal fault recovery timer
    QP::QTimeEvt _faultRecoveryTimer;
    /// @brief Internal fault recovery timer period in ticks
    uint32_t _faultRecoveryTimerInterval = bsp::TICKS_PER_SEC / 100U;
    /// @brief Fault states
    bool _faultStates[turret::Fault::NUM_FAULTS] = {false};

    /// @brief Standard nema 8 steps per revolution
    static constexpr uint16_t _stepsPerRev = 200U;
    /// @brief Gearbox reduction ratio
    static constexpr float _gearRatio = 10.0f;
    /// @brief Encoder counts per revolution
    static constexpr uint16_t _encoderCPR = 4000U;
    /// @brief Rate IIR filter learning rate
    static constexpr float _iirAlpha = 1.0f;

    /// @brief Encoder polling timer
    QP::QTimeEvt _encoderTimer;
    /// @brief Encoder polling timer frequency in Hz
    static constexpr uint32_t _encoderTimerFreq = 50U;
    /// @brief Encoder polling timer interval
    static constexpr uint32_t _encoderTimerInterval = bsp::TICKS_PER_SEC / _encoderTimerFreq;

    /// @brief Closed loop stepper control polling timer
    QP::QTimeEvt _clTimer;
    /// @brief Stepper closed loop control timer frequency in Hz
    static constexpr uint32_t _clTimerFreq = 20U;
    /// @brief Stepper closed loop control update interval
    static constexpr uint32_t _clTimerInterval = bsp::TICKS_PER_SEC / _clTimerFreq;
    /// @brief Closed loop stepper slew rate per interval (rad/s)
    static constexpr float _clSlewRate = 1.00f;

    /// @brief Maximum angular velocity setpoint. NEMA8 maximum angular velocity ~100 rad/s. Anything above and risk
    /// stalling.
    static constexpr float _maxRate = 100.0f / _gearRatio;

    /// @brief Encoder CLI streaming timer
    QP::QTimeEvt _encoderStreamTimer;
    /// @brief Encoder CLI streaming timer interval
    static constexpr uint32_t _encoderStreamTimerInterval = bsp::TICKS_PER_SEC / 5U;

    /// @brief Pan rate setpoint (rad/s)
    float _panRateSetpoint = 0.0f;
    /// @brief Tilt rate setpoint (rad/s)
    float _tiltRateSetpoint = 0.0f;

    /// @brief Pan closed loop controller command (rad/s)
    float _panRateCommand = 0.0f;
    /// @brief Tilt closed loop controller command (rad/s)
    float _tiltRateCommand = 0.0f;

    /// @brief Last pan encoder value
    uint16_t _lastPanEnc = 0u;
    /// @brief Last tilt encoder value
    uint16_t _lastTiltEnc = 0u;

    /// @brief Pan position offset relative to home
    uint16_t _panHomeOffset = 0u;
    /// @brief Tilt position offset relative to home
    uint16_t _tiltHomeOffset = 0u;

    /// @brief Last pan encoder measured angular rate (rad/s)
    float _lastPanRate = 0.0f;
    /// @brief Last tilt encoder measured angular rate (rad/s)
    float _lastTiltRate = 0.0f;

    /// @brief Stepper drivers
    a4988::A4988 _steppers[StepperID::NUM_STEPPERS];

private:
    /// @brief Private CLIAO signals
    enum PrivateSignals : QP::QSignal
    {
        INITIALIZED_SIG = bsp::PublicSignals::MAX_PUB_SIG,
        FAULT_SIG,
        SET_MODE_SIG,
        SET_RATE_DIRECT_SIG,
        SET_RATE_SIG,
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

    /// @brief Set angular rate evt
    class SetRateEvt : public QP::QEvt
    {
    public:
        SetRateEvt(QP::QSignal sig) : QP::QEvt(sig) {}
        StepperID stepper;
        float omega;
    };

    /// @brief Generic stepper control event
    class StepperControlEvt : public QP::QEvt
    {
    public:
        StepperControlEvt(QP::QSignal sig) : QP::QEvt(sig) {}
        StepperID stepper;
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
    void GetFreqResolutionForRate(float omega, float& freq, a4988::StepResolution& resolution);

    /// @brief Given a desired angular velocity, configure stepper drive pwm waveform
    /// @param[in] omega angular velocity
    /// @return Fault
    Fault SetPWMFromRate(StepperID stepper, float omega);

    /// @brief Initial state
    Q_STATE_DECL(initial);
    /// @brief Root state
    Q_STATE_DECL(root);
    /// @brief Initialize
    Q_STATE_DECL(initializing);
    /// @brief Active
    Q_STATE_DECL(active);
    /// @brief Active Open Loop
    Q_STATE_DECL(active_ol);
    /// @brief Active Closed Loop
    Q_STATE_DECL(active_cl);
    /// @brief Fault
    Q_STATE_DECL(error);
};  // class TurretAO

inline void TurretAO::SetMode(Mode mode)
{
    if (_isStarted)
    {
        SetModeEvt* evt = Q_NEW(SetModeEvt, PrivateSignals::SET_MODE_SIG);
        evt->mode = mode;
        POST(evt, this);
    }
}

inline void TurretAO::SetRateSetpointDirect(StepperID stepper, float omega)
{
    if (_isStarted)
    {
        SetRateEvt* evt = Q_NEW(SetRateEvt, PrivateSignals::SET_RATE_DIRECT_SIG);
        evt->stepper = stepper;
        evt->omega = omega;
        POST(evt, this);
    }
}

inline void TurretAO::SetRateSetpoint(StepperID stepper, float omega)
{
    if (_isStarted)
    {
        SetRateEvt* evt = Q_NEW(SetRateEvt, PrivateSignals::SET_RATE_SIG);
        evt->stepper = stepper;
        evt->omega = omega;
        POST(evt, this);
    }
}

inline void TurretAO::Home(StepperID stepper)
{
    if (_isStarted)
    {
        StepperControlEvt* evt = Q_NEW(StepperControlEvt, PrivateSignals::HOME_SIG);
        evt->stepper = stepper;
        POST(evt, this);
    }
}

inline void TurretAO::Enable(StepperID stepper)
{
    if (_isStarted)
    {
        StepperControlEvt* evt = Q_NEW(StepperControlEvt, PrivateSignals::ENABLE_SIG);
        evt->stepper = stepper;
        POST(evt, this);
    }
}

inline void TurretAO::Disable(StepperID stepper)
{
    if (_isStarted)
    {
        StepperControlEvt* evt = Q_NEW(StepperControlEvt, PrivateSignals::DISABLE_SIG);
        evt->stepper = stepper;
        POST(evt, this);
    }
}

inline void TurretAO::StartEncoderStream()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::START_ENCODER_STREAM_SIG);
        POST(&evt, this);
    }
}

inline void TurretAO::StopEncoderStream()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::STOP_ENCODER_STREAM_SIG);
        POST(&evt, this);
    }
}

inline void TurretAO::Reset()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::RESET_SIG);
        POST(&evt, this);
    }
}

}  // namespace turret

#endif
