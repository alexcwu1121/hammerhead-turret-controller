#ifndef TILT_HOME_AO_HPP_
#define TILT_HOME_AO_HPP_

#include "qpcpp.hpp"
#include "stepper_ao.hpp"

namespace stepper
{
/// @brief Fault codes
enum TiltHomeFault : uint8_t
{
    TILT_HOME_FAILED = 0U,
};

/// @brief Fault code to string table
/// @param fault
/// @return
constexpr const char* FaultToStr(TiltHomeFault fault)
{
    switch (fault)
    {
        case TiltHomeFault::TILT_HOME_FAILED:
        {
            return "TILT_HOME_FAILED";
        }
        default:
        {
            return "";
        }
    }
}

/// @brief Tilt stepper homing AO. Companion to StepperAO
class TiltHomeAO : public QP::QActive
{
public:
    /// @brief Constructor
    TiltHomeAO();
    TiltHomeAO(const TiltHomeAO&) = delete;
    TiltHomeAO& operator=(const TiltHomeAO&) = delete;
    TiltHomeAO(TiltHomeAO&&) = delete;
    TiltHomeAO& operator=(TiltHomeAO&&) = delete;

    /// @brief Get tilt stepper homing instance
    /// @return TiltHomeAO&
    static TiltHomeAO& Inst()
    {
        static TiltHomeAO inst;
        return inst;
    }

    /// @brief Start TiltHomeAO
    /// @param priority
    /// @param id
    void Start(const QP::QPrioSpec priority, bsp::SubsystemID id);

    /// @brief Request to home tilt stepper
    inline void Home();

    /// @brief Abort active homing sequence
    inline void Abort();

private:
    /// @brief Subsystem ID
    bsp::SubsystemID _id;
    /// @brief Event queue size
    static constexpr uint16_t _queueSize = 128U;
    /// @brief Event queue storage
    QP::QEvtPtr _queue[_queueSize] = {0};
    /// @brief Flag indicating if AO has executed initial transition
    bool _isStarted = false;

    /// @brief Traverse timeout in ticks (3 seconds)
    static constexpr uint32_t _traverseTimeout = bsp::TICKS_PER_SEC * 5.0f;
    /// @brief Traverse timeout timer
    QP::QTimeEvt _traverseTimer;

    /// @brief Stall detection threshold time in ticks
    static constexpr uint32_t _stallDetectThresholdTime = bsp::TICKS_PER_SEC * 0.5f;
    /// @brief Stall detection timer
    QP::QTimeEvt _stallDetectTimer;

    /// @brief Stall detection lower threshold rate in rad/s
    static constexpr float _stallDetectThresholdRate = 0.8f;

    /// TODO: make this configurable
    /// @brief Position offset relative to home in rad
    float _homeOffset = 0.2618f;

    /// @brief Rate at which stepper is considered settled at home (rad/s)
    static constexpr float _homeRateAcceptanceThreshold = 0.100f;

    /// @brief Last observed tilt stepper mode
    stepper::Mode _lastObservedMode = stepper::Mode::NUM_MODES;

    /// @brief Last observed tilt stepper mode before homing
    stepper::Mode _lastObservedModePreHoming = stepper::Mode::NUM_MODES;

private:  // NOLINT
    /// @brief Private CLIAO signals
    enum PrivateSignals : QP::QSignal
    {
        INITIALIZED_SIG = bsp::PublicSignals::MAX_PUB_SIG,
        HOME_SIG,
        TRAVERSE_TIMEOUT_SIG,
        ABORT_SIG,
        STALL_SIG,
        MAX_PRIV_SIG
    };

    /// @brief Initial state
    Q_STATE_DECL(initial);
    /// @brief Root state
    Q_STATE_DECL(root);
    /// @brief Idle
    Q_STATE_DECL(idle);
    /// @brief Homing
    Q_STATE_DECL(homing);
    /// @brief traverse until rate anomaly detected
    Q_STATE_DECL(traversing);
    /// @brief temporarily stalling during a traverse
    Q_STATE_DECL(stalling);
    /// @brief restore to home position, fixed offset
    Q_STATE_DECL(restoring);
};  // class TiltHomeAO

inline void TiltHomeAO::Home()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::HOME_SIG);
        POST(&evt, this);
    }
}

inline void TiltHomeAO::Abort()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::ABORT_SIG);
        POST(&evt, this);
    }
}

}  // namespace stepper

#endif
