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
    INIT_FAILED,
    NUM_FAULTS
};

/// @brief Fault code to string table
/// @param fault
/// @return
constexpr const char* FaultToStr(Fault fault)
{
    switch (fault)
    {
        case Fault::INIT_FAILED:
        {
            return "INIT_FAILED";
        }
        default:
        {
            return "";
        }
    }
}

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

    /// @brief Set a pan speed setpoint without damping (rad/s)
    inline void SetPanRateSetpointDirect(float omega);

    /// @brief Set a tilt speed setpoint without damping (rad/s)
    inline void SetTiltRateSetpointDirect(float omega);

    /// @brief Set a pan speed setpoint (rad/s)
    inline void SetPanRateSetpoint(float omega);

    /// @brief Set a tilt speed setpoint (rad/s)
    inline void SetTiltRateSetpoint(float omega);

    /// @brief Start pan homing sequence
    inline void HomePan();

    /// @brief Start tilt homing sequence
    inline void HomeTilt();

    /// @brief Enable pan motor
    inline void EnablePan();

    /// @brief Enable tilt motor
    inline void EnableTilt();

    /// @brief Disable pan motor
    inline void DisablePan();

    /// @brief Disable tilt motor
    inline void DisableTilt();

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

    /// @brief Pan stepper driver
    a4988::A4988 _panDriver;
    /// @brief Tilt stepper driver
    a4988::A4988 _tiltDriver;

private:
    /// @brief Private CLIAO signals
    enum PrivateSignals : QP::QSignal
    {
        INITIALIZED_SIG = bsp::PublicSignals::MAX_PUB_SIG,
        FAULT_SIG,
        SET_PAN_RATE_DIRECT_SIG,
        SET_TILT_RATE_DIRECT_SIG,
        HOME_PAN_SIG,
        HOME_TILT_SIG,
        ENABLE_PAN_SIG,
        ENABLE_TILT_SIG,
        DISABLE_PAN_SIG,
        DISABLE_TILT_SIG,
        RESET_SIG,
        MAX_PRIV_SIG
    };

    /// @brief Set angular rate evt
    class SetRateEvt : public QP::QEvt
    {
    public:
        SetRateEvt(QP::QSignal sig) : QP::QEvt(sig) {}
        float omega;
    };

    /// @brief Set and publish fault
    void SetFault(bsp::SubsystemID subsystem, uint8_t fault, bool active);

    /// @brief Initial state
    Q_STATE_DECL(initial);
    /// @brief Root state
    Q_STATE_DECL(root);
    /// @brief Initialize
    Q_STATE_DECL(initializing);
    /// @brief Active
    Q_STATE_DECL(active);
    /// @brief Fault
    Q_STATE_DECL(error);
};  // class TurretAO

inline void TurretAO::SetPanRateSetpointDirect(float omega)
{
    if (_isStarted)
    {
        SetRateEvt* evt = Q_NEW(SetRateEvt, PrivateSignals::SET_PAN_RATE_DIRECT_SIG);
        evt->omega = omega;
        POST(evt, this);
    }
}

inline void TurretAO::SetTiltRateSetpointDirect(float omega)
{
    if (_isStarted)
    {
        SetRateEvt* evt = Q_NEW(SetRateEvt, PrivateSignals::SET_TILT_RATE_DIRECT_SIG);
        evt->omega = omega;
        POST(evt, this);
    }
}

inline void TurretAO::HomePan()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::HOME_PAN_SIG);
        POST(&evt, this);
    }
}

inline void TurretAO::HomeTilt()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::HOME_TILT_SIG);
        POST(&evt, this);
    }
}

inline void TurretAO::EnablePan()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::ENABLE_PAN_SIG);
        POST(&evt, this);
    }
}

inline void TurretAO::EnableTilt()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::ENABLE_TILT_SIG);
        POST(&evt, this);
    }
}

inline void TurretAO::DisablePan()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::DISABLE_PAN_SIG);
        POST(&evt, this);
    }
}

inline void TurretAO::DisableTilt()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::DISABLE_TILT_SIG);
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
