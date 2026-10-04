#ifndef CONTROL_AO_HPP_
#define CONTROL_AO_HPP_

#include "bsp.hpp"

namespace control
{
/// @brief Fault codes
enum Fault : uint8_t
{
    NO_FAULT = 0U,
    CONTROL_INIT_FAILED,
    CONTROL_CAN_TX_FAILED,
    WATCHDOG_FAULT,
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
        case Fault::CONTROL_INIT_FAILED:
        {
            return "CONTROL_INIT_FAILED";
        }
        case Fault::CONTROL_CAN_TX_FAILED:
        {
            return "CONTROL_CAN_TX_FAILED";
        }
        case Fault::WATCHDOG_FAULT:
        {
            return "WATCHDOG_FAULT";
        }
        default:
        {
            return "";
        }
    }
}

/// @brief Control AO
class ControlAO : public QP::QActive
{
public:
    /// @brief Constructor
    ControlAO();
    ~ControlAO() = default;
    ControlAO(const ControlAO&) = delete;
    ControlAO& operator=(const ControlAO&) = delete;
    ControlAO(ControlAO&&) = delete;
    ControlAO& operator=(ControlAO&&) = delete;

    /// @brief Get instance
    /// @return ControlAO&
    static ControlAO& Inst()
    {
        static ControlAO inst;
        return inst;
    }

    /// @brief Start MCAO
    /// @param priority
    /// @param id
    void Start(const QP::QPrioSpec priority, bsp::SubsystemID id);  // NOLINT

    /// @brief Reset control AO
    inline void Reset();

    /// @brief Print system fault state
    inline void PrintFault();

    /// @brief Poke watchdog
    inline void PokeWatchdog();

    /// @brief Enable watchdog
    inline void EnableWatchdog();

    /// @brief Disable watchdog
    inline void DisableWatchdog();

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
    /// @brief Fault request timer
    QP::QTimeEvt _faultRequestTimer;
    /// @brief Fault request timer period in ticks
    uint32_t _faultRequestTimerInterval = bsp::TICKS_PER_SEC / 20U;
    /// @brief Latest faults from all subsystems, including control subsystem
    bool _faultStates[bsp::SubsystemID::NUM_SUBSYSTEMS][bsp::MAX_SUBSYSTEM_FAULTS] = {0};

    /// @brief CAN TX header
    CAN_TxHeaderTypeDef _canTxHeader;
    /// @brief CAN TX data
    uint8_t _canTxData[8];
    /// @brief CAN TX mailbox
    uint32_t _canTxMailbox;

    /// @brief Heartbeat CAN publish timer
    QP::QTimeEvt _heartbeatTimer;
    /// @brief Heartbeat CAN publish timer period
    static constexpr uint32_t _heartbeatTimerInterval = bsp::TICKS_PER_SEC / 5U;

    /// @brief Watchdog timer
    QP::QTimeEvt _watchdogTimer;
    /// @brief Watchdog timer period
    static constexpr uint32_t _watchdogTimerInterval = bsp::TICKS_PER_SEC / 1U;
    /// @brief Whether or not watchdog is enabled
    bool _watchdogEnable = false;

    /// @brief Track if this AO has successfully initialized once. Certain steps in initialization should be skipped
    /// after first time.
    bool _hasFirstTimeInit = false;

private:  // NOLINT
    /// @brief Private CLIAO signals
    enum PrivateSignals : QP::QSignal
    {
        INITIALIZED_SIG = bsp::PublicSignals::MAX_PUB_SIG,
        FAULT_SIG,
        RESET_SIG,
        SUBS_FAULT_REQUEST_SIG,
        PRINT_FAULT_SIG,
        HEARTBEAT_SIG,
        POKE_WATCHDOG_SIG,
        WATCHDOG_EXPIRED_SIG,
        ENABLE_WATCHDOG_SIG,
        DISABLE_WATCHDOG_SIG,
        MAX_PRIV_SIG
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
    /// @brief Protect the platform. Entered when watchdog expires.
    Q_STATE_DECL(selfprotect);
};  // class ControlAO

inline void ControlAO::Reset()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::RESET_SIG);
        POST(&evt, this);
    }
}

inline void ControlAO::PrintFault()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::PRINT_FAULT_SIG);
        POST(&evt, this);
    }
}

inline void ControlAO::PokeWatchdog()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::POKE_WATCHDOG_SIG);
        POST(&evt, this);
    }
}

inline void ControlAO::EnableWatchdog()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::ENABLE_WATCHDOG_SIG);
        POST(&evt, this);
    }
}

inline void ControlAO::DisableWatchdog()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::DISABLE_WATCHDOG_SIG);
        POST(&evt, this);
    }
}
}  // namespace control

#endif
