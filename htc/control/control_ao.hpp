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

private:  // NOLINT
    /// @brief Private CLIAO signals
    enum PrivateSignals : QP::QSignal
    {
        INITIALIZED_SIG = bsp::PublicSignals::MAX_PUB_SIG,
        FAULT_SIG,
        RESET_SIG,
        SUBS_FAULT_REQUEST_SIG,
        PRINT_FAULT_SIG,
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
}  // namespace control

#endif
