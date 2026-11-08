#ifndef IMU_AO_HPP_
#define IMU_AO_HPP_

#include "bmi270.hpp"
#include "bsp.hpp"

namespace imu
{
/// @brief IMU data event
class IMUEvt : public QP::QEvt
{
public:
    imu::IMUData data;
};

/// @brief IMU AO
class IMUAO : public QP::QActive
{
public:
    /// @brief Constructor
    IMUAO();
    IMUAO(const IMUAO&) = delete;
    IMUAO& operator=(const IMUAO&) = delete;
    IMUAO(IMUAO&&) = delete;
    IMUAO& operator=(IMUAO&&) = delete;

    /// @brief Get instance
    /// @return IMUAO&
    static IMUAO& Inst()
    {
        static IMUAO inst;
        return inst;
    }

    /// @brief Start MCAO
    /// @param priority
    /// @param id
    void Start(const QP::QPrioSpec priority, bsp::SubsystemID id);

    /// @brief Run IMU compensation
    inline void RunIMUCompensation();

    /// @brief Start IMU stream
    inline void StartIMUStream();

    /// @brief Stop IMU stream
    inline void StopIMUStream();

    /// @brief Reset IMU AO
    inline void Reset();

    /// @brief Rate control timer frequency (Hz)
    static constexpr uint32_t imuTimerFreq = 200U;
    /// @brief Rate control timer period in ticks
    static constexpr uint32_t imuTimerInterval = bsp::TICKS_PER_SEC / imuTimerFreq;

private:
    /// @brief Subsystem ID
    bsp::SubsystemID _id;
    /// @brief Event queue size
    static constexpr uint16_t _queueSize = 128U;
    /// @brief Event queue storage
    QP::QEvtPtr _queue[_queueSize] = {0};
    /// @brief Flag indicating if AO has executed initial transition
    bool _isStarted = false;
    /// @brief IMU device
    imu::BMI270 _imu;
    /// @brief IMU service timer
    QP::QTimeEvt _imuTimer;
    /// @brief IMU stream timer
    QP::QTimeEvt _imuStreamTimer;
    /// @brief IMU stream period
    static constexpr uint32_t _imuStreamTimerInterval = bsp::TICKS_PER_SEC / 10U;
    /// @brief Last IMU sample
    imu::IMUData _imuData = {0};
    /// @brief Internal fault recovery timer
    QP::QTimeEvt _faultRecoveryTimer;
    /// @brief Internal fault recovery timer period in ticks
    uint32_t _faultRecoveryTimerInterval = bsp::TICKS_PER_SEC / 100U;
    /// @brief Fault states
    bool _faultStates[imu::Fault::NUM_FAULTS] = {false};

private:
    /// @brief Private CLIAO signals
    enum PrivateSignals : QP::QSignal
    {
        INITIALIZED_SIG = bsp::PublicSignals::MAX_PUB_SIG,
        FAULT_SIG,
        RESET_SIG,
        IMU_SERVICE_SIG,
        RUN_IMU_COMPENSATION_SIG,
        START_IMU_STREAM,
        STOP_IMU_STREAM,
        IMU_STREAM_SIG,
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
};  // class IMUAO

inline void IMUAO::RunIMUCompensation()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::RUN_IMU_COMPENSATION_SIG);
        POST(&evt, this);
    }
}

inline void IMUAO::StartIMUStream()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::START_IMU_STREAM);
        POST(&evt, this);
    }
}

inline void IMUAO::StopIMUStream()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::STOP_IMU_STREAM);
        POST(&evt, this);
    }
}

inline void IMUAO::Reset()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::RESET_SIG);
        POST(&evt, this);
    }
}

}  // namespace imu

#endif
