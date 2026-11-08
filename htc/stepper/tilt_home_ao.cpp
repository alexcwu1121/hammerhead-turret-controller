#include "tilt_home_ao.hpp"

#include "bsp.hpp"
#include "cli_ao.hpp"
#include "qpcpp.hpp"
#include "stepper_ao.hpp"

namespace stepper
{
TiltHomeAO::TiltHomeAO() :
    QP::QActive(&initial),
    _traverseTimer(this, PrivateSignals::TRAVERSE_TIMEOUT_SIG),
    _stallDetectTimer(this, PrivateSignals::STALL_SIG)
{}

void TiltHomeAO::Start(const QP::QPrioSpec priority, bsp::SubsystemID id)
{
    _id = id;
    _isStarted = true;
    this->start(priority,      // QP prio. of the AO
                _queue,        // event queue storage
                _queueSize,    // queue size [events]
                nullptr, 0U);  // no stack storage
}

Q_STATE_DEF(TiltHomeAO, initial)
{
    Q_UNUSED_PAR(e);
    subscribe(bsp::PublicSignals::STEPPER_MODE_CHANGED_SIG);
    subscribe(bsp::PublicSignals::STEPPER_ENC_STATE_SIG);
    return tran(&idle);
}

Q_STATE_DEF(TiltHomeAO, root)
{
    QP::QState status_;
    switch (e->sig)
    {
        case bsp::PublicSignals::STEPPER_MODE_CHANGED_SIG:
        {
            if (Q_EVT_CAST(bsp::StepperModeChangedEvt)->id == bsp::SubsystemID::TILT_STEPPER_SUBSYSTEM)
            {
                // Update the last observed mode
                _lastObservedMode = static_cast<stepper::Mode>(Q_EVT_CAST(bsp::StepperModeChangedEvt)->mode);
            }

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

Q_STATE_DEF(TiltHomeAO, idle)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::HOME_SIG:
        {
            status_ = tran(&traversing);
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

Q_STATE_DEF(TiltHomeAO, homing)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // Ignore this homing request if tilt stepper isn't ready yet
            if (_lastObservedMode == stepper::Mode::NUM_MODES)
            {
                static QP::QEvt evt(PrivateSignals::ABORT_SIG);
                POST(&evt, this);

                status_ = Q_RET_HANDLED;
                break;
            }

            // remember what mode tilt stepper was in before homing so it can be restored
            _lastObservedModePreHoming = _lastObservedMode;

            status_ = Q_RET_HANDLED;
            break;
        }
        case Q_EXIT_SIG:
        {
            // restore tilt stepper to the mode it was in before homing
            switch (_lastObservedModePreHoming)
            {
                case stepper::Mode::OPEN_LOOP:
                {
                    StepperAO::TiltInst().SetMode(stepper::Mode::OPEN_LOOP);
                    break;
                }
                case stepper::Mode::CLOSED_LOOP:
                {
                    StepperAO::TiltInst().SetMode(stepper::Mode::CLOSED_LOOP);
                    break;
                }
                case stepper::Mode::CLOSED_LOOP_POS:
                {
                    StepperAO::TiltInst().SetMode(stepper::Mode::CLOSED_LOOP_POS);
                    break;
                }
                default:
                {
                    // we don't know somehow? it will be left in whatever mode homing leaves it in
                    break;
                }
            }

            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::ABORT_SIG:
        {
            status_ = tran(&idle);
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

Q_STATE_DEF(TiltHomeAO, traversing)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // start one-shot traverse watchdog timer
            _traverseTimer.armX(_traverseTimeout, 0U);

            // set max speed in positive direction and listen for a stall event
            StepperAO::TiltInst().SetMode(stepper::Mode::CLOSED_LOOP);
            StepperAO::TiltInst().SetRateSetpoint(maxRate);

            status_ = Q_RET_HANDLED;
            break;
        }
        case Q_EXIT_SIG:
        {
            // stop traverse watchdog timer
            _traverseTimer.disarm();

            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::TRAVERSE_TIMEOUT_SIG:
        {
            // jam motor to 0 speed
            StepperAO::TiltInst().SetRateSetpoint(0.0f);

            // Report fault
            bsp::FaultEvt* evt = Q_NEW(bsp::FaultEvt, bsp::PublicSignals::FAULT_SIG);
            evt->id = _id;
            evt->fault = TiltHomeFault::TILT_HOME_FAILED;
            evt->active = true;
            PUBLISH(evt, this);

            status_ = tran(&idle);
            break;
        }
        case bsp::PublicSignals::STEPPER_ENC_STATE_SIG:
        {
            status_ = Q_RET_HANDLED;
            if (Q_EVT_CAST(bsp::StepperEncStateEvt)->id == bsp::SubsystemID::TILT_STEPPER_SUBSYSTEM)
            {
                // if motor speed drops below threshold, transition to counting down the stall
                if (Q_EVT_CAST(bsp::StepperEncStateEvt)->rate < _stallDetectThresholdRate)
                {
                    status_ = tran(&stalling);
                }
            }
            break;
        }
        default:
        {
            status_ = super(&homing);
            break;
        }
    }
    return status_;
}

Q_STATE_DEF(TiltHomeAO, stalling)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // start one-shot stall detect timer
            _stallDetectTimer.armX(_stallDetectThresholdTime, 0U);

            status_ = Q_RET_HANDLED;
            break;
        }
        case Q_EXIT_SIG:
        {
            // stop stall detect timer
            _stallDetectTimer.disarm();

            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::STALL_SIG:
        {
            // progress to restoring state to home the stepper
            status_ = tran(&restoring);
            break;
        }
        case bsp::PublicSignals::STEPPER_ENC_STATE_SIG:
        {
            status_ = Q_RET_HANDLED;
            if (Q_EVT_CAST(bsp::StepperEncStateEvt)->id == bsp::SubsystemID::TILT_STEPPER_SUBSYSTEM)
            {
                // if motor speed exceeds threshold, go back to traversing
                if (Q_EVT_CAST(bsp::StepperEncStateEvt)->rate > _stallDetectThresholdRate)
                {
                    status_ = tran(&traversing);
                }
            }
            break;
        }
        default:
        {
            status_ = super(&traversing);
            break;
        }
    }
    return status_;
}

Q_STATE_DEF(TiltHomeAO, restoring)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // enter position mode and move to calibrated home position and wait until settled
            StepperAO::TiltInst().SetMode(stepper::Mode::CLOSED_LOOP_POS);
            StepperAO::TiltInst().SetPositionSetpoint(_homeOffset);

            status_ = Q_RET_HANDLED;
            break;
        }
        case bsp::PublicSignals::STEPPER_ENC_STATE_SIG:
        {
            status_ = Q_RET_HANDLED;
            if (Q_EVT_CAST(bsp::StepperEncStateEvt)->id == bsp::SubsystemID::TILT_STEPPER_SUBSYSTEM)
            {
                // finish homing when stepper rate is below threshold
                if (Q_EVT_CAST(bsp::StepperEncStateEvt)->rate < _homeRateAcceptanceThreshold)
                {
                    // Report success
                    bsp::FaultEvt* evt = Q_NEW(bsp::FaultEvt, bsp::PublicSignals::FAULT_SIG);
                    evt->id = _id;
                    evt->fault = TiltHomeFault::TILT_HOME_FAILED;
                    evt->active = false;
                    PUBLISH(evt, this);

                    // set new home position
                    StepperAO::TiltInst().SetHome(Q_EVT_CAST(bsp::StepperEncStateEvt)->absPos);

                    status_ = tran(&idle);
                }
            }
            break;
        }
        default:
        {
            status_ = super(&homing);
            break;
        }
    }
    return status_;
}
}  // namespace stepper
