#include "cli_ao.hpp"

#include <stdarg.h>

#include "cli_binding.hpp"
#include "thirdparty/printf.h"

// Define UART interrupt callback function to handle a received character with CLI
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    cli::CLIAO::Inst().ReceiveChar(huart);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART2)
    {
        // clear overrun fault if set
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) { __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF); }

        // restart listening
        cli::CLIAO::Inst().ReceiveChar(huart);
    }
}

cli::CLIAO::CLIAO() : QP::QActive(&initial), _processTimer(this, PrivateSignals::PROCESS_SIG, 0U) {}

void cli::CLIAO::Start(const QP::QPrioSpec priority, bsp::SubsystemID id)
{
    _id = id;
    _isStarted = true;
    this->start(priority,      // QP prio. of the AO
                _queue,        // event queue storage
                _queueSize,    // queue size [events]
                nullptr, 0U);  // no stack storage
}

void cli::CLIAO::Printf(const char* fmt, ...)
{
    if (_isStarted)
    {
        PrintEvt* evt = Q_NEW(PrintEvt, PrivateSignals::PRINT_SIG);

        // Format the string using snprintf
        va_list args;
        va_start(args, fmt);
        int length = vsnprintf(evt->buf, sizeof(evt->buf), fmt, args);
        va_end(args);

        // Check if string fits in buffer
        // TODO: automatically blockify and inject multiple events if
        //  buffer size exceeded
        if (length > 0) { CLIAO::Inst().POST(evt, this); }
        else
        {
            // Otherwise, gc the event so it doesn't leak
            QP::QF::gc(evt);
        }
    }
}

void cli::CLIAO::ReceiveChar(UART_HandleTypeDef* huart)
{
    if (huart == _uartCliPeriph && _isStarted)
    {
        static QP::QEvt evt(PrivateSignals::RX_CHAR_SIG);
        POST(&evt, this);
    }
}

Q_STATE_DEF(cli::CLIAO, initial)
{
    Q_UNUSED_PAR(e);
    // Subscribe to signals
    subscribe(bsp::PublicSignals::REQUEST_FAULT_SIG);
    return tran(&initializing);
}

Q_STATE_DEF(cli::CLIAO, root)
{
    QP::QState status_;
    switch (e->sig)
    {
        case PrivateSignals::RESET_SIG:
        {
            status_ = tran(&initializing);
            break;
        }
        case bsp::PublicSignals::REQUEST_FAULT_SIG:
        {
            // Publish all fault states
            for (uint8_t fault = 0U; fault < Fault::NUM_FAULTS; fault++)
            {
                bsp::FaultEvt* evt = Q_NEW(bsp::FaultEvt, bsp::PublicSignals::FAULT_SIG);
                evt->id = _id;
                evt->fault = fault;
                evt->active = _faultStates[fault];
                PUBLISH(evt, this);
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

Q_STATE_DEF(cli::CLIAO, initializing)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // Initialize the CLI configuration settings
            EmbeddedCliConfig* config = embeddedCliDefaultConfig();
            config->cliBuffer = _cliBuf;
            config->cliBufferSize = _cliBufSize;
            config->rxBufferSize = _cliRxBufSize;
            config->cmdBufferSize = _cliCmdBufSize;
            config->historyBufferSize = _cliHistorySize;
            config->maxBindingCount = _cliMaxBindingCount;

            // Create new CLI instance
            _cli = embeddedCliNew(config);

            // CLI init failed - most likely not enough memory
            if (_cli == NULL)
            {
                static QP::QEvt evt(PrivateSignals::FAULT_SIG);
                POST(&evt, this);
            }
            else
            {
                // Init succeeded
                static QP::QEvt evt(PrivateSignals::INITIALIZED_SIG);
                POST(&evt, this);
            }

            // Register character write function
            auto write_char_to_cli = [](EmbeddedCli* embeddedCli, char c) {
                uint8_t char_to_send = c;
                HAL_UART_Transmit(_uartCliPeriph, &char_to_send, 1, 100);
            };
            _cli->writeChar = write_char_to_cli;

            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::INITIALIZED_SIG:
        {
            // Add all the initial command bindings
            InitBindings(_cli);
            // Clear the cli
            onClear(_cli, NULL, NULL);
            // Start processing
            status_ = tran(&active);
            break;
        }
        case PrivateSignals::FAULT_SIG:
        {
            // Publish fault
            _faultStates[Fault::INIT_FAILED] = true;
            bsp::FaultEvt* evt = Q_NEW(bsp::FaultEvt, bsp::PublicSignals::FAULT_SIG);
            evt->id = _id;
            evt->fault = Fault::INIT_FAILED;
            evt->active = _faultStates[Fault::INIT_FAILED];
            PUBLISH(evt, this);
            status_ = tran(&error);
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

Q_STATE_DEF(cli::CLIAO, active)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // Arm uart receive interrupt
            HAL_UART_Receive_IT(_uartCliPeriph, _uartRxBuf, _uartRxBufSize);
            // Arm cli process timer
            _processTimer.armX(_processInterval, _processInterval);
            status_ = Q_RET_HANDLED;
            break;
        }
        case Q_EXIT_SIG:
        {
            // Disarm cli process timer
            _processTimer.disarm();
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::RX_CHAR_SIG:
        {
            // Receive char and rearm interrupt
            HAL_UART_Receive_IT(_uartCliPeriph, _uartRxBuf, _uartRxBufSize);
            embeddedCliReceiveChar(_cli, _uartRxBuf[0]);
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::PROCESS_SIG:
        {
            // Service CLI once
            embeddedCliProcess(_cli);
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::PRINT_SIG:
        {
            // Call embeddedCliPrint with the formatted string
            embeddedCliPrint(_cli, Q_EVT_CAST(PrintEvt)->buf);
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::PARAMS_UPDATED:
        {
            // Rearm cli process timer
            _processTimer.disarm();
            _processTimer.armX(_processInterval, _processInterval);
            status_ = super(&root);
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

Q_STATE_DEF(cli::CLIAO, error)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_EXIT_SIG:
        {
            // Clear all faults on exit
            for (uint8_t fault = 0U; fault < Fault::NUM_FAULTS; fault++)
            {
                if (_faultStates[fault])
                {
                    _faultStates[fault] = false;
                    bsp::FaultEvt* evt = Q_NEW(bsp::FaultEvt, bsp::PublicSignals::FAULT_SIG);
                    evt->id = _id;
                    evt->fault = fault;
                    evt->active = _faultStates[fault];
                    PUBLISH(evt, this);
                }
            }
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
