#include "main.h"

#include <atomic>
#include <cstring>

#include "can.h"
#include "crc.h"
#include "cstdint"
#include "gpio.h"
#include "stm32f3xx_it.h"
#include "tim.h"

/// @brief Number of ticks per second
constexpr uint32_t TICKS_PER_SEC {1000U};  // NOLINT
/// @brief Fault LED blink period in ms
constexpr uint32_t BLINK_PERIOD = 500U;  // NOLINT
/// @brief Payload chunk size in bytes, 8K
constexpr uint32_t CHUNK_SIZE = 0x2000U;  // NOLINT
/// @brief Max time to wait in idle state before booting back into main app in s
constexpr uint32_t IDLE_TIMEOUT = 5U;

/// @brief Flash max address
constexpr uint32_t FLASH_END = 0x8020000U;

/// @brief Main application flash address
constexpr uint32_t APP_FLASH_START = 0x8000000U;
/// @brief Main application flash section address, 64K
constexpr uint32_t APP_FLASH_SIZE = 0x10000U;

/// @brief Swap flash start address
constexpr uint32_t SWAP_FLASH_START = 0x8014000U;
/// @brief Main application flash section address, 48K (supplement with RAM)
constexpr uint32_t SWAP_FLASH_SIZE = 0xC000;
/// @brief Swap flash current offset
uint32_t swapFlashOffset = 0U;

/// @brief Latest chunk data
uint8_t chunk[CHUNK_SIZE];
/// @brief Latest chunk write write offset
uint32_t chunkOffset = 0U;

// We should only be accessing CAN Tx data in the Rx interrupts and those should pend on reentrance
// No need to serialize access.
/// @brief CAN TX header
CAN_TxHeaderTypeDef canTxHeader;
/// @brief CAN TX data
uint8_t canTxData[8];
/// @brief CAN TX mailbox
uint32_t canTxMailbox;

/// @brief CAN firmware loader states
enum State
{
    IDLE,       // waiting for request to transmit
    RECEIVING,  // receiving payload
    WRITING,    // writing to flash
    FAULT       // bad
};

/// @brief Current state
std::atomic<State> state(State::IDLE);

/// @brief CAN subscribed ids/opcodes base
constexpr uint16_t CAN_SUB_ID_BASE = 0x780;  // NOLINT
/// @brief CAN ids/opcodes
enum SubCANID
{
    REQUEST_START = CAN_SUB_ID_BASE,
    CHUNK,
    CHUNK_END,
    CHUNK_END_LAST,
    NUM_SUB_IDS
};

/// @brief CAN published ids/opcodes base
constexpr uint16_t CAN_PUB_ID_BASE = 0x7D0;  // NOLINT
/// @brief CAN ids/opcodes
enum PubCANID
{
    REQUEST_START_ACK_INFO = CAN_PUB_ID_BASE,
    NEXT_CHUNK,
    COMPLETE,
    NO_SPACE_IN_CHUNK,
    NO_SPACE_IN_SWAP,
    CHUNK_CRC_FAIL,
    FLASH_ERASE_FAIL,
    FLASH_PROGRAM_FAIL,
    NON_FINAL_CHUNK_NOT_MAXSIZE,
    NUM_PUB_IDS
};

/// @brief CAN interrupt handler
extern "C" void USB_LP_CAN_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan);
}

typedef void (*app_entry_t)(void);
/// @brief Jump to main application
/// @param
void JumpToMain()
{
    uint32_t app_msp = *(volatile uint32_t*)(APP_FLASH_START);

    // stop interrupts and everything else HAL is doing
    HAL_DeInit();

    // stop systick
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL = 0U;

    // disable nvic interrupts
    for (uint32_t i = 0U; i < 8U; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    __disable_irq();

    // point vector table and stack ptr to main application
    SCB->VTOR = APP_FLASH_START;
    __DSB();
    __ISB();

    __set_MSP(app_msp);

    uint32_t app_reset = *(volatile uint32_t*)(APP_FLASH_START + 4U);
    ((app_entry_t)app_reset)();
}

/// @brief Handle incoming can message in idle state
/// @param header
/// @param data
void HandleIdle(const CAN_RxHeaderTypeDef& header, const uint8_t* const data)
{
    switch (header.StdId)
    {
        case SubCANID::REQUEST_START:
        {
            // other device has asked to start netflash
            /// TODO: probably require a password...
            /// TODO: later add signing verification...

            // send ack with chunk size
            canTxHeader.DLC = sizeof(uint32_t);
            canTxHeader.StdId = PubCANID::REQUEST_START_ACK_INFO;
            memcpy(canTxData, &CHUNK_SIZE, sizeof(uint32_t));
            if (HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &canTxMailbox) != HAL_OK)
            {
                state = State::FAULT;
                break;
            }

            // On sending the ack, enter receiving state
            state = State::RECEIVING;
            break;
        }
        default: break;
    }
}

/// @brief Handle incoming can message in receiving state
/// @param header
/// @param data
void HandleReceiving(const CAN_RxHeaderTypeDef& header, const uint8_t* const data)
{
    switch (header.StdId)
    {
        case SubCANID::CHUNK:
        {
            // check to make sure we don't overrun
            if (chunkOffset + header.DLC > CHUNK_SIZE)
            {
                // send fault
                canTxHeader.DLC = 0U;
                canTxHeader.StdId = PubCANID::NO_SPACE_IN_CHUNK;
                HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &canTxMailbox);

                state = State::FAULT;
                return;
            }

            // regular contribution to a chunk
            memcpy(&chunk + chunkOffset, data, header.DLC);
            chunkOffset += header.DLC;
            break;
        }
        case SubCANID::CHUNK_END:
        case SubCANID::CHUNK_END_LAST:  // like chunk end, but last chunk in series so just keep it in ram
        {
            if (header.DLC == 4)
            {
                // change state to writing immedaitely
                state = State::WRITING;
                // we should not receive new chunk messages during writing
                // the other device should know to wait until the go ahead to send another chunk
                // if it breaks that rule, the net loader won't suffer for it

                // end a chunk with a checksum
                /// TODO: disable for now for debugging
                /*
                uint32_t crc_local = HAL_CRC_Calculate(&hcrc, (uint32_t*)chunk, chunkOffset);
                uint32_t crc_remote;
                memcpy(&crc_remote, data, header.DLC);

                // compare
                if (crc_local != crc_remote)
                {
                    // send fault
                    canTxHeader.DLC = 0U;
                    canTxHeader.StdId = PubCANID::CHUNK_CRC_FAIL;
                    HAL_CAN_AddTxMessage(&hcan, &_canTxHeader, _canTxData, &_canTxMailbox);

                    state = State::FAULT;
                    break;
                }
                */

                // if this is not the last chunk in payload, save to flash
                if (header.StdId == SubCANID::CHUNK_END)
                {
                    if (SWAP_FLASH_START + swapFlashOffset + chunkOffset > FLASH_END)
                    {
                        // send fault
                        canTxHeader.DLC = 0U;
                        canTxHeader.StdId = PubCANID::NO_SPACE_IN_SWAP;
                        HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &canTxMailbox);

                        state = State::FAULT;
                        return;
                    }

                    // non final chunks should be maximum chunk size
                    if (chunkOffset != CHUNK_SIZE)
                    {
                        // send fault
                        canTxHeader.DLC = 0U;
                        canTxHeader.StdId = PubCANID::NON_FINAL_CHUNK_NOT_MAXSIZE;
                        HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &canTxMailbox);

                        state = State::FAULT;
                        return;
                    }

                    FLASH_EraseInitTypeDef erase = {0};
                    erase.TypeErase = FLASH_TYPEERASE_PAGES;
                    erase.PageAddress = SWAP_FLASH_START + swapFlashOffset;
                    // This assumes each non-final chunk is the same (maximum) size
                    erase.NbPages = CHUNK_SIZE / FLASH_PAGE_SIZE;

                    HAL_FLASH_Unlock();

                    // erase flash in chunk first
                    uint32_t page_error;
                    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
                    {
                        HAL_FLASH_Lock();

                        // send fault
                        canTxHeader.DLC = 0U;
                        canTxHeader.StdId = PubCANID::FLASH_ERASE_FAIL;
                        HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &canTxMailbox);

                        state = State::FAULT;
                        return;
                    }

                    // write chunk to flash
                    for (uint32_t i = 0U; i < CHUNK_SIZE; i += 4)
                    {
                        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, SWAP_FLASH_START + swapFlashOffset + i,
                                              *(uint32_t*)(chunk + i)) != HAL_OK)
                        {
                            HAL_FLASH_Lock();

                            // send fault
                            canTxHeader.DLC = 0U;
                            canTxHeader.StdId = PubCANID::FLASH_PROGRAM_FAIL;
                            HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &canTxMailbox);

                            state = State::FAULT;
                            return;
                        }
                    }

                    HAL_FLASH_Lock();

                    // move flash offset ahead
                    swapFlashOffset += chunkOffset;

                    // clear chunk, reset chunk offset
                    memset(chunk, 0U, sizeof(chunk));
                    chunkOffset = 0U;

                    // Go back to listening
                    state = State::RECEIVING;

                    // send go-ahead for next chunk
                    canTxHeader.DLC = 0U;
                    canTxHeader.StdId = PubCANID::NEXT_CHUNK;
                    if (HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &canTxMailbox) != HAL_OK)
                    {
                        state = State::FAULT;
                        return;
                    }
                }
                else if (header.StdId == SubCANID::CHUNK_END_LAST)
                {
                    // this is the last chunk, so finish by copying everything from swap flash and ram into main
                    // application region little by little

                    // first erase all of application section
                    FLASH_EraseInitTypeDef erase = {0};
                    erase.TypeErase = FLASH_TYPEERASE_PAGES;
                    erase.PageAddress = APP_FLASH_START;
                    erase.NbPages = APP_FLASH_SIZE / FLASH_PAGE_SIZE;

                    HAL_FLASH_Unlock();

                    uint32_t page_error;
                    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
                    {
                        HAL_FLASH_Lock();

                        // send fault
                        canTxHeader.DLC = 0U;
                        canTxHeader.StdId = PubCANID::FLASH_ERASE_FAIL;
                        HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &canTxMailbox);

                        state = State::FAULT;
                        return;
                    }

                    // Copy swap contents into main application code page by page
                    uint8_t buffer[FLASH_PAGE_SIZE];
                    for (uint32_t offset = 0U; offset < swapFlashOffset; offset += FLASH_PAGE_SIZE)
                    {
                        // copy page into ram
                        memcpy(buffer, (const uint8_t*)(SWAP_FLASH_START + offset), FLASH_PAGE_SIZE);

                        // write to application region
                        for (uint32_t i = 0U; i < FLASH_PAGE_SIZE; i += 4U)
                        {
                            uint32_t word;
                            memcpy(&word, &buffer[i], sizeof(word));

                            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, APP_FLASH_START + offset + i, word) != HAL_OK)
                            {
                                HAL_FLASH_Lock();

                                // send fault
                                canTxHeader.DLC = 0U;
                                canTxHeader.StdId = PubCANID::FLASH_PROGRAM_FAIL;
                                HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &canTxMailbox);

                                state = State::FAULT;
                                return;
                            }
                        }
                    }

                    // Copy ram contents into main application code
                    for (uint32_t i = 0U; i < CHUNK_SIZE; i += 4U)
                    {
                        uint32_t word;
                        memcpy(&word, &chunk[i], sizeof(word));

                        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, APP_FLASH_START + swapFlashOffset + i, word) !=
                            HAL_OK)
                        {
                            HAL_FLASH_Lock();

                            // send fault
                            canTxHeader.DLC = 0U;
                            canTxHeader.StdId = PubCANID::FLASH_PROGRAM_FAIL;
                            HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &canTxMailbox);

                            state = State::FAULT;
                            return;
                        }
                    }

                    HAL_FLASH_Lock();

                    // Finished
                    canTxHeader.DLC = 0U;
                    canTxHeader.StdId = PubCANID::COMPLETE;
                    HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &canTxMailbox);

                    HAL_Delay(10);
                    // Reset to main application
                    JumpToMain();
                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

/// @brief Handle incoming can message in writing state
/// @param header
/// @param data
void HandleWriting(const CAN_RxHeaderTypeDef& header, const uint8_t* const data) {}

/// @brief Handle incoming can message in fault state
/// @param header
/// @param data
void HandleFault(const CAN_RxHeaderTypeDef& header, const uint8_t* const data) {}

/// @brief CAN receive fifo message callback
/// @param hcan
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) == HAL_OK)
    {
        /// TODO: Only standard IDs supported
        if (header.IDE == CAN_ID_STD)
        {
            switch (state)
            {
                case State::IDLE:
                {
                    HandleIdle(header, data);
                    break;
                }
                case State::RECEIVING:
                {
                    HandleReceiving(header, data);
                    break;
                }
                case State::WRITING:
                {
                    HandleWriting(header, data);
                    break;
                }
                case State::FAULT:
                {
                    HandleFault(header, data);
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
    }
}

void SystemClock_Config(void);
int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    // Initialize CAN bus and CRC
    MX_CAN_Init();
    MX_CRC_Init();

    // Start systick
    SysTick_Config(SystemCoreClock / TICKS_PER_SEC);

    // Set up CAN filters
    CAN_FilterTypeDef can_filter;
    can_filter.FilterBank = 0;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = CAN_SUB_ID_BASE << 5U;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = CAN_SUB_ID_BASE << 5U;
    can_filter.FilterMaskIdLow = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation = ENABLE;

    // Initialize can tx header and tx buffer
    canTxHeader.ExtId = 0x00;
    canTxHeader.IDE = CAN_ID_STD;
    canTxHeader.RTR = CAN_RTR_DATA;
    canTxHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK) { state = State::FAULT; }

    // Start CAN peripheral
    if (HAL_CAN_Start(&hcan) != HAL_OK) { state = State::FAULT; }

    // Enable recieve interrupt
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) { state = State::FAULT; }

    // CAN RX interrupt
    HAL_NVIC_SetPriority(USB_LP_CAN_RX0_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);

    // Computing CRC for byte array
    hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;

    // Listen for a request to transmit an update package. If a request to transmit does not come within 5 seconds, boot
    // back into main application. Once request received, respond with ack and maximum payload chunk size and start
    // expecting payload. Continuously load payload into buffer and expect a packet containing section_end code and
    // checksum at the end. On validation of buffer, write contents to flash swap region. Once done writing, send
    // continue to the other device and start waiting for next payload. Repeat until other device sends an end code.

    // On receiving end code from other device and firmware checksums validated, copy contents of flash swap region to
    // main application region and jump

    // The stm32f302 only has 32 kb of ram, and firmware images will be larger than that
    // The HTC does not have nvmemory aside from flash
    // We will thus use some part of flash as swap memory...
    /// TODO: add eeprom 25LC256 to next revision
    /// TODO: also consider compression

    // idle counter start
    uint32_t idleStart = HAL_GetTick();

    auto blink_state = GPIO_PIN_SET;
    for (;;)
    {
        if (state == State::IDLE)
        {
            if (HAL_GetTick() - idleStart > IDLE_TIMEOUT * TICKS_PER_SEC)
            {
                // trigger reset and go back to main application
                NVIC_SystemReset();
            }
        }

        // blinking LED represents board is in can load
        // blinking fast means something is wrong
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, blink_state);
        blink_state = (GPIO_PinState)!blink_state;
        if (state == State::FAULT) { HAL_Delay(BLINK_PERIOD / 10); }
        else { HAL_Delay(BLINK_PERIOD); }
    }

    return 0;
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) { Error_Handler(); }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}

/// @brief Application error handler callback
// TODO: Get rid of this
void Error_Handler(void)
{
    __disable_irq();
    for (;;) {}
}
