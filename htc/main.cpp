#include "bsp.hpp"
#include "cli_ao.hpp"
#include "control_ao.hpp"
#include "imu_ao.hpp"
#include "stepper_ao.hpp"
#include "tilt_home_ao.hpp"

int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_SPI3_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_CAN_Init();
    MX_TIM1_Init();
    MX_TIM17_Init();
    MX_USART2_UART_Init();

    // Init event pools
    static QF_MPOOL_EL(QP::QEvt) smlPoolSto[50];  // small (bare signals)
    QP::QF::poolInit(smlPoolSto, sizeof(smlPoolSto), sizeof(smlPoolSto[0]));
    static uint8_t mdPoolSto[50][32];  // medium (average data packets)
    QP::QF::poolInit(mdPoolSto, sizeof(mdPoolSto), sizeof(mdPoolSto[0]));
    static uint8_t lgPoolSto[7][512];  // large (logs or text)
    QP::QF::poolInit(lgPoolSto, sizeof(lgPoolSto), sizeof(lgPoolSto[0]));

    // Init publish-subscribe signals
    static QP::QSubscrList subscrSto[bsp::PublicSignals::MAX_PUB_SIG];
    QP::QActive::psInit(subscrSto, Q_DIM(subscrSto));

    // Init QF scheduler
    QP::QF::init();

    // Start AOs
    // ParamAO is highest priority because it doesn't do much + must start before everything else
    cli::CLIAO::Inst().Start(1U, bsp::SubsystemID::CLI_SUBSYSTEM);
    imu::IMUAO::Inst().Start(2U, bsp::SubsystemID::IMU_SUBSYSTEM);
    // stepper::PanHomeAO::Inst().Start(3U, bsp::SubsystemID::PAN_HOME_SUBSYSTEM);
    stepper::TiltHomeAO::Inst().Start(4U, bsp::SubsystemID::TILT_HOME_SUBSYSTEM);
    stepper::StepperAO::PanInst().Start(5U, bsp::SubsystemID::PAN_STEPPER_SUBSYSTEM);
    stepper::StepperAO::TiltInst().Start(6U, bsp::SubsystemID::TILT_STEPPER_SUBSYSTEM);
    control::ControlAO::Inst().Start(7U, bsp::SubsystemID::CONTROL_SUBSYSTEM);

    // Start QF scheduler
    return QP::QF::run();

    return 0;
}
