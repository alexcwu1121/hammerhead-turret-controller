#include "cli_binding.hpp"

#include <cstring>

#include "cli_ao.hpp"
#include "control_ao.hpp"
#include "imu_ao.hpp"
#include "stepper_ao.hpp"
#include "thirdparty/embedded_cli.h"
#include "tilt_home_ao.hpp"
//#include "pan_home_ao.hpp"

void cli::onClear(EmbeddedCli* cli, char* args, void* context)
{
    cli::CLIAO::Inst().Printf("\33[2J");
}

void cli::onIMU(EmbeddedCli* cli, char* args, void* context)
{
    // Get number of arguments
    uint16_t argc = embeddedCliGetTokenCount(args);
    bool handled = false;

    switch (argc)
    {
        case 1U:
        {
            const char* cmd_str = embeddedCliGetToken(args, 1U);
            if (strcmp(cmd_str, "run_compensation") == 0)
            {
                imu::IMUAO::Inst().RunIMUCompensation();
                handled = true;
            }
            else if (strcmp(cmd_str, "start_stream") == 0)
            {
                imu::IMUAO::Inst().StartIMUStream();
                handled = true;
            }
            else if (strcmp(cmd_str, "stop_stream") == 0)
            {
                imu::IMUAO::Inst().StopIMUStream();
                handled = true;
            }
            else if (strcmp(cmd_str, "reset") == 0)
            {
                imu::IMUAO::Inst().Reset();
                handled = true;
            }
            break;
        }
        default:
        {
            break;
        }
    }

    if (!handled)
    {
        // Help dialogue
        cli::CLIAO::Inst().Printf(
            "Usage:\n\r"
            "\timu run_compensation\n\r"
            "\timu start_stream\n\r"
            "\timu stop_stream\n\r"
            "\timu reset\n\r");
    }
}

void cli::onTurret(EmbeddedCli* cli, char* args, void* context)
{
    // Get number of arguments
    uint16_t argc = embeddedCliGetTokenCount(args);
    bool handled = false;

    switch (argc)
    {
        case 1U:
        {
            break;
        }
        case 2U:
        {
            const char* cmd_str = embeddedCliGetToken(args, 1U);
            const char* opt_str = embeddedCliGetToken(args, 2U);

            if (strcmp(cmd_str, "startencstream") == 0)
            {
                if (strcmp(opt_str, "pan") == 0 || strcmp(opt_str, "all") == 0)
                {
                    stepper::StepperAO::PanInst().StartEncoderStream();
                    handled = true;
                }

                if (strcmp(opt_str, "tilt") == 0 || strcmp(opt_str, "all") == 0)
                {
                    stepper::StepperAO::TiltInst().StartEncoderStream();
                    handled = true;
                }
            }
            else if (strcmp(cmd_str, "stopencstream") == 0)
            {
                if (strcmp(opt_str, "pan") == 0 || strcmp(opt_str, "all") == 0)
                {
                    stepper::StepperAO::PanInst().StopEncoderStream();
                    handled = true;
                }

                if (strcmp(opt_str, "tilt") == 0 || strcmp(opt_str, "all") == 0)
                {
                    stepper::StepperAO::TiltInst().StopEncoderStream();
                    handled = true;
                }
            }
            else if (strcmp(cmd_str, "disable") == 0)
            {
                if (strcmp(opt_str, "pan") == 0 || strcmp(opt_str, "all") == 0)
                {
                    stepper::StepperAO::PanInst().Disable();
                    handled = true;
                }

                if (strcmp(opt_str, "tilt") == 0 || strcmp(opt_str, "all") == 0)
                {
                    stepper::StepperAO::TiltInst().Disable();
                    handled = true;
                }
            }
            else if (strcmp(cmd_str, "enable") == 0)
            {
                if (strcmp(opt_str, "pan") == 0 || strcmp(opt_str, "all") == 0)
                {
                    stepper::StepperAO::PanInst().Enable();
                    handled = true;
                }

                if (strcmp(opt_str, "tilt") == 0 || strcmp(opt_str, "all") == 0)
                {
                    stepper::StepperAO::TiltInst().Enable();
                    handled = true;
                }
            }
            else if (strcmp(cmd_str, "home") == 0)
            {
                if (strcmp(opt_str, "pan") == 0 || strcmp(opt_str, "all") == 0)
                {
                    // stepper::PanHomeAO::Inst().Home();
                    handled = true;
                }

                if (strcmp(opt_str, "tilt") == 0 || strcmp(opt_str, "all") == 0)
                {
                    stepper::TiltHomeAO::Inst().Home();
                    handled = true;
                }
            }
            else if (strcmp(cmd_str, "aborthome") == 0)
            {
                if (strcmp(opt_str, "pan") == 0 || strcmp(opt_str, "all") == 0)
                {
                    // stepper::PanHomeAO::Inst().Abort();
                    handled = true;
                }

                if (strcmp(opt_str, "tilt") == 0 || strcmp(opt_str, "all") == 0)
                {
                    stepper::TiltHomeAO::Inst().Abort();
                    handled = true;
                }
            }
            break;
        }
        case 3U:
        {
            const char* cmd_str = embeddedCliGetToken(args, 1U);
            if (strcmp(cmd_str, "setmode") == 0)
            {
                const char* motor_str = embeddedCliGetToken(args, 2U);
                const char* mode_str = embeddedCliGetToken(args, 3U);
                stepper::Mode mode;
                if (strcmp(mode_str, "ol") == 0) { mode = stepper::Mode::OPEN_LOOP; }
                else if (strcmp(mode_str, "cl") == 0) { mode = stepper::Mode::CLOSED_LOOP; }
                else if (strcmp(mode_str, "clp") == 0) { mode = stepper::Mode::CLOSED_LOOP_POS; }
                else { break; }

                if (strcmp(motor_str, "pan") == 0 || strcmp(motor_str, "all") == 0)
                {
                    stepper::StepperAO::PanInst().SetMode(mode);
                    handled = true;
                }

                if (strcmp(motor_str, "tilt") == 0 || strcmp(motor_str, "all") == 0)
                {
                    stepper::StepperAO::TiltInst().SetMode(mode);
                    handled = true;
                }
            }
            else if (strcmp(cmd_str, "setratedirect") == 0)
            {
                const char* motor_str = embeddedCliGetToken(args, 2U);
                const char* rate_str = embeddedCliGetToken(args, 3U);
                float rate = strtofS(rate_str);

                if (strcmp(motor_str, "pan") == 0 || strcmp(motor_str, "all") == 0)
                {
                    stepper::StepperAO::PanInst().SetRateSetpointDirect(rate);
                    handled = true;
                }

                if (strcmp(motor_str, "tilt") == 0 || strcmp(motor_str, "all") == 0)
                {
                    stepper::StepperAO::TiltInst().SetRateSetpointDirect(rate);
                    handled = true;
                }
            }
            else if (strcmp(cmd_str, "setrate") == 0)
            {
                const char* motor_str = embeddedCliGetToken(args, 2U);
                const char* rate_str = embeddedCliGetToken(args, 3U);
                float rate = strtofS(rate_str);

                if (strcmp(motor_str, "pan") == 0 || strcmp(motor_str, "all") == 0)
                {
                    stepper::StepperAO::PanInst().SetRateSetpoint(rate);
                    handled = true;
                }

                if (strcmp(motor_str, "tilt") == 0 || strcmp(motor_str, "all") == 0)
                {
                    stepper::StepperAO::TiltInst().SetRateSetpoint(rate);
                    handled = true;
                }
            }
            else if (strcmp(cmd_str, "setpos") == 0)
            {
                const char* motor_str = embeddedCliGetToken(args, 2U);
                const char* pos_str = embeddedCliGetToken(args, 3U);
                float pos = strtofS(pos_str);

                if (strcmp(motor_str, "pan") == 0 || strcmp(motor_str, "all") == 0)
                {
                    stepper::StepperAO::PanInst().SetPositionSetpoint(pos);
                    handled = true;
                }

                if (strcmp(motor_str, "tilt") == 0 || strcmp(motor_str, "all") == 0)
                {
                    stepper::StepperAO::TiltInst().SetPositionSetpoint(pos);
                    handled = true;
                }
            }
            break;
        }
        default:
        {
            break;
        }
    }

    if (!handled)
    {
        // Help dialogue
        cli::CLIAO::Inst().Printf(
            "Usage:\n\r"
            "\tturret startencstream [all|pan|tilt]\n\r"
            "\tturret stopencstream [all|pan|tilt]\n\r"
            "\tturret setmode [all|pan|tilt] [ol|cl|clp]\n\r"
            "\tturret setratedirect [all|pan|tilt] [rate (rad/s)]\n\r"
            "\tturret setrate [all|pan|tilt] [rate (rad/s)]\n\r"
            "\tturret setpos [all|pan|tilt] [pos (rad)]\n\r"
            "\tturret disable [all|pan|tilt]\n\r"
            "\tturret enable [all|pan|tilt]\n\r"
            "\tturret home [all|pan|tilt]\n\r"
            "\tturret aborthome [all|pan|tilt]\n\r");
    }
}

void cli::onControl(EmbeddedCli* cli, char* args, void* context)
{
    // Get number of arguments
    uint16_t argc = embeddedCliGetTokenCount(args);
    bool handled = false;

    switch (argc)
    {
        case 1U:
        {
            const char* cmd_str = embeddedCliGetToken(args, 1U);
            if (strcmp(cmd_str, "print_fault") == 0)
            {
                control::ControlAO::Inst().PrintFault();
                handled = true;
            }
            else if (strcmp(cmd_str, "enable_watchdog") == 0)
            {
                control::ControlAO::Inst().EnableWatchdog();
                handled = true;
            }
            else if (strcmp(cmd_str, "disable_watchdog") == 0)
            {
                control::ControlAO::Inst().DisableWatchdog();
                handled = true;
            }
            break;
        }
        default:
        {
            break;
        }
    }

    if (!handled)
    {
        // Help dialogue
        cli::CLIAO::Inst().Printf(
            "Usage:\n\r"
            "\tcontrol print_fault\n\r"
            "\tcontrol enable_watchdog\n\r"
            "\tcontrol disable_watchdog\n\r");
    }
}

void cli::InitBindings(EmbeddedCli* cli)
{
    // Command binding for the clear command
    CliCommandBinding clear_binding = {
        .name = "clear", .help = "Clears the console", .tokenizeArgs = true, .context = NULL, .binding = onClear};
    embeddedCliAddBinding(cli, clear_binding);

    // Command binding for the IMU system command
    CliCommandBinding imu_binding = {
        .name = "imu", .help = "Manage IMU", .tokenizeArgs = true, .context = NULL, .binding = onIMU};
    embeddedCliAddBinding(cli, imu_binding);

    // Command binding for the turret system command
    CliCommandBinding turret_binding = {
        .name = "turret", .help = "Manage turret", .tokenizeArgs = true, .context = NULL, .binding = onTurret};
    embeddedCliAddBinding(cli, turret_binding);

    // Command binding for the control system command
    CliCommandBinding control_binding = {
        .name = "control", .help = "Manage control", .tokenizeArgs = true, .context = NULL, .binding = onControl};
    embeddedCliAddBinding(cli, control_binding);
}
