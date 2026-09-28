#include "cli_binding.hpp"

#include <cstring>

#include "cli_ao.hpp"
#include "imu_ao.hpp"
#include "thirdparty/embedded_cli.h"
#include "turret_ao.hpp"

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
            const char* motor_str = embeddedCliGetToken(args, 2U);

            if (strcmp(cmd_str, "disable") == 0)
            {
                if (strcmp(motor_str, "pan") == 0 || strcmp(motor_str, "all") == 0)
                {
                    turret::TurretAO::Inst().DisablePan();
                    handled = true;
                }

                if (strcmp(motor_str, "tilt") == 0 || strcmp(motor_str, "all") == 0)
                {
                    turret::TurretAO::Inst().DisableTilt();
                    handled = true;
                }
            }
            else if (strcmp(cmd_str, "enable") == 0)
            {
                if (strcmp(motor_str, "pan") == 0 || strcmp(motor_str, "all") == 0)
                {
                    turret::TurretAO::Inst().EnablePan();
                    handled = true;
                }

                if (strcmp(motor_str, "tilt") == 0 || strcmp(motor_str, "all") == 0)
                {
                    turret::TurretAO::Inst().EnableTilt();
                    handled = true;
                }
            }
            break;
        }
        case 3U:
        {
            const char* cmd_str = embeddedCliGetToken(args, 1U);
            if (strcmp(cmd_str, "setratedirect") == 0)
            {
                const char* motor_str = embeddedCliGetToken(args, 2U);
                const char* rate_str = embeddedCliGetToken(args, 3U);
                float rate = strtofS(rate_str);

                if (strcmp(motor_str, "pan") == 0 || strcmp(motor_str, "all") == 0)
                {
                    turret::TurretAO::Inst().SetPanRateSetpointDirect(rate);
                    handled = true;
                }

                if (strcmp(motor_str, "tilt") == 0 || strcmp(motor_str, "all") == 0)
                {
                    turret::TurretAO::Inst().SetTiltRateSetpointDirect(rate);
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
            "\tturret setratedirect [Stepper (all|pan|tilt)] [rate (rad/s)]\n\r"
            "\tturret disable [Stepper (all|pan|tilt)]\n\r"
            "\tturret enable [Stepper (all|pan|tilt)]\n\r");
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
}
