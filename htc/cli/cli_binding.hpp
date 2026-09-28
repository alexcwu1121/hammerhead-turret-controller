#ifndef INC_CLI_BINDING_H_
#define INC_CLI_BINDING_H_

class EmbeddedCli;

namespace cli
{
/// @brief Clear binding clears cli
/// @param cli
/// @param args
/// @param context
void onClear(EmbeddedCli* cli, char* args, void* context);

/// @brief IMU cli binding
/// @param cli
/// @param args
/// @param context
void onIMU(EmbeddedCli* cli, char* args, void* context);

/// @brief Turret cli binding
/// @param cli
/// @param args
/// @param context
void onTurret(EmbeddedCli* cli, char* args, void* context);

/// @brief Initialize bindings for a CLI
/// @param cli ptr to CLI instance
void InitBindings(EmbeddedCli* cli);

/// @brief Small strtof
/// @param s
/// @return
inline float strtofS(const char* s)
{
    bool neg = false;
    if (*s == '-')
    {
        neg = true;
        ++s;
    }

    float value = 0.0f;
    while (*s >= '0' && *s <= '9')
    {
        value = value * 10.0f + float(*s - '0');
        ++s;
    }

    if (*s == '.')
    {
        ++s;
        float factor = 0.1f;
        while (*s >= '0' && *s <= '9')
        {
            value += float(*s - '0') * factor;
            factor *= 0.1f;
            ++s;
        }
    }

    return neg ? -value : value;
}

/// @brief Small strtoul
/// @tparam T
/// @param s
/// @return
template <typename T>
inline T strtoulS(const char* s)
{
    T value = 0;
    while (*s >= '0' && *s <= '9')
    {
        value = value * 10 + (*s - '0');
        ++s;
    }
    return value;
}

/// @brief Small strtol
/// @tparam T
/// @param s
/// @return
template <typename T>
inline T strtolS(const char* s)
{
    bool neg = false;
    if (*s == '-')
    {
        neg = true;
        ++s;
    }

    T value = 0;
    while (*s >= '0' && *s <= '9')
    {
        value = value * 10 + (*s - '0');
        ++s;
    }

    return neg ? -value : value;
}
}  // namespace cli

#endif  // INC_CLI_BINDING_H_
