#ifndef PID_HPP_
#define PID_HPP_

namespace csys
{
/// @brief Simple discrete PID implementation
class PID
{
public:
    PID() = default;
    /// @brief PID constructor
    /// @param kp proportional gain
    /// @param ki integral gain
    /// @param kd derivative gain
    PID(float kp, float ki, float kd) : _kp(kp), _ki(ki), _kd(kd) {}  // NOLINT

    /// @brief Reset controller state
    void Reset()
    {
        _iAccum = 0.0f;
        _prevError = 0.0f;
    }

    /// @brief Set gains
    /// @param kp proportional gain
    /// @param ki integral gain
    /// @param kd derivative gain
    void SetGains(float kp, float ki, float kd)  // NOLINT
    {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }

    /// @brief Perform one update and
    /// @param setpoint setpoint
    /// @param obs new obseravtion
    /// @param dt time delta since the last update
    /// @return control effort
    float Update(float setpoint, float obs, float dt)  // NOLINT
    {
        float error = setpoint - obs;

        float effort = _kp * error;
        effort += _ki * _iAccum;
        effort += _kd * (error - _prevError) / dt;

        // integrate and save previous error
        _iAccum += error * dt;
        _prevError = error;

        return effort;
    }

private:
    /// @brief Proportional gain
    float _kp {0.0f};
    /// @brief Integral gain
    float _ki {0.0f};
    /// @brief Derivative gain
    float _kd {0.0f};

    /// @brief Integrator accumulator
    float _iAccum {0.0f};
    /// @brief Previous error for simple derivative
    float _prevError {0.0f};
};
}  // namespace csys

#endif
