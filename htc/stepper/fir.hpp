#ifndef FIR_HPP_
#define FIR_HPP_

#include <array>
#include <cstdint>

namespace csys
{
/// @brief Simple fixed-length FIR filter implementation
template <uint8_t S>
class FIR
{
public:
    /// @brief FIR constructor
    FIR()
    {
        Reset();
    }

    /// @brief Reset filter
    void Reset()
    {
        for (uint8_t i = 0; i < S; i++) { _buffer[i] = 0.0f; }
        _runningSum = 0.0f;
        _offset = 0u;
        _size = 0u;
    }

    /// @brief Update filter with one observation
    /// @param obs new obseravtion
    /// @return filter value
    float Update(float obs)  // NOLINT
    {
        if (_size < S) { _size++; }
        else
        {
            // If buffer is saturated, pop element at offset and subtract from running sum
            _runningSum -= _buffer[_offset];
        }

        // Insert obs at offset and add to sum
        _buffer[_offset] = obs;
        _runningSum += obs;

        // advance offset
        _offset = ++_offset % S;

        return _runningSum / _size;
    }

private:
    /// @brief Filter buffer
    std::array<float, S> _buffer;
    /// @brief Buffer insertion offset
    uint8_t _offset = 0u;
    /// @brief Running sum of buffer entries
    float _runningSum = 0.0f;
    /// @brief Running size of buffer
    uint8_t _size = 0u;
};
}  // namespace csys

#endif
