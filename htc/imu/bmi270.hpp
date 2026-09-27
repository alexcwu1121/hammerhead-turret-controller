#ifndef BMI270_HPP_
#define BMI270_HPP_

#include "bsp.hpp"

namespace imu
{
/// @brief IMU fault codes
enum Fault : uint8_t
{
    NO_FAULT = 0U,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT,
    INVALID_ID,
    INVALID_SIZE,
    INIT_FAILED,
    NUM_FAULTS
};

/// @brief Fault code to string table
/// @param fault
/// @return
constexpr const char* FaultToStr(Fault fault)
{
    switch (fault)
    {
        case Fault::NO_FAULT:
        {
            return "NO_FAULT";
        }
        case Fault::HAL_ERROR:
        {
            return "HAL_ERROR";
        }
        case Fault::HAL_BUSY:
        {
            return "HAL_BUSY";
        }
        case Fault::HAL_TIMEOUT:
        {
            return "HAL_TIMEOUT";
        }
        case Fault::INVALID_ID:
        {
            return "INVALID_ID";
        }
        case Fault::INVALID_SIZE:
        {
            return "INVALID_SIZE";
        }
        case Fault::INIT_FAILED:
        {
            return "INIT_FAILED";
        }
        default:
        {
            return "";
        }
    }
}

/// @brief 6DOF IMU data
struct IMUData
{
    float acc[3];
    float gyr[3];
};

class BMI270
{
public:
    /// @brief BMI270 register addresses
    enum Register : uint8_t
    {
        CHIP_ID = 0x00,
        ERR_REG = 0x02,
        STATUS = 0x03,
        PWR_CONF = 0x7C,
        PWR_CTRL = 0x7D,
        INIT_CTRL = 0x59,
        INIT_DATA = 0x5E,
        INTERNAL_STATUS = 0x21,
        FIFO_DATA = 0x26,
        ACC_CONF = 0x40,
        ACC_RANGE = 0x41,
        GYR_CONF = 0x42,
        GYR_RANGE = 0x43,
        AUX_X_LSB = 0x04,
        AUX_X_MSB = 0x05,
        AUX_Y_LSB = 0x06,
        AUX_Y_MSB = 0x07,
        AUX_Z_LSB = 0x08,
        AUX_Z_MSB = 0x09,
        AUX_R_LSB = 0x0A,
        AUX_R_MSB = 0x0B,
        ACC_X_LSB = 0x0C,
        ACC_X_MSB = 0x0D,
        ACC_Y_LSB = 0x0E,
        ACC_Y_MSB = 0x0F,
        ACC_Z_LSB = 0x10,
        ACC_Z_MSB = 0x11,
        GYR_X_LSB = 0x12,
        GYR_X_MSB = 0x13,
        GYR_Y_LSB = 0x14,
        GYR_Y_MSB = 0x15,
        GYR_Z_LSB = 0x16,
        GYR_Z_MSB = 0x17,
        GEN_SET_1 = 0x34,
        NV_CONF = 0x70,   // Accelerometer compensation enable
        OFFSET_0 = 0x71,  // Accelerometer x-axis offset compensation
        OFFSET_1 = 0x72,  // Accelerometer y-axis offset compensation
        OFFSET_2 = 0x73,  // Accelerometer z-axis offset compensation
        OFFSET_3 = 0x74,  // Gyrocsope x-axis offset compensation
        OFFSET_4 = 0x75,  // Gyrocsope y-axis offset compensation
        OFFSET_5 = 0x76,  // Gyrocsope z-axis offset compensation
        OFFSET_6 = 0x77,  // Gyroscope compensation enable
        CMD = 0x7E,
        GYR_CAS = 0x3C,
    };

    /// @brief Accelerometer ranges
    enum class AccRange : uint8_t
    {
        G_2 = 0x00,
        G_4 = 0x01,
        G_8 = 0x02,
        G_16 = 0x03,
    };

    /// @brief Accelerometer data rate
    enum class AccODR : uint8_t
    {
        ODR_0P78 = 0x01,
        ODR_1P5 = 0x02,
        ODR_3P1 = 0x03,
        ODR_6P25 = 0x04,
        ODR_12P5 = 0x05,
        ODR_25 = 0x06,
        ODR_50 = 0x07,
        ODR_100 = 0x08,
        ODR_200 = 0x09,
        ODR_400 = 0x0a,
        ODR_800 = 0x0b,
        ODR_1K6 = 0x0c,
    };

    /// @brief Gyroscope ranges
    enum class GyrRange : uint8_t
    {
        DPS_2000 = 0x00,
        DPS_1000 = 0x01,
        DPS_500 = 0x02,
        DPS_250 = 0x03,
        DPS_125 = 0x04,
    };

    /// @brief Gyroscope data rate
    enum class GyrODR : uint8_t
    {
        ODR_25 = 0x06,
        ODR_50 = 0x07,
        ODR_100 = 0x08,
        ODR_200 = 0x09,
        ODR_400 = 0x0a,
        ODR_800 = 0x0b,
        ODR_1K6 = 0x0c,
        ODR_3K2 = 0x0d,
    };

    /// @brief BMI270 Constructor
    /// @param csPort Chip-Select pin port
    /// @param csPinNum Chip-Select pin number
    /// @param spiDevice SPI interface handle
    BMI270(GPIO_TypeDef* csPort, uint16_t csPinNum, SPI_HandleTypeDef* spiDevice);
    BMI270(const BMI270&) = delete;
    BMI270& operator=(const BMI270&) = delete;
    BMI270(BMI270&&) = delete;
    BMI270& operator=(BMI270&&) = delete;

    /// @brief Initialize device
    /// @return Fault
    [[nodiscard]] Fault Initialize();

    /// @brief Read 6DOF IMU data
    /// @return
    [[nodiscard]] Fault ReadData(IMUData& data);

    /// @brief Read 6DOF IMU data from FIFO queue
    /// @return
    [[nodiscard]] Fault ReadDataFIFO(IMUData& data);

    /// @brief Set accelerometer ODR
    /// @param odr data rate
    /// @return Fault
    [[nodiscard]] Fault SetAccODR(AccODR odr);

    /// @brief Set accelerometer range
    /// @param range full scale range
    /// @return Fault
    [[nodiscard]] Fault SetAccRange(AccRange range);

    /// @brief Set gyroscope ODR
    /// @param odr data rate
    /// @return Fault
    [[nodiscard]] Fault SetGyrODR(GyrODR odr);

    /// @brief Set gyroscope range
    /// @param range full scale range
    /// @return Fault
    [[nodiscard]] Fault SetGyrRange(GyrRange range);

    /// @brief Run accelerometer and gyroscope compensation
    /// @return Fault
    [[nodiscard]] Fault RunCompensation();

    /// TODO: 0x6D ACC_SELF_TEST
    /// TODO: 0x6E GYR_SELF_TEST_AXES

private:
    /// @brief Chip-Select pin port
    GPIO_TypeDef* _csPort;
    /// @brief Chip-Select pin num
    uint16_t _csPinNum;
    /// @brief SPI device handle
    SPI_HandleTypeDef* _spiDevice;
    /// @brief Flag indicating if device ID has been verified
    bool _verified;  // VerifyID() must be called prior to any op to enable SPI
    /// @brief SPI transaction timeout in ms
    static constexpr uint32_t _timeout = 10U;
    /// @brief Initialization maximum retry/poll count
    static constexpr uint32_t _initRetryCount = 50U;
    /// @brief Initialization retry/poll period in ms
    static constexpr uint32_t _initRetryPeriod = 1U;
    /// @brief Offset compensation polling period in ms
    static constexpr uint32_t _compensationPollPeriod = 5U;
    /// @brief Offset compensation polling count
    static constexpr uint32_t _compensationPollCount = 10U;
    /// @brief NVM operation polling period in ms
    static constexpr uint32_t _nvmWaitPollPeriod = 1U;
    /// @brief NVM operation polling count
    static constexpr uint32_t _nvmWaitPollCount = 50U;
    /// @brief Compensation IIR learning rate
    static constexpr float _compIIRAlpha = 0.10;
    /// @brief Accelerometer offset resolution (g)
    static constexpr float _accOffsetRes = 0.0039f;
    /// @brief Gyroscope offset resolution (dps)
    static constexpr float _gyrOffsetRes = 0.061f;
    /// @brief Gyroscope odr
    GyrODR _gyrODR = GyrODR::ODR_1K6;
    /// @brief Accelerometer odr
    AccODR _accODR = AccODR::ODR_800;
    /// @brief Gyroscope range
    GyrRange _gyrRange = GyrRange::DPS_500;
    /// @brief Accelerometer range
    AccRange _accRange = AccRange::G_4;
    /// @brief Gyro cross axis coupling factor
    int8_t _gyrCas = 0;

    /// @brief Raw 6DOF IMU data
    struct IMUDataRaw
    {
        int16_t acc[3];
        int16_t gyr[3];
    };

    /// @brief Write N registers
    /// @param buf input buffer
    /// @param reg register to write to
    /// @param numRegisters number of registers to write
    /// @return Fault
    [[nodiscard]] Fault WriteRegisters(uint8_t* buf, Register reg, uint16_t numRegisters);

    /// @brief Read N registers
    /// @param buf output buffer
    /// @param reg register to read from
    /// @param numRegisters number of registers to read
    /// @return Fault
    [[nodiscard]] Fault ReadRegisters(uint8_t* buf, Register reg, uint16_t numRegisters);

    /// @brief Read raw IMU data from data registers
    /// @return Fault
    [[nodiscard]] Fault ReadDataRaw(IMUDataRaw& data);

    /// @brief Scale raw IMU data to currently configured range
    IMUData ScaleRaw(const IMUDataRaw& raw);
};
}  // namespace imu

#endif
