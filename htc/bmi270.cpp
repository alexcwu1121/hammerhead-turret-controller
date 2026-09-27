#include "bmi270.hpp"

namespace imu
{
/// @brief BMI270 device ID
constexpr uint8_t BMI270ID = 0x24;

/// @brief Commands
enum Command : uint8_t
{
    WRITE = 0x00,
    READ = 0x01,
};

/// @brief Advanced power save state
enum AdvPowerSave : uint8_t
{
    APS_OFF = 0x00,
    APS_ON = 0x01,
};

/// @brief FIFO read enable in low-power mode
enum FIFOSelfWakeUp : uint8_t
{
    FSW_OFF = 0x00,
    FSW_ON = 0x01,
};

/// @brief Fast powerup enable state
enum FupEn : uint8_t
{
    FUP_OFF = 0x00,
    FUP_ON = 0x01,
};

/// @brief Initialization control mode
enum InitCtrl : uint8_t
{
    INIT_ACTIVE = 0x00,
    INIT_INACTIVE = 0x01,
};

/// @brief Aux sensor enable
enum AuxEn : uint8_t
{
    AUX_OFF = 0x00,
    AUX_ON = 0x01,
};

/// @brief Gyro enable
enum GyrEn : uint8_t
{
    GYR_OFF = 0x00,
    GYR_ON = 0x01,
};

/// @brief Accelerometer enable
enum AccEn : uint8_t
{
    ACC_OFF = 0x00,
    ACC_ON = 0x01,
};

/// @brief Temperature sensor enable
enum TempEn : uint8_t
{
    TEMP_OFF = 0x00,
    TEMP_ON = 0x01,
};

/// @brief Internal status codes
enum InternalStatus : uint8_t
{
    NOT_INIT = 0x00,
    INIT_OK = 0x01,
    INIT_ERR = 0x02,
    DRV_ERR = 0x03,         // driver error
    SNS_STOP = 0x04,        // Sensor stopped
    NVM_ERROR = 0x05,       // nonvolatile memory error
    START_UP_ERROR = 0x06,  // init/nvm error
    COMPAT_ERROR = 0x07,    // compatibility error
};

/// @brief Minimal BMI270 config file
constexpr uint8_t bmi270_maximum_fifo_config_file[] = {
    0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x1a, 0x00, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00,
    0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0x90, 0x32, 0x21, 0x2e, 0x59, 0xf5,
    0x10, 0x30, 0x21, 0x2e, 0x6a, 0xf5, 0x1a, 0x24, 0x22, 0x00, 0x80, 0x2e, 0x3b, 0x00, 0xc8, 0x2e, 0x44, 0x47, 0x22,
    0x00, 0x37, 0x00, 0xa4, 0x00, 0xff, 0x0f, 0xd1, 0x00, 0x07, 0xad, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
    0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00,
    0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x11, 0x24, 0xfc, 0xf5, 0x80, 0x30, 0x40, 0x42, 0x50, 0x50, 0x00, 0x30, 0x12, 0x24, 0xeb,
    0x00, 0x03, 0x30, 0x00, 0x2e, 0xc1, 0x86, 0x5a, 0x0e, 0xfb, 0x2f, 0x21, 0x2e, 0xfc, 0xf5, 0x13, 0x24, 0x63, 0xf5,
    0xe0, 0x3c, 0x48, 0x00, 0x22, 0x30, 0xf7, 0x80, 0xc2, 0x42, 0xe1, 0x7f, 0x3a, 0x25, 0xfc, 0x86, 0xf0, 0x7f, 0x41,
    0x33, 0x98, 0x2e, 0xc2, 0xc4, 0xd6, 0x6f, 0xf1, 0x30, 0xf1, 0x08, 0xc4, 0x6f, 0x11, 0x24, 0xff, 0x03, 0x12, 0x24,
    0x00, 0xfc, 0x61, 0x09, 0xa2, 0x08, 0x36, 0xbe, 0x2a, 0xb9, 0x13, 0x24, 0x38, 0x00, 0x64, 0xbb, 0xd1, 0xbe, 0x94,
    0x0a, 0x71, 0x08, 0xd5, 0x42, 0x21, 0xbd, 0x91, 0xbc, 0xd2, 0x42, 0xc1, 0x42, 0x00, 0xb2, 0xfe, 0x82, 0x05, 0x2f,
    0x50, 0x30, 0x21, 0x2e, 0x21, 0xf2, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0xf0, 0x6f, 0x02, 0x30, 0x02, 0x42, 0x20,
    0x26, 0xe0, 0x6f, 0x02, 0x31, 0x03, 0x40, 0x9a, 0x0a, 0x02, 0x42, 0xf0, 0x37, 0x05, 0x2e, 0x5e, 0xf7, 0x10, 0x08,
    0x12, 0x24, 0x1e, 0xf2, 0x80, 0x42, 0x83, 0x84, 0xf1, 0x7f, 0x0a, 0x25, 0x13, 0x30, 0x83, 0x42, 0x3b, 0x82, 0xf0,
    0x6f, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x00, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x3e, 0x84,
    0x00, 0x40, 0x40, 0x42, 0x7e, 0x82, 0xe1, 0x7f, 0xf2, 0x7f, 0x98, 0x2e, 0x6a, 0xd6, 0x21, 0x30, 0x23, 0x2e, 0x61,
    0xf5, 0xeb, 0x2c, 0xe1, 0x6f};

/// @brief Populate command register
/// @param cmd Command (read/write)
/// @param reg Register
/// @return register
inline constexpr uint8_t CMD_REG(Command cmd, BMI270::Register reg)
{
    return ((cmd & 0x01) << 7) | (reg & 0x7F);
}

/// @brief Populate power conf register
/// @param aps advanced power save enable
/// @param fsw fifo self wakeup enable
/// @param fup fast powerup enable
/// @return register
inline constexpr uint8_t PWR_CONF_REG(AdvPowerSave aps, FIFOSelfWakeUp fsw, FupEn fup)
{
    return ((fup & 0x01) << 2) | ((fsw & 0x01) << 1) | (aps & 0x01);
}

/// @brief Populate power control register
/// @param aux Auxiliary sensor enable
/// @param gyr Gyro sensor enable
/// @param acc Accelerometer sensor enable
/// @param temp Temperature sensor enable
/// @return register
inline constexpr uint8_t PWR_CTRL_REG(AuxEn aux, GyrEn gyr, AccEn acc, TempEn temp)
{
    return ((temp & 0x01) << 3) | ((acc & 0x01) << 2) | ((gyr & 0x01) << 1) | (aux & 0x01);
}

/// @brief Populate gyroscope conf register
/// @param odr data rate
/// @return register
inline constexpr uint8_t GYR_CONF_REG(BMI270::GyrODR odr)
{
    return ((uint8_t)odr & 0x0F);
}

/// @brief Populate gyroscope range register
/// @param range
/// @return register
inline constexpr uint8_t GYR_RANGE_REG(BMI270::GyrRange range)
{
    return ((uint8_t)range & 0x0F);
}

/// @brief Populate accelerometer conf register
/// @param odr data rate
/// @return register
inline constexpr uint8_t ACC_CONF_REG(BMI270::AccODR odr)
{
    return ((uint8_t)odr & 0x0F);
}

/// @brief Populate accelerometer range register
/// @param range
/// @return register
inline constexpr uint8_t ACC_RANGE_REG(BMI270::AccRange range)
{
    return ((uint8_t)range & 0x0F);
}

BMI270::BMI270(GPIO_TypeDef* csPort, uint16_t csPinNum, SPI_HandleTypeDef* spiDevice) :
    _csPort(csPort),
    _csPinNum(csPinNum),
    _spiDevice(spiDevice),
    _verified(false)
{}

Fault BMI270::Initialize()
{
    // Read device ID
    uint8_t id;
    Fault fault = ReadRegisters(&id, Register::CHIP_ID, sizeof(id));
    if (fault != Fault::NO_FAULT) { return fault; }
    // Check for ID match
    if (id == BMI270ID) { _verified = true; }
    else { return Fault::INVALID_ID; }

    // Disable advanced power save mode
    static const uint8_t pwr_conf = PWR_CONF_REG(AdvPowerSave::APS_OFF, FIFOSelfWakeUp::FSW_OFF, FupEn::FUP_OFF);
    fault = WriteRegisters(&pwr_conf, Register::PWR_CONF, sizeof(pwr_conf));
    if (fault != Fault::NO_FAULT) { return fault; }

    // Wait 1ms for powerup sequence
    HAL_Delay(1U);

    // Prepare config load
    static const uint8_t start_init_ctrl = InitCtrl::INIT_ACTIVE;
    fault = WriteRegisters(&start_init_ctrl, Register::INIT_CTRL, sizeof(start_init_ctrl));
    if (fault != Fault::NO_FAULT) { return fault; }

    // Write config data
    fault =
        WriteRegisters(bmi270_maximum_fifo_config_file, Register::INIT_DATA, sizeof(bmi270_maximum_fifo_config_file));
    if (fault != Fault::NO_FAULT) { return fault; }

    // Complete config load
    static const uint8_t stop_init_ctrl = InitCtrl::INIT_INACTIVE;
    fault = WriteRegisters(&stop_init_ctrl, Register::INIT_CTRL, sizeof(stop_init_ctrl));
    if (fault != Fault::NO_FAULT) { return fault; }

    // Wait for initialization success
    uint8_t status;
    for (uint8_t i = 0; i < _initRetryCount; i++)
    {
        fault = ReadRegisters(&status, Register::INTERNAL_STATUS, sizeof(status));
        if (fault != Fault::NO_FAULT) { return fault; }
        if (status == InternalStatus::INIT_OK) { break; }
        HAL_Delay(_initRetryPeriod);
    }

    // Check for init success
    if (status != InternalStatus::INIT_OK)
    {
        // TODO: handle
        return Fault::INIT_FAILED;
    }

    // Enable gyro and accelerometer
    static const uint8_t pwr_ctrl = PWR_CTRL_REG(AuxEn::AUX_OFF, GyrEn::GYR_ON, AccEn::ACC_ON, TempEn::TEMP_OFF);
    fault = WriteRegisters(&pwr_ctrl, Register::PWR_CTRL, sizeof(pwr_ctrl));

    // Set default accelerometer ODR
    fault = SetAccODR(_accODR);

    // Set default accelerometer range
    fault = SetAccRange(_accRange);

    // Set default gyroscope ODR
    fault = SetGyrODR(_gyrODR);

    // Set default gyroscope range
    fault = SetGyrRange(_gyrRange);

    // Enable accelerometer offset compensation
    uint8_t nv_conf = {0};
    fault = ReadRegisters(&nv_conf, Register::NV_CONF, sizeof(nv_conf));
    nv_conf = nv_conf | (0x01 << 3);
    fault = WriteRegisters(&nv_conf, Register::NV_CONF, sizeof(nv_conf));
    if (fault != Fault::NO_FAULT) { return fault; }

    // Enable gyroscope offset compensation
    uint8_t offset_6 = {0};
    fault = ReadRegisters(&offset_6, Register::OFFSET_6, sizeof(offset_6));
    offset_6 = offset_6 | (0x01 << 6);
    fault = WriteRegisters(&offset_6, Register::OFFSET_6, sizeof(offset_6));
    if (fault != Fault::NO_FAULT) { return fault; }

    // Read gyro zx coupling compensation factor
    fault = ReadRegisters((uint8_t*)&_gyrCas, Register::GYR_CAS, sizeof(_gyrCas));
    // GYR_CAS is a 7-bit wide signed integer. Perform sign extension
    _gyrCas = (_gyrCas << 1) >> 1;

    return fault;
}

Fault BMI270::ReadData(IMUData& data)
{
    // Read raw data
    IMUDataRaw raw_data = {0};
    Fault fault = ReadDataRaw(raw_data);
    if (fault != Fault::NO_FAULT) { return NO_FAULT; }

    // Scale to range
    data = ScaleRaw(raw_data);

    return fault;
}

Fault BMI270::SetAccODR(AccODR odr)
{
    uint8_t acc_conf = ACC_CONF_REG(odr);
    return WriteRegisters(&acc_conf, Register::ACC_CONF, sizeof(acc_conf));
}

Fault BMI270::SetAccRange(AccRange range)
{
    uint8_t acc_range = ACC_RANGE_REG(range);
    return WriteRegisters(&acc_range, Register::ACC_RANGE, sizeof(acc_range));
}

Fault BMI270::SetGyrODR(GyrODR odr)
{
    uint8_t gyr_conf = GYR_CONF_REG(odr);
    return WriteRegisters(&gyr_conf, Register::GYR_CONF, sizeof(gyr_conf));
}

Fault BMI270::SetGyrRange(GyrRange range)
{
    uint8_t gyr_range = GYR_RANGE_REG(range);
    return WriteRegisters(&gyr_range, Register::GYR_RANGE, sizeof(gyr_range));
}

Fault BMI270::RunCompensation()
{
    /// TODO: Refactor into an asynchronous state machine

    // Disable ADS
    static const uint8_t pwr_conf = PWR_CONF_REG(AdvPowerSave::APS_OFF, FIFOSelfWakeUp::FSW_OFF, FupEn::FUP_OFF);
    Fault fault = WriteRegisters(&pwr_conf, Register::PWR_CONF, sizeof(pwr_conf));
    if (fault != Fault::NO_FAULT) { return fault; }

    // Disable accelerometer offset compensation temporarily
    uint8_t nv_conf = {0};
    fault = ReadRegisters(&nv_conf, Register::NV_CONF, sizeof(nv_conf));
    nv_conf &= ~(0x01 << 3);
    fault = WriteRegisters(&nv_conf, Register::NV_CONF, sizeof(nv_conf));
    if (fault != Fault::NO_FAULT) { return fault; }

    // Disable gyroscope offset compensation temporarily
    uint8_t offset_6 = {0};
    fault = ReadRegisters(&offset_6, Register::OFFSET_6, sizeof(offset_6));
    offset_6 &= ~(0x01 << 6);
    fault = WriteRegisters(&offset_6, Register::OFFSET_6, sizeof(offset_6));
    if (fault != Fault::NO_FAULT) { return fault; }

    // Read N samples from gyroscope and accelerometer
    // IMU must not be in motion
    bool init = false;
    IMUData avg = {0};
    // Read once after changing compensation enables
    fault = ReadData(avg);
    avg = {0};
    for (uint32_t i = 0; i < _compensationPollCount; i++)
    {
        // Read sample
        IMUData data = {0};
        fault = ReadData(data);
        if (fault != Fault::NO_FAULT) { return fault; }

        if (!init)
        {
            avg = data;
            init = true;
        }
        else
        {
            // Simple IIR filter
            auto iir = [](float& prior, float& obs, const float& alpha) { prior += alpha * (obs - prior); };
            iir(avg.acc[0], data.acc[0], _compIIRAlpha);
            iir(avg.acc[1], data.acc[1], _compIIRAlpha);
            iir(avg.acc[2], data.acc[2], _compIIRAlpha);
            iir(avg.gyr[0], data.gyr[0], _compIIRAlpha);
            iir(avg.gyr[1], data.gyr[1], _compIIRAlpha);
            iir(avg.gyr[2], data.gyr[2], _compIIRAlpha);
        }

        HAL_Delay(_compensationPollPeriod);
    }
    // Negate average and scale by resolution for bias offset
    avg.acc[0] = -avg.acc[0] / _accOffsetRes;
    avg.acc[1] = -avg.acc[1] / _accOffsetRes;
    avg.acc[2] = -avg.acc[2] / _accOffsetRes;
    avg.gyr[0] = -avg.gyr[0] / _gyrOffsetRes;
    avg.gyr[1] = -avg.gyr[1] / _gyrOffsetRes;
    avg.gyr[2] = -avg.gyr[2] / _gyrOffsetRes;

    /// TODO: Don't do accelerometer compensation until a procedure is defined
    /*
    // Compute compensation values and write to offset registers
    int8_t acc_offset[3] = {0};
    acc_offset[0]        = (int8_t)(((uint16_t)avg.acc[0] * 127) >> 15);
    acc_offset[1]        = (int8_t)(((uint16_t)avg.acc[1] * 127) >> 15);
    acc_offset[2]        = (int8_t)(((uint16_t)avg.acc[2] * 127) >> 15);
    // Write accelerometer compensation values
    /// TODO: assumes little endian
    fault = WriteRegisters((uint8_t*)(acc_offset), Register::OFFSET_0, sizeof(acc_offset[0]));
    fault = WriteRegisters((uint8_t*)(acc_offset + 1), Register::OFFSET_1, sizeof(acc_offset[1]));
    fault = WriteRegisters((uint8_t*)(acc_offset + 2), Register::OFFSET_2, sizeof(acc_offset[2]));
    */

    // Write LSB gyro compensation values
    uint8_t gyr_offset_lsb[3] = {0};
    gyr_offset_lsb[0] = (uint8_t)((int16_t)avg.gyr[0] & 0xFF);
    gyr_offset_lsb[1] = (uint8_t)((int16_t)avg.gyr[1] & 0xFF);
    gyr_offset_lsb[2] = (uint8_t)((int16_t)avg.gyr[2] & 0xFF);
    uint8_t gyr_offset_msb[3] = {0};
    gyr_offset_msb[0] = (uint8_t)(((int16_t)avg.gyr[0] >> 8) & 0xFF);
    gyr_offset_msb[1] = (uint8_t)(((int16_t)avg.gyr[1] >> 8) & 0xFF);
    gyr_offset_msb[2] = (uint8_t)(((int16_t)avg.gyr[2] >> 8) & 0xFF);
    fault = WriteRegisters(gyr_offset_lsb, Register::OFFSET_3, 1U);
    fault = WriteRegisters(gyr_offset_lsb + 1, Register::OFFSET_4, 1U);
    fault = WriteRegisters(gyr_offset_lsb + 2, Register::OFFSET_5, 1U);
    // Read offset 6 registers for other configurations before writing offsets
    offset_6 = 0;
    fault = ReadRegisters(&offset_6, Register::OFFSET_6, sizeof(offset_6));
    offset_6 = (offset_6 & ~0x03) | (gyr_offset_msb[0] & 0x03);
    offset_6 = (offset_6 & ~0x0C) | (gyr_offset_msb[1] << 2 & 0x0C);
    offset_6 = (offset_6 & ~0x30) | (gyr_offset_msb[2] << 4 & 0x30);
    // Write MSB gyro compensation values
    fault = WriteRegisters(&offset_6, Register::OFFSET_6, sizeof(offset_6));

    // Reenable accelerometer offset compensation
    nv_conf = {0};
    fault = ReadRegisters(&nv_conf, Register::NV_CONF, sizeof(nv_conf));
    nv_conf = nv_conf | (0x01 << 3);
    fault = WriteRegisters(&nv_conf, Register::NV_CONF, sizeof(nv_conf));
    if (fault != Fault::NO_FAULT) { return fault; }

    // Reenable gyroscope offset compensation
    offset_6 = {0};
    fault = ReadRegisters(&offset_6, Register::OFFSET_6, sizeof(offset_6));
    offset_6 = offset_6 | (0x01 << 6);
    fault = WriteRegisters(&offset_6, Register::OFFSET_6, sizeof(offset_6));
    if (fault != Fault::NO_FAULT) { return fault; }

    // Prepare NVM programming
    uint8_t gen_set_1[2] = {0};
    fault = ReadRegisters(gen_set_1, Register::GEN_SET_1, sizeof(gen_set_1));
    gen_set_1[1] = gen_set_1[1] | (0x01 << 2);
    fault = WriteRegisters(gen_set_1, Register::GEN_SET_1, sizeof(gen_set_1));
    if (fault != Fault::NO_FAULT) { return fault; }

    // Wait 40 ms
    HAL_Delay(40U);

    // Unlock NVM programming
    static const uint8_t nvm_unlock = 0x01;
    fault = WriteRegisters(&nvm_unlock, Register::NV_CONF, sizeof(nvm_unlock));

    // Program NVM
    static const uint8_t nvm_prog = 0xa0;
    fault = WriteRegisters(&nvm_prog, Register::CMD, sizeof(nvm_prog));
    if (fault != Fault::NO_FAULT) { return fault; }

    // Wait until write finishes
    for (uint32_t i = 0; i < _nvmWaitPollCount; i++)
    {
        uint8_t cmd_status = 0x00;
        fault = ReadRegisters(&cmd_status, Register::STATUS, sizeof(cmd_status));
        if (((cmd_status >> 4) & 0x01) == 0x01) { break; }
        HAL_Delay(_nvmWaitPollPeriod);
    }

    // Lock NVM programming
    static const uint8_t nvm_lock = 0x00;
    fault = WriteRegisters(&nvm_lock, Register::NV_CONF, sizeof(nvm_lock));

    return fault;
}

Fault BMI270::WriteRegisters(const uint8_t* buf, Register reg, uint16_t numRegisters)
{
    // Fail write operation if device ID not verified
    if (!_verified) { return Fault::INVALID_ID; }

    // Populate command byte
    uint8_t command = CMD_REG(Command::WRITE, reg);

    // Perform transaction
    HAL_GPIO_WritePin(_csPort, _csPinNum, GPIO_PIN_RESET);
    // Transmit command byte
    auto rc = HAL_SPI_Transmit(_spiDevice, &command, 1U, _timeout);
    // Write registers
    if (rc == HAL_StatusTypeDef::HAL_OK) { rc = HAL_SPI_Transmit(_spiDevice, (uint8_t*)buf, numRegisters, _timeout); }
    HAL_GPIO_WritePin(_csPort, _csPinNum, GPIO_PIN_SET);

    return static_cast<Fault>(rc);
}

Fault BMI270::ReadRegisters(uint8_t* buf, Register reg, uint16_t numRegisters)
{
    // Populate command byte
    uint8_t command = CMD_REG(Command::READ, reg);

    // Perform transaction
    HAL_GPIO_WritePin(_csPort, _csPinNum, GPIO_PIN_RESET);
    // Transmit command byte
    auto rc = HAL_SPI_Transmit(_spiDevice, &command, 1U, _timeout);
    // Discard the dummy byte
    HAL_SPI_Receive(_spiDevice, buf, 1U, _timeout);
    // Read registers
    if (rc == HAL_StatusTypeDef::HAL_OK) { rc = HAL_SPI_Receive(_spiDevice, buf, numRegisters, _timeout); }
    HAL_GPIO_WritePin(_csPort, _csPinNum, GPIO_PIN_SET);

    return static_cast<Fault>(rc);
}

Fault BMI270::ReadDataRaw(IMUDataRaw& data)
{
    uint8_t status;
    Fault fault = ReadRegisters(&status, Register::STATUS, sizeof(status));
    if (fault == Fault::NO_FAULT)
    {
        // Check for gyro data ready
        if (status & (0x01 << 6))
        {
            uint8_t gyr_temp[6] = {0};
            fault = ReadRegisters(gyr_temp, Register::GYR_X_LSB, sizeof(gyr_temp));
            data.gyr[0] = (gyr_temp[1] << 8) | gyr_temp[0];
            data.gyr[1] = (gyr_temp[3] << 8) | gyr_temp[2];
            data.gyr[2] = (gyr_temp[5] << 8) | gyr_temp[4];

            // Ratex = DATA_15<<8 + DATA_14 - GYR_CAS.factor_zx * (DATA_19<<8+DATA_18) / 2^9
            // Ratey = DATA_17<<8 + DATA_16
            // Ratez = DATA_19<<8 + DATA_18
            data.gyr[0] -= (int16_t)_gyrCas * data.gyr[2] / 512;
        }

        // Check for accelerometer data ready
        if (status & (0x01 << 7))
        {
            uint8_t acc_temp[6] = {0};
            fault = ReadRegisters(acc_temp, Register::ACC_X_LSB, sizeof(acc_temp));
            data.acc[0] = (acc_temp[1] << 8) | acc_temp[0];
            data.acc[1] = (acc_temp[3] << 8) | acc_temp[2];
            data.acc[2] = (acc_temp[5] << 8) | acc_temp[4];
        }
    }
    return fault;
}

IMUData BMI270::ScaleRaw(const IMUDataRaw& raw)
{
    IMUData data = {0};

    // Scale accelerometer readings
    float acc_range = 1.0f;
    switch (_accRange)
    {
        case AccRange::G_2:
        {
            acc_range = 2.0f;
            break;
        }
        case AccRange::G_4:
        {
            acc_range = 4.0f;
            break;
        }
        case AccRange::G_8:
        {
            acc_range = 8.0f;
            break;
        }
        case AccRange::G_16:
        {
            acc_range = 16.0f;
            break;
        }
        default:
        {
            acc_range = 1.0f;
            break;
        }
    }
    data.acc[0] = (acc_range / (float)INT16_MAX) * (float)raw.acc[0];
    data.acc[1] = (acc_range / (float)INT16_MAX) * (float)raw.acc[1];
    data.acc[2] = (acc_range / (float)INT16_MAX) * (float)raw.acc[2];

    // Scale gyro readings
    float gyr_range = 1.0f;
    switch (_gyrRange)
    {
        case GyrRange::DPS_125:
        {
            gyr_range = 125.0f;
            break;
        }
        case GyrRange::DPS_250:
        {
            gyr_range = 250.0f;
            break;
        }
        case GyrRange::DPS_500:
        {
            gyr_range = 500.0f;
            break;
        }
        case GyrRange::DPS_1000:
        {
            gyr_range = 1000.0f;
            break;
        }
        case GyrRange::DPS_2000:
        {
            gyr_range = 2000.0f;
            break;
        }
        default:
        {
            gyr_range = 1.0f;
            break;
        }
    }
    data.gyr[0] = (gyr_range / (float)INT16_MAX) * (float)raw.gyr[0];
    data.gyr[1] = (gyr_range / (float)INT16_MAX) * (float)raw.gyr[1];
    data.gyr[2] = (gyr_range / (float)INT16_MAX) * (float)raw.gyr[2];

    return data;
}
}  // namespace imu
