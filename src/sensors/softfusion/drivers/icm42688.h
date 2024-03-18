#pragma once

#include <cstdint>
#include <array>
#include <algorithm>

namespace SlimeVR::Sensors::SoftFusion::Drivers
{

// Driver uses acceleration range at 8g
// and gyroscope range at 1000dps
// Gyroscope ODR = 500Hz, accel ODR = 100Hz
// Timestamps reading not used, as they're useless (constant predefined increment)

template <typename I2CImpl>
struct ICM42688
{
    static constexpr uint8_t Address = 0x68;
    static constexpr auto Name = "ICM-42688";
    static constexpr auto Type = ImuID::ICM42688;

    static constexpr float GyrTs=1.0/(200.0 * 32.768 / 32.0);
    static constexpr float AccTs=1.0/(100.0 * 32.768 / 32.0);

    static constexpr float MagTs=1.0/100;

    static constexpr float GyroSensitivity = 32.8f;
    static constexpr float AccelSensitivity = 4096.0f;

    I2CImpl i2c;
    SlimeVR::Logging::Logger &logger;
    ICM42688(I2CImpl i2c, SlimeVR::Logging::Logger &logger)
    : i2c(i2c), logger(logger) {}

    struct Regs {
        struct WhoAmI {
            static constexpr uint8_t reg = 0x75;
            static constexpr uint8_t value = 0x47;
        };
        static constexpr uint8_t TempData = 0x1d;

        static constexpr uint8_t BankSel = 0x76;

        struct DeviceConfig {
            static constexpr uint8_t reg = 0x11;
            static constexpr uint8_t valueSwReset = 1;
        };
        struct IntfConfig0 {
            static constexpr uint8_t reg = 0x4c;
            static constexpr uint8_t value = (0 << 4) | (0 << 5) | (0 << 6); //fifo count in LE, sensor data in LE, fifo size in bytes
        };
        struct IntfConfig1 {
            static constexpr uint8_t reg = 0x4d; //Reset value: 0x91
            static constexpr uint8_t value = (0b1001 << 4) | (0 << 3) | (1 << 2) | (0b01); //Reserved[7:4](0b1001), ACCEL_LP_CLK_SEL[3], RTC_MODE[2], CLKSEL[1:0]
        };
        struct IntfConfig5 {
            static constexpr uint8_t reg = 0x7b;
            static constexpr uint8_t value = (0b10 << 1); // PIN9_FUNCTION = CLKIN
        };
        struct FifoConfig0 {
            static constexpr uint8_t reg = 0x16;
            static constexpr uint8_t value = (0b01 << 6); //stream to FIFO mode
        };
        struct FifoConfig1 {
            static constexpr uint8_t reg = 0x5f;
            static constexpr uint8_t value = 0b1 | (0b1 << 1) | (0b0 << 2); //fifo accel en=1, gyro=1, temp=0 todo: fsync, hires
        };
        struct GyroConfig {
            static constexpr uint8_t reg = 0x4f;
            static constexpr uint8_t value = (0b001 << 5) | 0b0111; //1000dps, odr=200Hz
        };
        struct AccelConfig {
            static constexpr uint8_t reg = 0x50;
            static constexpr uint8_t value = (0b001 << 5) | 0b1000; //8g, odr = 100Hz
        };
        struct PwrMgmt {
            static constexpr uint8_t reg = 0x4e;
            static constexpr uint8_t value = 0b11 | (0b11 << 2); //accel in low noise mode, gyro in low noise
        };

        // TODO: might be worth checking
        //GYRO_CONFIG1
        //GYRO_ACCEL_CONFIG0
        //ACCEL_CONFIG1

        static constexpr uint8_t FifoCount = 0x2e;
        static constexpr uint8_t FifoData = 0x30;
    };

    #pragma pack(push, 1)
    struct FifoEntryAligned {
        union {
            struct {
                int16_t accel[3];
                int16_t gyro[3];
                uint8_t temp;
                uint8_t timestamp[2]; // cannot do uint16_t because it's unaligned
            } part;
            uint8_t raw[15];
        };
    };
    #pragma pack(pop)

    static constexpr size_t FullFifoEntrySize = 16;

    bool initialize()
    {
        // perform initialization step
        i2c.writeReg(Regs::DeviceConfig::reg, Regs::DeviceConfig::valueSwReset);
        delay(20);

        i2c.writeReg(Regs::IntfConfig0::reg, Regs::IntfConfig0::value);
        //i2c.writeReg(0x4D, 0x90);
        i2c.writeReg(Regs::IntfConfig1::reg, Regs::IntfConfig1::value);
        i2c.writeReg(Regs::BankSel, 1);
        i2c.writeReg(Regs::IntfConfig5::reg, Regs::IntfConfig5::value);
        i2c.writeReg(Regs::BankSel, 0);
        i2c.writeReg(Regs::GyroConfig::reg, Regs::GyroConfig::value);
        i2c.writeReg(Regs::AccelConfig::reg, Regs::AccelConfig::value);
        i2c.writeReg(Regs::FifoConfig0::reg, Regs::FifoConfig0::value);
        i2c.writeReg(Regs::FifoConfig1::reg, Regs::FifoConfig1::value);
        i2c.writeReg(Regs::PwrMgmt::reg, Regs::PwrMgmt::value);
        delay(1);

        return true;
    }

    float getDirectTemp() const
    {
        const auto value = static_cast<int16_t>(i2c.readReg16(Regs::TempData));
        float result = ((float)value / 132.48f) + 25.0f;
        return result;
    }

    template <typename AccelCall, typename GyroCall>
    void bulkRead(AccelCall &&processAccelSample, GyroCall &&processGyroSample) {
        const auto fifo_bytes = i2c.readReg16(Regs::FifoCount);
        
        std::array<uint8_t, FullFifoEntrySize * 8> read_buffer; // max 8 readings
        
        // Flush FIFO if excessive data in FIFO
        if (fifo_bytes > read_buffer.size()) {
            i2c.writeReg(0x4B, 0x02);
            i2c.writeReg(0x4B, 0x00);
        }

        const auto bytes_to_read = std::min(static_cast<size_t>(read_buffer.size()),
            static_cast<size_t>(fifo_bytes)) / FullFifoEntrySize * FullFifoEntrySize;
        i2c.readBytes(Regs::FifoData, bytes_to_read, read_buffer.data());
        for (auto i=0u; i<bytes_to_read; i+=FullFifoEntrySize) {
            FifoEntryAligned entry;
            memcpy(entry.raw, &read_buffer[i+0x1], sizeof(FifoEntryAligned)); // skip fifo header
            processGyroSample(entry.part.gyro, GyrTs);

            if (entry.part.accel[0] != -32768) {
                processAccelSample(entry.part.accel, AccTs);
            }            
        }      
    }

};

} // namespace