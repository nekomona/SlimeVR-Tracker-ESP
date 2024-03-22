#pragma once

#include "softfusionsensor.h"

#include <vector>

namespace SlimeVR::Sensors
{

template <template<typename I2CImpl> typename T, typename I2CImpl>
class SoftFusionSensorBuffered : public SoftFusionSensor<T, I2CImpl>
{
    static constexpr auto BufferLength = 128;
    
    enum class RawDataPacketType:std::uint16_t {
        Unknown = 0,
        Gyro,
        Accel
    };

    struct SensorRawDataPacket {
        RawDataPacketType type;
        int16_t xyz[3];
    };

    std::vector<SensorRawDataPacket> m_buffer;

    using s = SoftFusionSensor<T, I2CImpl>;

    void setAcceleration(Vector3 a) 
    {
        s::acceleration = a;
        s::newAcceleration = true;
    }

    void setFusedRotation(Quat r) 
    {  
        s::fusedRotation = r * s::sensorOffset;
        bool changed = OPTIMIZE_UPDATES ? !s::lastFusedRotationSent.equalsWithEpsilon(s::fusedRotation) : true;
        if (ENABLE_INSPECTION || changed) {
            s::newFusedRotation = true;
            s::lastFusedRotationSent = s::fusedRotation;
        }
    }

    void processAccelSample(const int16_t xyz[3], const sensor_real_t timeDelta)
    {
        SensorRawDataPacket packet = {RawDataPacketType::Accel, xyz[0], xyz[1], xyz[2]};
        m_buffer.push_back(packet);
        if (m_buffer.size() >= BufferLength) {
            m_buffer.clear();
        }
    }

    void processGyroSample(const int16_t xyz[3], const sensor_real_t timeDelta)
    {
        SensorRawDataPacket packet = {RawDataPacketType::Gyro, xyz[0], xyz[1], xyz[2]};
        m_buffer.push_back(packet);
        if (m_buffer.size() >= BufferLength) {
            m_buffer.clear();
        }
    }

public:
    SoftFusionSensorBuffered(uint8_t id, uint8_t addrSuppl, float rotation, uint8_t sclPin, uint8_t sdaPin, uint8_t intPin)
    : SoftFusionSensor<T, I2CImpl>(id, addrSuppl, rotation, sclPin, sdaPin, intPin) {
        m_buffer.reserve(BufferLength);
    }

    void motionLoop() override
    {
        // sendTempIfNeeded();

        // read fifo updating fusion
        uint32_t now = micros();
        constexpr uint32_t targetPollIntervalMicros = 6000;
        uint32_t elapsed = now - s::m_lastPollTime;
        if (elapsed >= targetPollIntervalMicros) {
            s::m_lastPollTime = now - (elapsed - targetPollIntervalMicros);

            xSemaphoreTake(s::updateMutex, portMAX_DELAY);
            s::m_sensor.bulkRead(
                [&](const int16_t xyz[3], const sensor_real_t timeDelta) { processAccelSample(xyz, timeDelta); },
                [&](const int16_t xyz[3], const sensor_real_t timeDelta) { processGyroSample(xyz, timeDelta); }
            );
            xSemaphoreGive(s::updateMutex);

            optimistic_yield(100);
        }
    }

    bool hasNewDataToSend() override
    {
        if (xSemaphoreTake(s::updateMutex, 0) == pdTRUE) {
            for (auto & u : m_buffer) {
                if (u.type == RawDataPacketType::Gyro) {
                    s::processGyroSample(u.xyz, 0);
                } else if (u.type == RawDataPacketType::Accel) {
                    s::processAccelSample(u.xyz, 0);
                }
            }
            m_buffer.clear();

            xSemaphoreGive(s::updateMutex);
            optimistic_yield(100);
        }

        // send new fusion values when time is up
        uint32_t now = micros();
        constexpr float maxSendRateHz = 120.0f;
        constexpr uint32_t sendInterval = 1.0f/maxSendRateHz * 1e6;
        uint32_t elapsed = now - s::m_lastRotationPacketSent;
        if (elapsed >= sendInterval) {
            s::m_lastRotationPacketSent = now - (elapsed - sendInterval);

            setFusedRotation(s::m_fusion.getQuaternionQuat());
            setAcceleration(s::m_fusion.getLinearAccVec());
        }

        if (s::newFusedRotation) {
            return true;
        }
        return false;
    }

    void sendData() override
    {
        if (s::newFusedRotation) {
            s::newFusedRotation = false;
            Quat rotation = s::getFusedRotation();
            networkConnection.sendRotationData(s::sensorId, &rotation, DATA_TYPE_NORMAL, s::calibrationAccuracy);
            if (s::newAcceleration) {
                s::newAcceleration = false;
                networkConnection.sendSensorAcceleration(s::sensorId, s::acceleration);
            }
        }
    }

};

} // namespace
