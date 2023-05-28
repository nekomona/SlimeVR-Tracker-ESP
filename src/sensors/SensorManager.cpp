/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2022 TheDevMinerTV

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "SensorManager.h"
#include <i2cscan.h>
#include <StreamString.h>
#include "network/network.h"
#include "bno055sensor.h"
#include "bno080sensor.h"
#include "mpu9250sensor.h"
#include "mpu6050sensor.h"
#include "bmi160sensor.h"
#include "icm20948sensor.h"
#include "ErroneousSensor.h"
#include "sensoraddresses.h"

#if ESP32
    #include "driver/i2c.h"
#endif

namespace SlimeVR
{
    namespace Sensors
    {
        #define STR_WRAP(STR) #STR
        #define STR_WRAP2(STR) STR_WRAP(STR)
        #define IMU_DESC_STR_VAL STR_WRAP2(IMU_DESC_STR)

        // Sensor descriptor string format:
        // imuType(address,rotation,sclpin,sdapin,intpin);

        int SensorManager::getIMUParamCount(int imu_type)
        {
            switch (imu_type) {
            case IMU_BNO080: case IMU_BNO085: case IMU_BNO086:
                return 6;
            default: case IMU_BNO055: case IMU_MPU9250: case IMU_BMI160: case IMU_MPU6500: case IMU_MPU6050: case IMU_ICM20948:
                return 5;
            }
        }

        Sensor* SensorManager::buildSensor(String &desc, uint8_t sensorID)
        {
            // First parse descritor and check variables
            uint8_t imuType = 0;
            uint8_t address = 0;
            float rotation = 0.0f;
            uint8_t sclPin = 0;
            uint8_t sdaPin = 0;
            uint8_t intPin = 0;

            int nparam = sscanf(desc.c_str(), "%hhu(0x%hhx,%f,%hhu,%hhu,%hhu)",
                                            &imuType, &address, &rotation, &sclPin, &sdaPin, &intPin);

            m_Logger.trace("Building IMU with: id=%d, nparam=%d,\n\
                            imuType=0x%02X, address=%d, rotation=%f,\n\
                            sclPin=%d, sdaPin=%d, intPin=%d",
                            sensorID, nparam,
                            imuType, address, rotation,
                            sclPin, sdaPin, intPin);
            
            if (nparam < 1 || nparam < getIMUParamCount(imuType)) {
                m_Logger.error("IMU %d have only %d parameters (expect %d) from desc %s",
                                sensorID, nparam, getIMUParamCount(imuType), desc.c_str() );
            }
            
            // Convert degrees to angle
            rotation *= PI / 180.0f;

            // Now start detecting and building the IMU
            Sensor* sensor = NULL;
            
            // Clear and reset I2C bus for each sensor upon startup
            I2CSCAN::clearBus(sdaPin, sclPin);
            swapI2C(sclPin, sdaPin);

            if (I2CSCAN::isI2CExist(address)) {
                m_Logger.trace("IMU %d found at address 0x%02X", sensorID, address);
            } else {
                sensor = new ErroneousSensor(sensorID, imuType);
                return sensor;
            }

            switch (imuType) {
            case IMU_BNO080: case IMU_BNO085: case IMU_BNO086:
                sensor = new BNO080Sensor(sensorID, imuType, address, rotation, sclPin, sdaPin, intPin);
                break;
            case IMU_BNO055:
                sensor = new BNO055Sensor(sensorID, address, rotation, sclPin, sdaPin);
                break;
            case IMU_MPU9250:
                sensor = new MPU9250Sensor(sensorID, address, rotation, sclPin, sdaPin);
                break;
            case IMU_BMI160:
                sensor = new BMI160Sensor(sensorID, address, rotation, sclPin, sdaPin);
                break;
            case IMU_MPU6500: case IMU_MPU6050:
                sensor = new MPU6050Sensor(sensorID, imuType, address, rotation, sclPin, sdaPin);
                break;
            case IMU_ICM20948:
                sensor = new ICM20948Sensor(sensorID, address, rotation, sclPin, sdaPin);
                break;
            default:
                sensor = new ErroneousSensor(sensorID, imuType);
                break;
            }

            sensor->motionSetup();
            return sensor;
        }

        void SensorManager::swapI2C(uint8_t sclPin, uint8_t sdaPin)
        {
            if (sclPin != activeSCL || sdaPin != activeSDA || !running) {
                Wire.flush();
                #if ESP32 
                    if (running) {}
                    else {
                        // Reset HWI2C to avoid being affected by I2CBUS reset
                        Wire.end();
                    }
                    // Disconnect pins from HWI2C
                    pinMode(activeSCL, INPUT);
                    pinMode(activeSDA, INPUT);
                    
                    if (running) {
                        i2c_set_pin(I2C_NUM_0, sdaPin, sclPin, false, false, I2C_MODE_MASTER);
                    } else {
                        Wire.begin(static_cast<int>(sdaPin), static_cast<int>(sclPin), I2C_SPEED); 
                        Wire.setTimeOut(150);
                    }
                #else
                    Wire.begin(static_cast<int>(sdaPin), static_cast<int>(sclPin));
                #endif

                activeSCL = sclPin;
                activeSDA = sdaPin;
            }
        }

        void SensorManager::setup()
        {
            running = false;
            activeSCL = PIN_IMU_SCL;
            activeSDA = PIN_IMU_SDA;

            // Divide sensor from descriptor string by semicolon
            StreamString mstr;
            mstr.print(IMU_DESC_STR_VAL);

            String desc;
            uint8_t sensorID = 0;
            while (mstr.available() && sensorID < MAX_IMU_COUNT) {
                desc = mstr.readStringUntil(';');
                if (desc.endsWith(")")) { // Verify the end of descriptor
                    Sensor* sensor = buildSensor(desc, sensorID);
                    m_Sensors[sensorID] = sensor;
                    sensorID++;
                } else {
                    m_Logger.error("Bad sensor descriptor %s\n", desc.c_str());
                }
            }
        }

        void SensorManager::postSetup()
        {
            running = true;
            for (auto sensor : m_Sensors) {
                if (sensor->isWorking()) {
                    swapI2C(sensor->sclPin, sensor->sdaPin);
                    sensor->postSetup();
                }
            }

            #if ESP32
                for (auto sensor : m_Sensors) {
                    sensor->updateMutex = xSemaphoreCreateMutex();
                }
                xTaskCreatePinnedToCore(updateSensors, "sensors", 16*1024, this, 10, &sensorTask, ARDUINO_RUNNING_CORE);
            #endif
        }
        
        #if ESP32
            void SensorManager::updateSensors(void * pxParameter) {
                SensorManager *pthis = (SensorManager *)pxParameter;
                for (;;) {
                    for (auto sensor : pthis->m_Sensors) {
                        if (sensor->isWorking()) {
                            pthis->swapI2C(sensor->sclPin, sensor->sdaPin);
                            sensor->motionLoop();
                        }
                    }
                }
            }
        #endif

        void SensorManager::update()
        {
            #if ESP32
            #else
            // Gather IMU data
            for (auto sensor : m_Sensors) {
                if (sensor->isWorking()) {
                    swapI2C(sensor->sclPin, sensor->sdaPin);
                    sensor->motionLoop();
                }
            }
            #endif

            if (!ServerConnection::isConnected())
            {
                return;
            }

            // Send updates
            for (auto sensor : m_Sensors) {
                if (sensor->isWorking()) {
                    sensor->sendData();
                }
            }
        }

    }
}
