/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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

#include "serialcommands.h"
#include "logging/Logger.h"
#include <CmdCallback.hpp>
#include "GlobalVars.h"
#include "batterymonitor.h"
#include "utils.h"

#if ESP32
    #include "nvs_flash.h"
#endif

namespace SerialCommands {
    SlimeVR::Logging::Logger logger("SerialCommands");

    CmdCallback<6> cmdCallbacks;
    CmdParser cmdParser;
    CmdBuffer<64> cmdBuffer;
	bool cmdFromRemote = false;

    void cmdSet(CmdParser * parser) {
        if(parser->getParamCount() != 1 && parser->equalCmdParam(1, "WIFI")  ) {
            if(parser->getParamCount() < 3) {
                logger.error("CMD SET WIFI ERROR: Too few arguments");
                logger.info("Syntax: SET WIFI \"<SSID>\" \"<PASSWORD>\"");
            } else {
                WiFiNetwork::setWiFiCredentials(parser->getCmdParam(2), parser->getCmdParam(3));
                logger.info("CMD SET WIFI OK: New wifi credentials set, reconnecting");
            }
        } else {
            logger.error("CMD SET ERROR: Unrecognized variable to set");
        }
    }

    void printState() {
        logger.info(
            "SlimeVR Tracker, board: %d, hardware: %d, build: %d, firmware: %s, address: %s, mac: %s, status: %d, wifi state: %d",
            BOARD,
            HARDWARE_MCU,
            FIRMWARE_BUILD_NUMBER,
            FIRMWARE_VERSION,
            WiFiNetwork::getAddress().toString().c_str(),
            WiFi.macAddress().c_str(),
            statusManager.getStatus(),
            WiFiNetwork::getWiFiState()
        );
        Sensor* sensor1 = sensorManager.getFirst();
        Sensor* sensor2 = sensorManager.getSecond();
        logger.info(
            "Sensor 1: %s (%.3f %.3f %.3f %.3f) is working: %s, had data: %s",
            getIMUNameByType(sensor1->getSensorType()),
            UNPACK_QUATERNION(sensor1->getFusedRotation()),
            sensor1->isWorking() ? "true" : "false",
            sensor1->hadData ? "true" : "false"
        );
        logger.info(
            "Sensor 2: %s (%.3f %.3f %.3f %.3f) is working: %s, had data: %s",
            getIMUNameByType(sensor2->getSensorType()),
            UNPACK_QUATERNION(sensor2->getFusedRotation()),
            sensor2->isWorking() ? "true" : "false",
            sensor2->hadData ? "true" : "false"
        );
    }

    void cmdGet(CmdParser * parser) {
#if USE_REMOTE_COMMAND && ALLOW_REMOTE_WIFI_PROV
		if (cmdFromRemote && !networkConnection.isConnected()) return;
#endif

        if (parser->getParamCount() < 2) {
            return;
        }

        if (parser->equalCmdParam(1, "INFO")) {
            printState();
        }

        if (parser->equalCmdParam(1, "CONFIG")) {
            logger.info("BOARD=%d", BOARD);
            logger.info("IMU=%d", IMU);
            logger.info("SECOND_IMU=%d", SECOND_IMU);
            logger.info("IMU_ROTATION=%f", IMU_ROTATION);
            logger.info("SECOND_IMU_ROTATION=%f", SECOND_IMU_ROTATION);
            logger.info("BATTERY_MONITOR=%d", BATTERY_MONITOR);
            logger.info("BATTERY_SHIELD_RESISTANCE=%d", BATTERY_SHIELD_RESISTANCE);
            logger.info("BATTERY_SHIELD_R1=%d", BATTERY_SHIELD_R1);
            logger.info("BATTERY_SHIELD_R2=%d", BATTERY_SHIELD_R2);
            logger.info("PIN_IMU_SDA=%d", PIN_IMU_SDA);
            logger.info("PIN_IMU_SCL=%d", PIN_IMU_SCL);
            logger.info("PIN_IMU_INT=%d", PIN_IMU_INT);
            logger.info("PIN_IMU_INT_2=%d", PIN_IMU_INT_2);
            logger.info("PIN_BATTERY_LEVEL=%d", PIN_BATTERY_LEVEL);
            logger.info("LED_PIN=%d", LED_PIN);
            logger.info("LED_INVERTED=%d", LED_INVERTED);
        }

        if (parser->equalCmdParam(1, "TEST")) {
            logger.info(
                "[TEST] Board: %d, hardware: %d, build: %d, firmware: %s, address: %s, mac: %s, status: %d, wifi state: %d",
                BOARD,
                HARDWARE_MCU,
                FIRMWARE_BUILD_NUMBER,
                FIRMWARE_VERSION,
                WiFiNetwork::getAddress().toString().c_str(),
                WiFi.macAddress().c_str(),
                statusManager.getStatus(),
                WiFiNetwork::getWiFiState()
            );
            Sensor* sensor1 = sensorManager.getFirst();
            sensor1->motionLoop();
            logger.info(
                "[TEST] Sensor 1: %s (%.3f %.3f %.3f %.3f) is working: %s, had data: %s",
                getIMUNameByType(sensor1->getSensorType()),
                UNPACK_QUATERNION(sensor1->getFusedRotation()),
                sensor1->isWorking() ? "true" : "false",
                sensor1->hadData ? "true" : "false"
            );
            if(!sensor1->hadData) {
                logger.error("[TEST] Sensor 1 didn't send any data yet!");
            } else {
                logger.info("[TEST] Sensor 1 sent some data, looks working.");
            }
        }
    }

    void cmdReboot(CmdParser * parser) {
#if USE_REMOTE_COMMAND && ALLOW_REMOTE_WIFI_PROV
		if (cmdFromRemote && !networkConnection.isConnected()) return;
#endif

        logger.info("REBOOT");
        ESP.restart();
    }

    void cmdFactoryReset(CmdParser * parser) {
#if USE_REMOTE_COMMAND && ALLOW_REMOTE_WIFI_PROV
		if (cmdFromRemote && !networkConnection.isConnected()) return;
#endif

        logger.info("FACTORY RESET");

        configuration.reset();

        WiFi.disconnect(true); // Clear WiFi credentials
        #if ESP8266
            ESP.eraseConfig(); // Clear ESP config
        #elif ESP32
            nvs_flash_erase();
        #else
            #warning SERIAL COMMAND FACTORY RESET NOT SUPPORTED
            logger.info("FACTORY RESET NOT SUPPORTED");
            return;
        #endif

        #if defined(WIFI_CREDS_SSID) && defined(WIFI_CREDS_PASSWD)
            #warning FACTORY RESET does not clear your hardcoded WiFi credentials!
            logger.warn("FACTORY RESET does not clear your hardcoded WiFi credentials!");
        #endif

        delay(3000);
        ESP.restart();
    }

    void cmdTemperatureCalibration(CmdParser* parser) {
#if USE_REMOTE_COMMAND && ALLOW_REMOTE_WIFI_PROV
		if (cmdFromRemote && !networkConnection.isConnected()) return;
#endif

        if (parser->getParamCount() > 1) {
            if (parser->equalCmdParam(1, "PRINT")) {
                sensorManager.getFirst()->printTemperatureCalibrationState();
                sensorManager.getSecond()->printTemperatureCalibrationState();
                return;
            } else if (parser->equalCmdParam(1, "DEBUG")) {
                sensorManager.getFirst()->printDebugTemperatureCalibrationState();
                sensorManager.getSecond()->printDebugTemperatureCalibrationState();
                return;
            } else if (parser->equalCmdParam(1, "RESET")) {
                sensorManager.getFirst()->resetTemperatureCalibrationState();
                sensorManager.getSecond()->resetTemperatureCalibrationState();
                return;
            } else if (parser->equalCmdParam(1, "SAVE")) {
                sensorManager.getFirst()->saveTemperatureCalibration();
                sensorManager.getSecond()->saveTemperatureCalibration();
                return;
            }
        }
        logger.info("Usage:");
        logger.info("  TCAL PRINT: print current temperature calibration config");
        logger.info("  TCAL DEBUG: print debug values for the current temperature calibration profile");
        logger.info("  TCAL RESET: reset current temperature calibration in RAM (does not delete already saved)");
        logger.info("  TCAL SAVE: save current temperature calibration to persistent flash");
        logger.info("Note:");
        logger.info("  Temperature calibration config saves automatically when calibration percent is at 100%");
    }

    void setUp() {
        cmdCallbacks.addCmd("SET", &cmdSet);
        cmdCallbacks.addCmd("GET", &cmdGet);
        cmdCallbacks.addCmd("FRST", &cmdFactoryReset);
        cmdCallbacks.addCmd("REBOOT", &cmdReboot);
        cmdCallbacks.addCmd("TCAL", &cmdTemperatureCalibration);
    }

    void update() {
        cmdCallbacks.updateCmdProcessing(&cmdParser, &cmdBuffer, &Serial);
        #if USE_REMOTE_COMMAND
        if (networkRemoteCmd.isConnected()) {
			Stream & networkStream = networkRemoteCmd.getStream();
			cmdFromRemote = true;
			while (networkStream.available()) {
				cmdCallbacks.updateCmdProcessing(&cmdParser, &cmdBuffer, &networkStream);
			}
			cmdFromRemote = false;
		}
        #endif
    }
}
