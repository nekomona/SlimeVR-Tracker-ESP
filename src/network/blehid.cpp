#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLEHIDDevice.h>

#include "blehid.h"

namespace SlimeVR {
namespace Network {

SlimeVR::Logging::Logger b_Logger = SlimeVR::Logging::Logger("BLEHID");

bool deviceConnected = false;

class ServerStatCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        b_Logger.info("Connected");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        b_Logger.info("Disconnected");
    }
};

BLEServer *pServer = nullptr;
BLEHIDDevice *pHid = nullptr;
BLECharacteristic *pCharInput = nullptr;
BLECharacteristic *pCharOutput = nullptr;
BLECharacteristic *pCharacteristic = nullptr;

const uint8_t hid_report[] = { //This is where the amount, type, and value range of the inputs are declared
	0x06, 0x00, 0xFF, // USAGE_PAGE (Vendor)
	0x09, 0x00, // USAGE (Undefined)
	0xa1, 0x01, // COLLECTION (Application)
	0x09, 0x00, // USAGE (Undefined)
	0x15, 0x00, // LOGICAL_MINIMUM (0)
	0x26, 0xFF, 0x00, // LOGICAL_MAXIMUM (255)
	0x85, 0x01, //   REPORT_ID (1)
	0x75, 0x08, // REPORT_SIZE (8)
	0x95, 0x13, // REPORT_COUNT (19)
	0x81, 0x02, // INPUT (Data,Var,Abs)

	0xc0 // END_COLLECTION
};

void BLEHID::setup()
{
    BLEDevice::init("Smol SlimeVR");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerStatCallbacks());
	pHid = new BLEHIDDevice(pServer);
	pCharInput = pHid->inputReport(1);

	std::string name = "SlimeVR";
	pHid->manufacturer()->setValue(name);

	// Somehow VID / PID are in wrong byte ordering
	// Should be 0x2FE3, 0x5652 matchin the one in USB dongle
	pHid->pnp(0x02, 0xE32F, 0x5256, 0x0210);
	pHid->hidInfo(0x00, 0x02);

	BLESecurity *pSecurity = new BLESecurity();
	//  pSecurity->setKeySize();
	pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);

	pHid->reportMap((uint8_t*)hid_report, sizeof(hid_report));
	pHid->startServices();

	BLEAdvertising *pAdvertising = pServer->getAdvertising();
	pAdvertising->setAppearance(HID_GAMEPAD);
	pAdvertising->addServiceUUID(pHid->hidService()->getUUID());
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x08);
	pHid->setBatteryLevel(7);

    pServer->startAdvertising();

	b_Logger.info("BLEHID ready");
}

bool oldDeviceConnected = false;

uint8_t inputValues[20] = {0};

// Use 2B pack, transmit 19B with the first one dropped
// HOGP will fill report ID into the first byte to form 20B packet
#pragma pack(2)
struct HidSensorReport {
	uint8_t __type; // Replaced by BLE HOGP report ID
	uint8_t sensorId;
	uint8_t rssi;
	uint8_t battPerc;
	uint16_t battMV;
	uint16_t q[4]; // w, x, y, z
	uint16_t a[3]; // x, y, z
};

HidSensorReport rpt;
#pragma pack()

bool newReport = false;

void BLEHID::update()
{
    if (!deviceConnected && oldDeviceConnected) {
        pServer->startAdvertising();
        b_Logger.info("Advertising");
        oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
		rpt.rssi = 9;
		// Windows won't prime notification when reconnected
		// So set notification up manually here
		BLE2902 *p2902 = (BLE2902*)pCharInput->getDescriptorByUUID((uint16_t)0x2902);
		p2902->setNotifications(true);
    }
	if (deviceConnected) {
		if (newReport) {
			pCharInput->setValue((uint8_t *)&rpt.sensorId, sizeof(rpt)-1);
			pCharInput->notify();
			newReport = false;
		}
	}
}

bool BLEHID::isConnected()
{
	return deviceConnected;
}

void BLEHID::sendBatteryLevel(float voltage, float battperc)
{
	rpt.battPerc = (uint8_t)(battperc * 100);
	rpt.battMV = (uint16_t)(voltage * 1000);
}

#define INT16_TO_UINT16(x) ((uint16_t)32768 + (uint16_t)(x))
#define TO_FIXED_14(x) ((int16_t)((x) * (1 << 14)))
#define TO_FIXED_10(x) ((int16_t)((x) * (1 << 10)))

void BLEHID::sendSensorData(uint8_t sensorId, Quat & quat, float accel[3])
{
	rpt.sensorId = sensorId;
	rpt.q[0] = INT16_TO_UINT16(TO_FIXED_14(quat.w));
	rpt.q[1] = INT16_TO_UINT16(TO_FIXED_14(quat.x));
	rpt.q[2] = INT16_TO_UINT16(TO_FIXED_14(quat.y));
	rpt.q[3] = INT16_TO_UINT16(TO_FIXED_14(quat.z));
	rpt.a[0] = INT16_TO_UINT16(TO_FIXED_10(accel[0]));
	rpt.a[1] = INT16_TO_UINT16(TO_FIXED_10(accel[1]));
	rpt.a[2] = INT16_TO_UINT16(TO_FIXED_10(accel[2]));
	newReport = true;
}

}  // namespace Network
}  // namespace SlimeVR
