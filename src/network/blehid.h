#ifndef SLIMEVR_NETWORK_BLEHID_H_
#define SLIMEVR_NETWORK_BLEHID_H_

#include "globals.h"
#include "packets.h"
#include "logging/Logger.h"
#include <quat.h>

namespace SlimeVR {
namespace Network {

class BLEHID {
public:
	void setup();
	void update();

	bool isConnected();
	void sendBatteryLevel(float voltage, float battperc);
	void sendSensorData(uint8_t sensorId, Quat & quat, float accel[3]);
};

}  // namespace Network
}  // namespace SlimeVR

#endif  // SLIMEVR_NETWORK_BLEHID_H_
