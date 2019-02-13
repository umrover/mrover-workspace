#pragma once

namespace ctre {
	namespace phoenix {
		namespace unmanaged {
			/**
			 * Feed the robot enable.
			 * This function does nothing on a roborio during FRC use.
			 * @param timeoutMs Timeout before disabling
			 */
			void FeedEnable(int timeoutMs);
			/**
			 * @return true if enabled
			 */
			bool GetEnableState();
			/**
			 * Sets whether to enable transmitting
			 * This function does nothing on a roborio during FRC use.
			 * @param en True enables transmitting
			 */
			void SetTransmitEnable(bool en);
			/**
			 * @return true if transmitting is enabled
			 */
			bool GetTransmitEnable();
			/**
			 * @return Phoenix version
			 */
			int GetPhoenixVersion();
			void LoadPhoenix();
		}
	}
}
