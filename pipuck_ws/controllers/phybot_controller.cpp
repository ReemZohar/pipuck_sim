#include "phybot_controller.hpp"
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/utility/datatypes/color.h>

namespace argos {

	void CPhybotController::Init(TConfigurationNode& t_tree) {
		/* Get the actuators and sensors */
		m_pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
		m_pcColoredLEDs = GetActuator<CCI_PiPuckColorLEDsActuator>("pipuck_leds");
		m_pcSystem = GetSensor<CCI_PiPuckSystemSensor>("pipuck_system");
		m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
		m_pcRangefinders = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");
		m_pcRABSens = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
		m_pcRABAct = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
	}

	void CPhybotController::ControlStep() {
      m_pcWheels->SetLinearVelocity(5.0f, 5.0f);

		/* Clear the data buffer before writing new data */
		m_pcRABAct->ClearData();

		/* Only set data if size is at least 2 */
		if (m_pcRABAct->GetSize() >= 2) {
			m_pcRABAct->SetData(0, 42);
			m_pcRABAct->SetData(1, 13);
		}

		/* Log the size of the data we can send */
		RLOG << "RAB Max Data Size: " << m_pcRABAct->GetSize() << std::endl;

		/* Read RAB data */
		const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABSens->GetReadings();
		for(size_t i = 0; i < tPackets.size(); ++i) {
			RLOG << "Received RAB message from dist " << tPackets[i].Range
			     << " angle: " << tPackets[i].HorizontalBearing
			     << " size: " << tPackets[i].Data.Size();
			if (tPackets[i].Data.Size() >= 2) {
				RLOG << " with data[0]=" << (int)tPackets[i].Data[0]
				     << " data[1]=" << (int)tPackets[i].Data[1];
			}
			RLOG << std::endl;
		}
	}

	void CPhybotController::Reset() {
		/* Reset is empty for the Phase 0 skeleton.
		   State variables will be added in Phase 1 and reinitialized here. */
	}

	REGISTER_CONTROLLER(CPhybotController, "phybot_controller");
}
