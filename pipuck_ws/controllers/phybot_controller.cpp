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

		m_unTimestamp = 0;
	}

	void CPhybotController::ControlStep() {
      	m_pcWheels->SetLinearVelocity(5.0f, 5.0f);

		/* Serialize and broadcast an outgoing message */
		SPhybotMessage outMsg = {0, 0, 0, 0};
		m_pcRABAct->ClearData();
		m_pcRABAct->SetData(seriallizeMsg(outMsg));

		/* Read and deserialize incoming RAB messages */
		const CCI_RangeAndBearingSensor::TReadings& packets = m_pcRABSens->GetReadings();
		for(size_t i = 0; i < packets.size(); ++i) {
			if(packets[i].Data.Size() >= sizeof(SPhybotMessage)) {
				SPhybotMessage inMsg = deserializeMsg(packets[i].Data);
				RLOG << "Received: pressure=" << static_cast<float>(inMsg.senderEstPressure)
				     << " conductivity=" << static_cast<float>(inMsg.edgeConductivity)
				     << " flow=" << static_cast<float>(inMsg.edgeFlow)
				     << " timestamp=" << inMsg.timestamp << std::endl;
			}
		}

		m_unTimestamp++;
	}

	void CPhybotController::Reset() {
		m_unTimestamp = 0;
		m_messagesIn.clear();
		m_messagesOut.clear();
	}

	void CPhybotController::removeOldMessages() {
		if(m_unTimestamp <= m_unH) return;

		u_int32_t minTimestamp = m_unTimestamp - m_unH;
		removeOldMessages(m_messagesIn, minTimestamp);
		removeOldMessages(m_messagesOut, minTimestamp);
	}

	void CPhybotController::removeOldMessages(std::deque<SPhybotMessage>& messages, u_int32_t minTimestamp) {
		while(!messages.empty() && (minTimestamp > messages.front().timestamp)) {
			messages.pop_front();
		}
	}

	REGISTER_CONTROLLER(CPhybotController, "phybot_controller");
}
