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
		SPhybotMessage out_msg = {0, 0, 0, 0};
		m_pcRABAct->ClearData();
		m_pcRABAct->SetData(seriallize_msg(out_msg));

		/* Read and deserialize incoming RAB messages */
		const CCI_RangeAndBearingSensor::TReadings& packets = m_pcRABSens->GetReadings();
		for(size_t i = 0; i < packets.size(); ++i) {
			if(packets[i].Data.Size() >= sizeof(SPhybotMessage)) {
				SPhybotMessage in_msg = deserialize_msg(packets[i].Data);
				RLOG << "Received: pressure=" << static_cast<float>(in_msg.sender_est_pressure)
				     << " conductivity=" << static_cast<float>(in_msg.edge_conductivity)
				     << " flow=" << static_cast<float>(in_msg.edge_flow)
				     << " timestamp=" << in_msg.timestamp << std::endl;
			}
		}

		m_unTimestamp++;
	}

	void CPhybotController::Reset() {
		m_unTimestamp = 0;
		m_messages_in.clear();
		m_messages_out.clear();
	}

	void CPhybotController::remove_old_messages() {
		// No old messages yet scenario
		if(m_unTimestamp <= m_unH) return;

		u_int32_t min_timestamp = m_unTimestamp - m_unH;
		// Remove old messages from both incoming and outgoing message lists
		remove_old_messages(m_messages_in, min_timestamp);
		remove_old_messages(m_messages_out, min_timestamp);
	}

	void CPhybotController::remove_old_messages(std::deque<SPhybotMessage>& messages, u_int32_t min_timestamp) {
		// Remove old messages from the given message list
		while(!messages.empty() && (min_timestamp > messages.front().timestamp)) {
			messages.pop_front();
		}
	}

	REGISTER_CONTROLLER(CPhybotController, "phybot_controller");
}
