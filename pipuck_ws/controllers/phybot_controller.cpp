#include "phybot_controller.hpp"
#include "../algorithms/message/phybot_heavy_message.hpp"

namespace argos {

    void CPhybotController::Init(TConfigurationNode& t_tree) {
        m_pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
        m_pcColoredLEDs = GetActuator<CCI_PiPuckColorLEDsActuator>("pipuck_leds");
        m_pcSystem = GetSensor<CCI_PiPuckSystemSensor>("pipuck_system");
        m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
        m_pcRangefinders = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");
        m_pcRABSens = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
        m_pcRABAct = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");

        GetNodeAttribute(GetNode(t_tree, "alpha"), "value", m_fAlpha);
        GetNodeAttribute(GetNode(t_tree, "kp"), "value", m_fKp);
        CDiPL::setParameters(m_fKp, m_fAlpha);

        GetNodeAttribute(GetNode(t_tree, "gammaQ"), "value", m_fGammaQ);
        GetNodeAttribute(GetNode(t_tree, "deltaT"), "value", m_fDeltaT);
        GetNodeAttribute(GetNode(t_tree, "i0"), "value", m_fI0);
        GetNodeAttribute(GetNode(t_tree, "pMax"), "value", m_fPMax);

        GetNodeAttribute(GetNode(t_tree, "wp"), "value", m_fWp);
        GetNodeAttribute(GetNode(t_tree, "betaD"), "value", m_fBetaD);
        GetNodeAttribute(GetNode(t_tree, "alphaD"), "value", m_fAlphaD);
        GetNodeAttribute(GetNode(t_tree, "epsilonD"), "value", m_fEpsilonD);
        GetNodeAttribute(GetNode(t_tree, "ie"), "value", m_fIe);
        GetNodeAttribute(GetNode(t_tree, "gamma"), "value", m_fGamma);
        GetNodeAttribute(GetNode(t_tree, "k"), "value", m_fK);
        GetNodeAttribute(GetNode(t_tree, "ds"), "value", m_fDs);
        GetNodeAttribute(GetNode(t_tree, "H"), "value", m_unH);

        m_outMsg = std::make_unique<CPhybotHeavyMessage>();

        m_unTimestamp = 0;
        m_fEsimatedPressure = 0;
        m_fFoodReceived = 0;
        m_eRole = ERobotRole::NORMAL;
        for(u_int8_t i = 0; i < NUM_SECTORS; i++) {
            m_fSectorConductivities[i] = 0;
        }
    }

    void CPhybotController::ControlStep() {
        m_pcWheels->SetLinearVelocity(5.0f, 5.0f);

        m_outMsg->relativeLocation = 0;
        m_outMsg->senderEstPressure = 0;
        m_outMsg->edgeConductivity = 0;
        m_outMsg->edgeFlow = 0;
        m_outMsg->timestamp = 0;
        m_pcRABAct->ClearData();
        m_pcRABAct->SetData(m_outMsg->serialize());

        const CCI_RangeAndBearingSensor::TReadings& packets = m_pcRABSens->GetReadings();
        for(size_t i = 0; i < packets.size(); ++i) {
            std::unique_ptr<CPhybotMessage> inMsg = std::make_unique<CPhybotHeavyMessage>();
            CByteArray data = packets[i].Data;
            inMsg->deserialize(data);
            RLOG << "Received: pressure=" << static_cast<float>(inMsg->senderEstPressure)
                 << " conductivity=" << static_cast<float>(inMsg->edgeConductivity)
                 << " flow=" << static_cast<float>(inMsg->edgeFlow)
                 << " timestamp=" << inMsg->timestamp << std::endl;
            m_messageList.addMessage(std::move(inMsg), true);
        }
        m_messageList.removeOldMessages(m_unTimestamp, m_unH);

        m_unTimestamp++;
    }

    void CPhybotController::Reset() {
        m_unTimestamp = 0;
        m_fEsimatedPressure = 0;
        m_fFoodReceived = 0;
        m_eRole = ERobotRole::NORMAL;
        m_messageList.clear();
        for(u_int8_t i = 0; i < NUM_SECTORS; i++) {
            m_fSectorConductivities[i] = 0;
        }
    }

    void CPhybotController::updateLEDs() {
        if(m_eRole == ERobotRole::NORMAL) m_pcColoredLEDs->SetRingLEDs(CColor::WHITE);
        else if(m_eRole == ERobotRole::SOURCE) m_pcColoredLEDs->SetRingLEDs(CColor::GREEN);
        else m_pcColoredLEDs->SetRingLEDs(CColor::BLUE);
    }

    REGISTER_CONTROLLER(CPhybotController, "phybot_controller");
}
