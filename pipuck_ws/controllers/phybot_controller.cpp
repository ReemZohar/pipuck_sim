#include "phybot_controller.hpp"

namespace argos {


    void CPhybotController::Init(TConfigurationNode& t_tree) {
        m_pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
        m_pcColoredLEDs = GetActuator<CCI_PiPuckColorLEDsActuator>("pipuck_leds");
        m_pcSystem = GetSensor<CCI_PiPuckSystemSensor>("pipuck_system");
        m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
        m_pcRangefinders = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");
        m_pcRABSens = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
        m_pcRABAct = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");

        // Extracts all hyperparameters from the XML configuration file and sets them to the corresponding member variables.
        extractParameters(t_tree);

        // Sets the hyperparameters for the Di-PL algorithm
        CDiPL::setParameters(m_fKp, m_fAlpha, m_fDeltaT);

        m_outMsg = std::make_unique<CPhybotHeavyMessage>();

        m_unTimestamp = 0;
        m_fEstimatedPressure = 0;
        m_fFoodReceived = 0;
        m_eRole = ERobotRole::NORMAL;
        for(u_int8_t i = 0; i < NUM_SECTORS; i++) {
            m_fSectorConductivities[i] = 0;
        }
    }

    void CPhybotController::ControlStep() {
        m_pcWheels->SetLinearVelocity(5.0f, 5.0f);

        m_outMsg->m_fRelativeLocation = 0;
        m_outMsg->m_fSenderEstPressure = 0;
        m_outMsg->m_fEdgeConductivity = 0;
        m_outMsg->m_fEdgeFlow = 0;
        m_outMsg->m_unTimestamp = 0;
        m_pcRABAct->ClearData();
        m_pcRABAct->SetData(m_outMsg->serialize());

        std::vector<SNeighborReading> currentNeighborReadings;
        const CCI_RangeAndBearingSensor::TReadings& packets = m_pcRABSens->GetReadings();
        for(size_t i = 0; i < packets.size(); ++i) {
            std::unique_ptr<CPhybotMessage> inMsg = std::make_unique<CPhybotHeavyMessage>();
            CByteArray data = packets[i].Data;
            inMsg->deserialize(data);

            // Assign relative distance (Range) measured by RAB sensor
            inMsg->m_fRelativeLocation = packets[i].Range;

            // Calculate sector from horizontal bearing angle in [0, 2*pi)
            CRadians bearing = packets[i].HorizontalBearing;
            Real angle = bearing.UnsignedNormalize().GetValue();
            Real sectorAngle = (2.0 * M_PI) / NUM_SECTORS;
            u_int8_t sector = static_cast<u_int8_t>(angle / sectorAngle) % NUM_SECTORS;

            RLOG << "Received: pressure=" << static_cast<float>(inMsg->m_fSenderEstPressure)
                 << " conductivity=" << static_cast<float>(inMsg->m_fEdgeConductivity)
                 << " flow=" << static_cast<float>(inMsg->m_fEdgeFlow)
                 << " timestamp=" << inMsg->m_unTimestamp
                 << " sector=" << static_cast<int>(sector) << std::endl;

            SNeighborReading neighbor;
            neighbor.range = packets[i].Range;
            neighbor.bearing = packets[i].HorizontalBearing;
            neighbor.sector = sector;
            neighbor.msg = std::move(inMsg);

            currentNeighborReadings.push_back(std::move(neighbor));
        }

        // Selects the receiver with the lowest pressure per sector. One receiver per sector, valid until the messages are archived below.
        std::array<const CPhybotMessage*, NUM_SECTORS> chosenReceivers = selectSectorReceivers(currentNeighborReadings);

        // Di-PL per-tick updates (flux routing via chosenReceivers, conductivity)
        for(auto& neighbor : currentNeighborReadings) {
            if(neighbor.msg != nullptr) {
                m_messageList.addMessage(std::move(neighbor.msg), true);
            }
        }

        m_messageList.removeOldMessages(m_unTimestamp, m_unH);

        m_unTimestamp++;
    }

    std::array<const CPhybotMessage*, CPhybotController::NUM_SECTORS> CPhybotController::selectSectorReceivers(const std::vector<SNeighborReading>& neighbors) {
        std::array<const CPhybotMessage*, NUM_SECTORS> chosenReceivers{};
        Real minPressure[NUM_SECTORS];
        
        for(u_int8_t i = 0; i < NUM_SECTORS; ++i) {
            minPressure[i] = std::numeric_limits<Real>::max();
        }

        for(const auto& neighbor : neighbors) {
            if((neighbor.msg != nullptr) && (neighbor.sector < NUM_SECTORS)) {
                if(neighbor.msg->m_fSenderEstPressure < minPressure[neighbor.sector]) {
                    minPressure[neighbor.sector] = neighbor.msg->m_fSenderEstPressure;
                    chosenReceivers[neighbor.sector] = neighbor.msg.get();
                }
            }
        }

        return chosenReceivers;
    }


    void CPhybotController::Reset() {
        m_unTimestamp = 0;
        m_fEstimatedPressure = 0;
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

    void CPhybotController::extractParameters(TConfigurationNode& t_tree) {
        // Di-PL hyperparameters
        GetNodeAttribute(GetNode(t_tree, "alpha"), "value", m_fAlpha);
        GetNodeAttribute(GetNode(t_tree, "kp"), "value", m_fKp);
        GetNodeAttribute(GetNode(t_tree, "deltaT"), "value", m_fDeltaT);

        GetNodeAttribute(GetNode(t_tree, "gammaQ"), "value", m_fGammaQ);
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
    }

    REGISTER_CONTROLLER(CPhybotController, "phybot_controller");
}
