#ifndef PHYBOT_CONTROLLER_HPP
#define PHYBOT_CONTROLLER_HPP

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_color_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_system_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/utility/datatypes/color.h>

#include "../ds/message/phybot_message.hpp"
#include "../ds/message/phybot_heavy_message.hpp"
#include "../ds/neighbor_reading.hpp"
#include "../algorithms/robot_role.hpp"
#include "../ds/message/phybot_message_list.hpp"
#include "../algorithms/di_pl.hpp"

#include <array>
#include <cmath>
#include <deque>
#include <memory>
#include <limits>
#include <vector>


namespace argos {

    /**
     * @class CPhybotController
     * @brief Pi-Puck agent controller that runs the Di-PL adaptation loop.
     *
     * Owns the full per-agent state (role, estimated pressure, per-sector
     * conductivities, RAB message archive) and the sensor/actuator handles. Each
     * tick it turns neighbor RAB packets into sector-indexed readings, selects the
     * lowest-pressure receiver per sector, and broadcasts its own state so the
     * swarm can route flux between sources and targets. All hyperparameters are
     * loaded from the .argos XML tree.
     */
    class CPhybotController : public CCI_Controller {

    public:
        /**
         * @brief Initializes the controller from the .argos XML configuration.
         *
         * Wires all sensor/actuator handles, parses hyperparameters, configures
         * the Di-PL algorithm, and resets all state to starting values.
         *
         * @param t_tree Root node of the controller's XML configuration block.
         */
        void Init(TConfigurationNode& t_tree) override;

        /**
         * @brief Runs one tick of the controller loop.
         *
         * Reads RAB neighbor packets, maps bearings to sectors, selects the
         * lowest-pressure receiver per sector, archives messages, and broadcasts
         * the controller's own state.
         */
        void ControlStep() override;

        /**
         * @brief Resets all internal state for a simulation restart.
         */
        void Reset() override;

    private:
        CCI_PiPuckDifferentialDriveActuator* m_pcWheels = nullptr;
        CCI_PiPuckColorLEDsActuator* m_pcColoredLEDs = nullptr;
        CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera = nullptr;
        CCI_PiPuckRangefindersSensor* m_pcRangefinders = nullptr;
        CCI_PiPuckSystemSensor* m_pcSystem = nullptr;
        CCI_RangeAndBearingSensor* m_pcRABSens = nullptr;
        CCI_RangeAndBearingActuator* m_pcRABAct = nullptr;

        // Parameters
        static constexpr u_int8_t NUM_SECTORS = 8;
        static constexpr Real CONDUCTIVITY_TARGET = 0.001f;
        static constexpr u_int32_t CONDUCTIVITY_DECAY_STEPS = 150;
        static constexpr Real LENGTH_NORMALIZATION_FACTOR = 0.75f;

        // Di-PL parameters
        Real m_fAlpha;
        Real m_fKp;
        Real m_fGammaQ;
        Real m_fDeltaT;
        Real m_fI0;
        Real m_fPMax;
        Real m_fDInit;
        Real m_fQc;

        // Motion parameters
        Real m_fWp;
        Real m_fBetaD;
        Real m_fAlphaD;
        Real m_fEpsilonD;
        Real m_fIe;
        Real m_fGamma;
        Real m_fK;
        Real m_fDs;
        Real m_fCommunicationRange;
        Real m_fLambdaL;
        Real m_fAlphaC;
        Real m_fAlphaS;
        Real m_fWd;
        Real m_fEpsilon;
        u_int32_t m_unH;

        // State variables
        ERobotRole m_eRole;
        u_int32_t m_unTimestamp;
        Real m_fEstimatedPressure;
        Real m_fFoodReceived;
        Real m_fSectorConductivities[NUM_SECTORS];
        CPhybotMessageList m_messageList;
        std::unique_ptr<CPhybotMessage> m_outMsg;

        void updateLEDs();
        void extractParameters(TConfigurationNode& t_tree);

        /**
         * @brief Derives paper-defined values from direct configuration inputs.
         */
        void calculateDerivedParameters();

        /**
         * @brief Selects the lowest-pressure neighbor in each angular sector.
         *
         * @param neighbors List of decoded neighbor readings from the current tick.
         * @return Array of one receiver pointer per sector (nullptr if no neighbor was heard).
         */
        std::array<const CPhybotMessage*, NUM_SECTORS> selectSectorReceivers(const std::vector<SNeighborReading>& neighbors);
    };
}

#endif
