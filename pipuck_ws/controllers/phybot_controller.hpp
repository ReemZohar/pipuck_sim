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

#include "../algorithms/message/phybot_message.hpp"
#include "../algorithms/robot_role.hpp"

#include <deque>
#include <memory>

namespace argos {

    class CPhybotController : public CCI_Controller {

    public:
        CPhybotController() {}
        virtual ~CPhybotController() {}

        void Init(TConfigurationNode& t_tree) override;
        void ControlStep() override;
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

        // Di-PL parameters
        Real m_fAlpha;
        Real m_fKp;
        Real m_fGammaQ;
        Real m_fDeltaT;
        Real m_fI0;
        Real m_fPMax;

        // Motion parameters
        Real m_fWp;
        Real m_fBetaD;
        Real m_fAlphaD;
        Real m_fEpsilonD;
        Real m_fIe;
        Real m_fGamma;
        Real m_fK;
        Real m_fDs;
        u_int32_t m_unH;

        // State variables
        ERobotRole m_eRole;
        u_int32_t m_unTimestamp;
        Real m_fEsimatedPressure;
        Real m_fFoodReceived;
        Real m_fSectorConductivities[NUM_SECTORS];
        std::unique_ptr<CPhybotMessage> m_outMsg;
        std::deque<std::unique_ptr<CPhybotMessage>> m_messagesIn;
        std::deque<std::unique_ptr<CPhybotMessage>> m_messagesOut;

        void removeOldMessages();
        void removeOldMessages(std::deque<std::unique_ptr<CPhybotMessage>>& messages, u_int32_t minTimestamp);
        void updateLEDs();
    };
}
