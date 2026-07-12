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

#include "../common/phybot_message.h"
#include "../common/robot_role.h"

#include <deque>

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
      // Pi-Puck has 8 rangefinders
      static constexpr u_int8_t NUM_SECTORS = 8;
      u_int32_t m_unH;

      // State variables
      ERobotRole m_eRole;
      u_int32_t m_unTimestamp;
      _Float16 m_fEsimatedPressure;
      _Float16 m_fFoodReceived;
      _Float16 m_fSectorConductivities[NUM_SECTORS];
      std::deque<SPhybotMessage> m_messagesIn;
      std::deque<SPhybotMessage> m_messagesOut;

      void removeOldMessages();
      void removeOldMessages(std::deque<SPhybotMessage>& messages, u_int32_t minTimestamp);
   };
}
