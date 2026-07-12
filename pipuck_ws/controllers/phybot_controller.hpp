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
#include <common/phybot_message.h>
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
      u_int32_t m_unH;

      u_int32_t m_unTimestamp;
      std::deque<SPhybotMessage> m_lstMessagesIn;
      std::deque<SPhybotMessage> m_lstMessagesOut;

      // Removes old messages from the incoming and outgoing message lists
      void remove_old_messages();
      // Removes old messages (sent/received before the minimum timestamp) from the given message list
      void remove_old_messages(std::deque<SPhybotMessage>& messages, u_int32_t min_timestamp);
   };
}
