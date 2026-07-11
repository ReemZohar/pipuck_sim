#ifndef PHYBOT_MESSAGE_H
#define PHYBOT_MESSAGE_H

#include <cstdint>
#include <cstring>
#include <argos3/core/utility/datatypes/byte_array.h>

namespace argos {

   /**
    * @brief Message structure sent over the Range and Bearing (RAB) medium.
    * 
    * Conceptually represents the tuple <SenderPressure, EdgeConductivity, EdgeFlow, Timestamp>.
    * The spatial vector field of the tuple is sensed physically on reception and is not serialized.
    * 
    * NOTE: This struct must be exactly 10 bytes.
    */
   struct SPhybotMessage {
      _Float16 sender_est_pressure;      // Estimated pressure of the sender (p_j)
      _Float16 edge_conductivity;        // Conductivity of the shared channel (D_ji)
      _Float16 edge_flow;                // Flow along the channel (Q_ji)
      uint32_t timestamp;                // Time step counter (t)
   } __attribute__((packed));

   // CByteArray has no operator for _Float16, so we use raw byte copy to preserve the packed struct layout on the wire.
   inline CByteArray seriallize_msg(const SPhybotMessage& msg) {
      CByteArray c_bytes;
      c_bytes.AddBuffer(
         reinterpret_cast<const UInt8*>(&msg),
         sizeof(SPhybotMessage)
      );
      return c_bytes;
   }

   inline SPhybotMessage deserialize_msg(const CByteArray& c_bytes) {
      // Message was damaged scenario
      if(c_bytes.Size() < sizeof(SPhybotMessage)) {
         THROW_ARGOSEXCEPTION("CByteArray too small for SPhybotMessage");
      }
      SPhybotMessage msg;
      std::memcpy(&msg, c_bytes.ToCArray(), sizeof(SPhybotMessage));

      return msg;
   }

}

#endif
