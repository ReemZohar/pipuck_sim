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
      _Float16 senderEstPressure;      // Estimated pressure of the sender (p_j)
      _Float16 edgeConductivity;        // Conductivity of the shared channel (D_ji)
      _Float16 edgeFlow;                // Flow along the channel (Q_ji)
      uint32_t timestamp;               // Time step counter (t)
   } __attribute__((packed));

   // CByteArray has no operator for _Float16, so we use raw byte copy to preserve the packed struct layout on the wire.
   inline CByteArray seriallizeMsg(const SPhybotMessage& msg) {
      CByteArray cBytes;
      cBytes.AddBuffer(
         reinterpret_cast<const UInt8*>(&msg),
         sizeof(SPhybotMessage)
      );
      return cBytes;
   }

   inline SPhybotMessage deserializeMsg(const CByteArray& cBytes) {
      if(cBytes.Size() < sizeof(SPhybotMessage)) {
         THROW_ARGOSEXCEPTION("CByteArray too small for SPhybotMessage");
      }
      SPhybotMessage msg;
      std::memcpy(&msg, cBytes.ToCArray(), sizeof(SPhybotMessage));
      return msg;
   }

}

#endif
