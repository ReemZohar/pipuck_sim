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
     * This structure represents the field values more accurately than SPhybotDenseMessage, but is larger in size.
     */
    struct SPhybotMessage {
        Real senderEstPressure;
        Real edgeConductivity;
        Real edgeFlow;
        uint32_t timestamp;
    };

    inline CByteArray seriallizeMsg(const SPhybotMessage& msg) {
        CByteArray msgBytes;
        msgBytes.AddBuffer(
            reinterpret_cast<const UInt8*>(&msg),
            sizeof(SPhybotMessage)
        );
        return msgBytes;
    }

    inline SPhybotMessage deserializeMsg(const CByteArray& msgBytes) {
        if(msgBytes.Size() < sizeof(SPhybotMessage)) {
            THROW_ARGOSEXCEPTION("CByteArray too small for SPhybotMessage");
        }
        SPhybotMessage msg;
        std::memcpy(&msg, msgBytes.ToCArray(), sizeof(SPhybotMessage));
        return msg;
    }

}

#endif
