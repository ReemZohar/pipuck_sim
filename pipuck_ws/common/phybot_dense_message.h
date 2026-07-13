#ifndef PHYBOT_DENSE_MESSAGE_H
#define PHYBOT_DENSE_MESSAGE_H

#include <cstdint>
#include <cstring>
#include <argos3/core/utility/datatypes/byte_array.h>

namespace argos {

    /**
     * @brief Dense message structure sent over the Range and Bearing (RAB) medium.
     *
     * Conceptually represents the tuple <SenderPressure, EdgeConductivity, EdgeFlow, Timestamp>.
     * The spatial vector field of the tuple is sensed physically on reception and is not serialized.
     * This structure compresses the data as much as possible, the tradeoff is less accurate field representation.
     *
     * NOTE: Real Pi-Puck robots can only send 2 bytes of data.
     */
    struct SPhybotDenseMessage {
        _Float16 senderEstPressure;
        _Float16 edgeConductivity;
        _Float16 edgeFlow;
        uint32_t timestamp;
    } __attribute__((packed));

    // CByteArray has no operator for _Float16, so we use raw byte copy.
    inline CByteArray seriallizeMsg(const SPhybotDenseMessage& msg) {
        CByteArray msgBytes;
        msgBytes.AddBuffer(
            reinterpret_cast<const UInt8*>(&msg),
            sizeof(SPhybotDenseMessage)
        );
        return msgBytes;
    }

    inline SPhybotDenseMessage deserializeMsg(const CByteArray& msgBytes) {
        if(msgBytes.Size() < sizeof(SPhybotDenseMessage)) {
            THROW_ARGOSEXCEPTION("CByteArray too small for SPhybotDenseMessage");
        }
        SPhybotDenseMessage msg;
        std::memcpy(&msg, msgBytes.ToCArray(), sizeof(SPhybotDenseMessage));
        return msg;
    }

}

#endif
