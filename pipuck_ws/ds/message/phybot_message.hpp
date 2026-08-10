#ifndef PHYBOT_MESSAGE_H
#define PHYBOT_MESSAGE_H

#include <cstdint>
#include <argos3/core/utility/datatypes/byte_array.h>

namespace argos {

    /**
     * @class CPhybotMessage
     * @brief Abstract RAB message interface exchanged between Pi-Puck agents.
     *
     * Encodes the tuple:
     * <RelativeLocation, SenderPressure, EdgeConductivity, EdgeFlow, Timestamp>,
     * describing the sender's view of one channel so receivers
     * can update their pressure and conductivity estimates.
     */
    class CPhybotMessage {
    public:
        Real m_fRelativeLocation;
        Real m_fSenderEstPressure;
        Real m_fEdgeConductivity;
        Real m_fEdgeFlow;
        uint32_t m_unTimestamp;
        
        virtual ~CPhybotMessage() = default;

        /**
         * @brief Serializes the message fields into a byte array.
         * @return Byte representation ready for RAB transmission.
         */
        virtual CByteArray serialize() const = 0;

        /**
         * @brief Deserializes a byte array into the message fields.
         * @param msgBytes Byte array received from the RAB sensor.
         */
        virtual void deserialize(CByteArray& msgBytes) = 0;
    };

}

#endif
