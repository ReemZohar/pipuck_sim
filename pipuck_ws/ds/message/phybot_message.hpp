#ifndef PHYBOT_MESSAGE_H
#define PHYBOT_MESSAGE_H

#include <cstdint>
#include <argos3/core/utility/datatypes/byte_array.h>

namespace argos {

    /**
     * @brief Abstract interface for Pi-Puck RAB message serialization.
     *
     * Represents the tuple <RelativeLocation, SenderPressure, EdgeConductivity, EdgeFlow, Timestamp>.
     */
    class CPhybotMessage {
    public:
        Real m_fRelativeLocation;
        Real m_fSenderEstPressure;
        Real m_fEdgeConductivity;
        Real m_fEdgeFlow;
        uint32_t m_unTimestamp;
        
        virtual ~CPhybotMessage() = default;
        virtual CByteArray serialize() const = 0;
        virtual void deserialize(CByteArray& msgBytes) = 0;
    };

}

#endif
