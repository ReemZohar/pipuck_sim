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
        Real relativeLocation;
        Real senderEstPressure;
        Real edgeConductivity;
        Real edgeFlow;
        uint32_t timestamp;
        
        virtual ~CPhybotMessage() = default;
        virtual CByteArray serialize() const = 0;
        virtual void deserialize(CByteArray& msgBytes) = 0;
    };

}

#endif
