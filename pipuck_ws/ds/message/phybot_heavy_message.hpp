#ifndef PHYBOT_HEAVY_MESSAGE_H
#define PHYBOT_HEAVY_MESSAGE_H

#include "phybot_message.hpp"

namespace argos {

    /**
     * @class CPhybotHeavyMessage
     * @brief Full-precision RAB message variant using Real (double) for all fields.
     *
     * Uses the ARGoS byte-stream << / >> operators to (de)serialize the five tuple fields in a fixed order.
     */
    class CPhybotHeavyMessage : public CPhybotMessage {
    public:
        /**
         * @brief Serializes all five tuple fields into a byte array.
         * @return byte payload ready for RAB transmission.
         */
        CByteArray serialize() const override;

        /**
         * @brief Deserializes a byte array back into the tuple fields.
         * @param msgBytes byte payload received from the RAB sensor.
         */
        void deserialize(CByteArray& msgBytes) override;
    };

}

#endif
