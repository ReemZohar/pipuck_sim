#ifndef PHYBOT_HEAVY_MESSAGE_H
#define PHYBOT_HEAVY_MESSAGE_H

#include "phybot_message.hpp"

namespace argos {

    /**
     * @brief High-accuracy message using Real (double) for all fields.
     *
     * Serializes using << and >> operators for type-safe field-by-field transfer.
     */
    class CPhybotHeavyMessage : public CPhybotMessage {
    public:
        CByteArray serialize() const override;
        void deserialize(CByteArray& msgBytes) override;
    };

}

#endif
