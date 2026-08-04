#include "phybot_heavy_message.hpp"

namespace argos {

    CByteArray CPhybotHeavyMessage::serialize() const {
        CByteArray msgBytes;
        msgBytes << relativeLocation
                 << senderEstPressure
                 << edgeConductivity
                 << edgeFlow
                 << timestamp;
        return msgBytes;
    }

    void CPhybotHeavyMessage::deserialize(CByteArray& msgBytes) {
        msgBytes >> relativeLocation
                 >> senderEstPressure
                 >> edgeConductivity
                 >> edgeFlow
                 >> timestamp;
    }

}
