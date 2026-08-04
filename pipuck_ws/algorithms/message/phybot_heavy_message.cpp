#include "phybot_heavy_message.hpp"

namespace argos {

    CByteArray CPhybotHeavyMessage::serialize() const {
        CByteArray msgBytes;
        msgBytes << m_fRelativeLocation
                 << m_fSenderEstPressure
                 << m_fEdgeConductivity
                 << m_fEdgeFlow
                 << m_unTimestamp;
        return msgBytes;
    }

    void CPhybotHeavyMessage::deserialize(CByteArray& msgBytes) {
        msgBytes >> m_fRelativeLocation
                 >> m_fSenderEstPressure
                 >> m_fEdgeConductivity
                 >> m_fEdgeFlow
                 >> m_unTimestamp;
    }

}
