#include "di_pl.hpp"

namespace argos {
    Real CDiPL::m_fKp = 0;
    Real CDiPL::m_fAlpha = 0;

    void CDiPL::setParameters(Real Kp, Real alpha) {
        m_fKp = Kp;
        m_fAlpha = alpha;
    }

    Real CDiPL::updatePressure(const std::deque<std::unique_ptr<CPhybotMessage>>& msgList, Real oldPressure, Real xt) {
        return (1 - m_fAlpha) * oldPressure + m_fAlpha * g(xt, oldPressure);
    }

    Real CDiPL::calcIncomingPressure(const std::deque<std::unique_ptr<CPhybotMessage>>& incMsgList) {
        Real pressure = 0;

        for (const auto& msg : incMsgList) {
            pressure += msg->m_fSenderEstPressure;
            // Epsilon is added to avoid division by zero in case the edge conductivity is zero.
            pressure -= (msg->m_fRelativeLocation * msg->m_fEdgeFlow) / (msg->m_fEdgeConductivity + EPSILON);
        }

        // We return the calculated pressure while avoiding division by zero in case the message list is empty.
        return incMsgList.empty() ? 0 : pressure / incMsgList.size();
    }

    Real CDiPL::calcOutgoingPressure(const std::deque<std::unique_ptr<CPhybotMessage>>& outMsgList, Real totalPressures) {
        Real pressure = totalPressures;

        for (const auto& msg : outMsgList) {
            // Epsilon is added to avoid division by zero in case the edge conductivity is zero.
            pressure += (msg->m_fRelativeLocation * msg->m_fEdgeFlow) / (msg->m_fEdgeConductivity + EPSILON);
        }

        // We return the calculated pressure while avoiding division by zero in case the message list is empty.
        return outMsgList.empty() ? 0 : pressure / outMsgList.size();
    }

    Real CDiPL::g(Real newPressure, Real oldPressure) {
        return newPressure == 0 && ((m_fKp > 0) && (m_fKp < 1)) ? oldPressure * m_fKp : newPressure;
    }
}