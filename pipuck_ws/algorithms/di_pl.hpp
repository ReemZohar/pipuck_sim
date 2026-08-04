#ifndef DI_PL_H
#define DI_PL_H

#include <argos3/core/utility/datatypes/datatypes.h>

#include "message/phybot_message_list.hpp"

namespace argos {
    // TODO: update the documentation with a description of the di-pl algorithm
    /**
     * @brief Class for the Distributed Information Processing and Learning (DI-PL) algorithm.
     */
    class CDiPL {
        public:
        static void setParameters(Real Kp, Real alpha);
        static Real updatePressure(const CPhybotMessageList& msgList, Real oldPressure, bool isIncoming);
        static Real calcTotalIncomingFlux(const CPhybotMessageList& msgList, Real oldFlux);
        static Real calcOutgoingEstFlux(const std::unique_ptr<CPhybotMessage> msg, Real pressure);
        static Real calcOutgoingFlux(Real estFlux, Real totalFlux, Real totalEstFlux);
        static Real updateConductivity(Real oldConductivity, Real flux, u_int32_t timestepSize);

        private:
        static Real m_fKp;
        static Real m_fAlpha;

        static Real calcIncomingPressure(const CPhybotMessageList& msgList);
        static Real calcOutgoingPressure(const CPhybotMessageList& msgList);
        static Real g(Real newPressure);
    };
}

#endif