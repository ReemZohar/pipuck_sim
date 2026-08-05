#ifndef DI_PL_H
#define DI_PL_H

#include <argos3/core/utility/datatypes/datatypes.h>
#include <deque>

#include "../ds/message/phybot_message_list.hpp"

namespace argos {
    // TODO: update the documentation with a description of the di-pl algorithm
    /**
     * @brief Class for the Distributed Information Processing and Learning (DI-PL) algorithm.
     */
    class CDiPL {
        public:
        // Sets the algorithm hyperparameters
        static void setParameters(Real Kp, Real alpha, Real deltaT);
        static Real updatePressure(const std::deque<std::unique_ptr<CPhybotMessage>>& msgList, Real oldPressure, Real xt);
        static Real calcTotalIncomingFlux(const std::deque<std::unique_ptr<CPhybotMessage>>& incMsgList, Real oldFlux);
        static Real calcOutgoingEstFlux(const std::unique_ptr<CPhybotMessage>& msg, Real pressure);
        static Real calcOutgoingFlux(Real estFlux, Real totalFlux, Real totalEstFlux);
        static Real updateConductivity(Real oldConductivity, Real flux, u_int32_t timestepSize);
        static Real calcIncomingPressure(const std::deque<std::unique_ptr<CPhybotMessage>>& incMsgList);
        static Real calcOutgoingPressure(const std::deque<std::unique_ptr<CPhybotMessage>>& outMsgList, Real totalPressures);

        private:
        static constexpr Real EPSILON = 1e-6f;

        static Real m_fKp;
        static Real m_fAlpha;
        static Real m_fDeltaT;
        
        // Safe decay function
        static Real g(Real newPressure, Real oldPressure);
    };
}

#endif