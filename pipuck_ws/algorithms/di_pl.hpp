#ifndef DI_PL_H
#define DI_PL_H

#include <argos3/core/utility/datatypes/datatypes.h>
#include <deque>

#include "../ds/message/phybot_message_list.hpp"

namespace argos {
    // TODO: update the documentation with a description of the di-pl algorithm
    /**
     * @class CDiPL
     * @brief Stateless utility implementing the Distributed Plasticity Learning algorithm.
     *
     * Provides exponential-smoothing pressure updates with a safe decay, mean
     * incoming/outgoing pressure estimation, flux conservation across a vertex,
     * and the conductivity reinforcement/decay rule that prunes unused links.
     * All methods are static and share a single set of
     * hyperparameters configured once through setParameters().
     */
    class CDiPL {
        public:
        /**
         * @brief Sets the shared Di-PL hyperparameters.
         *
         * @param Kp Proportional gain for the safe decay function.
         * @param alpha Exponential smoothing factor for pressure updates.
         * @param deltaT Time step size for the conductivity update rule.
         */
        static void setParameters(Real Kp, Real alpha, Real deltaT);

        /**
         * @brief Updates the estimated pressure using exponential smoothing.
         *
         * @param msgList Message archive to aggregate over.
         * @param oldPressure Previous tick's pressure estimate.
         * @param xt New input value (e.g. local measurement).
         * @return Updated pressure estimate.
         */
        static Real updatePressure(const std::deque<std::unique_ptr<CPhybotMessage>>& msgList, Real oldPressure, Real xt);

        /**
         * @brief Computes the total incoming flux by summing edge flows.
         *
         * @param incMsgList Incoming message buffer.
         * @param oldFlux Previous total flux value.
         * @return Accumulated incoming flux.
         */
        static Real calcTotalIncomingFlux(const std::deque<std::unique_ptr<CPhybotMessage>>& incMsgList, Real oldFlux);

        /**
         * @brief Estimates the outgoing flux for a single edge.
         *
         * @param msg The message describing the channel to the receiver.
         * @param pressure The sender's current estimated pressure.
         * @return Estimated outgoing flux along that edge.
         */
        static Real calcOutgoingEstFlux(const std::unique_ptr<CPhybotMessage>& msg, Real pressure);

        /**
         * @brief Normalizes an estimated outgoing flux against the total.
         *
         * @param estFlux Raw estimated flux for one edge.
         * @param totalFlux Total incoming flux at the vertex.
         * @param totalEstFlux Sum of all estimated outgoing fluxes.
         * @return Normalized outgoing flux that conserves flow.
         */
        static Real calcOutgoingFlux(Real estFlux, Real totalFlux, Real totalEstFlux);

        /**
         * @brief Updates edge conductivity using the reinforcement/decay rule.
         *
         * @param oldConductivity Previous tick's conductivity value.
         * @param flux Current flux along the edge.
         * @param timestepSize Discrete time step size.
         * @return Updated conductivity.
         */
        static Real updateConductivity(Real oldConductivity, Real flux, u_int32_t timestepSize);

        /**
         * @brief Computes mean incoming pressure from neighbor messages.
         *
         * @param incMsgList Incoming message buffer.
         * @return Mean estimated pressure of incoming neighbors.
         */
        static Real calcIncomingPressure(const std::deque<std::unique_ptr<CPhybotMessage>>& incMsgList);

        /**
         * @brief Computes mean outgoing pressure toward receivers.
         *
         * @param outMsgList Outgoing message buffer.
         * @param totalPressures Accumulated pressure term to add.
         * @return Mean estimated pressure of outgoing receivers.
         */
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