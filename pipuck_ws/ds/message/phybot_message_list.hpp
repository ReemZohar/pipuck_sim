#ifndef PHYBOT_MESSAGE_LIST_H
#define PHYBOT_MESSAGE_LIST_H

#include <argos3/core/utility/datatypes/byte_array.h>
#include <deque>
#include <memory>

#include "phybot_message.hpp"


namespace argos {

    /**
     * @class CPhybotMessageList
     * @brief Agent's RAB message archive with sliding-window expiry.
     *
     * Maintains separate deques of unique_ptr messages for the incoming and outgoing
     * buffers, and drops messages that fall outside the H-timestamp sliding window
     * via removeOldMessages(). This archive is the message history M(i) that the
     * Di-PL aggregations iterate over.
     */
    class CPhybotMessageList {
    public:
        /**
         * @brief Adds a message to the list.
         *
         * @param msg Message to archive (ownership is transferred).
         * @param isIncoming True to route into the incoming buffer, false for outgoing.
         */
        void addMessage(std::unique_ptr<CPhybotMessage> msg, bool isIncoming);

        /**
         * @brief Clears both incoming and outgoing buffers.
         */
        void clear();

        /**
         * @brief Returns the incoming message buffer.
         * @return Const reference to the incoming message deque.
         */
        const std::deque<std::unique_ptr<CPhybotMessage>>& getIncomingMessages() const;

        /**
         * @brief Returns the outgoing message buffer.
         * @return Const reference to the outgoing message deque.
         */
        const std::deque<std::unique_ptr<CPhybotMessage>>& getOutgoingMessages() const;

        /**
         * @brief Drops messages older than the sliding-window threshold.
         *
         * @param timestamp Current simulation tick.
         * @param H Window size in ticks. messages older than (timestamp - H) are removed.
         */
        void removeOldMessages(u_int32_t timestamp, u_int32_t H);
        
    private:
        std::deque<std::unique_ptr<CPhybotMessage>> m_messagesIn;
        std::deque<std::unique_ptr<CPhybotMessage>> m_messagesOut;

        void removeOldMessages(std::deque<std::unique_ptr<CPhybotMessage>>& messages, u_int32_t minTimestamp);
    };

}

#endif