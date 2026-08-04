#ifndef PHYBOT_MESSAGE_LIST_H
#define PHYBOT_MESSAGE_LIST_H

#include <argos3/core/utility/datatypes/byte_array.h>
#include <deque>
#include <memory>

#include "phybot_message.hpp"


namespace argos {

    /**
     * @brief Manages incoming and outgoing RAB message buffers with sliding window expiry.
     */
    class CPhybotMessageList {
    public:
        void addMessage(std::unique_ptr<CPhybotMessage> msg, bool isIncoming);
        void clear();
        size_t getIncomingMsgSize() const;
        size_t getOutgoingMsgSize() const;
        void removeOldMessages(u_int32_t timestamp, u_int32_t H);
        
    private:
        std::deque<std::unique_ptr<CPhybotMessage>> m_messagesIn;
        std::deque<std::unique_ptr<CPhybotMessage>> m_messagesOut;

        void removeOldMessages(std::deque<std::unique_ptr<CPhybotMessage>>& messages, u_int32_t minTimestamp);
    };

}

#endif