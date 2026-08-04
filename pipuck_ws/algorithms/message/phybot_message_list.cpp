#include "phybot_message_list.hpp"

namespace argos {

    void CPhybotMessageList::addMessage(std::unique_ptr<CPhybotMessage> msg, bool isIncoming) {
        if (isIncoming) {
            m_messagesIn.push_back(std::move(msg));
        } else {
            m_messagesOut.push_back(std::move(msg));
        }
    }

    void CPhybotMessageList::clear() {
        m_messagesIn.clear();
        m_messagesOut.clear();
    }

    size_t CPhybotMessageList::getIncomingMsgSize() const {
        return m_messagesIn.size();
    }

    size_t CPhybotMessageList::getOutgoingMsgSize() const {
        return m_messagesOut.size();
    }

    void CPhybotMessageList::removeOldMessages(u_int32_t timestamp, u_int32_t H) {
        // If timestamp is less than or equal to H, there are no old messages to remove
        if(timestamp <= H) return;

        u_int32_t minTimestamp = timestamp - H;
        removeOldMessages(m_messagesIn, minTimestamp);
        removeOldMessages(m_messagesOut, minTimestamp);
    }


    void CPhybotMessageList::removeOldMessages(std::deque<std::unique_ptr<CPhybotMessage>>& messages, u_int32_t minTimestamp) {
        // Removes messages older than minTimestamp
        while (!messages.empty() && (messages.front()->m_unTimestamp < minTimestamp)) {
            messages.pop_front();
        }
    }

}