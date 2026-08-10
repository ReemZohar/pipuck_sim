#include <gtest/gtest.h>

#include "../../ds/message/phybot_message.hpp"
#include "../../ds/message/phybot_heavy_message.hpp"
#include "../../ds/message/phybot_message_list.hpp"

namespace argos {
namespace {

// Helper function to initialize a message with 0 values and the chosen timestamp.
void initMsg(CPhybotMessage& msg, uint32_t timestamp) {
    msg.m_fRelativeLocation = 0;
    msg.m_fSenderEstPressure = 0;
    msg.m_fEdgeConductivity = 0;
    msg.m_fEdgeFlow = 0;
    msg.m_unTimestamp = timestamp;
}

// Test that a newly created message list is empty.
TEST(PhybotMessageListTest, StartsEmpty) {
    CPhybotMessageList emptyList;

    // Both lists should be empty
    EXPECT_EQ(emptyList.getIncomingMessages().size(), 0);
    EXPECT_EQ(emptyList.getOutgoingMessages().size(), 0);
}

// Tests that 1 message is added successfully to each message list.
TEST(PhybotMessageListTest, AddsOneMessagePerList) {
    CPhybotMessageList list;
    size_t expectedInSize = 1, expectedOutSize = 1;

    // Random timestamp values for the messages
    uint32_t t1 = 1, t2 = 19;

    std::unique_ptr<CPhybotMessage> incMsg1 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*incMsg1, t1);
    std::unique_ptr<CPhybotMessage> outMsg1 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*outMsg1, t2);

    list.addMessage(std::move(incMsg1), true);
    list.addMessage(std::move(outMsg1), false);

    EXPECT_EQ(list.getIncomingMessages().size(), expectedInSize);
    EXPECT_EQ(list.getOutgoingMessages().size(), expectedOutSize);
}

// Tests the general case where several messages are added to each message list.
TEST(PhybotMessageListTest, AddsSeveralMessages) {
    CPhybotMessageList list;
    size_t expectedInSize = 4, expectedOutSize = 2;

    // Random timestamp values for the messages
    uint32_t t1 = 1, t2 = 73, t3 = 56, t4 = 21, t5 = 19, t6 = 1000;

    std::unique_ptr<CPhybotMessage> incMsg1 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*incMsg1, t1);
    std::unique_ptr<CPhybotMessage> incMsg2 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*incMsg2, t2);
    std::unique_ptr<CPhybotMessage> incMsg3 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*incMsg3, t3);
    std::unique_ptr<CPhybotMessage> incMsg4 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*incMsg4, t4);
    std::unique_ptr<CPhybotMessage> outMsg1 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*outMsg1, t5);
    std::unique_ptr<CPhybotMessage> outMsg2 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*outMsg2, t6);

    // 4 incoming and 2 outgoing message list test
    list.addMessage(std::move(incMsg1), true);
    list.addMessage(std::move(incMsg2), true);
    list.addMessage(std::move(incMsg3), true);
    list.addMessage(std::move(incMsg4), true);
    list.addMessage(std::move(outMsg1), false);
    list.addMessage(std::move(outMsg2), false);

    EXPECT_EQ(list.getIncomingMessages().size(), expectedInSize);
    EXPECT_EQ(list.getOutgoingMessages().size(), expectedOutSize);
}

// Testing a case where H is bigger than the current timestamp. no message should be removed.
TEST(PhybotMessageListTest, KeepsMessagesWhenHIsLarge) {
    CPhybotMessageList list;
    size_t expcInSize = 2, expcOutSize = 2;

    // timestamp values for the messages
    uint32_t t1 = 1, t2 = 2, t3 = 3, t4 = 4;
    uint32_t timestamp = 4;
    uint32_t H = 1000;

    std::unique_ptr<CPhybotMessage> incMsg1 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*incMsg1, t1);
    std::unique_ptr<CPhybotMessage> incMsg2 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*incMsg2, t2);
    std::unique_ptr<CPhybotMessage> outMsg1 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*outMsg1, t3);
    std::unique_ptr<CPhybotMessage> outMsg2 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*outMsg2, t4);

    // Add messages to the list
    list.addMessage(std::move(incMsg1), true);
    list.addMessage(std::move(incMsg2), true);
    list.addMessage(std::move(outMsg1), false);
    list.addMessage(std::move(outMsg2), false);
    
    list.removeOldMessages(timestamp, H);

    EXPECT_EQ(list.getIncomingMessages().size(), expcInSize);
    EXPECT_EQ(list.getOutgoingMessages().size(), expcOutSize);
}

// Testing the basic case where the list has 1 message in each list and both should be removed.
TEST(PhybotMessageListTest, RemovesExpiredMessage) {
    CPhybotMessageList list;

    // timestamp values for the messages
    uint32_t t1 = 5, t2 = 8;
    uint32_t timestamp = 20;
    uint32_t H = 10;

    std::unique_ptr<CPhybotMessage> incMsg = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*incMsg, t1);
    std::unique_ptr<CPhybotMessage> outMsg = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*outMsg, t2);

    // Add messages to the list
    list.addMessage(std::move(incMsg), true);
    list.addMessage(std::move(outMsg), false);

    list.removeOldMessages(timestamp, H);

    // Both lists should be empty after removal
    EXPECT_EQ(list.getIncomingMessages().size(), 0);
    EXPECT_EQ(list.getOutgoingMessages().size(), 0);
}

// Testing a general case where H is smaller than the current timestamp. (older messages should be removed)
TEST(PhybotMessageListTest, RemovesExpiredMessages) {
    
    CPhybotMessageList list;
    size_t expcInSize = 2, expcOutSize = 2;

    // timestamp values for the messages
    uint32_t tStay1 = 5, tStay2 = 7, tRemove1 = 0, tRemove2 = 3;
    uint32_t timestamp = 8;
    uint32_t H = 4;

    std::unique_ptr<CPhybotMessage> incMsg1 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*incMsg1, tRemove1);
    std::unique_ptr<CPhybotMessage> incMsg2 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*incMsg2, tRemove2);
    std::unique_ptr<CPhybotMessage> incMsg3 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*incMsg3, tStay1);
    std::unique_ptr<CPhybotMessage> incMsg4 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*incMsg4, tStay2);
    std::unique_ptr<CPhybotMessage> outMsg1 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*outMsg1, tRemove2);
    std::unique_ptr<CPhybotMessage> outMsg2 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*outMsg2, tStay1);
    std::unique_ptr<CPhybotMessage> outMsg3 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*outMsg3, tStay2);

    // Add messages to the list
    list.addMessage(std::move(incMsg1), true);
    list.addMessage(std::move(incMsg2), true);
    list.addMessage(std::move(incMsg3), true);
    list.addMessage(std::move(incMsg4), true);
    list.addMessage(std::move(outMsg1), false);
    list.addMessage(std::move(outMsg2), false);
    list.addMessage(std::move(outMsg3), false);

    // Incoming messages 1 and 2, and outgoing message 1 should be removed.
    list.removeOldMessages(timestamp, H);

    EXPECT_EQ(list.getIncomingMessages().size(), expcInSize);
    EXPECT_EQ(list.getOutgoingMessages().size(), expcOutSize);
}
} // namespace
} // namespace argos
