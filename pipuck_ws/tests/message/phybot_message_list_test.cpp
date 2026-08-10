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

// Tests that messages are added successfully to the message list.
TEST(PhybotMessageListTest, MessageAddition) {
    // Empty list test
    CPhybotMessageList emptyList;

    // Both lists should be empty
    EXPECT_EQ(emptyList.getIncomingMessages().size(), 0);
    EXPECT_EQ(emptyList.getOutgoingMessages().size(), 0);

    CPhybotMessageList list;
    size_t expectedInSize1 = 1, expectedOutSize1 = 1;
    size_t expectedInSize2 = 4, expectedOutSize2 = 2;

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

    // 1 incoming and 1 outgoing message list test
    list.addMessage(std::move(incMsg1), true);
    list.addMessage(std::move(outMsg1), false);
    
    EXPECT_EQ(list.getIncomingMessages().size(), expectedInSize1);
    EXPECT_EQ(list.getOutgoingMessages().size(), expectedOutSize1);

    // 4 incoming and 2 outgoing message list test
    list.addMessage(std::move(incMsg2), true);
    list.addMessage(std::move(incMsg3), true);
    list.addMessage(std::move(incMsg4), true);
    list.addMessage(std::move(outMsg2), false);

    EXPECT_EQ(list.getIncomingMessages().size(), expectedInSize2);
    EXPECT_EQ(list.getOutgoingMessages().size(), expectedOutSize2);
}

// Tests the removal of older messages from the list
TEST(PhybotMessageListTest, MessageRemoval) {
    // Testing a case where H is bigger than the current timestamp. no message should be removed.
    CPhybotMessageList bigHList;
    size_t expcBigHInSize = 2, expcBigHOutSize = 2;

    // timestamp values for the messages
    uint32_t t1 = 1, t2 = 2, t3 = 3, t4 = 4;
    uint32_t bigHTimestamp = 4;
    uint32_t bigH = 1000;

    std::unique_ptr<CPhybotMessage> incBigHMsg1 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*incBigHMsg1, t1);
    std::unique_ptr<CPhybotMessage> incBigHMsg2 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*incBigHMsg2, t2);
    std::unique_ptr<CPhybotMessage> outBigHMsg1 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*outBigHMsg1, t3);
    std::unique_ptr<CPhybotMessage> outBigHMsg2 = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*outBigHMsg2, t4);

    // Add messages to the list
    bigHList.addMessage(std::move(incBigHMsg1), true);
    bigHList.addMessage(std::move(incBigHMsg2), true);
    bigHList.addMessage(std::move(outBigHMsg1), false);
    bigHList.addMessage(std::move(outBigHMsg2), false);
    
    bigHList.removeOldMessages(bigHTimestamp, bigH);

    EXPECT_EQ(bigHList.getIncomingMessages().size(), expcBigHInSize);
    EXPECT_EQ(bigHList.getOutgoingMessages().size(), expcBigHOutSize);

    // Testing the basic case where the list has 1 message in each list and both should be removed.
    CPhybotMessageList basicList;

    // timestamp values for the messages
    uint32_t t5 = 5, t6 = 8;
    uint32_t basicTimestamp = 20;
    uint32_t basicH = 10;

    std::unique_ptr<CPhybotMessage> incBasicMsg = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*incBasicMsg, t5);
    std::unique_ptr<CPhybotMessage> outBasicMsg = std::make_unique<CPhybotHeavyMessage>();
    initMsg(*outBasicMsg, t6);

    // Add messages to the list
    basicList.addMessage(std::move(incBasicMsg), true);
    basicList.addMessage(std::move(outBasicMsg), false);

    basicList.removeOldMessages(basicTimestamp, basicH);

    // Both lists should be empty after removal
    EXPECT_EQ(basicList.getIncomingMessages().size(), 0);
    EXPECT_EQ(basicList.getOutgoingMessages().size(), 0);

    // Testing a general case where H is smaller than the current timestamp. (older messages should be removed)
    CPhybotMessageList normalList;
    size_t expcInSize = 2, expcOutSize = 2;

    // timestamp values for the messages
    uint32_t tStay1 = 5, tStay2 = 7, tRemove1 = 0, tRemove2 = 3;
    uint32_t normalTimestamp = 8;
    uint32_t normalH = 4;

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
    normalList.addMessage(std::move(incMsg1), true);
    normalList.addMessage(std::move(incMsg2), true);
    normalList.addMessage(std::move(incMsg3), true);
    normalList.addMessage(std::move(incMsg4), true);
    normalList.addMessage(std::move(outMsg1), false);
    normalList.addMessage(std::move(outMsg2), false);
    normalList.addMessage(std::move(outMsg3), false);

    // Incoming messages 1 and 2, and outgoing message 1 should be removed.
    normalList.removeOldMessages(normalTimestamp, normalH);

    EXPECT_EQ(normalList.getIncomingMessages().size(), expcInSize);
    EXPECT_EQ(normalList.getOutgoingMessages().size(), expcOutSize);
}
} // namespace
} // namespace argos
