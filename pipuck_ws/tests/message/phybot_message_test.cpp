#include <gtest/gtest.h>

#include <argos3/core/utility/datatypes/byte_array.h>

#include "../../ds/message/phybot_message.hpp"
#include "../../ds/message/phybot_heavy_message.hpp"

namespace argos {
namespace {

// Test that the serialized size of a message is preserved and matches the expected size.
TEST(PhybotMessageTest, SizePreserved) {
    // Heavy message testing
    std::unique_ptr<CPhybotHeavyMessage> heavyMsg = std::make_unique<CPhybotHeavyMessage>();
    // Sum of all field sizes (double is 12 bytes in ARGoS's byte array)
    size_t expectedSize = 52;

    // Random field values
    heavyMsg->m_fRelativeLocation = 0.5;
    heavyMsg->m_fSenderEstPressure = 12.0;
    heavyMsg->m_fEdgeConductivity = 1.0;
    heavyMsg->m_fEdgeFlow = 0.25;
    heavyMsg->m_unTimestamp = 42;

    CByteArray msgBytes = heavyMsg->serialize();

    EXPECT_EQ(msgBytes.Size(), expectedSize);
}

TEST(PhybotMessageTest, SerializeDeserialize) {
    // Normal heavy message test
    std::unique_ptr<CPhybotHeavyMessage> heavyMsg = std::make_unique<CPhybotHeavyMessage>();

    // Random field values
    heavyMsg->m_fRelativeLocation = 0.5;
    heavyMsg->m_fSenderEstPressure = 12.0;
    heavyMsg->m_fEdgeConductivity = 1.0;
    heavyMsg->m_fEdgeFlow = 0.25;
    heavyMsg->m_unTimestamp = 42;

    CByteArray msgBytes = heavyMsg->serialize();

    std::unique_ptr<CPhybotHeavyMessage> received = std::make_unique<CPhybotHeavyMessage>();
    received->deserialize(msgBytes);

    // Checks that the deserialized values match the original values
    EXPECT_DOUBLE_EQ(received->m_fRelativeLocation, heavyMsg->m_fRelativeLocation);
    EXPECT_DOUBLE_EQ(received->m_fSenderEstPressure, heavyMsg->m_fSenderEstPressure);
    EXPECT_DOUBLE_EQ(received->m_fEdgeConductivity, heavyMsg->m_fEdgeConductivity);
    EXPECT_DOUBLE_EQ(received->m_fEdgeFlow, heavyMsg->m_fEdgeFlow);
    EXPECT_EQ(received->m_unTimestamp, heavyMsg->m_unTimestamp);

    // Test with maximum values
    std::unique_ptr<CPhybotHeavyMessage> heavyMaxMsg = std::make_unique<CPhybotHeavyMessage>();

    // Maximum possible field values
    heavyMaxMsg->m_fRelativeLocation = std::numeric_limits<Real>::max();
    heavyMaxMsg->m_fSenderEstPressure = std::numeric_limits<Real>::max();
    heavyMaxMsg->m_fEdgeConductivity = std::numeric_limits<Real>::max();
    heavyMaxMsg->m_fEdgeFlow = std::numeric_limits<Real>::max();
    heavyMaxMsg->m_unTimestamp = std::numeric_limits<uint32_t>::max();

    CByteArray maxMsgBytes = heavyMaxMsg->serialize();

    std::unique_ptr<CPhybotHeavyMessage> receivedMaxMsg = std::make_unique<CPhybotHeavyMessage>();
    receivedMaxMsg->deserialize(maxMsgBytes);

    // Checks that the deserialized values match the original values
    EXPECT_DOUBLE_EQ(receivedMaxMsg->m_fRelativeLocation, std::numeric_limits<Real>::max());
    EXPECT_DOUBLE_EQ(receivedMaxMsg->m_fSenderEstPressure, std::numeric_limits<Real>::max());
    EXPECT_DOUBLE_EQ(receivedMaxMsg->m_fEdgeConductivity, std::numeric_limits<Real>::max());
    EXPECT_DOUBLE_EQ(receivedMaxMsg->m_fEdgeFlow, std::numeric_limits<Real>::max());
    EXPECT_EQ(receivedMaxMsg->m_unTimestamp, std::numeric_limits<uint32_t>::max());

    // Test with minimum values
    std::unique_ptr<CPhybotHeavyMessage> heavyMinMsg = std::make_unique<CPhybotHeavyMessage>();

    // Minimum possible field values
    heavyMinMsg->m_fRelativeLocation = std::numeric_limits<Real>::min();
    heavyMinMsg->m_fSenderEstPressure = std::numeric_limits<Real>::min();
    heavyMinMsg->m_fEdgeConductivity = std::numeric_limits<Real>::min();
    heavyMinMsg->m_fEdgeFlow = std::numeric_limits<Real>::min();
    heavyMinMsg->m_unTimestamp = std::numeric_limits<uint32_t>::min();

    CByteArray minMsgBytes = heavyMinMsg->serialize();

    std::unique_ptr<CPhybotHeavyMessage> receivedMinMsg = std::make_unique<CPhybotHeavyMessage>();
    receivedMinMsg->deserialize(minMsgBytes);

    // Checks that the deserialized values match the original values
    EXPECT_DOUBLE_EQ(receivedMinMsg->m_fRelativeLocation, std::numeric_limits<Real>::min());
    EXPECT_DOUBLE_EQ(receivedMinMsg->m_fSenderEstPressure, std::numeric_limits<Real>::min());
    EXPECT_DOUBLE_EQ(receivedMinMsg->m_fEdgeConductivity, std::numeric_limits<Real>::min());
    EXPECT_DOUBLE_EQ(receivedMinMsg->m_fEdgeFlow, std::numeric_limits<Real>::min());
    EXPECT_EQ(receivedMinMsg->m_unTimestamp, std::numeric_limits<uint32_t>::min());
}

} // namespace
} // namespace argos
