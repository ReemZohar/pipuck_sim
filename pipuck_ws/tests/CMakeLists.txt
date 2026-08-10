include(GoogleTest)

function(add_phybot_test target)
  add_executable(${target} ${ARGN})
  target_link_libraries(${target}
    PRIVATE
      GTest::gtest_main
      argos3core_${ARGOS_BUILD_FOR}
      argos3plugin_${ARGOS_BUILD_FOR}_pipuck
  )
  gtest_discover_tests(${target})
endfunction()

# Add a target when promoting a snippet into a real test, for example:
# add_phybot_test(test_message
#   test_message.cpp
#   ../ds/message/phybot_heavy_message.cpp
# )
