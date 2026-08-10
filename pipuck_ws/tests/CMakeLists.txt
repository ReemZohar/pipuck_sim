function(add_phybot_test test_target)
  add_executable(${test_target} ${ARGN})
  target_link_libraries(${test_target}
    PRIVATE
      GTest::gtest_main
      argos3core_${ARGOS_BUILD_FOR}
      argos3plugin_${ARGOS_BUILD_FOR}_pipuck
  )
  set_property(GLOBAL APPEND PROPERTY PHYBOT_TEST_TARGETS ${test_target})
endfunction()

# Each test file should be added as a separate add_phybot_test() call.
add_phybot_test(
  phybot_message_test
  ../ds/message/phybot_heavy_message.cpp
  message/phybot_message_test.cpp
)

# Used by run_tests.sh to get the list of test executables to run.
get_property(phybot_test_targets GLOBAL PROPERTY PHYBOT_TEST_TARGETS)
set(phybot_test_executables "")
foreach(phybot_test_target IN LISTS phybot_test_targets)
  string(APPEND phybot_test_executables "$<TARGET_FILE:${phybot_test_target}>\n")
endforeach()
file(GENERATE
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_executables.txt"
  CONTENT "${phybot_test_executables}"
)
