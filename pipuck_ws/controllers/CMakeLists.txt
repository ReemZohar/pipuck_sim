set(SRC
  phybot_controller.cpp
  ../algorithms/message/phybot_heavy_message.cpp
)

add_library(pipuck_phybot_controller MODULE ${SRC})

target_link_libraries(pipuck_phybot_controller
  PUBLIC
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_pipuck

)

target_include_directories(pipuck_phybot_controller
  PUBLIC
    ${ARGOS_INCLUDE_DIRS}
)

set_target_properties(pipuck_phybot_controller PROPERTIES
  OUTPUT_NAME pipuck_phybot_controller
)
