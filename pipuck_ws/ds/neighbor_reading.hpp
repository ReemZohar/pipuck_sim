#ifndef NEIGHBOR_READING_H
#define NEIGHBOR_READING_H

#include <memory>
#include <argos3/core/utility/datatypes/datatypes.h>

#include "message/phybot_message.hpp"

namespace argos {

    /**
     * @struct SNeighborReading
     * @brief Aggregates everything the controller learns about one neighbor in a tick.
     *
     * Associates the decoded RAB message with the range and horizontal bearing
     * measured by the local sensor, plus the angular sector derived from that bearing,
     * so controllers can perform per-sector receiver selection without recomputing
     * the bearing-to-sector mapping.
     */
    struct SNeighborReading {
        std::unique_ptr<CPhybotMessage> msg;
        Real range;
        CRadians bearing;
        u_int8_t sector;
    };

}

#endif
