#ifndef NEIGHBOR_READING_H
#define NEIGHBOR_READING_H

#include <memory>
#include <argos3/core/utility/datatypes/datatypes.h>

#include "message/phybot_message.hpp"

namespace argos {

    struct SNeighborReading {
        std::unique_ptr<CPhybotMessage> msg;
        Real range;
        CRadians bearing;
        u_int8_t sector;
    };

}

#endif
