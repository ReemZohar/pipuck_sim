#include "loop_functions.hpp"
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

namespace argos {

    /****************************************/
    /****************************************/

    bool CTestLoopFunctions::IsExperimentFinished() {
        return false;
    }

    /****************************************/
    /****************************************/

    REGISTER_LOOP_FUNCTIONS(CTestLoopFunctions, "test_loop_functions");

}
