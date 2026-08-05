#include "loop_functions.hpp"

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
