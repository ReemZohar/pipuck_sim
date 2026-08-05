#ifndef TEST_LOOP_FUNCTIONS_H
#define TEST_LOOP_FUNCTIONS_H

namespace argos {
    class CEmbodiedEntity;
}

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>


namespace argos {

    class CTestLoopFunctions : public CLoopFunctions {

    public:
        CTestLoopFunctions() {}
        virtual ~CTestLoopFunctions() {}

        virtual bool IsExperimentFinished() override;
    };
}

#endif
