#ifndef TEST_LOOP_FUNCTIONS_H
#define TEST_LOOP_FUNCTIONS_H

namespace argos {
    class CEmbodiedEntity;
}

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>


namespace argos {

    /**
     * @class CTestLoopFunctions
     * @brief Global experiment loop functions for the swarm simulation.
     *
     * The ARGoS hook that runs outside of any single robot: it currently decides
     * when the experiment ends, and per the project's architecture it is the only
     * place where global (non-local) swarm state may be observed or written.
     */
    class CTestLoopFunctions : public CLoopFunctions {

    public:
        CTestLoopFunctions() {}
        virtual ~CTestLoopFunctions() {}

        /**
         * @brief Checks whether the experiment should terminate.
         * @return True if the experiment is finished, false to continue.
         */
        virtual bool IsExperimentFinished() override;
    };
}

#endif
