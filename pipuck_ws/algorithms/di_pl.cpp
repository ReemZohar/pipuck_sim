#include "di_pl.hpp"

namespace argos {
    Real CDiPL::m_fKp = 0;
    Real CDiPL::m_fAlpha = 0;

    void CDiPL::setParameters(Real Kp, Real alpha) {
        m_fKp = Kp;
        m_fAlpha = alpha;
    }
}