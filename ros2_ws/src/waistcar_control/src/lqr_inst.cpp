#include "lqr_inst.h"
#include "ct/optcon/lqr/riccati/CARE-impl.hpp"
// #include "ct/optcon/lqr/riccati/DARE-impl.hpp"
#include "ct/optcon/lqr/LQR-impl.hpp"

namespace ct {
namespace optcon {
template class LQR<5,3>;
template class CARE<5, 3>; // Explicitly instantiate CARE<5, 3>
}
}