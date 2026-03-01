#pragma once
#include "ct_lqr.h"

namespace my_lqr {
    using LQR53 = ct::optcon::LQR<5,3>;
}
extern template class ct::optcon::LQR<5, 3>;
extern template class ct::optcon::CARE<5, 3>;


