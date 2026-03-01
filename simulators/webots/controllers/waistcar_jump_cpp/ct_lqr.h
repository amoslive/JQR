/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#ifndef INCLUDE_CT_LQR_BASE_H_
#define INCLUDE_CT_LQR_BASE_H_
// #define CT_USE_LAPACK

// Include file for convenience
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/StdVector>
#include <unsupported/Eigen/MatrixFunctions>

// #include <boost/property_tree/ptree.hpp>
// #include <boost/property_tree/info_parser.hpp>
// #include <ct/core/Types.hpp>
// #include <ct/core/Systems.hpp>


#include "ct/optcon/lqr/riccati/CARE.hpp"
// #include "ct/optcon/lqr/riccati/DARE.hpp"
#include "ct/optcon/lqr/LQR.hpp"
#include "ct/optcon/lqr/riccati/CARE-impl.hpp"
// #include "ct/optcon/lqr/riccati/DARE-impl.hpp"
#include "ct/optcon/lqr/LQR-impl.hpp"

#endif /* INCLUDE_CT_OPTCON_OPTCON_H_ */
