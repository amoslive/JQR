/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#ifndef INCLUDE_CT_CORE_CORE_H_
#define INCLUDE_CT_CORE_CORE_H_

#include <iosfwd>
#include <vector>
#include <functional>
#include <algorithm>
#include <cstdlib>
#include <pthread.h>
#include <mutex>

#ifdef CPPADCG
#include <cppad/cg.hpp>
#endif

#ifdef CPPAD
#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>
#include <cppad/example/atomic_two/eigen_mat_inv.hpp>
#include "internal/autodiff/CppadParallel.h"
#endif

// Include file for convenience
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/StdVector>
#include <unsupported/Eigen/MatrixFunctions>

// Declarations
#include "Common.hpp"
#include "Types.hpp"
#include "Control.hpp"
#include "Systems.hpp"
#include "Integration.hpp"
#include "Geometry.hpp"
#include "Internal.hpp"
#include "Math.hpp"
#include "Simulation.hpp"
#include "Switching.hpp"

#include "templateDir.h"

#ifdef PLOTTING_ENABLED
#include "plot/plot.h"
#include "plot/matplotlibcpp.h"
#endif


// Implementations
#include "Common-impl.hpp"
#include "Types-impl.hpp"
#include "Control-impl.hpp"
#include "Systems-impl.hpp"
#include "Integration-impl.hpp"
#include "Internal-impl.hpp"
#include "Math-impl.hpp"
#include "Geometry-impl.hpp"
#include "Simulation-impl.hpp"

// keep standard header guard (easy debugging)
// header guard is identical to the one in core-prespec.h
#endif  // INCLUDE_CT_CORE_CORE_H_
