// wrap MBSim with directors (and all protected class data)
%module(directors="1", allprotected="1") _mbsim_part050


// include the general mbsim SWIG configuration (used by all MBSim modules)
%import fmatvec.i
%include "mbsim_include.i"
%import mbsim.i

// includes needed in the generated swig c++ code
%{
#include <config.h> // to use consistent preprocessor defines
namespace MBSim {
  class CriteriaFunction;
  class DampingFunction;
  template<typename Sig> class DistanceFunction;
  class FuncPairPlanarContourPoint;
  class FunctionBase;
  template<typename Sig> class Function;
  class GlobalCriteriaFunction;
  class GlobalResidualCriteriaFunction;
  class GlobalShiftCriteriaFunction;
  class LocalCriteriaFunction;
  class LocalResidualCriteriaFunction;
  class LocalShiftCriteriaFunction;
  class NewtonJacobianFunction;
  class NumericalNewtonJacobianFunction;
  class StandardDampingFunction;
}
#include "mbsim/utils/index.h"
#include "mbsim/links/link.h"
#include "mbsim/links/mechanical_link.h"
#include "mbsim/links/frame_link.h"
#include "mbsim/frames/frame.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/frames/fixed_contour_frame.h"
#include "mbsim/frames/floating_contour_frame.h"
#include "mbsim/frames/floating_relative_contour_frame.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/frames/floating_relative_frame.h"
using namespace MBSim; // SWIGs namespace handling seems to be buggy -> this fixes this
using namespace fmatvec; // SWIGs namespace handling seems to be buggy -> this fixes this
%}



// wrap the following classes
%rename(lambda_) MBSim::Link::lambda; // lambda is a python keyword -> rename it to lambda_
%include "mbsim/utils/index.h"
%include "mbsim/links/link.h"
%include "mbsim/links/mechanical_link.h"
%include "mbsim/links/frame_link.h"
%include "mbsim/frames/frame.h"
%include "mbsim/frames/contour_frame.h"
%include "mbsim/frames/fixed_contour_frame.h"
%include "mbsim/frames/floating_contour_frame.h"
%include "mbsim/frames/floating_relative_contour_frame.h"
%include "mbsim/frames/fixed_relative_frame.h"
%include "mbsim/frames/floating_relative_frame.h"
