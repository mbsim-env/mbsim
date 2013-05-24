/*
 * DO NOT ADD ANYTHING IN THIS FILE!!!
 *
 * If you want to add new classes to the ObjectFactory add the corresponding
 *
 * MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(BaseType, NewType, MBSIMNS"NewType")
 *
 * line to the .cc file where the new class is defined!
 * Where NewType is the class name of the new class to be added to the ObjectFactory,
 * BaseType is the TOP LEVEL base class of NewType and
 * MBSIMNS"NewType" is the full qualified XML element name of NewType
 *
 * All below MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(...) line
 * should also be moved the the corrsponding *.cc files. They are just added here
 * temporarily during the change of the ObjectFactory code.
 */
#include "config.h"
#include "mbsim/objectfactory.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_23_bta.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_rcm.h"
#include "mbsimFlexibleBody/defines.h"

#define COMMA ,

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimFlexibleBody;

/*
 * DO NOT ADD ANYTHING IN THIS FILE!!! (see also the comment on top of this file)
 */
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(FlexibleBody, FlexibleBody1s23BTA, MBSIMFLEXIBLEBODYNS"FlexibleBody1s23BTA")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(FlexibleBody, FlexibleBody1s33RCM, MBSIMFLEXIBLEBODYNS"FlexibleBody1s33RCMCantilever")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(FlexibleBody, FlexibleBody1s33RCM, MBSIMFLEXIBLEBODYNS"FlexibleBody1s33RCMRing")
/*
 * DO NOT ADD ANYTHING IN THIS FILE!!! (see also the comment on top of this file)
 */
