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
#include <config.h>
#include "mbsim/objectfactory.h"
#include "mbsimHydraulics/defines.h"
#include "mbsimHydraulics/rigid_line.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimHydraulics/hnode_mec.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/controlvalve43.h"
#include "mbsimHydraulics/checkvalve.h"
#include "mbsimHydraulics/leakage_line.h"
#include "mbsimHydraulics/dimensionless_line.h"
#include "mbsimHydraulics/hydraulic_sensor.h"
#include "mbsimHydraulics/elastic_line_galerkin.h"
#include "mbsimHydraulics/elastic_line_variational.h"

#define COMMA ,

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimHydraulics;

/*
 * DO NOT ADD ANYTHING IN THIS FILE!!! (see also the comment on top of this file)
 */
MBSIM_OBJECTFACTORY_REGISTERXMLNAMEASSINGLETON(Environment, HydraulicEnvironment, MBSIMHYDRAULICSNS"HydraulicEnvironment")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SerialResistanceLinePressureLoss,  MBSIMHYDRAULICSNS"SerialResistanceLinePressureLoss")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, ParallelResistanceLinePressureLoss,  MBSIMHYDRAULICSNS"ParallelResistanceLinePressureLoss")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, ZetaLinePressureLoss,  MBSIMHYDRAULICSNS"ZetaLinePressureLoss")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, ZetaPosNegLinePressureLoss,  MBSIMHYDRAULICSNS"ZetaPosNegLinePressureLoss")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, LaminarTubeFlowLinePressureLoss,  MBSIMHYDRAULICSNS"LaminarTubeFlowLinePressureLoss")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TurbulentTubeFlowLinePressureLoss,  MBSIMHYDRAULICSNS"TurbulentTubeFlowLinePressureLoss")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, CurveFittedLinePressureLoss,  MBSIMHYDRAULICSNS"CurveFittedLinePressureLoss")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TabularLinePressureLoss,  MBSIMHYDRAULICSNS"TabularLinePressureLoss")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, RelativeAreaZetaClosablePressureLoss,  MBSIMHYDRAULICSNS"RelativeAreaZetaClosablePressureLoss")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, GapHeightClosablePressureLoss,  MBSIMHYDRAULICSNS"GapHeightClosablePressureLoss")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, ReynoldsClosablePressureLoss,  MBSIMHYDRAULICSNS"ReynoldsClosablePressureLoss")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, RelativeAlphaClosablePressureLoss,  MBSIMHYDRAULICSNS"RelativeAlphaClosablePressureLoss")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, GammaCheckvalveClosablePressureLoss,  MBSIMHYDRAULICSNS"GammaCheckvalveClosablePressureLoss")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, IdelchickCheckvalveClosablePressureLoss,  MBSIMHYDRAULICSNS"IdelchickCheckvalveClosablePressureLoss")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, ConeCheckvalveClosablePressureLoss,  MBSIMHYDRAULICSNS"ConeCheckvalveClosablePressureLoss")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, PlaneLeakagePressureLoss,  MBSIMHYDRAULICSNS"PlaneLeakagePressureLoss")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, EccentricCircularLeakagePressureLoss,  MBSIMHYDRAULICSNS"EccentricCircularLeakagePressureLoss")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, RealCircularLeakagePressureLoss,  MBSIMHYDRAULICSNS"RealCircularLeakagePressureLoss")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, UnidirectionalZetaPressureLoss,  MBSIMHYDRAULICSNS"UnidirectionalZetaPressureLoss")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, RigidLine,  MBSIMHYDRAULICSNS"RigidLine")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, ClosableRigidLine,  MBSIMHYDRAULICSNS"ClosableRigidLine")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, UnidirectionalRigidLine,  MBSIMHYDRAULICSNS"UnidirectionalRigidLine")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, PlaneLeakageLine,  MBSIMHYDRAULICSNS"PlaneLeakageLine")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, CircularLeakageLine,  MBSIMHYDRAULICSNS"CircularLeakageLine")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, PlaneLeakage0DOF,  MBSIMHYDRAULICSNS"PlaneLeakage0DOF")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, CircularLeakage0DOF,  MBSIMHYDRAULICSNS"CircularLeakage0DOF")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, ConstrainedLine,  MBSIMHYDRAULICSNS"ConstrainedLine")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, FluidPump,  MBSIMHYDRAULICSNS"FluidPump")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, StatelessOrifice,  MBSIMHYDRAULICSNS"StatelessOrMBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, ")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, ElasticLineGalerkin,  MBSIMHYDRAULICSNS"ElasticLineGalerkin")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, ElasticLineVariational,  MBSIMHYDRAULICSNS"ElasticLineVariational")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, ConstrainedNode, MBSIMHYDRAULICSNS"ConstrainedNode")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, EnvironmentNode, MBSIMHYDRAULICSNS"EnvironmentNode")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, ElasticNode, MBSIMHYDRAULICSNS"ElasticNode")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, RigidNode, MBSIMHYDRAULICSNS"RigidNode")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, RigidCavitationNode, MBSIMHYDRAULICSNS"RigidCavitationNode")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, PressurePump, MBSIMHYDRAULICSNS"PressurePump")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, ConstrainedNodeMec, MBSIMHYDRAULICSNS"ConstrainedNodeMec")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, EnvironmentNodeMec, MBSIMHYDRAULICSNS"EnvironmentNodeMec")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, ElasticNodeMec, MBSIMHYDRAULICSNS"ElasticNodeMec")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, RigidNodeMec, MBSIMHYDRAULICSNS"RigidNodeMec")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, FlowSensor, MBSIMHYDRAULICSNS"FlowSensor")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, PressureSensor, MBSIMHYDRAULICSNS"PressureSensor")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, TemperatureSensor, MBSIMHYDRAULICSNS"TemperatureSensor")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, KinematicViscositySensor, MBSIMHYDRAULICSNS"KinematicViscositySensor")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, Controlvalve43, MBSIMHYDRAULICSNS"Controlvalve43")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, Checkvalve, MBSIMHYDRAULICSNS"Checkvalve")
/*
 * DO NOT ADD ANYTHING IN THIS FILE!!! (see also the comment on top of this file)
 */
