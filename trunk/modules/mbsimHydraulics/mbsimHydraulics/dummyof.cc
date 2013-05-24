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
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, SerialResistanceLinePressureLoss,  MBSIMHYDRAULICSNS"SerialResistanceLinePressureLoss")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, ParallelResistanceLinePressureLoss,  MBSIMHYDRAULICSNS"ParallelResistanceLinePressureLoss")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, ZetaLinePressureLoss,  MBSIMHYDRAULICSNS"ZetaLinePressureLoss")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, ZetaPosNegLinePressureLoss,  MBSIMHYDRAULICSNS"ZetaPosNegLinePressureLoss")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, LaminarTubeFlowLinePressureLoss,  MBSIMHYDRAULICSNS"LaminarTubeFlowLinePressureLoss")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, TurbulentTubeFlowLinePressureLoss,  MBSIMHYDRAULICSNS"TurbulentTubeFlowLinePressureLoss")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, CurveFittedLinePressureLoss,  MBSIMHYDRAULICSNS"CurveFittedLinePressureLoss")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, TabularLinePressureLoss,  MBSIMHYDRAULICSNS"TabularLinePressureLoss")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, RelativeAreaZetaClosablePressureLoss,  MBSIMHYDRAULICSNS"RelativeAreaZetaClosablePressureLoss")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, GapHeightClosablePressureLoss,  MBSIMHYDRAULICSNS"GapHeightClosablePressureLoss")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, ReynoldsClosablePressureLoss,  MBSIMHYDRAULICSNS"ReynoldsClosablePressureLoss")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, RelativeAlphaClosablePressureLoss,  MBSIMHYDRAULICSNS"RelativeAlphaClosablePressureLoss")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, GammaCheckvalveClosablePressureLoss,  MBSIMHYDRAULICSNS"GammaCheckvalveClosablePressureLoss")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, IdelchickCheckvalveClosablePressureLoss,  MBSIMHYDRAULICSNS"IdelchickCheckvalveClosablePressureLoss")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, ConeCheckvalveClosablePressureLoss,  MBSIMHYDRAULICSNS"ConeCheckvalveClosablePressureLoss")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, PlaneLeakagePressureLoss,  MBSIMHYDRAULICSNS"PlaneLeakagePressureLoss")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, EccentricCircularLeakagePressureLoss,  MBSIMHYDRAULICSNS"EccentricCircularLeakagePressureLoss")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, RealCircularLeakagePressureLoss,  MBSIMHYDRAULICSNS"RealCircularLeakagePressureLoss")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, UnidirectionalZetaPressureLoss,  MBSIMHYDRAULICSNS"UnidirectionalZetaPressureLoss")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, RigidLine,  MBSIMHYDRAULICSNS"RigidLine")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, ClosableRigidLine,  MBSIMHYDRAULICSNS"ClosableRigidLine")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, UnidirectionalRigidLine,  MBSIMHYDRAULICSNS"UnidirectionalRigidLine")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, PlaneLeakageLine,  MBSIMHYDRAULICSNS"PlaneLeakageLine")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, CircularLeakageLine,  MBSIMHYDRAULICSNS"CircularLeakageLine")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, PlaneLeakage0DOF,  MBSIMHYDRAULICSNS"PlaneLeakage0DOF")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, CircularLeakage0DOF,  MBSIMHYDRAULICSNS"CircularLeakage0DOF")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, ConstrainedLine,  MBSIMHYDRAULICSNS"ConstrainedLine")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, FluidPump,  MBSIMHYDRAULICSNS"FluidPump")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, StatelessOrifice,  MBSIMHYDRAULICSNS"StatelessOrMBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, ")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, ElasticLineGalerkin,  MBSIMHYDRAULICSNS"ElasticLineGalerkin")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, ElasticLineVariational,  MBSIMHYDRAULICSNS"ElasticLineVariational")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, ConstrainedNode, MBSIMHYDRAULICSNS"ConstrainedNode")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, EnvironmentNode, MBSIMHYDRAULICSNS"EnvironmentNode")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, ElasticNode, MBSIMHYDRAULICSNS"ElasticNode")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, RigidNode, MBSIMHYDRAULICSNS"RigidNode")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, RigidCavitationNode, MBSIMHYDRAULICSNS"RigidCavitationNode")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, PressurePump, MBSIMHYDRAULICSNS"PressurePump")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, ConstrainedNodeMec, MBSIMHYDRAULICSNS"ConstrainedNodeMec")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, EnvironmentNodeMec, MBSIMHYDRAULICSNS"EnvironmentNodeMec")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, ElasticNodeMec, MBSIMHYDRAULICSNS"ElasticNodeMec")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, RigidNodeMec, MBSIMHYDRAULICSNS"RigidNodeMec")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, FlowSensor, MBSIMHYDRAULICSNS"FlowSensor")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, PressureSensor, MBSIMHYDRAULICSNS"PressureSensor")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, TemperatureSensor, MBSIMHYDRAULICSNS"TemperatureSensor")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, KinematicViscositySensor, MBSIMHYDRAULICSNS"KinematicViscositySensor")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORYASSINGLETON(MBSim::Environment, HydraulicEnvironment, MBSIMHYDRAULICSNS"HydraulicEnvironment")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, Controlvalve43, MBSIMHYDRAULICSNS"Controlvalve43")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, Checkvalve, MBSIMHYDRAULICSNS"Checkvalve")
/*
 * DO NOT ADD ANYTHING IN THIS FILE!!! (see also the comment on top of this file)
 */
