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
#include "mbsimControl/defines.h"
#include "mbsimControl/signal_.h"
#include "mbsimControl/sensor.h"
#include "mbsimControl/extern_signal_source.h"
#include "mbsimControl/object_sensors.h"
#include "mbsimControl/function_sensor.h"
#include "mbsimControl/signal_processing_system_sensor.h"
#include "mbsimControl/frame_sensors.h"
#include "mbsimControl/link_sensors.h"
#include "mbsimControl/signal_manipulation.h"
#include "mbsimControl/linear_transfer_system.h"
#include "mbsimControl/massless_spring_damper.h"
#include "mbsimControl/actuator.h"


#define COMMA ,

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimControl;

/*
 * DO NOT ADD ANYTHING IN THIS FILE!!! (see also the comment on top of this file)
 */
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, LinearTransferSystem, MBSIMCONTROLNS"LinearTransferSystem")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, MasslessSpringDamper, MBSIMCONTROLNS"MasslessSpringDamper")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, ExternSignalSource, MBSIMCONTROLNS"ExternSignalSource")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, GeneralizedPositionSensor, MBSIMCONTROLNS"GeneralizedPositionSensor")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, GeneralizedVelocitySensor, MBSIMCONTROLNS"GeneralizedVelocitySensor")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, AbsolutePositionSensor, MBSIMCONTROLNS"AbsolutePositionSensor")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, AbsoluteVelocitySensor, MBSIMCONTROLNS"AbsoluteVelocitySensor")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, AbsoluteAngularPositionSensor, MBSIMCONTROLNS"AbsoluteAngularPositionSensor")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, AbsoluteAngularVelocitySensor, MBSIMCONTROLNS"AbsoluteAngularVelocitySensor")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, RelativePositionSensor, MBSIMCONTROLNS"RelativePositionSensor")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, RelativeVelocitySensor, MBSIMCONTROLNS"RelativeVelocitySensor")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, RelativeAngularPositionSensor, MBSIMCONTROLNS"RelativeAngularPositionSensor")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, RelativeAngularVelocitySensor, MBSIMCONTROLNS"RelativeAngularVelocitySensor")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, FunctionSensor, MBSIMCONTROLNS"FunctionSensor")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, Function1_SSEvaluation, MBSIMCONTROLNS"Function1_SSEvaluation")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, Function2_SSSEvaluation, MBSIMCONTROLNS"Function2_SSSEvaluation")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, SignalProcessingSystemSensor, MBSIMCONTROLNS"SignalProcessingSystemSensor")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, LinkDistanceSensor, MBSIMCONTROLNS"LinkDistanceSensor")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, LinkVelocitySensor, MBSIMCONTROLNS"LinkVelocitySensor")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, SignalAddition, MBSIMCONTROLNS"SignalAddition")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, SignalOffset, MBSIMCONTROLNS"SignalOffset")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, SignalMultiplication, MBSIMCONTROLNS"SignalMultiplication")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, SignalMux, MBSIMCONTROLNS"SignalMux")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, SignalDemux, MBSIMCONTROLNS"SignalDemux")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, SignalLimitation, MBSIMCONTROLNS"SignalLimitation")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, SignalTimeDiscretization, MBSIMCONTROLNS"SignalTimeDiscretization")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, SignalOperation, MBSIMCONTROLNS"SignalOperation")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, SpecialSignalOperation, MBSIMCONTROLNS"SpecialSignalOperation")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, Actuator, MBSIMCONTROLNS"Actuator")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, PIDController, MBSIMCONTROLNS"PIDController")
/*
 * DO NOT ADD ANYTHING IN THIS FILE!!! (see also the comment on top of this file)
 */
