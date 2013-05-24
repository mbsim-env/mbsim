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
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, LinearTransferSystem, MBSIMCONTROLNS"LinearTransferSystem")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, MasslessSpringDamper, MBSIMCONTROLNS"MasslessSpringDamper")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, ExternSignalSource, MBSIMCONTROLNS"ExternSignalSource")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, GeneralizedPositionSensor, MBSIMCONTROLNS"GeneralizedPositionSensor")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, GeneralizedVelocitySensor, MBSIMCONTROLNS"GeneralizedVelocitySensor")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, AbsolutePositionSensor, MBSIMCONTROLNS"AbsolutePositionSensor")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, AbsoluteVelocitySensor, MBSIMCONTROLNS"AbsoluteVelocitySensor")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, AbsoluteAngularPositionSensor, MBSIMCONTROLNS"AbsoluteAngularPositionSensor")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, AbsoluteAngularVelocitySensor, MBSIMCONTROLNS"AbsoluteAngularVelocitySensor")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, RelativePositionSensor, MBSIMCONTROLNS"RelativePositionSensor")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, RelativeVelocitySensor, MBSIMCONTROLNS"RelativeVelocitySensor")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, RelativeAngularPositionSensor, MBSIMCONTROLNS"RelativeAngularPositionSensor")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, RelativeAngularVelocitySensor, MBSIMCONTROLNS"RelativeAngularVelocitySensor")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, FunctionSensor, MBSIMCONTROLNS"FunctionSensor")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, Function1_SSEvaluation, MBSIMCONTROLNS"Function1_SSEvaluation")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, Function2_SSSEvaluation, MBSIMCONTROLNS"Function2_SSSEvaluation")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, SignalProcessingSystemSensor, MBSIMCONTROLNS"SignalProcessingSystemSensor")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, LinkDistanceSensor, MBSIMCONTROLNS"LinkDistanceSensor")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, LinkVelocitySensor, MBSIMCONTROLNS"LinkVelocitySensor")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, SignalAddition, MBSIMCONTROLNS"SignalAddition")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, SignalOffset, MBSIMCONTROLNS"SignalOffset")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, SignalMultiplication, MBSIMCONTROLNS"SignalMultiplication")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, SignalMux, MBSIMCONTROLNS"SignalMux")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, SignalDemux, MBSIMCONTROLNS"SignalDemux")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, SignalLimitation, MBSIMCONTROLNS"SignalLimitation")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, SignalTimeDiscretization, MBSIMCONTROLNS"SignalTimeDiscretization")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, SignalOperation, MBSIMCONTROLNS"SignalOperation")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, SpecialSignalOperation, MBSIMCONTROLNS"SpecialSignalOperation")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, Actuator, MBSIMCONTROLNS"Actuator")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, PIDController, MBSIMCONTROLNS"PIDController")
/*
 * DO NOT ADD ANYTHING IN THIS FILE!!! (see also the comment on top of this file)
 */
