#include <config.h>
#include "mbsimControl/objectfactory.h"
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

using namespace std;

namespace MBSimControl {

  ObjectFactory *ObjectFactory::instance=NULL;

  void ObjectFactory::initialize() {
    if(instance==0) {
      instance=new ObjectFactory;
      MBSim::ObjectFactory::getInstance()->registerObjectFactory(instance);
    }
  }

  MBSim::ExtraDynamic * ObjectFactory::createExtraDynamic(TiXmlElement *element) {
    if(element==0) return 0;
    if (element->ValueStr()==MBSIMCONTROLNS"LinearTransferSystem")
      return new LinearTransferSystem(element->Attribute("name"));
    if (element->ValueStr()==MBSIMCONTROLNS"MasslessSpringDamper")
      return new MasslessSpringDamper(element->Attribute("name"));
    return 0;
  }

  MBSim::Link* ObjectFactory::createLink(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMCONTROLNS"ExternSignalSource")
      return new ExternSignalSource(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"GeneralizedPositionSensor")
      return new GeneralizedPositionSensor(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"GeneralizedVelocitySensor")
      return new GeneralizedVelocitySensor(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"AbsolutePositionSensor")
      return new AbsolutePositionSensor(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"AbsoluteVelocitySensor")
      return new AbsoluteVelocitySensor(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"AbsoluteAngularPositionSensor")
      return new AbsoluteAngularPositionSensor(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"AbsoluteAngularVelocitySensor")
      return new AbsoluteAngularVelocitySensor(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"RelativePositionSensor")
      return new RelativePositionSensor(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"RelativeVelocitySensor")
      return new RelativeVelocitySensor(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"RelativeAngularPositionSensor")
      return new RelativeAngularPositionSensor(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"RelativeAngularVelocitySensor")
      return new RelativeAngularVelocitySensor(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"FunctionSensor")
      return new FunctionSensor(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"Function1_SSEvaluation")
      return new Function1_SSEvaluation(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"Function2_SSSEvaluation")
      return new Function2_SSSEvaluation(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"SignalProcessingSystemSensor")
      return new SignalProcessingSystemSensor(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"LinkDistanceSensor")
      return new LinkDistanceSensor(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"LinkVelocitySensor")
      return new LinkVelocitySensor(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"SignalAddition")
      return new SignalAddition(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"SignalOffset")
      return new SignalOffset(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"SignalMultiplication")
      return new SignalMultiplication(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"SignalMux")
      return new SignalMux(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"SignalDemux")
      return new SignalDemux(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"SignalLimitation")
      return new SignalLimitation(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"SignalTimeDiscretization")
      return new SignalTimeDiscretization(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"SignalOperation")
      return new SignalOperation(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"SpecialSignalOperation")
      return new SpecialSignalOperation(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"Actuator")
      return new Actuator(element->Attribute("name"));
    return 0;
  }

  MBSim::ObjectFactoryBase::MM_PRINSPRE& ObjectFactory::getPriorityNamespacePrefix() {
    static MBSim::ObjectFactoryBase::MM_PRINSPRE priorityNamespacePrefix;

    if(priorityNamespacePrefix.empty()) {
      priorityNamespacePrefix.insert(P_PRINSPRE( 50, P_NSPRE(MBSIMCONTROLNS_, "")));
      priorityNamespacePrefix.insert(P_PRINSPRE( 40, P_NSPRE(MBSIMCONTROLNS_, "ctl")));
      priorityNamespacePrefix.insert(P_PRINSPRE( 30, P_NSPRE(MBSIMCONTROLNS_, "control")));
      priorityNamespacePrefix.insert(P_PRINSPRE( 20, P_NSPRE(MBSIMCONTROLNS_, "mbsimcontrol")));
    }

    return priorityNamespacePrefix;
  }

}
