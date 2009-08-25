#include "config.h"
#include "mbsimControl/objectfactory.h"
#include "mbsimControl/signal_.h"
#include "mbsimControl/sensor.h"
#include "mbsimControl/object_sensors.h"
#include "mbsimControl/function_sensor.h"
#include "mbsimControl/frame_sensors.h"
#include "mbsimControl/signal_manipulation.h"
#include "mbsimControl/linear_transfer_system.h"

using namespace std;

namespace MBSim {

  ControlObjectFactory *ControlObjectFactory::instance=NULL;

  void ControlObjectFactory::initialize() {
    if(instance==0) {
      instance=new ControlObjectFactory;
      ObjectFactory::getInstance()->registerObjectFactory(instance);
    }
  }

  Link* ControlObjectFactory::createLink(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMCONTROLNS"GeneralizedPositionSensor")
      return new GeneralizedPositionSensor(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"GeneralizedVelocitySensor")
      return new GeneralizedVelocitySensor(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"AbsolutePositionSensor")
      return new AbsolutPositionSensor(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"AbsoluteVelocitySensor")
      return new AbsolutVelocitySensor(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"AbsoluteAngularPositionSensor")
      return new AbsolutAngularPositionSensor(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"AbsoluteAngularVelocitySensor")
      return new AbsolutAngularVelocitySensor(element->Attribute("name"));
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
    if(element->ValueStr()==MBSIMCONTROLNS"SignalAddition")
      return new SignalAddition(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"SignalMux")
      return new SignalMux(element->Attribute("name"));
    if(element->ValueStr()==MBSIMCONTROLNS"SignalTimeDiscretization")
      return new SignalTimeDiscretization(element->Attribute("name"));
    return 0;
  }

}
