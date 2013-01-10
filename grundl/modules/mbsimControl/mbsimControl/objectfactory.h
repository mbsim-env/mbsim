#ifndef _MBSIMCONTROL_OBJECTFACTORY_H_
#define _MBSIMCONTROL_OBJECTFACTORY_H_

#include "mbsim/objectfactory.h"
#include "mbxmlutilstinyxml/tinyxml.h"

#define MBSIMCONTROLNS_ "http://mbsim.berlios.de/MBSimControl"
#define MBSIMCONTROLNS "{"MBSIMCONTROLNS_"}"

namespace MBSim {
  class ExtraDynamic;
  class Link;
}

namespace MBSimControl {

  class ObjectFactory : protected MBSim::ObjectFactoryBase {
    private:
      static ObjectFactory *instance;
      ObjectFactory() {}
    public:
      // This static function must be called before the ObjectFactory is used to create
      // objects from MBSimObjectFactory
      static void initialize();
    protected:
      MBSim::ExtraDynamic * createExtraDynamic(TiXmlElement *element);
      MBSim::Link* createLink(TiXmlElement *element);
      MBSim::ObjectFactoryBase::MM_PRINSPRE& getPriorityNamespacePrefix();
  };

}

#endif
