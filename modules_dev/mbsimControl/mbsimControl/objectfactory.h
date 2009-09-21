#ifndef _MBSIMCONTROL_OBJECTFACTORY_H_
#define _MBSIMCONTROL_OBJECTFACTORY_H_

#include "mbsim/objectfactory.h"
#include "mbsimtinyxml/tinyxml-src/tinyxml.h"

#define MBSIMCONTROLNS "{http://mbsim.berlios.de/MBSimControl}"

namespace MBSim {

  class ControlObjectFactory : protected ObjectFactory {
    private:
      static ControlObjectFactory *instance;
      ControlObjectFactory() {}
    public:
      // This static function must be called before the ObjectFactory is used to create
      // objects from MBSimObjectFactory
      static void initialize();
    protected:
      ExtraDynamic * createExtraDynamic(TiXmlElement *element);
      Link* createLink(TiXmlElement *element);
  };

}

#endif
