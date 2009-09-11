#ifndef _MBSIMHYDRAULICS_OBJECTFACTORY_H_
#define _MBSIMHYDRAULICS_OBJECTFACTORY_H_

#include "mbsim/objectfactory.h"
#include "mbsimtinyxml/tinyxml-src/tinyxml.h"
#include "mbsim/utils/function.h"

#define MBSIMHYDRAULICSNS "{http://mbsim.berlios.de/MBSimHydraulics}"

namespace MBSim {

  class HydraulicsObjectFactory : protected ObjectFactory {
    private:
      static HydraulicsObjectFactory *instance;
      HydraulicsObjectFactory() {}
    public:
      // This static function must be called before the ObjectFactory is usend to create
      // objects from MBSimObjectFactory
      static void initialize();
    protected:
      Group* createGroup(TiXmlElement *element);
      Object* createObject(TiXmlElement *element);
      Link* createLink(TiXmlElement *element);
      Environment *getEnvironment(TiXmlElement *element);
      Function1<double, double> * createFunction1_SS(TiXmlElement *element);
  };

}

#endif
