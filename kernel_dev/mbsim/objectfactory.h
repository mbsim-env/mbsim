#ifndef _MBSIMXML_OBJECTFACTORY_H_
#define _MBSIMXML_OBJECTFACTORY_H_

#include "mbsimtinyxml/tinyxml.h"
#include "mbsim/group.h"
#include "mbsim/object.h"
#include "mbsim/kinematics.h"
#include "mbsim/link_mechanics.h"

namespace MBSim {

class ObjectFactory {
  public:
    static Group* createGroup(TiXmlElement *element);
    static Object* createObject(TiXmlElement *element);
    static Translation* createTranslation(TiXmlElement *element);
    static LinkMechanics* createLinkMechanics(TiXmlElement *element);
};

}

#endif
