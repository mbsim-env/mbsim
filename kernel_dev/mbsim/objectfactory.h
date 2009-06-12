#ifndef _MBSIMXML_OBJECTFACTORY_H_
#define _MBSIMXML_OBJECTFACTORY_H_

#include "mbsimtinyxml/tinyxml-src/tinyxml.h"
#include "mbsim/group.h"
#include "mbsim/object.h"
#include "mbsim/kinematics.h"
#include "mbsim/link.h"
#include "mbsim/integrators/integrator.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/contour.h"

namespace MBSim {

class ObjectFactory {
  public:
    static Group* createGroup(TiXmlElement *element);
    static Object* createObject(TiXmlElement *element);
    static Translation* createTranslation(TiXmlElement *element);
    static Rotation* createRotation(TiXmlElement *element);
    static Link* createLink(TiXmlElement *element);
    static Integrator* createIntegrator(TiXmlElement *element);
    static GeneralizedForceLaw *createGeneralizedForceLaw(TiXmlElement *element);
    static GeneralizedImpactLaw *createGeneralizedImpactLaw(TiXmlElement *element);
    static FrictionForceLaw *createFrictionForceLaw(TiXmlElement *element);
    static FrictionImpactLaw *createFrictionImpactLaw(TiXmlElement *element);
    static Contour *createContour(TiXmlElement *element);
};

}

#endif
