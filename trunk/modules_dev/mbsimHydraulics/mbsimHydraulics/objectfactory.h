#ifndef _MBSIMHYDRAULICS_OBJECTFACTORY_H_
#define _MBSIMHYDRAULICS_OBJECTFACTORY_H_

#include "mbsim/objectfactory.h"
#include "mbsimtinyxml/tinyxml-src/tinyxml.h"
#include "mbsim/group.h"
#include "mbsim/object.h"
#include "mbsim/kinematics.h"
#include "mbsim/link.h"
#include "mbsim/integrators/integrator.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/contour.h"

#define MBSIMHYDRAULICSNS "{http://mbsim.berlios.de/MBSimHydraulics}"

namespace MBSim {

class HydraulicsObjectFactory : protected ObjectFactory {
  private:
    static HydraulicsObjectFactory instance;
    HydraulicsObjectFactory();
  protected:
    //Group* createGroup(TiXmlElement *element);
    //Object* createObject(TiXmlElement *element);
    //Translation* createTranslation(TiXmlElement *element);
    //Rotation* createRotation(TiXmlElement *element);
    Link* createLink(TiXmlElement *element);
    //Integrator* createIntegrator(TiXmlElement *element);
    //GeneralizedForceLaw *createGeneralizedForceLaw(TiXmlElement *element);
    //GeneralizedImpactLaw *createGeneralizedImpactLaw(TiXmlElement *element);
    //FrictionForceLaw *createFrictionForceLaw(TiXmlElement *element);
    //FrictionImpactLaw *createFrictionImpactLaw(TiXmlElement *element);
    //Contour *createContour(TiXmlElement *element);
};

}

#endif
