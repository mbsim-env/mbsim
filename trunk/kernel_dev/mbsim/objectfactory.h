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
#include "mbsim/environment.h"
#include <set>

namespace MBSim {

class ObjectFactory {
  protected:
    ObjectFactory() {}
    virtual ~ObjectFactory() {}
  private:
    static ObjectFactory *instance;
    std::set<ObjectFactory*> factories;
  public:
    static ObjectFactory* getInstance() { return instance?instance:instance=new ObjectFactory; }
    void registerObjectFactory(ObjectFactory *fac) { factories.insert(fac); }
    void unregisterObjectFactory(ObjectFactory *fac) { factories.erase(fac); }

    virtual Group* createGroup(TiXmlElement *element);
    virtual Object* createObject(TiXmlElement *element);
    virtual Translation* createTranslation(TiXmlElement *element);
    virtual Rotation* createRotation(TiXmlElement *element);
    virtual Link* createLink(TiXmlElement *element);
    virtual Integrator* createIntegrator(TiXmlElement *element);
    virtual GeneralizedForceLaw *createGeneralizedForceLaw(TiXmlElement *element);
    virtual GeneralizedImpactLaw *createGeneralizedImpactLaw(TiXmlElement *element);
    virtual FrictionForceLaw *createFrictionForceLaw(TiXmlElement *element);
    virtual FrictionImpactLaw *createFrictionImpactLaw(TiXmlElement *element);
    virtual Contour *createContour(TiXmlElement *element);
    virtual Environment *getEnvironment(TiXmlElement *element);
};

class MBSimObjectFactory : protected ObjectFactory {
  private:
    static MBSimObjectFactory *instance;
    MBSimObjectFactory() {}
  public:
    // This static function must be called before the ObjectFactory is usend to create
    // objects from MBSimObjectFactory
    static void initialize();
  protected:
    Group* createGroup(TiXmlElement *element);
    Object* createObject(TiXmlElement *element);
    Translation* createTranslation(TiXmlElement *element);
    Rotation* createRotation(TiXmlElement *element);
    Link* createLink(TiXmlElement *element);
    Integrator* createIntegrator(TiXmlElement *element);
    GeneralizedForceLaw *createGeneralizedForceLaw(TiXmlElement *element);
    GeneralizedImpactLaw *createGeneralizedImpactLaw(TiXmlElement *element);
    FrictionForceLaw *createFrictionForceLaw(TiXmlElement *element);
    FrictionImpactLaw *createFrictionImpactLaw(TiXmlElement *element);
    Contour *createContour(TiXmlElement *element);
    Environment *getEnvironment(TiXmlElement *element);
};

}

#endif
