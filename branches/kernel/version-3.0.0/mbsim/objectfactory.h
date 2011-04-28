#ifndef _MBSIM_OBJECTFACTORY_H_
#define _MBSIM_OBJECTFACTORY_H_

#include "mbsimtinyxml/tinyxml-src/tinyxml.h"
#include "mbsim/group.h"
#include "mbsim/object.h"
#include "mbsim/extra_dynamic.h"
#include "mbsim/kinematics.h"
#include "mbsim/link.h"
#include "mbsim/integrators/integrator.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/contour.h"
#include "mbsim/environment.h"
#include "mbsim/utils/contour_functions.h"
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
    virtual ExtraDynamic * createExtraDynamic(TiXmlElement *element);
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
    virtual Jacobian *createJacobian(TiXmlElement *element);
    virtual Function1<double,double> *createFunction1_SS(TiXmlElement *element);
    virtual Function1<fmatvec::Vec,double> *createFunction1_VS(TiXmlElement *element);
    virtual Function2<double,double,double> *createFunction2_SSS(TiXmlElement *element);
    virtual Function2<fmatvec::Vec,fmatvec::Vec,double> *createFunction2_VVS(TiXmlElement *element);
    virtual Function3<fmatvec::Mat,fmatvec::Vec,fmatvec::Vec,double> *createFunction3_MVVS(TiXmlElement *element);
    virtual ContourFunction1s * createContourFunction1s(TiXmlElement * element);
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
    ExtraDynamic * createExtraDynamic(TiXmlElement *element) {return 0; }
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
    Jacobian *createJacobian(TiXmlElement *element);
    Function1<double,double> *createFunction1_SS(TiXmlElement *element);
    Function1<fmatvec::Vec,double> *createFunction1_VS(TiXmlElement *element);
    Function2<double,double,double> *createFunction2_SSS(TiXmlElement *element);
    Function2<fmatvec::Vec,fmatvec::Vec,double> *createFunction2_VVS(TiXmlElement *element);
    Function3<fmatvec::Mat,fmatvec::Vec,fmatvec::Vec,double> *createFunction3_MVVS(TiXmlElement *element);
    ContourFunction1s * createContourFunction1s(TiXmlElement * element) {return 0; }
};

}

#endif
