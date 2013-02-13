#ifndef _MBSIM_OBJECTFACTORY_H_
#define _MBSIM_OBJECTFACTORY_H_

#include "mbxmlutilstinyxml/tinyxml.h"
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
#include <map>

namespace MBSim {

class ObjectFactoryBase {
  protected:
    ObjectFactoryBase() {}
    virtual ~ObjectFactoryBase() {}
    typedef std::pair<std::string, std::string> P_NSPRE;
    typedef std::map<std::string, std::string> M_NSPRE;
    typedef std::pair<double, P_NSPRE> P_PRINSPRE;
  public:
    typedef std::multimap<double, P_NSPRE> MM_PRINSPRE;
    virtual Group* createGroup(TiXmlElement *element) { return NULL; }
    virtual Object* createObject(TiXmlElement *element) { return NULL; }
    virtual ExtraDynamic * createExtraDynamic(TiXmlElement *element) { return NULL; }
    virtual Translation* createTranslation(TiXmlElement *element) { return NULL; }
    virtual Rotation* createRotation(TiXmlElement *element) { return NULL; }
    virtual Link* createLink(TiXmlElement *element) { return NULL; }
    virtual Integrator* createIntegrator(TiXmlElement *element) { return NULL; }
    virtual GeneralizedForceLaw *createGeneralizedForceLaw(TiXmlElement *element) { return NULL; }
    virtual GeneralizedImpactLaw *createGeneralizedImpactLaw(TiXmlElement *element) { return NULL; }
    virtual FrictionForceLaw *createFrictionForceLaw(TiXmlElement *element) { return NULL; }
    virtual FrictionImpactLaw *createFrictionImpactLaw(TiXmlElement *element) { return NULL; }
    virtual Contour *createContour(TiXmlElement *element) { return NULL; }
    virtual Environment *getEnvironment(TiXmlElement *element) { return NULL; }
    virtual Jacobian *createJacobian(TiXmlElement *element) { return NULL; }
    virtual Function1<double,double> *createFunction1_SS(TiXmlElement *element) { return NULL; }
    virtual Function1<fmatvec::Vec,double> *createFunction1_VS(TiXmlElement *element) { return NULL; }
    virtual Function1<fmatvec::VecV,double> *createFunction1_VVS(TiXmlElement *element) { return NULL; }
    virtual Function1<fmatvec::Vec3,double> *createFunction1_V3S(TiXmlElement *element) { return NULL; }
    virtual Function2<double,double,double> *createFunction2_SSS(TiXmlElement *element) { return NULL; }
    virtual Function2<fmatvec::Vec,fmatvec::Vec,double> *createFunction2_VVS(TiXmlElement *element) { return NULL; }
    virtual Function3<fmatvec::Mat3V,fmatvec::Vec,fmatvec::Vec,double> *createFunction3_MVVS(TiXmlElement *element) { return NULL; }
    virtual InfluenceFunction* createInfluenceFunction(TiXmlElement * element) { return NULL; }
    virtual ContourFunction1s * createContourFunction1s(TiXmlElement * element) { return NULL; }
    virtual MM_PRINSPRE& getPriorityNamespacePrefix() {
      static MM_PRINSPRE ret;
      return ret;
    }
};

class ObjectFactory : public ObjectFactoryBase {
  protected:
    ObjectFactory() {}
    virtual ~ObjectFactory() {}
  private:
    static ObjectFactory *instance;
    std::set<ObjectFactoryBase*> factories;
  public:
    static ObjectFactory* getInstance() { return instance?instance:instance=new ObjectFactory; }
    void registerObjectFactory(ObjectFactoryBase *fac) { factories.insert(fac); }
    void unregisterObjectFactory(ObjectFactory *fac) { factories.erase(fac); }

    Group* createGroup(TiXmlElement *element);
    Object* createObject(TiXmlElement *element);
    ExtraDynamic * createExtraDynamic(TiXmlElement *element);
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
    Function1<fmatvec::VecV,double> *createFunction1_VVS(TiXmlElement *element);
    Function1<fmatvec::Vec3,double> *createFunction1_V3S(TiXmlElement *element);
    Function2<double,double,double> *createFunction2_SSS(TiXmlElement *element);
    Function2<fmatvec::Vec,fmatvec::Vec,double> *createFunction2_VVS(TiXmlElement *element);
    Function3<fmatvec::Mat3V,fmatvec::Vec,fmatvec::Vec,double> *createFunction3_MVVS(TiXmlElement *element);
    InfluenceFunction* createInfluenceFunction(TiXmlElement * element);
    ContourFunction1s * createContourFunction1s(TiXmlElement * element);
    M_NSPRE getNamespacePrefixMapping();
};

class MBSimObjectFactory : protected ObjectFactoryBase {
  private:
    static MBSimObjectFactory *instance;
    MBSimObjectFactory() {}
  public:
    // This static function must be called before the ObjectFactory is used to create
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
    Function1<fmatvec::VecV,double> *createFunction1_VVS(TiXmlElement *element);
    Function1<fmatvec::Vec3,double> *createFunction1_V3S(TiXmlElement *element);
    Function2<double,double,double> *createFunction2_SSS(TiXmlElement *element);
    Function2<fmatvec::Vec,fmatvec::Vec,double> *createFunction2_VVS(TiXmlElement *element);
    Function3<fmatvec::Mat3V,fmatvec::Vec,fmatvec::Vec,double> *createFunction3_MVVS(TiXmlElement *element);
    InfluenceFunction* createInfluenceFunction(TiXmlElement * element);
    ContourFunction1s * createContourFunction1s(TiXmlElement * element) {return 0; }
    MM_PRINSPRE& getPriorityNamespacePrefix();
};

}

#endif
