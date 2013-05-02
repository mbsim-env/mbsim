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
    virtual Group* createGroup(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual Object* createObject(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual ExtraDynamic * createExtraDynamic(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual Translation* createTranslation(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual Rotation* createRotation(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual Link* createLink(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual Observer* createObserver(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual Integrator* createIntegrator(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual GeneralizedForceLaw *createGeneralizedForceLaw(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual GeneralizedImpactLaw *createGeneralizedImpactLaw(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual FrictionForceLaw *createFrictionForceLaw(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual FrictionImpactLaw *createFrictionImpactLaw(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual Contour *createContour(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual Environment *getEnvironment(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual Jacobian *createJacobian(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual Function1<double,double> *createFunction1_SS(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual Function1<fmatvec::Vec,double> *createFunction1_VS(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual Function1<fmatvec::VecV,double> *createFunction1_VVS(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual Function1<fmatvec::Vec3,double> *createFunction1_V3S(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual Function2<double,double,double> *createFunction2_SSS(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual Function2<fmatvec::Vec,fmatvec::Vec,double> *createFunction2_VVS(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual Function3<fmatvec::Mat3xV,fmatvec::Vec,fmatvec::Vec,double> *createFunction3_MVVS(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual ContourFunction1s * createContourFunction1s(MBXMLUtils::TiXmlElement * element) { return NULL; }
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

    Group* createGroup(MBXMLUtils::TiXmlElement *element);
    Object* createObject(MBXMLUtils::TiXmlElement *element);
    ExtraDynamic * createExtraDynamic(MBXMLUtils::TiXmlElement *element);
    Translation* createTranslation(MBXMLUtils::TiXmlElement *element);
    Rotation* createRotation(MBXMLUtils::TiXmlElement *element);
    Link* createLink(MBXMLUtils::TiXmlElement *element);
    Observer* createObserver(MBXMLUtils::TiXmlElement *element);
    Integrator* createIntegrator(MBXMLUtils::TiXmlElement *element);
    GeneralizedForceLaw *createGeneralizedForceLaw(MBXMLUtils::TiXmlElement *element);
    GeneralizedImpactLaw *createGeneralizedImpactLaw(MBXMLUtils::TiXmlElement *element);
    FrictionForceLaw *createFrictionForceLaw(MBXMLUtils::TiXmlElement *element);
    FrictionImpactLaw *createFrictionImpactLaw(MBXMLUtils::TiXmlElement *element);
    Contour *createContour(MBXMLUtils::TiXmlElement *element);
    Environment *getEnvironment(MBXMLUtils::TiXmlElement *element);
    Jacobian *createJacobian(MBXMLUtils::TiXmlElement *element);
    Function1<double,double> *createFunction1_SS(MBXMLUtils::TiXmlElement *element);
    Function1<fmatvec::Vec,double> *createFunction1_VS(MBXMLUtils::TiXmlElement *element);
    Function1<fmatvec::VecV,double> *createFunction1_VVS(MBXMLUtils::TiXmlElement *element);
    Function1<fmatvec::Vec3,double> *createFunction1_V3S(MBXMLUtils::TiXmlElement *element);
    Function2<double,double,double> *createFunction2_SSS(MBXMLUtils::TiXmlElement *element);
    Function2<fmatvec::Vec,fmatvec::Vec,double> *createFunction2_VVS(MBXMLUtils::TiXmlElement *element);
    Function3<fmatvec::Mat3xV,fmatvec::Vec,fmatvec::Vec,double> *createFunction3_MVVS(MBXMLUtils::TiXmlElement *element);
    ContourFunction1s * createContourFunction1s(MBXMLUtils::TiXmlElement * element);
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
    Group* createGroup(MBXMLUtils::TiXmlElement *element);
    Object* createObject(MBXMLUtils::TiXmlElement *element);
    ExtraDynamic * createExtraDynamic(MBXMLUtils::TiXmlElement *element) {return 0; }
    Translation* createTranslation(MBXMLUtils::TiXmlElement *element);
    Rotation* createRotation(MBXMLUtils::TiXmlElement *element);
    Link* createLink(MBXMLUtils::TiXmlElement *element);
    Observer* createObserver(MBXMLUtils::TiXmlElement *element);
    Integrator* createIntegrator(MBXMLUtils::TiXmlElement *element);
    GeneralizedForceLaw *createGeneralizedForceLaw(MBXMLUtils::TiXmlElement *element);
    GeneralizedImpactLaw *createGeneralizedImpactLaw(MBXMLUtils::TiXmlElement *element);
    FrictionForceLaw *createFrictionForceLaw(MBXMLUtils::TiXmlElement *element);
    FrictionImpactLaw *createFrictionImpactLaw(MBXMLUtils::TiXmlElement *element);
    Contour *createContour(MBXMLUtils::TiXmlElement *element);
    Environment *getEnvironment(MBXMLUtils::TiXmlElement *element);
    Jacobian *createJacobian(MBXMLUtils::TiXmlElement *element);
    Function1<double,double> *createFunction1_SS(MBXMLUtils::TiXmlElement *element);
    Function1<fmatvec::Vec,double> *createFunction1_VS(MBXMLUtils::TiXmlElement *element);
    Function1<fmatvec::VecV,double> *createFunction1_VVS(MBXMLUtils::TiXmlElement *element);
    Function1<fmatvec::Vec3,double> *createFunction1_V3S(MBXMLUtils::TiXmlElement *element);
    Function2<double,double,double> *createFunction2_SSS(MBXMLUtils::TiXmlElement *element);
    Function2<fmatvec::Vec,fmatvec::Vec,double> *createFunction2_VVS(MBXMLUtils::TiXmlElement *element);
    Function3<fmatvec::Mat3xV,fmatvec::Vec,fmatvec::Vec,double> *createFunction3_MVVS(MBXMLUtils::TiXmlElement *element);
    ContourFunction1s * createContourFunction1s(MBXMLUtils::TiXmlElement * element) {return 0; }
    MM_PRINSPRE& getPriorityNamespacePrefix();
};

}

#endif
