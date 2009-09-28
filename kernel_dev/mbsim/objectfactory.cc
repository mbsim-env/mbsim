#include "config.h"
#include "mbsim/objectfactory.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/group.h"
#include "mbsim/tree.h"
#include "mbsim/rigid_body.h"
#include "mbsim/kinetic_excitation.h"
#include "mbsim/spring_damper.h"
#include "mbsim/extern_generalized_io.h"
#include "mbsim/joint.h"
#include "mbsim/contact.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/line.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/contours/circle_hollow.h"
#include "mbsim/contours/frustum2d.h"
#include "mbsim/contours/area.h"
#include "mbsim/contours/edge.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/contours/plane.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/planewithfrustum.h"
#include "mbsim/contours/contour_interpolation.h"
#include "mbsim/contours/contour_quad.h"
#include "mbsim/contours/cuboid.h"
#include "mbsim/contours/compound_contour.h"
#include "mbsim/contours/contour1s_analytical.h"
#include "mbsim/utils/function_library.h"
#include "mbsim/integrators/dopri5_integrator.h"
#include "mbsim/integrators/radau5_integrator.h"
#include "mbsim/integrators/lsode_integrator.h"
#include "mbsim/integrators/lsodar_integrator.h"
#include "mbsim/integrators/time_stepping_integrator.h"
#include "mbsim/utils/contour_functions.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  ObjectFactory* ObjectFactory::instance=NULL;

  Group* ObjectFactory::createGroup(TiXmlElement *element) {
    Group *obj;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->createGroup(element))) return obj;
    return 0;
  }
  
  Object* ObjectFactory::createObject(TiXmlElement *element) {
    Object *obj;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->createObject(element))) return obj;
    return 0;
  }
  
  ExtraDynamic* ObjectFactory::createExtraDynamic(TiXmlElement *element) {
    ExtraDynamic *obj;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->createExtraDynamic(element))) return obj;
    return 0;
  }
  
  Translation* ObjectFactory::createTranslation(TiXmlElement *element) {
    Translation *obj;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->createTranslation(element))) return obj;
    return 0;
  }
  
  Rotation* ObjectFactory::createRotation(TiXmlElement *element) {
    Rotation *obj;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->createRotation(element))) return obj;
    return 0;
  }
  
  Link* ObjectFactory::createLink(TiXmlElement *element) {
    Link *obj;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->createLink(element))) return obj;
    return 0;
  }
  
  Integrator* ObjectFactory::createIntegrator(TiXmlElement *element) {
    Integrator *obj;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->createIntegrator(element))) return obj;
    return 0;
  }

  GeneralizedForceLaw* ObjectFactory::createGeneralizedForceLaw(TiXmlElement *element) {
    GeneralizedForceLaw *obj;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->createGeneralizedForceLaw(element))) return obj;
    return 0;
  }

  GeneralizedImpactLaw* ObjectFactory::createGeneralizedImpactLaw(TiXmlElement *element) {
    GeneralizedImpactLaw *obj;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->createGeneralizedImpactLaw(element))) return obj;
    return 0;
  }
  
  FrictionForceLaw *ObjectFactory::createFrictionForceLaw(TiXmlElement *element) {
    FrictionForceLaw *obj;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->createFrictionForceLaw(element))) return obj;
    return 0;
  }

  FrictionImpactLaw *ObjectFactory::createFrictionImpactLaw(TiXmlElement *element) {
    FrictionImpactLaw *obj;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->createFrictionImpactLaw(element))) return obj;
    return 0;
  }

  Contour *ObjectFactory::createContour(TiXmlElement *element) {
    Contour *obj;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->createContour(element))) return obj;
    return 0;
  }

  Environment *ObjectFactory::getEnvironment(TiXmlElement *element) {
    Environment *obj;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->getEnvironment(element))) return obj;
    return 0;
  }

  Jacobian *ObjectFactory::createJacobian(TiXmlElement *element) {
    Jacobian *obj;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->createJacobian(element))) return obj;
    return 0;
  }

  Function1<double,double> *ObjectFactory::createFunction1_SS(TiXmlElement *element) {
    Function1<double,double> *obj;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->createFunction1_SS(element))) return obj;
    return 0;
  }

  Function1<Vec,double> *ObjectFactory::createFunction1_VS(TiXmlElement *element) {
    Function1<Vec,double> *obj;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->createFunction1_VS(element))) return obj;
    return 0;
  }

  Function2<double,double,double> *ObjectFactory::createFunction2_SSS(TiXmlElement *element) {
    Function2<double,double,double> *obj;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->createFunction2_SSS(element))) return obj;
    return 0;
  }

  Function2<Vec,Vec,double> *ObjectFactory::createFunction2_VVS(TiXmlElement *element) {
    Function2<Vec,Vec,double> *obj;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->createFunction2_VVS(element))) return obj;
    return 0;
  }

  Function3<Mat,Vec,Vec,double> *ObjectFactory::createFunction3_MVVS(TiXmlElement *element) {
    Function3<Mat,Vec,Vec,double> *obj;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((obj=(*i)->createFunction3_MVVS(element))) return obj;
    return 0;
  }

  ContourFunction1s *ObjectFactory::createContourFunction1s(TiXmlElement *element) {
    ContourFunction1s * u;
    for(set<ObjectFactory*>::iterator i=factories.begin(); i!=factories.end(); i++)
      if((u=(*i)->createContourFunction1s(element))) return u;
    return 0;
  }



  MBSimObjectFactory *MBSimObjectFactory::instance=NULL;

  void MBSimObjectFactory::initialize() {
    if(instance==0) {
      instance=new MBSimObjectFactory;
      ObjectFactory::getInstance()->registerObjectFactory(instance);
    }
  }

  Group* MBSimObjectFactory::createGroup(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"DynamicSystemSolver")
      return new DynamicSystemSolver(element->Attribute("name"));
    else if(element->ValueStr()==MBSIMNS"Group")
      return new Group(element->Attribute("name"));
    return 0;
  }
  
  Object* MBSimObjectFactory::createObject(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"RigidBody")
      return new RigidBody(element->Attribute("name"));
    return 0;
  }
  
  Translation* MBSimObjectFactory::createTranslation(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"LinearTranslation")
      return new LinearTranslation;
    if(element->ValueStr()==MBSIMNS"TimeDependentTranslation1D")
      return new TimeDependentTranslation1D;
    return 0;
  }
  
  Rotation* MBSimObjectFactory::createRotation(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"RotationAboutFixedAxis")
      return new RotationAboutFixedAxis;
    if(element->ValueStr()==MBSIMNS"TimeDependentRotation1D")
      return new TimeDependentRotation1D;
    if(element->ValueStr()==MBSIMNS"CardanAngles")
      return new CardanAngles;
    return 0;
  }
  
  Link* MBSimObjectFactory::createLink(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"KineticExcitation")
      return new KineticExcitation(element->Attribute("name"));
    if(element->ValueStr()==MBSIMNS"SpringDamper")
      return new SpringDamper(element->Attribute("name"));
    if(element->ValueStr()==MBSIMNS"Joint")
      return new Joint(element->Attribute("name"));
    if(element->ValueStr()==MBSIMNS"Contact")
      return new Contact(element->Attribute("name"));
    if(element->ValueStr()==MBSIMNS"ExternGeneralizedIO")
      return new ExternGeneralizedIO(element->Attribute("name"));
    return 0;
  }
  
  Integrator* MBSimObjectFactory::createIntegrator(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMINTNS"DOPRI5Integrator")
      return new DOPRI5Integrator;
    if(element->ValueStr()==MBSIMINTNS"RADAU5Integrator")
      return new RADAU5Integrator;
    if(element->ValueStr()==MBSIMINTNS"LSODEIntegrator")
      return new LSODEIntegrator;
    if(element->ValueStr()==MBSIMINTNS"LSODARIntegrator")
      return new LSODARIntegrator;
    if(element->ValueStr()==MBSIMINTNS"TimeSteppingIntegrator")
      return new TimeSteppingIntegrator;
    return 0;
  }

  GeneralizedForceLaw* MBSimObjectFactory::createGeneralizedForceLaw(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"BilateralConstraint")
      return new BilateralConstraint;
    if(element->ValueStr()==MBSIMNS"UnilateralConstraint")
      return new UnilateralConstraint;
    if(element->ValueStr()==MBSIMNS"RegularizedBilateralConstraint")
      return new RegularizedBilateralConstraint;
    if(element->ValueStr()==MBSIMNS"RegularizedUnilateralConstraint")
      return new RegularizedUnilateralConstraint;
    return 0;
  }

  GeneralizedImpactLaw* MBSimObjectFactory::createGeneralizedImpactLaw(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"BilateralImpact")
      return new BilateralImpact;
    if(element->ValueStr()==MBSIMNS"UnilateralNewtonImpact")
      return new UnilateralNewtonImpact;
    return 0;
  }
  
  FrictionForceLaw *MBSimObjectFactory::createFrictionForceLaw(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"SpatialCoulombFriction")
      return new SpatialCoulombFriction;
    if(element->ValueStr()==MBSIMNS"PlanarCoulombFriction")
      return new PlanarCoulombFriction;
    if(element->ValueStr()==MBSIMNS"RegularizedSpatialFriction")
      return new RegularizedSpatialFriction;
    return 0;
  }

  FrictionImpactLaw *MBSimObjectFactory::createFrictionImpactLaw(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"SpatialCoulombImpact")
      return new SpatialCoulombImpact;
    if(element->ValueStr()==MBSIMNS"PlanarCoulombImpact")
      return new PlanarCoulombImpact;
    return 0;
  }

  Contour *MBSimObjectFactory::createContour(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"CircleHollow")
      return new CircleHollow(element->Attribute("name"));
    if(element->ValueStr()==MBSIMNS"CircleSolid")
      return new CircleSolid(element->Attribute("name"));
    if(element->ValueStr()==MBSIMNS"Frustum")
      return new Frustum(element->Attribute("name"));
    if(element->ValueStr()==MBSIMNS"Line")
      return new Line(element->Attribute("name"));
    if(element->ValueStr()==MBSIMNS"Plane")
      return new Plane(element->Attribute("name"));
    if(element->ValueStr()==MBSIMNS"PlaneWithFrustum")
      return new PlaneWithFrustum(element->Attribute("name"));
    if(element->ValueStr()==MBSIMNS"Point")
      return new Point(element->Attribute("name"));
    if(element->ValueStr()==MBSIMNS"Sphere")
      return new Sphere(element->Attribute("name"));
    if(element->ValueStr()==MBSIMNS"Contour1sAnalytical")
      return new Contour1sAnalytical(element->Attribute("name"));
    return 0;
  }

  Environment *MBSimObjectFactory::getEnvironment(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"MBSimEnvironment")
      return MBSimEnvironment::getInstance();
    return 0;
  }

  Jacobian *MBSimObjectFactory::createJacobian(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"ConstantJacobian")
      return new ConstantJacobian;
    return 0;
  }

  Function1<double,double> *MBSimObjectFactory::createFunction1_SS(TiXmlElement *element) {
    if(element->ValueStr()==MBSIMNS"ConstantFunction1_SS")
      return new ConstantFunction1<double,double>;
    if(element->ValueStr()==MBSIMNS"Function1_SS_from_VS")
      return new Function1_SS_from_VS();
    return 0;
  }
  
  Function1<Vec,double> *MBSimObjectFactory::createFunction1_VS(TiXmlElement *element) {
    if(element->ValueStr()==MBSIMNS"ConstantFunction1_VS")
      return new ConstantFunction1<Vec,double>;
    if(element->ValueStr()==MBSIMNS"PiecewisePolynom1_VS")
      return new PPolynom;
    if(element->ValueStr()==MBSIMNS"QuadraticFunction1_VS")
      return new QuadraticFunction1_VS;
    if(element->ValueStr()==MBSIMNS"SinusFunction1_VS")
      return new SinusFunction1_VS;
    if(element->ValueStr()==MBSIMNS"PositiveSinusFunction1_VS")
      return new PositiveSinusFunction1_VS;
    if(element->ValueStr()==MBSIMNS"StepFunction1_VS")
      return new StepFunction1_VS;
    if(element->ValueStr()==MBSIMNS"TabularFunction1_VS")
      return new TabularFunction1_VS;
    if(element->ValueStr()==MBSIMNS"PeriodicTabularFunction1_VS")
      return new PeriodicTabularFunction1_VS;
    if(element->ValueStr()==MBSIMNS"Function1_VS_from_SS")
      return new Function1_VS_from_SS;
    return 0;
  }
  
  Function2<double,double,double> *MBSimObjectFactory::createFunction2_SSS(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"ConstantFunction2_SSS")
      return new ConstantFunction2<double,double,double>;
    if(element->ValueStr()==MBSIMNS"LinearSpringDamperForce")
      return new LinearSpringDamperForce;
    if(element->ValueStr()==MBSIMNS"NonlinearSpringDamperForce")
      return new NonlinearSpringDamperForce;
    if(element->ValueStr()==MBSIMNS"LinearRegularizedUnilateralConstraint")
      return new LinearRegularizedUnilateralConstraint;
    if(element->ValueStr()==MBSIMNS"LinearRegularizedBilateralConstraint")
      return new LinearRegularizedBilateralConstraint;
    return 0;
  }

  Function2<Vec,Vec,double> *MBSimObjectFactory::createFunction2_VVS(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"LinearRegularizedCoulombFriction")
      return new LinearRegularizedCoulombFriction;
    if(element->ValueStr()==MBSIMNS"LinearRegularizedCoulombFriction")
      return new LinearRegularizedCoulombFriction;
    if(element->ValueStr()==MBSIMNS"LinearRegularizedStribeckFriction")
      return new LinearRegularizedStribeckFriction;
    return 0;
  }

  Function3<Mat,Vec,Vec,double> *MBSimObjectFactory::createFunction3_MVVS(TiXmlElement *element) {
    return 0;
  }

}
