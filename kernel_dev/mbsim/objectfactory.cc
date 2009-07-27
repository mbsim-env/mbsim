#include "config.h"
#include "mbsim/objectfactory.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/group.h"
#include "mbsim/tree.h"
#include "mbsim/rigid_body.h"
#include "mbsim/linear_spring_damper.h"
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
#include "mbsim/contours/planewithfrustum.h"
#include "mbsim/contours/contour_interpolation.h"
#include "mbsim/contours/contour_quad.h"
#include "mbsim/contours/cuboid.h"
#include "mbsim/contours/compound_contour.h"
#include "mbsim/integrators/dopri5_integrator.h"
#include "mbsim/integrators/radau5_integrator.h"
#include "mbsim/integrators/lsodar_integrator.h"
#include "mbsim/integrators/time_stepping_integrator.h"

using namespace std;

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


  MBSimObjectFactory MBSimObjectFactory::instance;

  MBSimObjectFactory::MBSimObjectFactory() : ObjectFactory() {
    ObjectFactory::getInstance()->registerObjectFactory(this);
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
    return 0;
  }
  
  Rotation* MBSimObjectFactory::createRotation(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"RotationAboutFixedAxis")
      return new RotationAboutFixedAxis;
    if(element->ValueStr()==MBSIMNS"CardanAngles")
      return new CardanAngles;
    return 0;
  }
  
  Link* MBSimObjectFactory::createLink(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"LinearSpringDamper")
      return new LinearSpringDamper(element->Attribute("name"));
    if(element->ValueStr()==MBSIMNS"Joint")
      return new Joint(element->Attribute("name"));
    if(element->ValueStr()==MBSIMNS"Contact")
      return new Contact(element->Attribute("name"));
    return 0;
  }
  
  Integrator* MBSimObjectFactory::createIntegrator(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMINTNS"DOPRI5Integrator")
      return new DOPRI5Integrator;
    if(element->ValueStr()==MBSIMINTNS"RADAU5Integrator")
      return new RADAU5Integrator;
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
    if(element->ValueStr()==MBSIMNS"LinearRegularizedBilateralConstraint")
      return new LinearRegularizedBilateralConstraint;
    if(element->ValueStr()==MBSIMNS"LinearRegularizedUnilateralConstraint")
      return new LinearRegularizedUnilateralConstraint;
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
    if(element->ValueStr()==MBSIMNS"LinearRegularizedSpatialCoulombFriction")
      return new LinearRegularizedSpatialCoulombFriction;
    return 0;
  }

  FrictionImpactLaw *MBSimObjectFactory::createFrictionImpactLaw(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"SpatialCoulombImpact")
      return new SpatialCoulombImpact;
    return 0;
  }

  Contour *MBSimObjectFactory::createContour(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"Plane")
      return new Plane(element->Attribute("name"));
    if(element->ValueStr()==MBSIMNS"Sphere")
      return new Sphere(element->Attribute("name"));
    return 0;
  }

  Environment *MBSimObjectFactory::getEnvironment(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"MBSimEnvironment")
      return MBSimEnvironment::getInstance();
    return 0;
  }

}
