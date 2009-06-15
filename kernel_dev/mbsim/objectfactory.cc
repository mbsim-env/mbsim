#include "config.h"
#include "mbsim/objectfactory.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/group.h"
#include "mbsim/tree.h"
#include "mbsim/rigid_body.h"
#include "mbsim/linear_spring_damper.h"
#include "mbsim/joint.h"
#include "mbsim/contact.h"
#include "mbsim/integrators/dopri5_integrator.h"
#include "mbsim/integrators/radau5_integrator.h"
#include "mbsim/integrators/lsodar_integrator.h"
#include "mbsim/integrators/time_stepping_integrator.h"

using namespace std;

namespace MBSim {

  Group* ObjectFactory::createGroup(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"DynamicSystemSolver")
      return new DynamicSystemSolver(element->Attribute("name"));
    else if(element->ValueStr()==MBSIMNS"Group")
      return new Group(element->Attribute("name"));
    return 0;
  }
  
  Object* ObjectFactory::createObject(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"RigidBody")
      return new RigidBody(element->Attribute("name"));
    return 0;
  }
  
  Translation* ObjectFactory::createTranslation(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"LinearTranslation")
      return new LinearTranslation;
    return 0;
  }
  
  Rotation* ObjectFactory::createRotation(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"RotationAboutFixedAxis")
      return new RotationAboutFixedAxis;
    if(element->ValueStr()==MBSIMNS"CardanAngles")
      return new CardanAngles;
    return 0;
  }
  
  Link* ObjectFactory::createLink(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"LinearSpringDamper")
      return new LinearSpringDamper(element->Attribute("name"));
    if(element->ValueStr()==MBSIMNS"Joint")
      return new Joint(element->Attribute("name"));
    if(element->ValueStr()==MBSIMNS"Contact")
      return new Contact(element->Attribute("name"));
    return 0;
  }
  
  Integrator* ObjectFactory::createIntegrator(TiXmlElement *element) {
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

  GeneralizedForceLaw* ObjectFactory::createGeneralizedForceLaw(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"BilateralConstraint")
      return new BilateralConstraint;
    if(element->ValueStr()==MBSIMNS"UnilateralConstraint")
      return new UnilateralConstraint;
    if(element->ValueStr()==MBSIMNS"LinearRegularizedBilateralConstraint")
      return new LinearRegularizedBilateralConstraint;
    return 0;
  }

  GeneralizedImpactLaw* ObjectFactory::createGeneralizedImpactLaw(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"BilateralImpact")
      return new BilateralImpact;
    if(element->ValueStr()==MBSIMNS"UnilateralNewtonImpact")
      return new UnilateralNewtonImpact;
    return 0;
  }
  
  FrictionForceLaw *ObjectFactory::createFrictionForceLaw(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"SpatialCoulombFriction")
      return new SpatialCoulombFriction;
    return 0;
  }

  FrictionImpactLaw *ObjectFactory::createFrictionImpactLaw(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"SpatialCoulombImpact")
      return new SpatialCoulombImpact;
    return 0;
  }

  Contour *ObjectFactory::createContour(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"Plane")
      return new Plane(element->Attribute("name"));
    if(element->ValueStr()==MBSIMNS"Sphere")
      return new Sphere(element->Attribute("name"));
    return 0;
  }

}
