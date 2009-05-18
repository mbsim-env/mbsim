#include "config.h"
#include "mbsim/objectfactory.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/group.h"
#include "mbsim/tree.h"
#include "mbsim/rigid_body.h"
#include "mbsim/linear_spring_damper.h"
#include "mbsim/joint.h"
#include "mbsim/integrators/dopri5_integrator.h"

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
    return 0;
  }
  
  LinkMechanics* ObjectFactory::createLinkMechanics(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"LinearSpringDamper")
      return new LinearSpringDamper(element->Attribute("name"));
    if(element->ValueStr()==MBSIMNS"Joint")
      return new Joint(element->Attribute("name"));
    return 0;
  }
  
  Integrator* ObjectFactory::createIntegrator(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMINTNS"DOPRI5Integrator")
      return new DOPRI5Integrator;
    return 0;
  }

  GeneralizedForceLaw* ObjectFactory::createGeneralizedForceLaw(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"BilateralConstraint")
      return new BilateralConstraint;
    return 0;
  }

  GeneralizedImpactLaw* ObjectFactory::createGeneralizedImpactLaw(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMNS"BilateralImpact")
      return new BilateralImpact;
    return 0;
  }

}
