#include "config.h"
#include "mbsimHydraulics/objectfactory.h"
#include "mbsimHydraulics/hydnode_mec.h"
#include "mbsimHydraulics/environment.h"

using namespace std;

namespace MBSim {

  HydraulicsObjectFactory HydraulicsObjectFactory::instance;

  HydraulicsObjectFactory::HydraulicsObjectFactory() : ObjectFactory() {
    ObjectFactory::getInstance()->registerObjectFactory(this);
  }

  //Group* HydraulicsObjectFactory::createGroup(TiXmlElement *element) {
  //  return 0;
  //}
  
  //Object* HydraulicsObjectFactory::createObject(TiXmlElement *element) {
  //  return 0;
  //}
  
  //Translation* HydraulicsObjectFactory::createTranslation(TiXmlElement *element) {
  //  return 0;
  //}
  
  //Rotation* HydraulicsObjectFactory::createRotation(TiXmlElement *element) {
  //  return 0;
  //}
  
  Link* HydraulicsObjectFactory::createLink(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMHYDRAULICSNS"HydNodeMecElastic")
      return new HydNodeMecElastic(element->Attribute("name"));
    return 0;
  }
  
  //Integrator* HydraulicsObjectFactory::createIntegrator(TiXmlElement *element) {
  //  return 0;
  //}

  //GeneralizedForceLaw* HydraulicsObjectFactory::createGeneralizedForceLaw(TiXmlElement *element) {
  //  return 0;
  //}

  //GeneralizedImpactLaw* HydraulicsObjectFactory::createGeneralizedImpactLaw(TiXmlElement *element) {
  //  return 0;
  //}
  
  //FrictionForceLaw *HydraulicsObjectFactory::createFrictionForceLaw(TiXmlElement *element) {
  //  return 0;
  //}

  //FrictionImpactLaw *HydraulicsObjectFactory::createFrictionImpactLaw(TiXmlElement *element) {
  //  return 0;
  //}

  //Contour *HydraulicsObjectFactory::createContour(TiXmlElement *element) {
  //  return 0;
  //}

  Environment* HydraulicsObjectFactory::getEnvironment(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMHYDRAULICSNS"HydraulicEnvironment")
      return HydraulicEnvironment::getInstance();
    return 0;
  }

}
