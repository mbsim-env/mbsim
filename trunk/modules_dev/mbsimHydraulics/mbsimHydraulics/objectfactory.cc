#include "config.h"
#include "mbsimHydraulics/objectfactory.h"
#include "mbsimHydraulics/hydline.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimHydraulics/hydnode_mec.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/controlvalve43.h"

using namespace std;

namespace MBSim {

  HydraulicsObjectFactory *HydraulicsObjectFactory::instance=NULL;

  void HydraulicsObjectFactory::initialize() {
    if(instance==0) {
      instance=new HydraulicsObjectFactory;
      ObjectFactory::getInstance()->registerObjectFactory(instance);
    }
  }

  PressureLoss * HydraulicsObjectFactory::createPressureLoss(TiXmlElement * element) {
    if (element==0) return 0;
    if (element->ValueStr()==MBSIMHYDRAULICSNS"PressureLossZeta")
      return new PressureLossZeta(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"PressureLossLaminarTubeFlow")
      return new PressureLossLaminarTubeFlow(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"PressureLossCurveFit")
      return new PressureLossCurveFit(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"VariablePressureLossAreaZeta")
      return new VariablePressureLossAreaZeta(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"RegularizedVariablePressureLossAreaZeta")
      return new RegularizedVariablePressureLossAreaZeta(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"VariablePressureLossControlvalveAreaAlpha")
      return new VariablePressureLossControlvalveAreaAlpha(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"RegularizedVariablePressureLossControlvalveAreaAlpha")
      return new RegularizedVariablePressureLossControlvalveAreaAlpha(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"VariablePressureLossCheckvalveGamma")
      return new VariablePressureLossCheckvalveGamma(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"RegularizedVariablePressureLossCheckvalveGamma")
      return new RegularizedVariablePressureLossCheckvalveGamma(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"VariablePressureLossCheckvalveIdelchick")
      return new VariablePressureLossCheckvalveIdelchick(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"RegularizedVariablePressureLossCheckvalveIdelchick")
      return new RegularizedVariablePressureLossCheckvalveIdelchick(element->Attribute("name"));
  }

  Object * HydraulicsObjectFactory::createObject(TiXmlElement * element) {
    if (element==0) return 0;
    if (element->ValueStr()==MBSIMHYDRAULICSNS"HydLine")
      return new HydLine(element->Attribute("name"));
    return 0;
  }

  Link* HydraulicsObjectFactory::createLink(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMHYDRAULICSNS"HydNodeConstrained")
      return new HydNodeConstrained(element->Attribute("name"));
    if(element->ValueStr()==MBSIMHYDRAULICSNS"HydNodeEnvironment")
      return new HydNodeEnvironment(element->Attribute("name"));
    if(element->ValueStr()==MBSIMHYDRAULICSNS"HydNodeElastic")
      return new HydNodeElastic(element->Attribute("name"));
    if(element->ValueStr()==MBSIMHYDRAULICSNS"HydNodeRigid")
      return new HydNodeRigid(element->Attribute("name"));
    if(element->ValueStr()==MBSIMHYDRAULICSNS"HydNodeMecConstrained")
      return new HydNodeMecConstrained(element->Attribute("name"));
    if(element->ValueStr()==MBSIMHYDRAULICSNS"HydNodeMecEnvironment")
      return new HydNodeMecEnvironment(element->Attribute("name"));
    if(element->ValueStr()==MBSIMHYDRAULICSNS"HydNodeMecElastic")
      return new HydNodeMecElastic(element->Attribute("name"));
    if(element->ValueStr()==MBSIMHYDRAULICSNS"HydNodeMecRigid")
      return new HydNodeMecRigid(element->Attribute("name"));
    return 0;
  }


  Environment* HydraulicsObjectFactory::getEnvironment(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMHYDRAULICSNS"HydraulicEnvironment")
      return HydraulicEnvironment::getInstance();
    return 0;
  }


  Group * HydraulicsObjectFactory::createGroup(TiXmlElement * element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMHYDRAULICSNS"Controlvalve43")
      return new Controlvalve43(element->Attribute("name"));
    if(element->ValueStr()==MBSIMHYDRAULICSNS"RegularizedControlvalve43")
      return new RegularizedControlvalve43(element->Attribute("name"));
    return 0;
  }

}
