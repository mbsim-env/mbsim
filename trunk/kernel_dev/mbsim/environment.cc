#include "config.h"
#include "mbsim/environment.h"
#include "mbsim/element.h"
#include "fmatvec.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  MBSimEnvironment *MBSimEnvironment::instance=NULL;

  void MBSimEnvironment::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMNS"accelerationOfGravity");
    setAccelerationOfGravity(Vec(e->GetText()));
  }

}
