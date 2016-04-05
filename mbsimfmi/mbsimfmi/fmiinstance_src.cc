#include "../config.h"
#include <fmiinstance.h>
#include <mbsim/dynamic_system_solver.h>

#include <mbxmlutilshelper/shared_library.h>
#include "../general/mbsimsrc_fmi.h"

using namespace std;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace boost::filesystem;

namespace MBSimFMI {

  void FMIInstance::addModelParametersAndCreateDSS(vector<boost::shared_ptr<Variable> > &varSim) {
    // get the model shared library
    path mbsimsrclibfile=path(MBXMLUtils::getFMUSharedLibPath()).parent_path().parent_path().parent_path()/
      "model"/("libmbsimfmi_model"+SHEXT);

    modelLib=boost::make_shared<SharedLibrary>(absolute(mbsimsrclibfile).string());
    DynamicSystemSolver *dssPtr;
    modelLib->getSymbol<mbsimSrcFMIPtr>("mbsimSrcFMI")(dssPtr);
    dss.reset(dssPtr);
  }

}
