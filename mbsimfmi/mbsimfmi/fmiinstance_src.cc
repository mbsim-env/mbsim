#include "../config.h"
#include <fmiinstance.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/integrators/integrator.h>

#include <mbxmlutilshelper/shared_library.h>
#include "../general/mbsimsrc_fmi.h"

using namespace std;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace boost::filesystem;

namespace MBSimFMI {

  void FMIInstance::addModelParametersAndCreateSystem(vector<std::shared_ptr<Variable> > &varSim) {
    // get the model shared library
    path mbsimsrclibfile=path(MBXMLUtils::getFMUSharedLibPath()).parent_path().parent_path().parent_path()/
      "model"/("libmbsimfmi_model"+SHEXT);

    // create DynamicSystemSolver
    DynamicSystemSolver *dssPtr;
    MBSimIntegrator::Integrator *integratorPtr;
    SharedLibrary::getSymbol<mbsimSrcFMIPtr>(canonical(mbsimsrclibfile).string(), "mbsimSrcFMI")(dssPtr, integratorPtr);
    dss.reset(dssPtr);

    if(cosim) {
      if(!integratorPtr)
        throw std::runtime_error("This is a cosim FMU but no Integrator is provided.");
      integrator.reset(integratorPtr);
    }
  }

}
