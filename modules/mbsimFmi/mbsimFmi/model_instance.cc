#include "model_instance.h"
namespace fmi {
fmiCallbackFunctions ModelInstance::functions;

ModelInstance::ModelInstance(MBSim::DynamicSystemSolver *s,fmiString instanceName, fmiString GUID, fmiCallbackFunctions functions, fmiBoolean loggingOn):
  r(NUMBER_OF_REALS),
  i(NUMBER_OF_INTEGERS),
  b(NUMBER_OF_BOOLEANS),
  s(NUMBER_OF_STRINGS),
  isPositive(NUMBER_OF_EVENT_INDICATORS),
  system(s),
  time(0),
  instanceName(instanceName),
  GUID(GUID),
  loggingOn(loggingOn),
  state(modelError)
{
  ModelInstance::setFmiCallbacksFunctions(functions);
  system->initialize();
  printf("qInd %d\n",system->getqSize());
  system->initz(this->r);
}

ModelInstance::~ModelInstance(){
}

void ModelInstance::initialize(){
  system->initz(this->r);
}
std::vector<fmiReal> ModelInstance::update(){
  return (std::vector<fmiReal>) system->zdot(r,time);
}

}//end namespace fmi
