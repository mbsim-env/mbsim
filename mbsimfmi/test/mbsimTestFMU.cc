// includes
#include <cassert>
#include <cfenv>
#include <mbxmlutilshelper/shared_library.h>
#include <fmiFunctions.h>
#include <fmiModelFunctions.h>
#include <fmiFunctions.h>
#include <boost/filesystem.hpp>
#include <cstdarg>
#include <cstdio>
#include <iostream>

// typedefs for FMI function (must be in sync with fmiModelFunction.h)
typedef fmiStatus (*t_fmiSetString)(fmiComponent, const fmiValueReference[], size_t, const fmiString[]);
typedef fmiComponent (*t_fmiInstantiateModel)(fmiString, fmiString, fmiCallbackFunctions_me, fmiBoolean);
typedef fmiComponent (*t_fmiInstantiateSlave)(fmiString instanceName, fmiString fmuGUID, fmiString fmuLocation,
                                              fmiString mimeType, fmiReal timeout, fmiBoolean visible,
                                              fmiBoolean interactive, fmiCallbackFunctions_cosim functions, fmiBoolean loggingOn);
typedef fmiStatus (*t_fmiInitialize)(fmiComponent, fmiBoolean, fmiReal, fmiEventInfo*);
typedef fmiStatus (*t_fmiInitializeSlave)(fmiComponent c, fmiReal tStart, fmiBoolean StopTimeDefined, fmiReal tStop);
typedef fmiStatus (*t_fmiTerminate)(fmiComponent);
typedef fmiStatus (*t_fmiTerminateSlave)(fmiComponent);
typedef void (*t_fmiFreeModelInstance)(fmiComponent);
typedef void (*t_fmiFreeSlaveInstance)(fmiComponent);

using namespace std;
using namespace boost::filesystem;
using namespace MBXMLUtils;

namespace {

// some platform dependent file suffixes, directory names, ...
#ifdef _WIN32
  std::string SHEXT(".dll");
  #ifdef _WIN64
    const string FMIOS("win64");
  #else
    const string FMIOS("win32");
  #endif
#else
  std::string SHEXT(".so");
  #ifdef __x86_64__
    const string FMIOS("linux64");
  #else
    const string FMIOS("linux32");
  #endif
#endif

bool cosim;

// FMI callback function
extern "C"
void fmiCallbackLoggerImpl(fmiComponent c, fmiString instanceName, fmiStatus status, fmiString category, fmiString message, ...) {
  cout<<"Message from "<<instanceName<<" with category "<<category<<":"<<endl;
  va_list ap;
  va_start(ap, message);
  vprintf(message, ap);
  va_end(ap);
  cout<<endl;
}

// set FMI parmeter
void setOutput(const path &fmuFile, void *comp, const string &modelIdentifier, fmiValueReference vr, const string &output) {
  const char* str=output.c_str();
  if(SharedLibrary::getSymbol<t_fmiSetString>(fmuFile.string(), modelIdentifier+"_fmiSetString")(comp, &vr, 1, &str)!=fmiOK)
    throw runtime_error("fmisetString failed");
}

// load, init and deinit one FMU
void singleLoad(const path &fmuFile, fmiCallbackFunctions_me cbFuncs_me, fmiCallbackFunctions_cosim cbFuncs_cosim, const string &guid, const string &modelIdentifier, int outputVR) {
  void *comp;
  if(!cosim) {
    comp=SharedLibrary::getSymbol<t_fmiInstantiateModel>(fmuFile.string(), 
      modelIdentifier+"_fmiInstantiateModel")("fmu1", guid.c_str(), cbFuncs_me, fmiFalse);
    if(!comp)
      throw runtime_error("fmiInstantiateModel failed");
  }
  else {
    comp=SharedLibrary::getSymbol<t_fmiInstantiateSlave>(fmuFile.string(), 
      modelIdentifier+"_fmiInstantiateSlave")("fmu1", guid.c_str(), "dummy_location", "dummy_mimetype", 0,
      fmiFalse, fmiFalse, cbFuncs_cosim, fmiFalse);
    if(!comp)
      throw runtime_error("fmiInstantiateSlave failed");
  }

  setOutput(fmuFile, comp, modelIdentifier, outputVR, "fmuOutput");

  if(!cosim) {
    fmiEventInfo ei;
    if(SharedLibrary::getSymbol<t_fmiInitialize>(fmuFile.string(), modelIdentifier+"_fmiInitialize")(comp, fmiFalse, 0.0, &ei)!=fmiOK)
      throw runtime_error("fmiInitialize failed");
    if(!ei.iterationConverged || ei.terminateSimulation)
      throw runtime_error("fmiInitialize failed by eventInfo");
  }
  else {
    if(SharedLibrary::getSymbol<t_fmiInitializeSlave>(fmuFile.string(), modelIdentifier+"_fmiInitializeSlave")(comp, 0.0, fmiFalse, 0.0)!=fmiOK)
      throw runtime_error("fmiInitializeSlave failed");
  }

  if(!cosim) {
    if(SharedLibrary::getSymbol<t_fmiTerminate>(fmuFile.string(), modelIdentifier+"_fmiTerminate")(comp)!=fmiOK)
      throw runtime_error("fmiTerminate failed");

    SharedLibrary::getSymbol<t_fmiFreeModelInstance>(fmuFile.string(), modelIdentifier+"_fmiFreeModelInstance")(comp);
  }
  else {
    if(SharedLibrary::getSymbol<t_fmiTerminateSlave>(fmuFile.string(), modelIdentifier+"_fmiTerminateSlave")(comp)!=fmiOK)
      throw runtime_error("fmiTerminateSlave failed");

    SharedLibrary::getSymbol<t_fmiFreeSlaveInstance>(fmuFile.string(), modelIdentifier+"_fmiFreeSlaveInstance")(comp);
  }
}

// load, init and deinit two FMU
void doubleLoad(const path &fmuFile, fmiCallbackFunctions_me cbFuncs_me, fmiCallbackFunctions_cosim cbFuncs_cosim, const string &guid, const string &modelIdentifier, int outputVR) {
  void *comp1;
  if(!cosim) {
    comp1=SharedLibrary::getSymbol<t_fmiInstantiateModel>(fmuFile.string(),
      modelIdentifier+"_fmiInstantiateModel")("fmu1", guid.c_str(), cbFuncs_me, fmiFalse);
    if(!comp1)
      throw runtime_error(modelIdentifier+"_fmiInstantiateModel failed");
  }
  else {
    comp1=SharedLibrary::getSymbol<t_fmiInstantiateSlave>(fmuFile.string(), 
      modelIdentifier+"_fmiInstantiateSlave")("fmu1", guid.c_str(), "dummy_location", "dummy_mimetype", 0,
      fmiFalse, fmiFalse, cbFuncs_cosim, fmiFalse);
    if(!comp1)
      throw runtime_error("fmiInstantiateSlave failed");
  }

  setOutput(fmuFile, comp1, modelIdentifier, outputVR, "fmuOutput1");

  void *comp2;
  if(!cosim) {
    comp2=SharedLibrary::getSymbol<t_fmiInstantiateModel>(fmuFile.string(), 
      modelIdentifier+"_fmiInstantiateModel")("fmu2", guid.c_str(), cbFuncs_me, fmiFalse);
    if(!comp2)
      throw runtime_error(modelIdentifier+"_fmiInstantiateModel failed");
  }
  else {
    comp2=SharedLibrary::getSymbol<t_fmiInstantiateSlave>(fmuFile.string(), 
      modelIdentifier+"_fmiInstantiateSlave")("fmu1", guid.c_str(), "dummy_location", "dummy_mimetype", 0,
      fmiFalse, fmiFalse, cbFuncs_cosim, fmiFalse);
    if(!comp2)
      throw runtime_error("fmiInstantiateSlave failed");
  }

  setOutput(fmuFile, comp2, modelIdentifier, outputVR, "fmuOutput2");



  fmiEventInfo ei;

  if(!cosim) {
    if(SharedLibrary::getSymbol<t_fmiInitialize>(fmuFile.string(), modelIdentifier+"_fmiInitialize")(comp1, fmiFalse, 0.0, &ei)!=fmiOK)
      throw runtime_error("fmiInitialize failed");
    if(!ei.iterationConverged || ei.terminateSimulation)
      throw runtime_error("fmiInitialize failed by eventInfo");

    if(SharedLibrary::getSymbol<t_fmiInitialize>(fmuFile.string(), modelIdentifier+"_fmiInitialize")(comp2, fmiFalse, 0.0, &ei)!=fmiOK)
      throw runtime_error("fmiInitialize failed");
    if(!ei.iterationConverged || ei.terminateSimulation)
      throw runtime_error("fmiInitialize failed by eventInfo");
  }
  else {
    if(SharedLibrary::getSymbol<t_fmiInitializeSlave>(fmuFile.string(), modelIdentifier+"_fmiInitializeSlave")(comp1, 0.0, fmiFalse, 0.0)!=fmiOK)
      throw runtime_error("fmiInitializeSlave failed");

    if(SharedLibrary::getSymbol<t_fmiInitializeSlave>(fmuFile.string(), modelIdentifier+"_fmiInitializeSlave")(comp2, 0.0, fmiFalse, 0.0)!=fmiOK)
      throw runtime_error("fmiInitializeSlave failed");
  }



  if(!cosim) {
    if(SharedLibrary::getSymbol<t_fmiTerminate>(fmuFile.string(), modelIdentifier+"_fmiTerminate")(comp1)!=fmiOK)
      throw runtime_error("fmiTerminate failed");

    SharedLibrary::getSymbol<t_fmiFreeModelInstance>(fmuFile.string(), modelIdentifier+"_fmiFreeModelInstance")(comp1);

    if(SharedLibrary::getSymbol<t_fmiTerminate>(fmuFile.string(), modelIdentifier+"_fmiTerminate")(comp2)!=fmiOK)
      throw runtime_error("fmiTerminate failed");

    SharedLibrary::getSymbol<t_fmiFreeModelInstance>(fmuFile.string(), modelIdentifier+"_fmiFreeModelInstance")(comp2);
  }
  else {
    if(SharedLibrary::getSymbol<t_fmiTerminateSlave>(fmuFile.string(), modelIdentifier+"_fmiTerminateSlave")(comp1)!=fmiOK)
      throw runtime_error("fmiTerminateSlave failed");

    SharedLibrary::getSymbol<t_fmiFreeSlaveInstance>(fmuFile.string(), modelIdentifier+"_fmiFreeSlaveInstance")(comp1);

    if(SharedLibrary::getSymbol<t_fmiTerminateSlave>(fmuFile.string(), modelIdentifier+"_fmiTerminateSlave")(comp2)!=fmiOK)
      throw runtime_error("fmiTerminateSlave failed");

    SharedLibrary::getSymbol<t_fmiFreeSlaveInstance>(fmuFile.string(), modelIdentifier+"_fmiFreeSlaveInstance")(comp2);
  }
}

}



int main(int argc, char *argv[]) {
#ifndef _WIN32
  assert(feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW)!=-1);
#endif

  if(argc!=3) {
    cout<<"Usage: "<<argv[0]<<" --me|--cosim <FMU dir>"<<endl;
    return 1;
  }

  try {
    cosim=(string(argv[1])=="--cosim");

    // configuration
    string fmu="mbsim" + SHEXT;
    string guid="mbsimfmi_guid";
    string modelIdentifier="mbsim";
    int outputVR=0; // the VR of the "Output directory"

    // arguments
    path fmuFile=canonical(path(argv[2])/"binaries"/FMIOS/fmu);

    // callbacks
    fmiCallbackFunctions_me cbFuncs_me;
    cbFuncs_me.logger=&fmiCallbackLoggerImpl;
    cbFuncs_me.allocateMemory=&calloc;
    cbFuncs_me.freeMemory=&free;
    fmiCallbackFunctions_cosim cbFuncs_cosim;
    cbFuncs_cosim.logger=&fmiCallbackLoggerImpl;
    cbFuncs_cosim.allocateMemory=&calloc;
    cbFuncs_cosim.freeMemory=&free;
    cbFuncs_cosim.stepFinished=nullptr;

    // FMU calls

    cout<<"First single load and init started"<<endl;
    singleLoad(fmuFile, cbFuncs_me, cbFuncs_cosim, guid, modelIdentifier, outputVR);
    cout<<"First single load and init done"<<endl;

    cout<<"Second single load and init started"<<endl;
    singleLoad(fmuFile, cbFuncs_me, cbFuncs_cosim, guid, modelIdentifier, outputVR);
    cout<<"Second single load and init done"<<endl;

    cout<<"Double load and init started"<<endl;
    doubleLoad(fmuFile, cbFuncs_me, cbFuncs_cosim, guid, modelIdentifier, outputVR);
    cout<<"Double load and init done"<<endl;

    return 0;
  }
  catch(const std::exception &ex) {
    cout<<"Exception: "<<ex.what()<<endl;
  }
  catch(...) {
    cout<<"Unknown exception."<<endl;
  }
  return 1;
}
