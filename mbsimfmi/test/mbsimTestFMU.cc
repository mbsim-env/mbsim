// includes
#include <mbxmlutilshelper/shared_library.h>
#include <fmiModelFunctions.h>
#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>
#include <cstdarg>
#include <cstdio>
#include <iostream>

// typedefs for FMI function (must be in sync with fmiModelFunction.h)
typedef fmiStatus (*t_fmiSetString)(fmiComponent, const fmiValueReference[], size_t, const fmiString[]);
typedef fmiComponent (*t_fmiInstantiateModel)(fmiString, fmiString, fmiCallbackFunctions, fmiBoolean);
typedef fmiStatus (*t_fmiInitialize)(fmiComponent, fmiBoolean, fmiReal, fmiEventInfo*);
typedef fmiStatus (*t_fmiTerminate)(fmiComponent);
typedef void (*t_fmiFreeModelInstance)(fmiComponent);

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
void singleLoad(const path &fmuFile, fmiCallbackFunctions cbFuncs, const string &guid, const string &modelIdentifier, int outputVR) {
  void *comp=SharedLibrary::getSymbol<t_fmiInstantiateModel>(fmuFile.string(), 
    modelIdentifier+"_fmiInstantiateModel")("fmu1", guid.c_str(), cbFuncs, fmiFalse);
  if(!comp)
    throw runtime_error("fmiInstantiateModel failed");

  setOutput(fmuFile, comp, modelIdentifier, outputVR, "fmuOutput");

  fmiEventInfo ei;
  if(SharedLibrary::getSymbol<t_fmiInitialize>(fmuFile.string(), modelIdentifier+"_fmiInitialize")(comp, fmiFalse, 0.0, &ei)!=fmiOK)
    throw runtime_error("fmiInitialize failed");
  if(!ei.iterationConverged || ei.terminateSimulation)
    throw runtime_error("fmiInitialize failed by eventInfo");

  if(SharedLibrary::getSymbol<t_fmiTerminate>(fmuFile.string(), modelIdentifier+"_fmiTerminate")(comp)!=fmiOK)
    throw runtime_error("fmiTerminate failed");

  SharedLibrary::getSymbol<t_fmiFreeModelInstance>(fmuFile.string(), modelIdentifier+"_fmiFreeModelInstance")(comp);
}

// load, init and deinit two FMU
void doubleLoad(const path &fmuFile, fmiCallbackFunctions cbFuncs, const string &guid, const string &modelIdentifier, int outputVR) {
  void *comp1=SharedLibrary::getSymbol<t_fmiInstantiateModel>(fmuFile.string(),
    modelIdentifier+"_fmiInstantiateModel")("fmu1", guid.c_str(), cbFuncs, fmiFalse);
  if(!comp1)
    throw runtime_error(modelIdentifier+"_fmiInstantiateModel failed");

  setOutput(fmuFile, comp1, modelIdentifier, outputVR, "fmuOutput1");

  void *comp2=SharedLibrary::getSymbol<t_fmiInstantiateModel>(fmuFile.string(), 
    modelIdentifier+"_fmiInstantiateModel")("fmu2", guid.c_str(), cbFuncs, fmiFalse);
  if(!comp2)
    throw runtime_error(modelIdentifier+"_fmiInstantiateModel failed");

  setOutput(fmuFile, comp2, modelIdentifier, outputVR, "fmuOutput2");



  fmiEventInfo ei;

  if(SharedLibrary::getSymbol<t_fmiInitialize>(fmuFile.string(), modelIdentifier+"_fmiInitialize")(comp1, fmiFalse, 0.0, &ei)!=fmiOK)
    throw runtime_error("fmiInitialize failed");
  if(!ei.iterationConverged || ei.terminateSimulation)
    throw runtime_error("fmiInitialize failed by eventInfo");

  if(SharedLibrary::getSymbol<t_fmiInitialize>(fmuFile.string(), modelIdentifier+"_fmiInitialize")(comp2, fmiFalse, 0.0, &ei)!=fmiOK)
    throw runtime_error("fmiInitialize failed");
  if(!ei.iterationConverged || ei.terminateSimulation)
    throw runtime_error("fmiInitialize failed by eventInfo");



  if(SharedLibrary::getSymbol<t_fmiTerminate>(fmuFile.string(), modelIdentifier+"_fmiTerminate")(comp1)!=fmiOK)
    throw runtime_error("fmiTerminate failed");

  SharedLibrary::getSymbol<t_fmiFreeModelInstance>(fmuFile.string(), modelIdentifier+"_fmiFreeModelInstance")(comp1);

  if(SharedLibrary::getSymbol<t_fmiTerminate>(fmuFile.string(), modelIdentifier+"_fmiTerminate")(comp2)!=fmiOK)
    throw runtime_error("fmiTerminate failed");

  SharedLibrary::getSymbol<t_fmiFreeModelInstance>(fmuFile.string(), modelIdentifier+"_fmiFreeModelInstance")(comp2);
}

}



int main(int argc, char *argv[]) {
  if(argc!=2) {
    cout<<"Usage: "<<argv[0]<<" <FMU dir>"<<endl;
    return 1;
  }

  try {
    // configuration
    string fmu="mbsim" + SHEXT;
    string guid="mbsimfmi_guid";
    string modelIdentifier="mbsim";
    int outputVR=0; // the VR of the "Output directory"

    // arguments
    path fmuFile=canonical(path(argv[1])/"binaries"/FMIOS/fmu);

    // callbacks
    fmiCallbackFunctions cbFuncs;
    cbFuncs.logger=&fmiCallbackLoggerImpl;
    cbFuncs.allocateMemory=&calloc;
    cbFuncs.freeMemory=&free;

    // FMU calls

    cout<<"First single load and init started"<<endl;
    singleLoad(fmuFile, cbFuncs, guid, modelIdentifier, outputVR);
    cout<<"First single load and init done"<<endl;

    cout<<"Second single load and init started"<<endl;
    singleLoad(fmuFile, cbFuncs, guid, modelIdentifier, outputVR);
    cout<<"Second single load and init done"<<endl;

    cout<<"Double load and init started"<<endl;
    doubleLoad(fmuFile, cbFuncs, guid, modelIdentifier, outputVR);
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
