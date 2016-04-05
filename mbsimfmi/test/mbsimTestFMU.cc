// includes
#include <mbxmlutilshelper/shared_library.h>
#include <fmiModelFunctions.h>
#include <boost/make_shared.hpp>
#include <boost/filesystem/path.hpp>
#include <cstdarg>

// typedefs for FMI function (must be in sync with fmiModelFunction.h)
typedef fmiStatus (*t_fmiSetString)(fmiComponent, const fmiValueReference[], size_t, const fmiString[]);
typedef fmiComponent (*t_fmiInstantiateModel)(fmiString, fmiString, fmiCallbackFunctions, fmiBoolean);
typedef fmiStatus (*t_fmiInitialize)(fmiComponent, fmiBoolean, fmiReal, fmiEventInfo*);
typedef fmiStatus (*t_fmiTerminate)(fmiComponent);
typedef void (*t_fmiFreeModelInstance)(fmiComponent);

using namespace std;
using namespace boost;
using namespace boost::filesystem;
using namespace MBXMLUtils;

namespace {

// some platform dependent file suffixes, directory names, ...
#ifdef _WIN32
  #ifdef _WIN64
    const string FMIOS("win64");
  #else
    const string FMIOS("win32");
  #endif
#else
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
void setOutput(const shared_ptr<SharedLibrary> &fmu, void *comp, const string &modelIdentifier, fmiValueReference vr, const string &output) {
  const char* str=output.c_str();
  if(fmu->getSymbol<t_fmiSetString>(modelIdentifier+"_fmiSetString")(comp, &vr, 1, &str)!=fmiOK)
    throw runtime_error("fmisetString failed");
}

// load, init and deinit one FMU
void singleLoad(const path &fmuFile, fmiCallbackFunctions cbFuncs, const string &guid, const string &modelIdentifier, int outputVR) {
  shared_ptr<SharedLibrary> fmu=make_shared<SharedLibrary>(fmuFile.string());

  void *comp=fmu->getSymbol<t_fmiInstantiateModel>(modelIdentifier+"_fmiInstantiateModel")("fmu1", guid.c_str(), cbFuncs, fmiFalse);
  if(!comp)
    throw runtime_error("fmiInstantiateModel failed");

  setOutput(fmu, comp, modelIdentifier, outputVR, "fmuOutput");

  fmiEventInfo ei;
  if(fmu->getSymbol<t_fmiInitialize>(modelIdentifier+"_fmiInitialize")(comp, fmiFalse, 0.0, &ei)!=fmiOK)
    throw runtime_error("fmiInitialize failed");
  if(!ei.iterationConverged || ei.terminateSimulation)
    throw runtime_error("fmiInitialize failed by eventInfo");

  if(fmu->getSymbol<t_fmiTerminate>(modelIdentifier+"_fmiTerminate")(comp)!=fmiOK)
    throw runtime_error("fmiTerminate failed");

  fmu->getSymbol<t_fmiFreeModelInstance>(modelIdentifier+"_fmiFreeModelInstance")(comp);

  fmu.reset();
}

// load, init and deinit two FMU
void doubleLoad(const path &fmuFile, fmiCallbackFunctions cbFuncs, const string &guid, const string &modelIdentifier, int outputVR) {
  shared_ptr<SharedLibrary> fmu1=make_shared<SharedLibrary>(fmuFile.string());

  void *comp1=fmu1->getSymbol<t_fmiInstantiateModel>(modelIdentifier+"_fmiInstantiateModel")("fmu1", guid.c_str(), cbFuncs, fmiFalse);
  if(!comp1)
    throw runtime_error(modelIdentifier+"_fmiInstantiateModel failed");

  setOutput(fmu1, comp1, modelIdentifier, outputVR, "fmuOutput1");

  shared_ptr<SharedLibrary> fmu2=make_shared<SharedLibrary>(fmuFile.string());

  void *comp2=fmu2->getSymbol<t_fmiInstantiateModel>(modelIdentifier+"_fmiInstantiateModel")("fmu2", guid.c_str(), cbFuncs, fmiFalse);
  if(!comp2)
    throw runtime_error(modelIdentifier+"_fmiInstantiateModel failed");

  setOutput(fmu2, comp2, modelIdentifier, outputVR, "fmuOutput2");



  fmiEventInfo ei;

  if(fmu1->getSymbol<t_fmiInitialize>(modelIdentifier+"_fmiInitialize")(comp1, fmiFalse, 0.0, &ei)!=fmiOK)
    throw runtime_error("fmiInitialize failed");
  if(!ei.iterationConverged || ei.terminateSimulation)
    throw runtime_error("fmiInitialize failed by eventInfo");

  if(fmu2->getSymbol<t_fmiInitialize>(modelIdentifier+"_fmiInitialize")(comp2, fmiFalse, 0.0, &ei)!=fmiOK)
    throw runtime_error("fmiInitialize failed");
  if(!ei.iterationConverged || ei.terminateSimulation)
    throw runtime_error("fmiInitialize failed by eventInfo");



  if(fmu1->getSymbol<t_fmiTerminate>(modelIdentifier+"_fmiTerminate")(comp1)!=fmiOK)
    throw runtime_error("fmiTerminate failed");

  fmu1->getSymbol<t_fmiFreeModelInstance>(modelIdentifier+"_fmiFreeModelInstance")(comp1);

  fmu1.reset();

  if(fmu2->getSymbol<t_fmiTerminate>(modelIdentifier+"_fmiTerminate")(comp2)!=fmiOK)
    throw runtime_error("fmiTerminate failed");

  fmu2->getSymbol<t_fmiFreeModelInstance>(modelIdentifier+"_fmiFreeModelInstance")(comp2);

  fmu1.reset();
}

}



int main(int argc, char *argv[]) {
  if(argc!=2) {
    cout<<"Usage: "<<argv[0]<<" <FMU dir>"<<endl;
    return 1;
  }

  try {
    // configuration
    string fmu="mbsim.so";
    string guid="mbsimfmi_guid";
    string modelIdentifier="mbsim";
    int outputVR=0; // the VR of the "Output directory"

    // arguments
    path fmuFile=path(argv[1])/"binaries"/FMIOS/fmu;

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
