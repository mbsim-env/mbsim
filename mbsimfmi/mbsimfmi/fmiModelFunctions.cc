// includes
#include "config.h"
#include <string>
#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include <mbxmlutilshelper/shared_library.h>

// include the fmi header
extern "C" {
  #define MODEL_IDENTIFIER mbsim
  #include <fmiinstancebase.h> // this includes 3rdparty/fmiModelFunctions
}

// define getFMUWrapperSharedLibPath() which returns the FMU wrapper shared library path = .../binaries/<OS>/<fmuname>.[so|dll]
#define MBXMLUTILS_SHAREDLIBNAME FMUWrapper
#include <mbxmlutilshelper/getsharedlibpath_impl.h>

#define DLLEXPORT __attribute__((visibility("default")))

// use namespaces
using namespace std;
using namespace MBSimFMI;
using namespace MBXMLUtils;

namespace {
  // some platform dependent values
#ifdef _WIN32
  string SHEXT(".dll");
  string LIBDIR="bin";
#else
  string SHEXT(".so");
  string LIBDIR="lib";
#endif

  // FMI instance struct of mbsim.so: hold the real SharedLibrary and the real instance
  struct Instance {
    Instance(const std::shared_ptr<FMIInstanceBase> &instance_) : instance(instance_) {}
    std::shared_ptr<FMIInstanceBase> instance;
  };
}

// define all FMI function as C functions
extern "C" {

  // global FMI function.
  DLLEXPORT const char* fmiGetModelTypesPlatform() {
    return "standard32";
  }

  // global FMI function.
  DLLEXPORT const char* fmiGetVersion() {
    return "1.0";
  }

  // FMI instantiate function: just calls the FMIInstanceBase ctor
  // Convert exceptions to FMI logger calls and return no instance.
  DLLEXPORT fmiComponent fmiInstantiateModel(fmiString instanceName_, fmiString GUID, fmiCallbackFunctions functions, fmiBoolean loggingOn) {
    try {
      string fmuDir=MBXMLUtils::getFMUWrapperSharedLibPath();
      size_t s=string::npos;
      // replace /./ with /
      while((s=fmuDir.find("/./"))!=string::npos)
        fmuDir.replace(s, 3, "/");
      while((s=fmuDir.find("\\.\\"))!=string::npos)
        fmuDir.replace(s, 3, "/");
      while((s=fmuDir.find("\\./"))!=string::npos)
        fmuDir.replace(s, 3, "/");
      while((s=fmuDir.find("/.\\"))!=string::npos)
        fmuDir.replace(s, 3, "/");
      // remove trailing /binaries/<os>/mbism.so
      for(int i=0; i<3; ++i)
        s=fmuDir.find_last_of("/\\", s)-1;
      // load main mbsim FMU library
      fmiInstanceCreatePtr fmiInstanceCreate=SharedLibrary::getSymbol<fmiInstanceCreatePtr>(
        fmuDir.substr(0, s+1)+"/resources/local/"+LIBDIR+"/libmbsimXXX_fmi"+SHEXT, "fmiInstanceCreate");
      return new Instance(fmiInstanceCreate(instanceName_, GUID, functions, loggingOn));
    }
    // note: we can not use the instance here since the creation has failed
    catch(const exception &ex) {
      string instanceName=instanceName_; // passing instanceName_ to logger is not allowed acording the FMI standard
      functions.logger(NULL, instanceName.c_str(), fmiError, "error", ex.what());
      return NULL;
    }
    catch(...) {
      string instanceName=instanceName_; // passing instanceName_ to logger is not allowed acording the FMI standard
      functions.logger(NULL, instanceName.c_str(), fmiError, "error", "Unknown error");
      return NULL;
    }
  }

  // FMI free instance function: just calls the FMIInstanceBase dtor.
  // No exception handling needed since the dtor must not throw.
  DLLEXPORT void fmiFreeModelInstance(fmiComponent c) {
    // must not throw
    delete static_cast<Instance*>(c);
  }

  // All other FMI functions: just calls the corresponding member function in FMIInstanceBase
  // Convert exceptions to call of logError which itself passed these to the FMI logge and return with fmiError.
  #define FMIFUNC(fmiFuncName, instanceMemberName, Sig, sig) \
  DLLEXPORT fmiStatus fmiFuncName Sig { \
    std::shared_ptr<FMIInstanceBase> instance=static_cast<Instance*>(c)->instance; \
    try { \
      instance->instanceMemberName sig; \
      return fmiOK; \
    } \
    catch(const exception &ex) { \
      instance->logException(ex); \
      return fmiError; \
    } \
    catch(...) { \
      instance->logException(runtime_error("Unknwon error")); \
      return fmiError; \
    } \
  }

  // All other FMI function (see above macro)
  FMIFUNC(fmiSetDebugLogging, setDebugLogging,
    (fmiComponent c, fmiBoolean loggingOn),
    (loggingOn))

  FMIFUNC(fmiSetTime, setTime,
    (fmiComponent c, fmiReal time),
    (time))

  FMIFUNC(fmiSetContinuousStates, setContinuousStates,
    (fmiComponent c, const fmiReal x[], size_t nx),
    (x, nx))

  FMIFUNC(fmiCompletedIntegratorStep, completedIntegratorStep,
    (fmiComponent c, fmiBoolean* callEventUpdate),
    (callEventUpdate))

  FMIFUNC(fmiSetReal, setDoubleValue,
    (fmiComponent c, const fmiValueReference vr[], size_t nvr, const fmiReal value[]),
    (vr, nvr, value))

  FMIFUNC(fmiSetInteger, setIntValue,
    (fmiComponent c, const fmiValueReference vr[], size_t nvr, const fmiInteger value[]),
    (vr, nvr, value))

  FMIFUNC(fmiSetBoolean, setBoolValue,
    (fmiComponent c, const fmiValueReference vr[], size_t nvr, const fmiBoolean value[]),
    (vr, nvr, value))

  FMIFUNC(fmiSetString, setStringValue,
    (fmiComponent c, const fmiValueReference vr[], size_t nvr, const fmiString value[]),
    (vr, nvr, value))

  FMIFUNC(fmiInitialize, initialize,
    (fmiComponent c, fmiBoolean toleranceControlled, fmiReal relativeTolerance, fmiEventInfo* eventInfo),
    (toleranceControlled, relativeTolerance, eventInfo))

  FMIFUNC(fmiGetDerivatives, getDerivatives,
    (fmiComponent c, fmiReal derivatives[], size_t nx),
    (derivatives, nx))

  FMIFUNC(fmiGetEventIndicators, getEventIndicators,
    (fmiComponent c, fmiReal eventIndicators[], size_t ni),
    (eventIndicators, ni))

  FMIFUNC(fmiGetReal, getDoubleValue,
    (fmiComponent c, const fmiValueReference vr[], size_t nvr, fmiReal value[]),
    (vr, nvr, value))

  FMIFUNC(fmiGetInteger, getIntValue,
    (fmiComponent c, const fmiValueReference vr[], size_t nvr, fmiInteger value[]),
    (vr, nvr, value))

  FMIFUNC(fmiGetBoolean, getBoolValue,
    (fmiComponent c, const fmiValueReference vr[], size_t nvr, fmiBoolean value[]),
    (vr, nvr, value))

  FMIFUNC(fmiGetString, getStringValue,
    (fmiComponent c, const fmiValueReference vr[], size_t nvr, fmiString value[]),
    (vr, nvr, value))

  FMIFUNC(fmiEventUpdate, eventUpdate,
    (fmiComponent c, fmiBoolean intermediateResults, fmiEventInfo* eventInfo),
    (intermediateResults, eventInfo))

  FMIFUNC(fmiGetContinuousStates, getContinuousStates,
    (fmiComponent c, fmiReal states[], size_t nx),
    (states, nx))

  FMIFUNC(fmiGetNominalContinuousStates, getNominalContinuousStates,
    (fmiComponent c, fmiReal x_nominal[], size_t nx),
    (x_nominal, nx))

  FMIFUNC(fmiGetStateValueReferences, getStateValueReferences,
    (fmiComponent c, fmiValueReference vrx[], size_t nx),
    (vrx, nx))

  FMIFUNC(fmiTerminate, terminate,
    (fmiComponent c),
    ())

}
