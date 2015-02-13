// includes
#include "config.h"
#include <string>
#include <stdexcept>
#include <boost/shared_ptr.hpp>

// include the fmi header
extern "C" {
  #define MODEL_IDENTIFIER mbsim
  #include <fmiinstancebase.h> // this includes 3rdparty/fmiModelFunctions
}

// use namespaces
using namespace std;
using namespace MBSimFMI;

class SharedLibrary {//MFMF use real SharedLibrary from MBXMLUtils but with minimal dependencies
  public:
    SharedLibrary(const string &filename) {}
    void *getAddress(const string &symbolName) { return NULL; }
};

// define all FMI function as C functions
extern "C" {

  // global FMI function.
  const char* fmiGetModelTypesPlatform() {
    return "standard32";
  }

  // global FMI function.
  const char* fmiGetVersion() {
    return "1.0";
  }

  // FMI instantiate function: just calls the FMIInstanceBase ctor
  // Convert exceptions to FMI logger calls and return no instance.
  fmiComponent fmiInstantiateModel(fmiString instanceName_, fmiString GUID, fmiCallbackFunctions functions, fmiBoolean loggingOn) {
    try {
      SharedLibrary lib("MFMF"); // use sharedLibdir/resources/../libmbsimXXX_fmi.so.0; also copy mbsim.so to FMU in createFMU
      fmiInstanceCreatePtr fmiInstanceCreate=reinterpret_cast<fmiInstanceCreatePtr>(lib.getAddress("fmiInstanceCreate"));
      return new pair<SharedLibrary, boost::shared_ptr<FMIInstanceBase> >(lib,
        fmiInstanceCreate(instanceName_, GUID, functions, loggingOn));
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
  void fmiFreeModelInstance(fmiComponent c) {
    // must not throw
    delete static_cast<pair<SharedLibrary, boost::shared_ptr<FMIInstanceBase> >*>(c);
  }

  // All other FMI functions: just calls the corresponding member function in FMIInstanceBase
  // Convert exceptions to call of logError which itself passed these to the FMI logge and return with fmiError.
  #define FMIFUNC(fmiFuncName, instanceMemberName, Sig, sig) \
  fmiStatus fmiFuncName Sig { \
    boost::shared_ptr<FMIInstanceBase> instance=static_cast<pair<SharedLibrary, boost::shared_ptr<FMIInstanceBase> >*>(c)->second; \
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
