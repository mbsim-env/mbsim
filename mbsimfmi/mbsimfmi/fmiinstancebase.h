#ifndef _MBSIMFMI_FMIINSTANCEBASE_H_
#define _MBSIMFMI_FMIINSTANCEBASE_H_

#include <exception>

extern "C" {
  #include <3rdparty/fmiModelFunctions.h>
}

namespace MBSimFMI {

  /*! A pure virtual MBSim FMI instance base class */
  class FMIInstanceBase {
    public:
      virtual ~FMIInstanceBase() {}
      virtual void logException(const std::exception &ex)=0;
      virtual void setDebugLogging           (fmiBoolean loggingOn)=0;
      virtual void setTime                   (fmiReal time_)=0;
      virtual void setContinuousStates       (const fmiReal x[], size_t nx)=0;
      virtual void completedIntegratorStep   (fmiBoolean* callEventUpdate)=0;

      // we cannot use templates for these here since these functions must be virtual
      virtual void setDoubleValue            (const fmiValueReference vr[], size_t nvr, const fmiReal value[])=0;
      virtual void setIntValue               (const fmiValueReference vr[], size_t nvr, const fmiInteger value[])=0;
      virtual void setBoolValue              (const fmiValueReference vr[], size_t nvr, const fmiBoolean value[])=0;
      virtual void setStringValue            (const fmiValueReference vr[], size_t nvr, const fmiString value[])=0;

      virtual void initialize                (fmiBoolean toleranceControlled, fmiReal relativeTolerance, fmiEventInfo* eventInfo)=0;
      virtual void getDerivatives            (fmiReal derivatives[], size_t nx)=0;
      virtual void getEventIndicators        (fmiReal eventIndicators[], size_t ni)=0;

      // we cannot use templates for these here since these functions must be virtual
      virtual void getDoubleValue            (const fmiValueReference vr[], size_t nvr, fmiReal value[])=0;
      virtual void getIntValue               (const fmiValueReference vr[], size_t nvr, fmiInteger value[])=0;
      virtual void getBoolValue              (const fmiValueReference vr[], size_t nvr, fmiBoolean value[])=0;
      virtual void getStringValue            (const fmiValueReference vr[], size_t nvr, fmiString value[])=0;

      virtual void eventUpdate               (fmiBoolean intermediateResults, fmiEventInfo* eventInfo)=0;
      virtual void getContinuousStates       (fmiReal states[], size_t nx)=0;
      virtual void getNominalContinuousStates(fmiReal x_nominal[], size_t nx)=0;
      virtual void getStateValueReferences   (fmiValueReference vrx[], size_t nx)=0;
      virtual void terminate                 ()=0;
  };

  extern "C"
  typedef std::shared_ptr<FMIInstanceBase> (*fmiInstanceCreatePtr)(fmiString instanceName_, fmiString GUID,
                                                                      fmiCallbackFunctions functions, fmiBoolean loggingOn);
  extern "C"
  std::shared_ptr<FMIInstanceBase> fmiInstanceCreate(fmiString instanceName_, fmiString GUID,
                                                       fmiCallbackFunctions functions, fmiBoolean loggingOn);

}

#endif
