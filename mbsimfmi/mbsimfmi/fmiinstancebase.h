#ifndef _MBSIMFMI_FMIINSTANCEBASE_H_
#define _MBSIMFMI_FMIINSTANCEBASE_H_

#include <exception>
#include <memory>
#include <mbxmlutilshelper/shared_library.h>

extern "C" {
  #include <3rdparty/fmiFunctions.h>
  #include <3rdparty/fmiModelFunctions.h>
}

namespace MBSimFMI {

  /*! A pure virtual MBSim FMI instance base class */
  class FMIInstanceBase {
    public:
      virtual ~FMIInstanceBase() = default;
      virtual void logException(const std::exception &ex)=0;
      virtual void setDebugLogging           (fmiBoolean loggingOn)=0;

      // we cannot use templates for these here since these functions must be virtual
      virtual void setDoubleValue            (const fmiValueReference vr[], size_t nvr, const fmiReal value[])=0;
      virtual void setIntValue               (const fmiValueReference vr[], size_t nvr, const fmiInteger value[])=0;
      virtual void setBoolValue              (const fmiValueReference vr[], size_t nvr, const fmiBoolean value[])=0;
      virtual void setStringValue            (const fmiValueReference vr[], size_t nvr, const fmiString value[])=0;

      // we cannot use templates for these here since these functions must be virtual
      virtual void getDoubleValue            (const fmiValueReference vr[], size_t nvr, fmiReal value[])=0;
      virtual void getIntValue               (const fmiValueReference vr[], size_t nvr, fmiInteger value[])=0;
      virtual void getBoolValue              (const fmiValueReference vr[], size_t nvr, fmiBoolean value[])=0;
      virtual void getStringValue            (const fmiValueReference vr[], size_t nvr, fmiString value[])=0;

      virtual void terminate                 ()=0;

      /* me special funcs */
      virtual void setTime                   (fmiReal time_)=0;
      virtual void setContinuousStates       (const fmiReal x[], size_t nx)=0;
      virtual void completedIntegratorStep   (fmiBoolean* callEventUpdate)=0;
      virtual void initialize_me             (fmiBoolean toleranceControlled, fmiReal relativeTolerance, fmiEventInfo* eventInfo)=0;
      virtual void getDerivatives            (fmiReal derivatives[], size_t nx)=0;
      virtual void getEventIndicators        (fmiReal eventIndicators[], size_t ni)=0;
      virtual void eventUpdate               (fmiBoolean intermediateResults, fmiEventInfo* eventInfo)=0;
      virtual void getContinuousStates       (fmiReal states[], size_t nx)=0;
      virtual void getNominalContinuousStates(fmiReal x_nominal[], size_t nx)=0;
      virtual void getStateValueReferences   (fmiValueReference vrx[], size_t nx)=0;

      /* cosim special funcs */
      virtual void initialize_cosim(fmiReal tStart, fmiBoolean StopTimeDefined, fmiReal tStop)=0;
      virtual void resetSlave()=0;
      virtual void setRealInputDerivatives(const fmiValueReference vr[], size_t nvr, const fmiInteger order[], const fmiReal value[])=0;
      virtual void getRealOutputDerivatives(const fmiValueReference vr[], size_t nvr, const fmiInteger order[], fmiReal value[])=0;
      virtual void cancelStep()=0;
      virtual void doStep(fmiReal currentCommunicationPoint, fmiReal communicationStepSize, fmiBoolean newStep)=0;
      virtual void getStatus(const fmiStatusKind s, fmiStatus* value)=0;
      virtual void getDoubleStatus(const fmiStatusKind s, fmiReal* value)=0;
      virtual void getIntStatus(const fmiStatusKind s, fmiInteger* value)=0;
      virtual void getBoolStatus(const fmiStatusKind s, fmiBoolean* value)=0;
      virtual void getStringStatus(const fmiStatusKind s, fmiString* value)=0;
  };

  extern "C"
  typedef std::shared_ptr<FMIInstanceBase> (*fmiInstanceCreatePtr)(bool cosim, fmiString instanceName_, fmiString GUID,
                                                                   fmiCallbackLogger logger, fmiBoolean loggingOn);
  extern "C"
  std::shared_ptr<FMIInstanceBase> fmiInstanceCreate(bool cosim, fmiString instanceName_, fmiString GUID,
                                                     fmiCallbackLogger logger, fmiBoolean loggingOn);

}

#endif
