#ifndef _MBSIMFMI_FMIINSTANCE_H_
#define _MBSIMFMI_FMIINSTANCE_H_

#include <string>
#include <map>
#include <fmatvec/atom.h>
#include "fmiinstancebase.h"

// define getFMUSharedLibPath() which returns the FMU shared library path = .../resources/local/[lib|bin]/<name>.[so|dll]
#define MBXMLUTILS_SHAREDLIBNAME FMU
#include <mbxmlutilshelper/getsharedlibpath.h>

// fmi function declarations must be included as extern C
extern "C" {
  #include <3rdparty/fmiFunctions.h>
  #include <3rdparty/fmiModelFunctions.h>
}

#include <../general/fmi_variables.h>

namespace MBSim {
  class DynamicSystemSolver;
}

namespace MBSimIntegrator {
  class Integrator;
}

namespace MBSimControl {
  class ExternSignalSource;
  class ExternSignalSink;
}

namespace MBSimFMI {

  /*! A MBSim FMI instance */
  class FMIInstance : public FMIInstanceBase, virtual public fmatvec::Atom {
    friend std::shared_ptr<FMIInstanceBase> fmiInstanceCreate(bool cosim, fmiString instanceName_, fmiString GUID,
                                                              fmiCallbackLogger logger, fmiBoolean loggingOn);
    public:
      //! dtor used in fmiFreeModelInstance
      ~FMIInstance() override;

      //! print exception using FMI logger
      void logException(const std::exception &ex) override;

      // Wrapper for all other FMI functions (except fmiInstantiateModel and fmiFreeModelInstance, see above)

      void setDebugLogging           (fmiBoolean loggingOn) override;

      // wrap the virtual none template functions to the corresponding template function
      void setDoubleValue(const fmiValueReference vr[], size_t nvr, const fmiReal value[]) override    { setValue<double>     (vr, nvr, value); }
      void setIntValue   (const fmiValueReference vr[], size_t nvr, const fmiInteger value[]) override { setValue<int>        (vr, nvr, value); }
      void setBoolValue  (const fmiValueReference vr[], size_t nvr, const fmiBoolean value[]) override { setValue<bool>       (vr, nvr, value); }
      void setStringValue(const fmiValueReference vr[], size_t nvr, const fmiString value[]) override  { setValue<std::string>(vr, nvr, value); }
      // used in fmiSetReal, fmiSetInteger, fmiSetBoolean and fmiSetString
      template<typename CppDatatype, typename FMIDatatype>
      void setValue      (const fmiValueReference vr[], size_t nvr, const FMIDatatype value[]);

      // wrap the virtual none template functions to the corresponding template function
      void getDoubleValue(const fmiValueReference vr[], size_t nvr, fmiReal value[]) override    { getValue<double>     (vr, nvr, value); }
      void getIntValue   (const fmiValueReference vr[], size_t nvr, fmiInteger value[]) override { getValue<int>        (vr, nvr, value); }
      void getBoolValue  (const fmiValueReference vr[], size_t nvr, fmiBoolean value[]) override { getValue<bool>       (vr, nvr, value); }
      void getStringValue(const fmiValueReference vr[], size_t nvr, fmiString value[]) override  { getValue<std::string>(vr, nvr, value); }
      // used in fmiGetReal, fmiGetInteger, fmiGetBoolean and fmiGetString
      template<typename CppDatatype, typename FMIDatatype>
      void getValue      (const fmiValueReference vr[], size_t nvr, FMIDatatype value[]);

      void terminate                 () override;

      /* me special functions */
      void setTime                   (fmiReal time_) override;
      void setContinuousStates       (const fmiReal x[], size_t nx) override;
      void completedIntegratorStep   (fmiBoolean* callEventUpdate) override;
      void initialize_me             (fmiBoolean toleranceControlled, fmiReal relativeTolerance, fmiEventInfo* eventInfo) override;
      void getDerivatives            (fmiReal derivatives[], size_t nx) override;
      void getEventIndicators        (fmiReal eventIndicators[], size_t ni) override;
      void eventUpdate               (fmiBoolean intermediateResults, fmiEventInfo* eventInfo) override;
      void getContinuousStates       (fmiReal states[], size_t nx) override;
      void getNominalContinuousStates(fmiReal x_nominal[], size_t nx) override;
      void getStateValueReferences   (fmiValueReference vrx[], size_t nx) override;

      /* cosim special functions */
      void initialize_cosim(fmiReal tStart, fmiBoolean StopTimeDefined, fmiReal tStop) override;
      void resetSlave() override;
      void setRealInputDerivatives(const fmiValueReference vr[], size_t nvr, const fmiInteger order[], const fmiReal value[]) override;
      void getRealOutputDerivatives(const fmiValueReference vr[], size_t nvr, const fmiInteger order[], fmiReal value[]) override;
      void cancelStep() override;
      void doStep(fmiReal currentCommunicationPoint, fmiReal communicationStepSize, fmiBoolean newStep) override;
      void getStatus(const fmiStatusKind s, fmiStatus* value) override;
      void getDoubleStatus(const fmiStatusKind s, fmiReal* value) override;
      void getIntStatus(const fmiStatusKind s, fmiInteger* value) override;
      void getBoolStatus(const fmiStatusKind s, fmiBoolean* value) override;
      void getStringStatus(const fmiStatusKind s, fmiString* value) override;

    private:
      //! ctor used in fmiInstantiateModel
      FMIInstance(bool cosim, fmiString instanceName_, fmiString GUID, fmiCallbackLogger logger_, fmiBoolean loggingOn);

      void rethrowVR(size_t vr, const std::exception &ex=std::runtime_error("Unknown exception."));

      bool cosim;

      // store FMI instanceName and logger
      std::string instanceName;
      fmiCallbackLogger logger;

      // XML parser (none validating)
      std::shared_ptr<MBXMLUtils::DOMParser> parser;

      // the system
      std::shared_ptr<MBSim::DynamicSystemSolver> dss;
      // the integrator
      std::shared_ptr<MBSimIntegrator::Integrator> integrator;

      // system time
      double timeStore; // do not use this variable, use time
      std::reference_wrapper<double> time;
      // system state
      fmatvec::Vec zStore; // do not use this variable, use z
      std::reference_wrapper<fmatvec::Vec> z;
      // system stop vector (0 = no shift in this index; 1 = shift in this index)
      fmatvec::Vec svLast;

      enum DriftCompensation {
        none,
        positionLevel,
        velocityLevel
      };
      DriftCompensation driftCompensation;

      // variables store for all predefined variables (variables not owned by dss)
      PredefinedParameterStruct predefinedParameterStruct;

      // all FMI variables
      std::vector<std::shared_ptr<Variable> > var;

      int completedStepCounter;
      double nextPlotTime;

      void addModelParametersAndCreateSystem(std::vector<std::shared_ptr<Variable> > &varSim);

      void initialize();
  };

}

#endif
