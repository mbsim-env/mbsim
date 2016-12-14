#ifndef _MBSIMFMI_FMIINSTANCE_H_
#define _MBSIMFMI_FMIINSTANCE_H_

#include <string>
#include <map>
#include <utils.h>
#include <fmatvec/atom.h>
#include "fmiinstancebase.h"

// fmi function declarations must be included as extern C
extern "C" {
  #include <3rdparty/fmiModelFunctions.h>
}

#include <../general/fmi_variables.h>

namespace MBSim {
  class DynamicSystemSolver;
}

namespace MBSimControl {
  class ExternSignalSource;
  class ExternSignalSink;
}

namespace MBSimFMI {

  /*! A MBSim FMI instance */
  class FMIInstance : public FMIInstanceBase, virtual public fmatvec::Atom {
    friend std::shared_ptr<FMIInstanceBase> fmiInstanceCreate(fmiString instanceName_, fmiString GUID,
                                                              fmiCallbackFunctions functions, fmiBoolean loggingOn);
    public:
      //! dtor used in fmiFreeModelInstance
      ~FMIInstance();

      //! print exception using FMI logger
      void logException(const std::exception &ex);

      // Wrapper for all other FMI functions (except fmiInstantiateModel and fmiFreeModelInstance, see above)

      void setDebugLogging           (fmiBoolean loggingOn);
      void setTime                   (fmiReal time_);
      void setContinuousStates       (const fmiReal x[], size_t nx);
      void completedIntegratorStep   (fmiBoolean* callEventUpdate);

      // wrap the virtual none template functions to the corresponding template function
      void setDoubleValue(const fmiValueReference vr[], size_t nvr, const fmiReal value[])    { setValue<double>     (vr, nvr, value); }
      void setIntValue   (const fmiValueReference vr[], size_t nvr, const fmiInteger value[]) { setValue<int>        (vr, nvr, value); }
      void setBoolValue  (const fmiValueReference vr[], size_t nvr, const fmiBoolean value[]) { setValue<bool>       (vr, nvr, value); }
      void setStringValue(const fmiValueReference vr[], size_t nvr, const fmiString value[])  { setValue<std::string>(vr, nvr, value); }
      // used in fmiSetReal, fmiSetInteger, fmiSetBoolean and fmiSetString
      template<typename CppDatatype, typename FMIDatatype>
      void setValue      (const fmiValueReference vr[], size_t nvr, const FMIDatatype value[]);

      void initialize                (fmiBoolean toleranceControlled, fmiReal relativeTolerance, fmiEventInfo* eventInfo);
      void getDerivatives            (fmiReal derivatives[], size_t nx);
      void getEventIndicators        (fmiReal eventIndicators[], size_t ni);

      // wrap the virtual none template functions to the corresponding template function
      void getDoubleValue(const fmiValueReference vr[], size_t nvr, fmiReal value[])    { getValue<double>     (vr, nvr, value); }
      void getIntValue   (const fmiValueReference vr[], size_t nvr, fmiInteger value[]) { getValue<int>        (vr, nvr, value); }
      void getBoolValue  (const fmiValueReference vr[], size_t nvr, fmiBoolean value[]) { getValue<bool>       (vr, nvr, value); }
      void getStringValue(const fmiValueReference vr[], size_t nvr, fmiString value[])  { getValue<std::string>(vr, nvr, value); }
      // used in fmiGetReal, fmiGetInteger, fmiGetBoolean and fmiGetString
      template<typename CppDatatype, typename FMIDatatype>
      void getValue      (const fmiValueReference vr[], size_t nvr, FMIDatatype value[]);

      void eventUpdate               (fmiBoolean intermediateResults, fmiEventInfo* eventInfo);
      void getContinuousStates       (fmiReal states[], size_t nx);
      void getNominalContinuousStates(fmiReal x_nominal[], size_t nx);
      void getStateValueReferences   (fmiValueReference vrx[], size_t nx);
      void terminate                 ();

    private:
      //! ctor used in fmiInstantiateModel
      FMIInstance(fmiString instanceName_, fmiString GUID, fmiCallbackFunctions functions, fmiBoolean loggingOn);

      void rethrowVR(size_t vr, const std::exception &ex=std::runtime_error("Unknown exception."));

      // store FMI instanceName and logger
      std::string instanceName;
      fmiCallbackLogger logger;

      // stream buffers for MBSim objects
      LoggerBuffer infoBuffer;
      LoggerBuffer warnBuffer;
      LoggerBuffer debugBuffer;

      // XML parser (none validating)
      std::shared_ptr<MBXMLUtils::DOMParser> parser;

      // the system
      std::shared_ptr<MBSim::DynamicSystemSolver> dss;

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

      void addModelParametersAndCreateDSS(std::vector<std::shared_ptr<Variable> > &varSim);
  };

}

#endif
