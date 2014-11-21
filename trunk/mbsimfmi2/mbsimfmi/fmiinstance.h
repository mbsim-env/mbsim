#ifndef _MBSIMFMI_FMIINSTANCE_H_
#define _MBSIMFMI_FMIINSTANCE_H_

#include <string>
#include <map>
#include <utils.h>
#include <fmatvec/atom.h>
extern "C" {
  #include <extern/fmiModelFunctions.h>
}

namespace MBSim {
  class DynamicSystemSolver;
  class ExternGeneralizedIO;
}

namespace MBSimControl {
  class ExternSignalSource;
  class ExternSignalSink;
}

namespace MBSimFMI {

  /*! A MBSim FMI instance */
  class FMIInstance : public fmatvec::Atom {
    public:
      //! ctor used in fmiInstantiateModel
      FMIInstance(fmiString instanceName_, fmiString GUID, fmiCallbackFunctions functions, fmiBoolean loggingOn);

      //! ctor used in fmiFreeModelInstance
      ~FMIInstance();

      //! print exception using FMI logger
      void logException(const std::exception &ex);

      // Wrapper for all other FMI functions
      void setDebugLogging           (fmiBoolean loggingOn);
      void setTime                   (fmiReal time_);
      void setContinuousStates       (const fmiReal x[], size_t nx);
      void completedIntegratorStep   (fmiBoolean* callEventUpdate);
      void setReal                   (const fmiValueReference vr[], size_t nvr, const fmiReal value[]);
      void setInteger                (const fmiValueReference vr[], size_t nvr, const fmiInteger value[]);
      void setBoolean                (const fmiValueReference vr[], size_t nvr, const fmiBoolean value[]);
      void setString                 (const fmiValueReference vr[], size_t nvr, const fmiString value[]);
      void initialize                (fmiBoolean toleranceControlled, fmiReal relativeTolerance, fmiEventInfo* eventInfo);
      void getDerivatives            (fmiReal derivatives[], size_t nx);
      void getEventIndicators        (fmiReal eventIndicators[], size_t ni);
      void getReal                   (const fmiValueReference vr[], size_t nvr, fmiReal value[]);
      void getInteger                (const fmiValueReference vr[], size_t nvr, fmiInteger value[]);
      void getBoolean                (const fmiValueReference vr[], size_t nvr, fmiBoolean value[]);
      void getString                 (const fmiValueReference vr[], size_t nvr, fmiString value[]);
      void eventUpdate               (fmiBoolean intermediateResults, fmiEventInfo* eventInfo);
      void getContinuousStates       (fmiReal states[], size_t nx);
      void getNominalContinuousStates(fmiReal x_nominal[], size_t nx);
      void getStateValueReferences   (fmiValueReference vrx[], size_t nx);
      void terminate                 ();

    private:
      // store FMI instanceName and logger
      std::string instanceName;
      fmiCallbackLogger logger;

      // stream buffers for MBSim objects
      LoggerBuffer infoBuffer;
      LoggerBuffer warnBuffer;
      LoggerBuffer debugBuffer;

      // the system
      boost::shared_ptr<MBSim::DynamicSystemSolver> dss;

      // map valueReference to union
      enum Type {
        GeneralizedIO_h,
        GeneralizedIO_x,
        GeneralizedIO_v,
        SignalSource,
        SignalSink
      };
      union TypeValue {
        MBSim::ExternGeneralizedIO *generalizedIO;
        MBSimControl::ExternSignalSource *signalSource;
        MBSimControl::ExternSignalSink *signalSink;
      };
      std::vector<std::pair<Type, TypeValue> > vrUnion;

      // the system time
      double time;

      // store for valueReference before fmiInitialize is called
      std::map<size_t, double> vrReal;
  };

}

#endif
