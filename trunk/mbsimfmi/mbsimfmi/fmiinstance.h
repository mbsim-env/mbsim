#ifndef _MBSIMFMI_FMIINSTANCE_H_
#define _MBSIMFMI_FMIINSTANCE_H_

#include <string>
#include <map>
#include <utils.h>
#include <fmatvec/atom.h>

// fmi function declarations must be included as extern C
extern "C" {
  #include <3rdparty/fmiModelFunctions.h>
}

#include <../general/fmi_variables.h>

namespace MBSim {
  class DynamicSystemSolver;
  class ExternGeneralizedIO;
}

namespace MBSimControl {
  class ExternSignalSource;
  class ExternSignalSink;
}

namespace MBXMLUtils {
  class SharedLibrary;
}

namespace MBSimFMI {

  //! This class is uses for ALL FMI variables during the preprocessing phase:
  //! between fmiInstantiateModel and fmiInitialize.
  //! The value of the variable is store by the class itself: as a member variable.
  template<class Datatype>
  class PreVariable : public Variable {
    public:
      PreVariable(Type type, const Datatype &defaultValue=Datatype());
      std::string getValueAsString() { throw std::runtime_error("Internal error: getValueAsString not allowed"); }
      const Datatype& getValue(const Datatype&);
      void setValue(const Datatype &v);
    protected:
      Datatype value;
  };

  /*! A MBSim FMI instance */
  class FMIInstance : virtual public fmatvec::Atom {
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

      template<typename CppDatatype, typename FMIDatatype>
      void setValue                  (const fmiValueReference vr[], size_t nvr, const FMIDatatype value[]);

      void initialize                (fmiBoolean toleranceControlled, fmiReal relativeTolerance, fmiEventInfo* eventInfo);
      void getDerivatives            (fmiReal derivatives[], size_t nx);
      void getEventIndicators        (fmiReal eventIndicators[], size_t ni);

      template<typename CppDatatype, typename FMIDatatype>
      void getValue                  (const fmiValueReference vr[], size_t nvr, FMIDatatype value[]);

      void eventUpdate               (fmiBoolean intermediateResults, fmiEventInfo* eventInfo);
      void getContinuousStates       (fmiReal states[], size_t nx);
      void getNominalContinuousStates(fmiReal x_nominal[], size_t nx);
      void getStateValueReferences   (fmiValueReference vrx[], size_t nx);
      void terminate                 ();

    private:

      void rethrowVR(size_t vr, const std::string &what="");

      // store FMI instanceName and logger
      std::string instanceName;
      fmiCallbackLogger logger;

      // stream buffers for MBSim objects
      LoggerBuffer infoBuffer;
      LoggerBuffer warnBuffer;
      LoggerBuffer debugBuffer;

      // XML parser
      boost::shared_ptr<MBXMLUtils::DOMParser> parser;

      // the system
      boost::shared_ptr<MBSim::DynamicSystemSolver> dss;

      // system time
      double time;
      // system state
      fmatvec::Vec z;
      // system state derivative
      fmatvec::Vec zd;
      // system stop vector
      fmatvec::Vec sv, svLast;
      // system stop vector indicator (0 = no shift in this index; 1 = shift in this index)
      fmatvec::VecInt jsv;

      // variables store for all predefined variables (variables not owned by dss)
      PredefinedVariables predefinedVar;

      // all FMI variables
      std::vector<boost::shared_ptr<Variable> > var;

      int completedStepCounter;
      double nextPlotTime;

#if MBSIMSRCFMI
      // shared library of a user supplied model
      boost::shared_ptr<MBXMLUtils::SharedLibrary> shLib;
#endif
  };

}

#endif
