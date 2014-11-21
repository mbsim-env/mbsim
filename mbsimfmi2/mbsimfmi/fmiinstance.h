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

#include <../general/fmi_variables_impl.h>

namespace MBSim {
  class DynamicSystemSolver;
  class ExternGeneralizedIO;
}

namespace MBSimControl {
  class ExternSignalSource;
  class ExternSignalSink;
}

namespace MBSimFMI {

  //! This class is uses for ALL FMI variables during the preprocessing phase:
  //! between fmiInstantiateModel and fmiInitialize.
  //! The value of the variable is store by the class itself: as a member variable.
  class PreVariable : public Variable {
    public:
      PreVariable(Type type_, char datatype_, const std::string &defaultValue);
      std::string getName() { throw std::runtime_error("Internal error: getName not allowed"); }
      std::string getDescription() { throw std::runtime_error("Internal error: getDescription not allowed"); }
      char getDatatype() { return datatype; }
      std::string getValueAsString() { throw std::runtime_error("Internal error: getValueAsString not allowed"); }
      Type getType() { return type; }
      const double& getValue(const double&);
      const int& getValue(const int&);
      const bool& getValue(const bool&);
      const std::string& getValue(const std::string&);
      void setValue(const double &v);
      void setValue(const int &v);
      void setValue(const bool &v);
      void setValue(const std::string &v);
    protected:
      Type type;
      char datatype;

      double doubleValue;
      int integerValue;
      bool booleanValue;
      std::string stringValue;
  };

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

      template<typename CppType, typename FMIType>
      void setValue                  (const fmiValueReference vr[], size_t nvr, const FMIType value[]);

      void initialize                (fmiBoolean toleranceControlled, fmiReal relativeTolerance, fmiEventInfo* eventInfo);
      void getDerivatives            (fmiReal derivatives[], size_t nx);
      void getEventIndicators        (fmiReal eventIndicators[], size_t ni);

      template<typename CppType, typename FMIType>
      void getValue                  (const fmiValueReference vr[], size_t nvr, FMIType value[]);

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

      // variables store for all "hard coded" variables (variables not owned by dss)
      HardCodedVariables hardCodedVar;

      // all FMI variables
      std::vector<boost::shared_ptr<Variable> > var;

      int completedStepCounter;
      double nextPlotEvent;
  };

}

#endif
