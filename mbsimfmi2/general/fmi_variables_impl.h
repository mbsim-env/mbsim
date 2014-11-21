#ifndef _MBSIMFMI_FMI_VARIABLES_IMPL_H_
#define _MBSIMFMI_FMI_VARIABLES_IMPL_H_

#include <mbsim/dynamic_system_solver.h>
#include <mbsimControl/extern_signal_source.h>
#include <mbsimControl/extern_signal_sink.h>
#include <mbsim/extern_generalized_io.h>
#include <boost/algorithm/string/replace.hpp>
#include <boost/ref.hpp>
#include <boost/lexical_cast.hpp>

// local helper functions
namespace {
  //! convert a MBSim path to a FMI structured variable name
  std::string mbsimPathToFMIName(std::string path);

  //! get all links form sys, recursively
  void getAllLinks(const MBSim::DynamicSystem *sys, std::vector<MBSim::Link*> &link);
}

namespace MBSimFMI {

class Variable;

//! enumeration for PredefinedVariables::plotMode
enum PlotMode {
  EverynthCompletedStep = 1,
  SampleTime            = 2
};
//! Struct holding all predefined FMI variables (variables which are not part of the MBSim dss)
struct PredefinedVariables {
  std::string outputDir; // the MBSim output directory
  int plotMode;          // the MBSim plotting mode
  int plotEachNStep;     // plot at each n-th completed integrator step
  double plotStepSize;   // plot in equidistand time steps
};

//! create all FMI variables using a MBSim dss: predefined and dynamic ones from dss
inline void createAllVariables(const MBSim::DynamicSystemSolver *dss, std::vector<boost::shared_ptr<Variable> > &var,
                               PredefinedVariables &predefinedVar);

//! Type of the variable
enum Type {
  Parameter,
  Input,
  Output
};

//! Abstract base class for all FMI variables
// MISSING: create a exception wrapper with add the VR to all exceptoins throw in by this class
class Variable {
  public:
    typedef std::vector<std::pair<std::string, std::string> > EnumListCont;
    typedef boost::shared_ptr<EnumListCont> EnumList;

    //! ctor
    Variable(const std::string &name_, const std::string &desc_, Type type_, char datatypeChar_, const EnumList &enumList_=EnumList()) :
      name(name_), desc(desc_), type(type_), datatypeChar(datatypeChar_), enumList(enumList_) {}

    //! FMI name
    std::string getName() { return name; }
    //! FMI description
    std::string getDescription() { return desc; }
    //! Variable type
    Type getType() { return type; }
    //! FMI variable datatype
    char getDatatypeChar() { return datatypeChar; }
    //! Enumeration list.
    //! Only allowed for datatype int (='i'). A empty pointer means integer type a non empty pointer a enumeration type.
    const EnumList& getEnumerationList() { return enumList; }

    //! get the current value as a string (usable to add it to e.g. XML)
    virtual std::string getValueAsString()=0;

    // value setter

    //! set FMI variable of type real. The default implementation throws, if not overloaded.
    virtual void setValue(const double &v) {
      throw std::runtime_error("Setting this variable is not allowed or is not of type real.");
    }
    //! set FMI variable of type integer. The default implementation throws, if not overloaded.
    virtual void setValue(const int &v) {
      throw std::runtime_error("Setting this variable is not allowed or is not of type integer.");
    }
    //! set FMI variable of type boolean. The default implementation throws, if not overloaded.
    virtual void setValue(const bool &v) {
      throw std::runtime_error("Setting this variable is not allowed or is not of type boolean.");
    }
    //! set FMI variable of type string. The default implementation throws, if not overloaded.
    virtual void setValue(const std::string &v) {
      throw std::runtime_error("Setting this variable is not allowed or is not of type string.");
    }

    // value getter

    //! get FMI variable of type real. The default implementation throws, if not overloaded.
    //! Note: the argument just exists to be able to overload all type with the name function name.
    virtual const double& getValue(const double&) {
      throw std::runtime_error("This variable is not of type real.");
    }
    //! get FMI variable of type integer. The default implementation throws, if not overloaded.
    //! Note: the argument just exists to be able to overload all type with the name function name.
    virtual const int& getValue(const int&) {
      throw std::runtime_error("This variable is not of type integer.");
    }
    //! get FMI variable of type boolean. The default implementation throws, if not overloaded.
    //! Note: the argument just exists to be able to overload all type with the name function name.
    virtual const bool& getValue(const bool&) {
      throw std::runtime_error("This variable is not of type boolean.");
    }
    //! get FMI variable of type string. The default implementation throws, if not overloaded.
    //! Note: the argument just exists to be able to overload all type with the name function name.
    virtual const std::string& getValue(const std::string&) {
      throw std::runtime_error("This variable is not of type string.");
    }

  protected:
    std::string name, desc;
    Type type;
    char datatypeChar;
    EnumList enumList;
};



//! map c++ type to FMI datatype character
template<typename Datatype> struct MapDatatypeToFMIDatatypeChar;
template<> struct MapDatatypeToFMIDatatypeChar<double     > { static const char value='r'; };
template<> struct MapDatatypeToFMIDatatypeChar<int        > { static const char value='i'; };
template<> struct MapDatatypeToFMIDatatypeChar<bool       > { static const char value='b'; };
template<> struct MapDatatypeToFMIDatatypeChar<std::string> { static const char value='s'; };

//! A FMI parameter which stores the value in a externally provided reference.
//! This is used for predefined variables.
template<typename Datatype>
class PredefinedParameter : public Variable {
  public:
    PredefinedParameter(const std::string &name_, const std::string &desc_, Datatype &v, EnumList enumList=EnumList()) :
      Variable(name_, desc_, Parameter, MapDatatypeToFMIDatatypeChar<Datatype>::value, enumList), value(v) {}
    std::string getValueAsString() { return boost::lexical_cast<std::string>(value); }
    void setValue(const Datatype &v) { value=v; }
    const Datatype& getValue(const Datatype&) { return value; }
  protected:
    Datatype &value;
};

//! FMI input variable for the force of MBSim::ExternGeneralizedIO
class ExternGeneralizedIOForceInput : public Variable {
  public:
    ExternGeneralizedIOForceInput(MBSim::ExternGeneralizedIO *io_) : Variable(mbsimPathToFMIName(io_->getPath()),
      "ExternGeneralizedIO force", Input, 'r'), io(io_) {}
    std::string getValueAsString() { return boost::lexical_cast<std::string>(getValue(double())); }
    void setValue(const double &v) { io->setGeneralizedForce(v); }
    const double& getValue(const double&) { return io->getla()(0); }
  protected:
    MBSim::ExternGeneralizedIO *io;
};

//! FMI output variable for position of MBSim::ExternGeneralizedIO
class ExternGeneralizedIOPositionOutput : public Variable {
  public:
    ExternGeneralizedIOPositionOutput(MBSim::ExternGeneralizedIO *io_) : Variable(mbsimPathToFMIName(io_->getPath()),
      "ExternGeneralizedIO position", Output, 'r'), io(io_) {}
    std::string getValueAsString() { return boost::lexical_cast<std::string>(getValue(double())); }
    const double& getValue(const double&) { return io->getGeneralizedPosition(); }
  protected:
    MBSim::ExternGeneralizedIO *io;
};

//! FMI output variable for velocity of MBSim::ExternGeneralizedIO
class ExternGeneralizedIOVelocityOutput : public Variable {
  public:
    ExternGeneralizedIOVelocityOutput(MBSim::ExternGeneralizedIO *io_) : Variable(mbsimPathToFMIName(io_->getPath()),
      "ExternGeneralizedIO velocity", Output, 'r'), io(io_) {}
    std::string getValueAsString() { return boost::lexical_cast<std::string>(getValue(double())); }
    const double& getValue(const double&) { return io->getGeneralizedVelocity(); }
  protected:
    MBSim::ExternGeneralizedIO *io;
};

//! FMI input variable for MBSim::ExternSignalSource
class ExternSignalSourceInput : public Variable {
  public:
    ExternSignalSourceInput(MBSimControl::ExternSignalSource *sig_) : Variable(mbsimPathToFMIName(sig_->getPath()),
      "ExternSignalSource", Input, 'r'), sig(sig_) {}
    std::string getValueAsString() { return boost::lexical_cast<std::string>(getValue(double())); }
    void setValue(const double &v) { sig->setSignal(fmatvec::VecV(1, fmatvec::INIT, v)); }
    const double& getValue(const double&) { value=sig->getSignal()(0); return value; }
  protected:
    MBSimControl::ExternSignalSource *sig;
    double value; // MISSING: replace this varaible if getSignal returns a const reference!!!
};

//! FMI output variable for MBSim::ExternSignalSink
class ExternSignalSinkOutput : public Variable {
  public:
    ExternSignalSinkOutput(MBSimControl::ExternSignalSink *sig_) : Variable(mbsimPathToFMIName(sig_->getPath()),
      "ExternSignalSink", Output, 'r'), sig(sig_) {}
    std::string getValueAsString() { return boost::lexical_cast<std::string>(getValue(double())); }
    const double& getValue(const double&) { value=sig->getSignal()(0); return value; }
  protected:
    MBSimControl::ExternSignalSink *sig;
    double value; // MISSING: replace this varaible if getSignal returns a const reference!!!
};

void createAllVariables(const MBSim::DynamicSystemSolver *dss, std::vector<boost::shared_ptr<Variable> > &var,
                        PredefinedVariables &predefinedVar) {
  // create predefined parameters

  // output directory
  var.push_back(boost::make_shared<PredefinedParameter<std::string> >("Output directory",
    "MBSim output directory for all files: *.mbsim.h5, *.ombv.h5, *.ombv.xml, ...", boost::ref(predefinedVar.outputDir)));
  (*--var.end())->setValue(std::string(".")); // default value: current dir

  // plot mode
  // generate enumeration list
  Variable::EnumList plotModeList=boost::make_shared<Variable::EnumListCont>();
  plotModeList->push_back(std::make_pair<std::string, std::string>("Every n-th completed step", // PlotMode::EverynthCompletedStep
    "Plot each n-th completed integrator step, with n = 'Plot.each n-th step'."));
  plotModeList->push_back(std::make_pair<std::string, std::string>("Constant sample time", // PlotMode::SampleTime
    "Plot in equidistant sample times, with sample time = 'Plot.sample time'."));
  // add variable
  var.push_back(boost::make_shared<PredefinedParameter<int> >("Plot.mode",
    "Write to *.mbsim.h5 and *.ombv.h5 files at every ...", boost::ref(predefinedVar.plotMode), plotModeList));
  (*--var.end())->setValue(int(1)); // default value: every n-th completed integrator step

  // plot at each n-th integrator step
  var.push_back(boost::make_shared<PredefinedParameter<int> >("Plot.each n-th step",
    "... n-th completed integrator step", boost::ref(predefinedVar.plotEachNStep)));
  (*--var.end())->setValue(int(5)); // default value: every 5-th step

  // plot every dt
  var.push_back(boost::make_shared<PredefinedParameter<double> >("Plot.sample time",
    "... sample point with this sample time", boost::ref(predefinedVar.plotStepSize)));
  (*--var.end())->setValue(double(0.001)); // default value: every 1ms

  // ADD HERE MORE PREDEFINED PARAMETERS

  // create all input/output variables for links in the dss
  std::vector<MBSim::Link*> l;
  getAllLinks(dss, l);
  for(std::vector<MBSim::Link*>::const_iterator it=l.begin(); it!=l.end(); ++it) {
    // for ExternGeneralizedIO create three variables: force input, position output and velocity output
    MBSim::ExternGeneralizedIO *genIO=dynamic_cast<MBSim::ExternGeneralizedIO*>(*it);
    if(genIO) {
      var.push_back(boost::make_shared<ExternGeneralizedIOForceInput>(genIO));
      (*--var.end())->setValue(double(0.0)); // default value
      var.push_back(boost::make_shared<ExternGeneralizedIOPositionOutput>(genIO));
      var.push_back(boost::make_shared<ExternGeneralizedIOVelocityOutput>(genIO));
    }
    // for ExternSignalSource create one input variable
    MBSimControl::ExternSignalSource *sigSource=dynamic_cast<MBSimControl::ExternSignalSource*>(*it);
    if(sigSource) {
      var.push_back(boost::make_shared<ExternSignalSourceInput>(sigSource));
      (*--var.end())->setValue(double(0.0)); // default value
    }
    // for ExternSignalSink create one output variable
    MBSimControl::ExternSignalSink *sigSink=dynamic_cast<MBSimControl::ExternSignalSink*>(*it);
    if(sigSink) {
      var.push_back(boost::make_shared<ExternSignalSinkOutput>(sigSink));
    }
    // ADD HERE MORE MBSIM TYPES WHICH SHOULD BECOME FMI INPUTS/OUTPUTS
  }
}

}

namespace {

  std::string mbsimPathToFMIName(std::string path) {
    boost::replace_all(path, "/", "."); // replace the MBSim separator / by the FMI seperator .
    return path.substr(1); // skip the starting spearotor character
  }

  void getAllLinks(const MBSim::DynamicSystem *sys, std::vector<MBSim::Link*> &link) {
    // add all link of dss
    const std::vector<MBSim::Link*> &l=sys->getLinks();
    link.insert(link.end(), l.begin(), l.end());
  
    // call recursively for all DynamicSystem
    const std::vector<MBSim::DynamicSystem*> &s=sys->getDynamicSystems();
    for(std::vector<MBSim::DynamicSystem*>::const_iterator it=s.begin(); it!=s.end(); ++it)
      getAllLinks(*it, link);
  }

}

#endif
