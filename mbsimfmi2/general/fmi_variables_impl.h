#include <mbsim/dynamic_system_solver.h>
#include <mbsimControl/extern_signal_source.h>
#include <mbsimControl/extern_signal_sink.h>
#include <mbsim/extern_generalized_io.h>
#include <boost/algorithm/string/replace.hpp>
#include <boost/ref.hpp>

// local helper functions
namespace {
  //! convert a MBSim path to a FMI structured variable name
  std::string mbsimPathToFMIName(std::string path);

  //! get all links form sys, recursively
  void getAllLinks(const MBSim::DynamicSystem *sys, std::vector<MBSim::Link*> &link);
}

namespace MBSimFMI {

class Variable;

//! Struct holding all "hard coded" FMI variables (variables which are not part of the MBSim dss)
struct FMIParameters {
  std::string outputDir; // the MBSim output directory
};

//! create all FMI variables: "hard coded" and dynamic ones from dss
inline void createAllVariables(const MBSim::DynamicSystemSolver *dss, std::vector<boost::shared_ptr<Variable> > &vrMap,
                               FMIParameters &fmiPar);

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
    //! FMI name
    virtual std::string getName()=0;
    //! FMI description
    virtual std::string getDescription()=0;
    //! Variable type
    virtual Type getType()=0;
    //! FMI variable datatype
    virtual char getDatatype()=0;
    //! Default value as a string
    virtual std::string getDefault() {
      throw std::runtime_error("No default value available for this variable.");
    }
    //! set FMI variable. The default implementation throws, if not overloaded.
    virtual void setValue(double v) {
      throw std::runtime_error("Setting this variable is not allowed or is not of type real.");
    }
    //! get FMI variable. The default implementation throws, if not overloaded.
    virtual double getValue(double) {
      throw std::runtime_error("This variable is not of type real.");
    }
    //! set FMI variable. The default implementation throws, if not overloaded.
    virtual void setValue(int v) {
      throw std::runtime_error("Setting this variable is not allowed or is not of type integer.");
    }
    //! get FMI variable. The default implementation throws, if not overloaded.
    virtual int getValue(int) {
      throw std::runtime_error("This variable is not of type integer.");
    }
    //! set FMI variable. The default implementation throws, if not overloaded.
    virtual void setValue(bool v) {
      throw std::runtime_error("Setting this variable is not allowed or is not of type boolean.");
    }
    //! get FMI variable. The default implementation throws, if not overloaded.
    virtual bool getValue(bool) {
      throw std::runtime_error("This variable is not of type boolean.");
    }
    //! set FMI variable. The default implementation throws, if not overloaded.
    virtual void setValue(const std::string &v) {
      throw std::runtime_error("Setting this variable is not allowed or is not of type string.");
    }
    //! get FMI variable. The default implementation throws, if not overloaded.
    virtual const char* getValue(const char*) {
      throw std::runtime_error("This variable is not of type string.");
    }
};



//! Parameter without a relation to dss (not processed by mbsimxml)
class FMIStringParameter : public Variable {
  public:
    FMIStringParameter(const std::string &name_, const std::string &desc_, const std::string &defaultValue_, std::string &v) :
      name(name_), desc(desc_), defaultValue(defaultValue_), value(v) {}
    std::string getName() { return name; }
    std::string getDescription() { return desc; }
    Type getType() { return Parameter; }
    char getDatatype() { return 's'; }
    std::string getDefault() { return defaultValue; }
    void setValue(const std::string &v) { value=v; }
    const char* getValue(const char*) { return value.c_str(); }
  protected:
    std::string name, desc;
    std::string defaultValue;
    std::string &value;
};

//! FMI variable for force of ExternGeneralizedIO
class ExternGeneralizedIOForce : public Variable {
  public:
    ExternGeneralizedIOForce(MBSim::ExternGeneralizedIO *io_) : io(io_) {}
    std::string getName() { return mbsimPathToFMIName(io->getPath())+".h"; }
    std::string getDescription() { return "ExternGeneralizedIO force"; }
    Type getType() { return Input; }
    char getDatatype() { return 'r'; }
    std::string getDefault() { return "0"; }
    void setValue(double v) { io->setGeneralizedForce(v); }
    double getValue(double) { return io->getla()(0); }
  protected:
    MBSim::ExternGeneralizedIO *io;
};

//! FMI variable for position of ExternGeneralizedIO
class ExternGeneralizedIOPosition : public Variable {
  public:
    ExternGeneralizedIOPosition(MBSim::ExternGeneralizedIO *io_) : io(io_) {}
    std::string getName() { return mbsimPathToFMIName(io->getPath())+".x"; }
    std::string getDescription() { return "ExternGeneralizedIO position"; }
    Type getType() { return Output; }
    char getDatatype() { return 'r'; }
    double getValue(double) { return io->getGeneralizedPosition(); }
  protected:
    MBSim::ExternGeneralizedIO *io;
};

//! FMI variable for velocity of ExternGeneralizedIO
class ExternGeneralizedIOVelocity : public Variable {
  public:
    ExternGeneralizedIOVelocity(MBSim::ExternGeneralizedIO *io_) : io(io_) {}
    std::string getName() { return mbsimPathToFMIName(io->getPath())+".v"; }
    std::string getDescription() { return "ExternGeneralizedIO velocity"; }
    Type getType() { return Output; }
    char getDatatype() { return 'r'; }
    double getValue(double) { return io->getGeneralizedVelocity(); }
  protected:
    MBSim::ExternGeneralizedIO *io;
};

//! FMI variable for ExternSignalSource
class ExternSignalSource : public Variable {
  public:
    ExternSignalSource(MBSimControl::ExternSignalSource *sig_) : sig(sig_) {}
    std::string getName() { return mbsimPathToFMIName(sig->getPath()); }
    std::string getDescription() { return "ExternSignalSource"; }
    Type getType() { return Input; }
    char getDatatype() { return 'r'; }
    std::string getDefault() { return "0"; }
    void setValue(double v) { sig->setSignal(fmatvec::VecV(1, fmatvec::INIT, v)); }
    double getValue(double) { return sig->getSignal()(0); }
  protected:
    MBSimControl::ExternSignalSource *sig;
};

//! FMI variable for ExternSignalSink
class ExternSignalSink : public Variable {
  public:
    ExternSignalSink(MBSimControl::ExternSignalSink *sig_) : sig(sig_) {}
    std::string getName() { return mbsimPathToFMIName(sig->getPath()); }
    std::string getDescription() { return "ExternSignalSink"; }
    Type getType() { return Output; }
    char getDatatype() { return 'r'; }
    double getValue(double) { return sig->getSignal()(0); }
  protected:
    MBSimControl::ExternSignalSink *sig;
};

void createAllVariables(const MBSim::DynamicSystemSolver *dss, std::vector<boost::shared_ptr<Variable> > &vrMap, FMIParameters &fmiPar) {
  // create "hard coded" parameters
  vrMap.push_back(boost::make_shared<FMIStringParameter>("Output directory",
    "MBSim output directory for all files: *.mbsim.h5, *.ombv.h5, *.ombv.xml, ...",
    boost::filesystem::temp_directory_path().string(), boost::ref(fmiPar.outputDir)));

  // create all input/output variables for links
  std::vector<MBSim::Link*> l;
  getAllLinks(dss, l);
  for(std::vector<MBSim::Link*>::const_iterator it=l.begin(); it!=l.end(); ++it) {
    MBSim::ExternGeneralizedIO *genIO=dynamic_cast<MBSim::ExternGeneralizedIO*>(*it);
    if(genIO) {
      vrMap.push_back(boost::make_shared<ExternGeneralizedIOForce>(genIO));
      vrMap.push_back(boost::make_shared<ExternGeneralizedIOPosition>(genIO));
      vrMap.push_back(boost::make_shared<ExternGeneralizedIOVelocity>(genIO));
    }
    MBSimControl::ExternSignalSource *sigSource=dynamic_cast<MBSimControl::ExternSignalSource*>(*it);
    if(sigSource)
      vrMap.push_back(boost::make_shared<ExternSignalSource>(sigSource));
    MBSimControl::ExternSignalSink *sigSink=dynamic_cast<MBSimControl::ExternSignalSink*>(*it);
    if(sigSink)
      vrMap.push_back(boost::make_shared<ExternSignalSink>(sigSink));
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
