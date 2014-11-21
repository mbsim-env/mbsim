#include <mbsim/dynamic_system_solver.h>
#include <mbsimControl/extern_signal_source.h>
#include <mbsimControl/extern_signal_sink.h>
#include <mbsim/extern_generalized_io.h>
#include <boost/algorithm/string/replace.hpp>

namespace {
  //! convert a MBSim path to a FMI structure variable name
  std::string pathToName(std::string path);
}

namespace MBSimFMI {

//! Type of the variable
enum Type {
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
    //! set FMI variable. The default implementation throws, if not overloaded.
    virtual void setRealValue(double v) {
      throw std::runtime_error("Setting this variable is not allowed or is not of type real.");
    }
    //! get FMI variable. The default implementation throws, if not overloaded.
    virtual double getRealValue() {
      throw std::runtime_error("This variable is not of type real.");
    }

};

//! create all FMI variables from sys
inline void createAllVariables(const MBSim::DynamicSystem *sys, std::vector<boost::shared_ptr<Variable> > &vrMap);



//! FMI variable for force of ExternGeneralizedIO
class ExternGeneralizedIOForce : public Variable {
  public:
    ExternGeneralizedIOForce(MBSim::ExternGeneralizedIO *io_) : io(io_) {}
    std::string getName() { return pathToName(io->getPath())+".h"; }
    std::string getDescription() { return "ExternGeneralizedIO force"; }
    Type getType() { return Input; }
    void setRealValue(double v) { io->setGeneralizedForce(v); }
    double getRealValue() { return io->getla()(0); }
  protected:
    MBSim::ExternGeneralizedIO *io;
};

//! FMI variable for position of ExternGeneralizedIO
class ExternGeneralizedIOPosition : public Variable {
  public:
    ExternGeneralizedIOPosition(MBSim::ExternGeneralizedIO *io_) : io(io_) {}
    std::string getName() { return pathToName(io->getPath())+".x"; }
    std::string getDescription() { return "ExternGeneralizedIO position"; }
    Type getType() { return Output; }
    double getRealValue() { return io->getGeneralizedPosition(); }
  protected:
    MBSim::ExternGeneralizedIO *io;
};

//! FMI variable for velocity of ExternGeneralizedIO
class ExternGeneralizedIOVelocity : public Variable {
  public:
    ExternGeneralizedIOVelocity(MBSim::ExternGeneralizedIO *io_) : io(io_) {}
    std::string getName() { return pathToName(io->getPath())+".v"; }
    std::string getDescription() { return "ExternGeneralizedIO velocity"; }
    Type getType() { return Output; }
    double getRealValue() { return io->getGeneralizedVelocity(); }
  protected:
    MBSim::ExternGeneralizedIO *io;
};

//! FMI variable for ExternSignalSource
class ExternSignalSource : public Variable {
  public:
    ExternSignalSource(MBSimControl::ExternSignalSource *sig_) : sig(sig_) {}
    std::string getName() { return pathToName(sig->getPath()); }
    std::string getDescription() { return "ExternSignalSource"; }
    Type getType() { return Input; }
    void setRealValue(double v) { sig->setSignal(fmatvec::VecV(1, fmatvec::INIT, v)); }
    double getRealValue() { return sig->getSignal()(0); }
  protected:
    MBSimControl::ExternSignalSource *sig;
};

//! FMI variable for ExternSignalSink
class ExternSignalSink : public Variable {
  public:
    ExternSignalSink(MBSimControl::ExternSignalSink *sig_) : sig(sig_) {}
    std::string getName() { return pathToName(sig->getPath()); }
    std::string getDescription() { return "ExternSignalSink"; }
    Type getType() { return Output; }
    double getRealValue() { return sig->getSignal()(0); }
  protected:
    MBSimControl::ExternSignalSink *sig;
};



void createAllVariables(const MBSim::DynamicSystem *sys, std::vector<boost::shared_ptr<Variable> > &vrMap) {
  // loop over all links
  const std::vector<MBSim::Link*> &l=sys->getLinks();
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

  // recursively walk the DynamicSystem sys
  const std::vector<MBSim::DynamicSystem*> &s=sys->getDynamicSystems();
  for(std::vector<MBSim::DynamicSystem*>::const_iterator it=s.begin(); it!=s.end(); ++it)
    createAllVariables(*it, vrMap);
}

}

namespace {

  std::string pathToName(std::string path) {
    boost::replace_all(path, "/", "."); // replace the MBSim separator / by the FMI seperator .
    return path.substr(1); // skip the starting spearotor character
  }

}
