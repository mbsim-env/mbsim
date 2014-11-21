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
template<typename Datatype>
class Variable {
  public:
    //! FMI name
    virtual std::string getName()=0;
    //! FMI description
    virtual std::string getDescription()=0;
    //! Variable type
    virtual Type getType()=0;
    //! set FMI variable
    virtual void setValue(const Datatype &v) { throw std::runtime_error("Setting this variable is not allowed."); } // MISSING: try to extend this error message with the valueReferernce in the calling function
    //! get FMI variable
    virtual Datatype getValue()=0;
};

//! create all FMI variables from sys
inline void createAllVariables(const MBSim::DynamicSystem *sys, std::vector<boost::shared_ptr<Variable<double> > > &vrReal);



//! FMI variable for force of ExternGeneralizedIO
class ExternGeneralizedIOForce : public Variable<double> {
  public:
    ExternGeneralizedIOForce(MBSim::ExternGeneralizedIO *io_) : io(io_) {}
    std::string getName() { return pathToName(io->getPath())+".h"; }
    std::string getDescription() { return "ExternGeneralizedIO force"; }
    Type getType() { return Input; }
    void setValue(const double &v) { io->setGeneralizedForce(v); }
    double getValue() { return io->getla()(0); }
  protected:
    MBSim::ExternGeneralizedIO *io;
};

//! FMI variable for position of ExternGeneralizedIO
class ExternGeneralizedIOPosition : public Variable<double> {
  public:
    ExternGeneralizedIOPosition(MBSim::ExternGeneralizedIO *io_) : io(io_) {}
    std::string getName() { return pathToName(io->getPath())+".x"; }
    std::string getDescription() { return "ExternGeneralizedIO position"; }
    Type getType() { return Output; }
    double getValue() { return io->getGeneralizedPosition(); }
  protected:
    MBSim::ExternGeneralizedIO *io;
};

//! FMI variable for velocity of ExternGeneralizedIO
class ExternGeneralizedIOVelocity : public Variable<double> {
  public:
    ExternGeneralizedIOVelocity(MBSim::ExternGeneralizedIO *io_) : io(io_) {}
    std::string getName() { return pathToName(io->getPath())+".v"; }
    std::string getDescription() { return "ExternGeneralizedIO velocity"; }
    Type getType() { return Output; }
    double getValue() { return io->getGeneralizedVelocity(); }
  protected:
    MBSim::ExternGeneralizedIO *io;
};

//! FMI variable for ExternSignalSource
class ExternSignalSource : public Variable<double> {
  public:
    ExternSignalSource(MBSimControl::ExternSignalSource *sig_) : sig(sig_) {}
    std::string getName() { return pathToName(sig->getPath()); }
    std::string getDescription() { return "ExternSignalSource"; }
    Type getType() { return Input; }
    void setValue(const double &v) { sig->setSignal(fmatvec::VecV(1, fmatvec::INIT, v)); }
    double getValue() { return sig->getSignal()(0); }
  protected:
    MBSimControl::ExternSignalSource *sig;
};

//! FMI variable for ExternSignalSink
class ExternSignalSink : public Variable<double> {
  public:
    ExternSignalSink(MBSimControl::ExternSignalSink *sig_) : sig(sig_) {}
    std::string getName() { return pathToName(sig->getPath()); }
    std::string getDescription() { return "ExternSignalSink"; }
    Type getType() { return Output; }
    double getValue() { return sig->getSignal()(0); }
  protected:
    MBSimControl::ExternSignalSink *sig;
};



void createAllVariables(const MBSim::DynamicSystem *sys, std::vector<boost::shared_ptr<Variable<double> > > &vrReal) {
  // loop over all links
  const std::vector<MBSim::Link*> &l=sys->getLinks();
  for(std::vector<MBSim::Link*>::const_iterator it=l.begin(); it!=l.end(); ++it) {
    MBSim::ExternGeneralizedIO *genIO=dynamic_cast<MBSim::ExternGeneralizedIO*>(*it);
    if(genIO) {
      vrReal.push_back(boost::make_shared<ExternGeneralizedIOForce>(genIO));
      vrReal.push_back(boost::make_shared<ExternGeneralizedIOPosition>(genIO));
      vrReal.push_back(boost::make_shared<ExternGeneralizedIOVelocity>(genIO));
    }
    MBSimControl::ExternSignalSource *sigSource=dynamic_cast<MBSimControl::ExternSignalSource*>(*it);
    if(sigSource)
      vrReal.push_back(boost::make_shared<ExternSignalSource>(sigSource));
    MBSimControl::ExternSignalSink *sigSink=dynamic_cast<MBSimControl::ExternSignalSink*>(*it);
    if(sigSink)
      vrReal.push_back(boost::make_shared<ExternSignalSink>(sigSink));
  }

  // recursively walk the DynamicSystem sys
  const std::vector<MBSim::DynamicSystem*> &s=sys->getDynamicSystems();
  for(std::vector<MBSim::DynamicSystem*>::const_iterator it=s.begin(); it!=s.end(); ++it)
    createAllVariables(*it, vrReal);
}

}

namespace {

  std::string pathToName(std::string path) {
    boost::replace_all(path, "/", "."); // replace the MBSim separator / by the FMI seperator .
    return path.substr(1); // skip the starting spearotor character
  }

}
