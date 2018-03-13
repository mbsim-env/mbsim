#include "config.h"
#include <cstring>
#include <boost/regex.hpp>
#include "mbsimflatxml.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/solver.h"
#include <boost/filesystem.hpp>
#include <boost/timer/timer.hpp>
#include <mbxmlutilshelper/last_write_time.h>

using namespace std;
using namespace MBSim;

int main(int argc, char *argv[]) {
  try {
    DynamicSystemSolver::installSignalHandler();

    vector<string> args;
    for(int i=1; i<argc; ++i)
      args.emplace_back(argv[i]);

    // defaults for --stdout and --stderr
    if(find(args.begin(), args.end(), "--stdout")==args.end() &&
       find(args.begin(), args.end(), "--stderr")==args.end()) {
      args.push_back("--stdout"); args.push_back("info~Info: ~");
      args.push_back("--stderr"); args.push_back("warn~Warn: ~");
      args.push_back("--stderr"); args.push_back("error~~");
      args.push_back("--stderr"); args.push_back("depr~Depr: ~");
      args.push_back("--stdout"); args.push_back("status~~\r");
    }

    // disable all streams
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Info      , std::make_shared<bool>(false));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Warn      , std::make_shared<bool>(false));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Debug     , std::make_shared<bool>(false));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Error     , std::make_shared<bool>(false));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Deprecated, std::make_shared<bool>(false));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Status    , std::make_shared<bool>(false));

    // handle --stdout and --stderr args
    vector<string>::iterator it;
    while((it=find_if(args.begin(), args.end(), [](const string &x){ return x=="--stdout" || x=="--stderr"; }))!=args.end()) {
      if(*it!="--stdout" && *it!="--stderr") continue;

      ostream &ostr=*it=="--stdout"?cout:cerr;
      auto itn=next(it);
      if(itn==args.end()) {
        cerr<<"Invalid argument"<<endl;
        return 1;
      }
      fmatvec::Atom::MsgType msgType;
      if     (itn->substr(0, 5)=="info~"  ) msgType=fmatvec::Atom::Info;
      else if(itn->substr(0, 5)=="warn~"  ) msgType=fmatvec::Atom::Warn;
      else if(itn->substr(0, 6)=="debug~" ) msgType=fmatvec::Atom::Debug;
      else if(itn->substr(0, 6)=="error~" ) msgType=fmatvec::Atom::Error;
      else if(itn->substr(0, 5)=="depr~"  ) msgType=fmatvec::Atom::Deprecated;
      else if(itn->substr(0, 7)=="status~") msgType=fmatvec::Atom::Status;
      else throw runtime_error("Unknown message stream.");
      static boost::regex re(".*~(.*)~(.*)", boost::regex::extended);
      boost::smatch m;
      if(!boost::regex_match(*itn, m, re)) {
        cerr<<"Invalid argument"<<endl;
        return 1;
      }
      fmatvec::Atom::setCurrentMessageStream(msgType, std::make_shared<bool>(true),
        std::make_shared<fmatvec::PrePostfixedStream>(m.str(1), m.str(2), ostr));

      args.erase(itn);
      args.erase(it);
    }

    Solver *solver;
    DynamicSystemSolver *dss;
  
    bool doNotIntegrate=false;
    if(find(args.begin(), args.end(), "--donotintegrate")!=args.end())
      doNotIntegrate=true;
    bool stopAfterFirstStep=false;
    if(find(args.begin(), args.end(), "--stopafterfirststep")!=args.end())
      stopAfterFirstStep=true;
  
    if(MBSimXML::preInit(args, dss, solver)!=0) return 0; 
    MBSimXML::initDynamicSystemSolver(args, dss);
  
    if(doNotIntegrate==false) {
      if(stopAfterFirstStep)
        MBSimXML::plotInitialState(solver, dss);
      else {
        boost::timer::cpu_timer t;
        t.start();
        solver->setSystem(dss);
        solver->execute();
        t.stop();
        fmatvec::Atom::msgStatic(fmatvec::Atom::Info)<<"Integration CPU times: "<<t.format()<<endl;
      }
      // Remove the following block if --lastframe works in OpenMBV.
      // If this is removed openmbv should be opened with the --lastframe option.
      // Currently we use this block if --stopafterfirststep is given to reload the XML/H5 file in OpenMBV again
      // after the first step has been written since this is not possible by the file locking mechanism in OpenMBVCppInterface.
      if(stopAfterFirstStep) {
        // touch the OpenMBV files
        boost::myfilesystem::last_write_time((dss->getName()+".ombv.xml").c_str(), boost::posix_time::microsec_clock::universal_time());
        boost::myfilesystem::last_write_time((dss->getName()+".ombv.h5" ).c_str(), boost::posix_time::microsec_clock::universal_time());
      }
    }

    MBSimXML::postMain(args, solver, dss);
  }
  catch(const exception &e) {
    fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<e.what()<<endl;
    return 1;
  }
  catch(...) {
    fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<"Unknown exception."<<endl;
    return 1;
  }

  return 0;
}
