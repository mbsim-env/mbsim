#include "config.h"
#include <cstring>
#include "mbsimflatxml.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/dynamic_system_solver.h"
#include <boost/filesystem.hpp>
#include <mbxmlutilshelper/last_write_time.h>
#include <boost/program_options.hpp>

using namespace std;
using namespace MBSim;
namespace po = boost::program_options;

int main(int argc, char *argv[]) {
  try {

    Integrator *integrator;
    DynamicSystemSolver *dss;

      po::options_description desc("Usage: mbsimflatxml [--donotintegrate|--savestatevector|--stopafterfirststep]\n                    <mbsimprjfile>\n   or: mbsimflatxml --printNamespacePrefixMapping\n\nCopyright (C) 2004-2009 MBSim Development Team\nThis is free software; see the source for copying conditions. There is NO\nwarranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.\n\nLicensed under the GNU Lesser General Public License (LGPL)\n\nOptions");
      desc.add_options()
        ("donotintegrate", "Stop after the initialization stage, do not integrate")
        ("stopafterfirststep", "Stop after outputting the first step (usually at t=0). This generates a HDF5 output file with only one time serie")
        ("task", po::value<string>()->default_value("simulation"), "Define the simulation task (simulation, eigenanalysis, eigenmode, eigenmodes, eigenmotion)")
        ("equilibrium",  "Determine the equilibrium state of the system")
        ("autoupdate", "Auto update eigenanalysis")
        ("mode", po::value<int>(), "Eigenmode number")
        ("amplitude", po::value<double>(), "Amplitude")
        ("savefinalstatevector", "Save the state vector to the file \"statevector.asc\" after integration")
        ("printNamespacePrefixMapping", "Print the recommended mapping of XML namespaces to XML prefix")
        //           ("help,h", "Print help message")
        ;
      po::options_description hidden("Hidden options");
      hidden.add_options()
        ("projectfile", po::value<string>(), "input file")
        ;        

      po::positional_options_description p;
      p.add("projectfile", -1);

      po::options_description all;
      all.add(desc).add(hidden);

      po::options_description visible("Allowed options");
      visible.add(desc);

      po::variables_map vm;
      po::store(po::command_line_parser(argc, argv).
          options(all).positional(p).run(), vm);
      po::notify(vm);

      if(vm.count("projectfile")) {
        cout << "Project files is: " 
          << vm["projectfile"].as< string >() << endl;
      }
      else {
        cout << visible << endl;
        return 1;
      }
      if(vm.count("task")) {
        string task = vm["task"].as<string>();
        cout << "task is: " << task << endl;
        if(task=="eigenmode") {
          if (vm.count("mode")) {
            cout << "mode is: " << vm["mode"].as<int>() << endl;
          }
        }
        else if(vm.count("mode")) {
          cout << "error" <<endl;
          return 1;
        }
      }
      if(vm.count("donotintegrate")) {
        cout << "dot not integrate "<< endl;
      } 
      if(vm.count("savefinalstatevector")) {
        cout << "save final state vector"<< endl;
      } 
   
  
    if(MBSimXML::preInit(vm, dss, integrator)!=0) return 0; 
    MBSimXML::initDynamicSystemSolver(vm, dss);
  
    if(not(vm.count("donotintegrate"))) {
      if(vm.count("stopafterfirststep"))
//      if(stopAfterFirstStep)
        MBSimXML::plotInitialState(integrator, dss);
      else if(vm.count("task")) {
//      else if(eigenAnalysis)
        string task = vm["task"].as< string >();
        if(task=="simulation")
          MBSimXML::main(integrator, dss);
        else
          MBSimXML::eigenAnalysis(vm, integrator, dss);
      }
      else
        MBSimXML::main(integrator, dss);
      // Remove the following block if --lastframe works in OpenMBV.
      // If this is removed openmbv should be opened with the --lastframe option.
      // Currently we use this block if --stopafterfirststep is given to reload the XML/H5 file in OpenMBV again
      // after the first step has been written since this is not possible by the file locking mechanism in OpenMBVCppInterface.
      if(vm.count("stopafterfirststep")) {
//      if(stopAfterFirstStep) 
        // touch the OpenMBV files
        boost::myfilesystem::last_write_time((dss->getName()+".ombv.xml").c_str(), boost::posix_time::microsec_clock::universal_time());
        boost::myfilesystem::last_write_time((dss->getName()+".ombv.h5" ).c_str(), boost::posix_time::microsec_clock::universal_time());
      }
    }

    MBSimXML::postMain(vm, integrator, dss);
  }
  catch(const MBSimError &e) {
    cerr<<e.what()<<endl;
    return 1;
  }
  catch(const H5::Exception &e) {
    cerr<<"HDF5 exception: "<<e.getCDetailMsg()<<endl<<
          "function: "<<e.getCFuncName()<<endl;
    return 1;
  }
  catch(const exception &e) {
    cerr<<"Exception: "<<e.what()<<endl;
    return 1;
  }
  catch(...) {
    cerr<<"Unknown exception"<<endl;
    return 1;
  }

  return 0;
}
