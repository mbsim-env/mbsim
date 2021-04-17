#ifndef _MBSIMFLATXML_H_
#define _MBSIMFLATXML_H_

#include <string>
#include <sstream>
#include <set>
#include <boost/filesystem/fstream.hpp>
#include <utility>
#include <mbxmlutilshelper/dom.h>

namespace MBSim {

  const MBXMLUtils::NamespaceURI MBSIMXML("http://www.mbsim-env.de/MBSimXML");

  class DynamicSystemSolver;
  class Solver;

  class MBSimXML {
    public:
      static int preInit(std::list<std::string> args, std::unique_ptr<DynamicSystemSolver>& dss, std::unique_ptr<Solver>& solver);
      static void initDynamicSystemSolver(const std::list<std::string> &args, const std::unique_ptr<DynamicSystemSolver>& dss);
      static void plotInitialState(const std::unique_ptr<Solver>& solver, const std::unique_ptr<DynamicSystemSolver>& dss);
      static void main(const std::unique_ptr<Solver>& solver, const std::unique_ptr<DynamicSystemSolver>& dss, bool doNotIntegrate, bool stopAfterFirstStep, bool savestatevector, bool savestatetable);

      //! Load all MBSim modules and return a list of all loaded module libraries.
      static std::set<boost::filesystem::path> loadModules(const std::set<boost::filesystem::path> &searchDirs={});//MISSING remove ={} if mbsimfmi supports searchDirs
  };

}

#endif
