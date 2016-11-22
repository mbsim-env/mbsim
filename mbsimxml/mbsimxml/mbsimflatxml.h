#ifndef _MBSIMFLATXML_H_
#define _MBSIMFLATXML_H_

#include <string>
#include <sstream>
#include <set>
#include <boost/filesystem/fstream.hpp>
#include <mbxmlutilshelper/dom.h>

namespace MBSim {

  const MBXMLUtils::NamespaceURI MBSIMXML("http://www.mbsim-env.de/MBSimXML");

  //! A string buffer which prefixes every line.
  class PrefixedStringBuf : public std::stringbuf {
    public:
      //! Prefix each line with prefix_ and print to str_.
      PrefixedStringBuf(const std::string &prefix_, std::ostream &outstr_) :
        std::stringbuf(std::ios_base::out), prefix(prefix_), outstr(outstr_) {}
    protected:
      int sync();
      std::string prefix;
      std::ostream &outstr;
  };

  class DynamicSystemSolver;
  class Solver;

  class MBSimXML {
    public:
      static int preInit(std::vector<std::string> args, DynamicSystemSolver*& dss, Solver*& solver);
      static void initDynamicSystemSolver(const std::vector<std::string> &args, DynamicSystemSolver*& dss);
      static void plotInitialState(Solver*& solver, DynamicSystemSolver*& dss);
      static void postMain(const std::vector<std::string> &args, Solver *&solver, DynamicSystemSolver*& dss);

      //! Load all MBSim modules and return a list of all loaded module libraries.
      static std::set<boost::filesystem::path> loadModules(const std::set<boost::filesystem::path> &searchDirs={});//MISSING remove ={} if mbsimfmi supports searchDirs
  };

}

#endif
