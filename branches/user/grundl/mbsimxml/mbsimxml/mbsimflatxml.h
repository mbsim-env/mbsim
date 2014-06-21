#ifndef _MBSIMFLATXML_H_
#define _MBSIMFLATXML_H_

#include <string>
#include <sstream>

namespace MBSim {

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
  class Integrator;

  class MBSimXML {
    public:
      static int preInit(int argc, char *argv[], DynamicSystemSolver*& dss, Integrator*& integrator);
      static void initDynamicSystemSolver(int argc, char *argv[], DynamicSystemSolver*& dss);
      static void plotInitialState(Integrator*& integrator, DynamicSystemSolver*& dss);
      static void main(Integrator *&integrator, DynamicSystemSolver *&dss);
      static void postMain(int argc, char *argv[], Integrator *&integrator, DynamicSystemSolver*& dss);
  };

}

#endif
