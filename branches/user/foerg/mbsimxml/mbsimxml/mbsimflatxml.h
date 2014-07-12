#ifndef _MBSIMFLATXML_H_
#define _MBSIMFLATXML_H_

#include <xercesc/util/XercesDefs.hpp>
#include <boost/shared_ptr.hpp>
#include <mbxmlutilshelper/dom.h>
#include <string>
#include <sstream>
#include <boost/program_options.hpp>

namespace XERCES_CPP_NAMESPACE {
  class DOMDocument;
}

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
      static int preInit(const boost::program_options::variables_map &vm, DynamicSystemSolver*& dss, Integrator*& integrator);
      static void initDynamicSystemSolver(const boost::program_options::variables_map &vm, DynamicSystemSolver*& dss);
      static void plotInitialState(Integrator* integrator, DynamicSystemSolver* dss);
      static void eigenAnalysis(const boost::program_options::variables_map &vm, Integrator*& integrator, DynamicSystemSolver*& dss);
      static void main(Integrator *integrator, DynamicSystemSolver *dss);
      static void postMain(const boost::program_options::variables_map &vm, Integrator *&integrator, DynamicSystemSolver*& dss);
  };

}

#endif
