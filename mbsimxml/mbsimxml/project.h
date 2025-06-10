#ifndef _MBSIM_PROJECT_H_
#define _MBSIM_PROJECT_H_

#include <memory>
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/solver.h"

namespace MBSim {

  class DynamicSystemSolver;
  class Solver;

  class MBSimProject {
    public:
      MBSimProject(const std::string &name_="Project") : name(name_) { }
      void initializeUsingXML(xercesc::DOMElement *element); 
      MBSim::DynamicSystemSolver* getDynamicSystemSolver() { return dss.get(); }
      MBSim::Solver* getSolver() { return solver.get(); }
    private:
      std::string name;
      std::shared_ptr<DynamicSystemSolver> dss;
      std::shared_ptr<Solver> solver;
  };

  MBSimProject readFlatXMLFile(const std::string &fileName);

}

#endif


