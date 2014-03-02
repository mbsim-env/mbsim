#ifndef _MBSIMFLATXML_H_
#define _MBSIMFLATXML_H_

using namespace std;

namespace MBSim {

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
