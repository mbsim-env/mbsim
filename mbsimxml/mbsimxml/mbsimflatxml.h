#ifndef _MBSIMFLATXML_H_
#define _MBSIMFLATXML_H_

using namespace std;

namespace MBSim {

  class DynamicSystemSolver;
  class Integrator;

  class MBSimXML {
    public:
      static int preInitDynamicSystemSolver(int argc, char *argv[], DynamicSystemSolver*& dss);
      static int initDynamicSystemSolver(int argc, char *argv[], DynamicSystemSolver*& dss);
      static int initIntegrator(int argc, char *argv[], Integrator *&integrator);
      static int main(Integrator *&integrator, DynamicSystemSolver *&dss);
      static int postMain(int argc, char *argv[], Integrator *&integrator, DynamicSystemSolver*& dss);
  };

}

#endif
