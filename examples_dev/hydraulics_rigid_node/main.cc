#include <mbsim/integrators/integrators.h>
#include <mbsim/dynamic_system_solver.h>

#include "mbsimHydraulics/environment.h"
#include "system.h"

using namespace std;
using namespace MBSim;
using namespace fmatvec;

int main (int argc, char* argv[]) {

  vector<double> integrationTime;
  vector<string> simulationName;


  //  for (int valveType=0; valveType<2; valveType++) {
  //    string valveTypeString = (valveType==0) ? "regularizedValves" : "cornerValves";
  int valveType=0;
  string valveTypeString = "";

  for (int nodeType=0; nodeType<2; nodeType++) {
    string nodeTypeString = (nodeType==0) ? "elasticNodes" : "rigidNodes";

    int iintegratorMax=((valveType==0)?6:2);
    if (iintegratorMax==2) // no ThetaTimeSteppingIntegrator
      iintegratorMax--;
    for (int iintegrator=0; iintegrator<iintegratorMax; iintegrator++) {
      if (iintegrator==1) // no ThetaTimeSteppingIntegrator
        iintegrator++;
      string nameintegrator;
      if (iintegrator==0)
        nameintegrator = "TimeStepping";
      else if (iintegrator==1)
        nameintegrator = "ThetaTimeStepping";
      else if (iintegrator==2)
        nameintegrator = "EulerExplicit";
      else if (iintegrator==3)
        nameintegrator = "RADAU5";
      else if (iintegrator==4)
        nameintegrator = "DOPRI5";
      else if (iintegrator==5)
        nameintegrator = "LSODE";

      int isolverMax=((iintegrator<=1)?4:1);
//        if ((valveType==1)&&(isolverMax==4))
//          isolverMax--; // TODO no Rootfinding in HydlinePressureloss yet
        for (int isolver=0; isolver<isolverMax; isolver++) {
          string namesolver;
          if(isolver==0)
            namesolver = "LinearEquations";
          else if (isolver==1)
            namesolver = "GaussSeidel";
          else if (isolver==2)
            namesolver = "FixedPointSingle";
          else if (isolver==3)
            namesolver = "RootFinding";

      simulationName.push_back(valveTypeString+"_"+nodeTypeString+"_"+nameintegrator+"_"+namesolver);
      DynamicSystemSolver * dss = new DynamicSystemSolver(simulationName.back());
      dss->addGroup(new System("HS", (nodeType==1)/*  * (valveType==1)*/));
      HydraulicEnvironment::getInstance()->setBasicBulkModulus(2e11);
      HydraulicEnvironment::getInstance()->setConstantSpecificMass(800);
      HydraulicEnvironment::getInstance()->setWalterUbbelohdeKinematicViscosity(40, 55e-6, 100, 10e-6);
      HydraulicEnvironment::getInstance()->setKappa(1.3);
      HydraulicEnvironment::getInstance()->setEnvironmentPressure(1e5);
      HydraulicEnvironment::getInstance()->setTemperature(50);
      HydraulicEnvironment::getInstance()->initializeFluidData();

      MBSimEnvironment::getInstance()->setAccelerationOfGravity("[0;-9.81;0]");

      if (isolver==0) {
        dss->setConstraintSolver(LinearEquations);
        dss->setImpactSolver(LinearEquations);
      }
      else if (isolver==1) {
        dss->setConstraintSolver(GaussSeidel);
        dss->setImpactSolver(GaussSeidel);
      }
      else if(isolver==2) {
        dss->setConstraintSolver(FixedPointSingle);
        dss->setImpactSolver(FixedPointSingle);
      }
      else if(isolver==3) {
        dss->setConstraintSolver(RootFinding);
        dss->setImpactSolver(RootFinding);
      }
      dss->setgdTol(1e-9);
      dss->init();

      double tEnd=1.;
      double dtPlot=1e-2;
      // if elasticNodes or regularizedValves
      double stepSizeFactor=(((nodeType==0)/*||(valveType==0)*/)?1.e-2:1.);
      clock_t startTime, endTime;
      cout << "\n" << simulationName.back() << endl;
      cerr << "\n" << simulationName.back() << endl;
      if (iintegrator==0) {
        TimeSteppingIntegrator in;
        in.setStepSize(stepSizeFactor*dtPlot);
        in.setStartTime(0);
        in.setEndTime(tEnd);
        in.setPlotStepSize(dtPlot);
        in.setOutput(true);
        startTime=clock();
        in.integrate(*dss);
        endTime=clock();
      }
      else if (iintegrator==1) {
        ThetaTimeSteppingIntegrator in;
        in.setStepSize(stepSizeFactor*dtPlot);
        in.setTheta(.5);
        in.setStartTime(0);
        in.setEndTime(tEnd);
        in.setPlotStepSize(dtPlot);
        in.setOutput(true);
        startTime=clock();
        in.integrate(*dss);
        endTime=clock();
      }
      else if (iintegrator==2) {
        EulerExplicitIntegrator in;
        in.setStepSize(stepSizeFactor*dtPlot);
        in.setStartTime(0);
        in.setEndTime(tEnd);
        in.setPlotStepSize(dtPlot);
        in.setOutput(true);
        startTime=clock();
        in.integrate(*dss);
        endTime=clock();
      }
      else if (iintegrator==3) {
        RADAU5Integrator in;
        in.setStartTime(0);
        in.setEndTime(tEnd);
        in.setPlotStepSize(dtPlot);
        in.setMaximalStepSize(dtPlot/2.);
        in.setOutput(true);
        startTime=clock();
        in.integrate(*dss);
        endTime=clock();
      }
      else if (iintegrator==4) {
        DOPRI5Integrator in;
        in.setStartTime(0);
        in.setEndTime(tEnd);
        in.setPlotStepSize(dtPlot);
        in.setMaximalStepSize(dtPlot/4.);
        in.setOutput(true);
        startTime=clock();
        in.integrate(*dss);
        endTime=clock();
      }
      else if (iintegrator==5) {
        LSODEIntegrator in;
        in.setStartTime(0);
        in.setEndTime(tEnd);
        in.setPlotStepSize(dtPlot);
        in.setMaximalStepSize(dtPlot/4.);
        in.setOutput(true);
        startTime=clock();
        in.integrate(*dss);
        endTime=clock();
      }
      integrationTime.push_back(double(endTime-startTime)/CLOCKS_PER_SEC);

      dss->closePlot();
      delete dss;
    }
  }
  //    }
}

cout << endl;
for (unsigned int i=0; i<integrationTime.size(); i++)
cout << "Integrator \"" << simulationName[i] << "\": Integration time = " << integrationTime[i] << " [s]." << endl;

return 0;
}


