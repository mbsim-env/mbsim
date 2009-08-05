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

  for (int isetvalued=0; isetvalued<2; isetvalued++) {

    bool setvalued = (isetvalued==0);
    bool unilateral; 
    string valuetype = (isetvalued==0) ? "setvalued" : "singlevalued";

    for (int iintegrator=0; iintegrator<6; iintegrator++) {
      if (iintegrator==2)
        iintegrator++;
      string nameintegrator;
      switch (iintegrator) {
        case 0: 
          nameintegrator = "EulerExplicit";
          unilateral = false;
          break;
        case 1:
          nameintegrator = "TimeStepping";
          unilateral = true;
          break;
        case 2: //TODO
          nameintegrator = "ThetaTimeStepping";
          unilateral = true;
          break;
        case 3:
          nameintegrator = "RADAU5";
          unilateral = false;
          break;
        case 4:
          nameintegrator = "DOPRI5";
          unilateral = false;
          break;
        case 5:
          nameintegrator = "LSODE";
          unilateral = false;
          break;
      }

      for (int isolver=0; isolver<3; isolver++) {
        string namesolver;
        if(isolver==0)
          namesolver = "LinearEquations";
        else if (isolver==1)
          namesolver = "GaussSeidel";
        else if (isolver==2)
          namesolver = "FixedPointSingle";
        else if (isolver==3)
          namesolver = "RootFinding";

        simulationName.push_back(nameintegrator+"_"+valuetype+"_"+namesolver);
        DynamicSystemSolver * dss = new DynamicSystemSolver(simulationName.back());
        dss->addDynamicSystem(new System("HS", setvalued, unilateral));
        HydraulicEnvironment::getInstance()->setBasicBulkModulus(2e8);
        HydraulicEnvironment::getInstance()->setConstantSpecificMass(800);
        HydraulicEnvironment::getInstance()->setConstantKinematicViscosity(12e-6);
        HydraulicEnvironment::getInstance()->setEnvironmentPressure(1e5);
        HydraulicEnvironment::getInstance()->setKappa(1.3);
        HydraulicEnvironment::getInstance()->setTemperature(50);
        HydraulicEnvironment::getInstance()->initializeFluidData();


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
        else if(isolver==3) { // TODO
          dss->setConstraintSolver(RootFinding);
          dss->setImpactSolver(RootFinding);
        }
        dss->setgdTol(1e-9);
        dss->init();
        cout << "Use Integrator \"" << simulationName.back()  << "." << endl;

        double tEnd=0.5;
        double dtPlot=1e-3;
        double stepSizeFactor=(setvalued?.1:.001);
        clock_t startTime, endTime;
        if (iintegrator==0) {
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
        else if (iintegrator==1) {
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
        else if (iintegrator==2) {
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
        else if (iintegrator==3) {
          RADAU5Integrator in;
          in.setStartTime(0);
          in.setEndTime(tEnd);
          in.setPlotStepSize(dtPlot);
          in.setMaximalStepSize(dtPlot);
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
          in.setMaximalStepSize(dtPlot);
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
          in.setMaximalStepSize(dtPlot);
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
  }

  cout << endl;
  for (unsigned int i=0; i<integrationTime.size(); i++)
    cout << "Integrator \"" << simulationName[i] << "\": Integration time = " << integrationTime[i] << " [s]." << endl;

  return 0;
}


