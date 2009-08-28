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
    string valuetype = (isetvalued==0) ? "setvalued" : "singlevalued";

    for (int iintegrator=0; iintegrator<6; iintegrator++) {
      string nameintegrator;
      if (iintegrator==0)
        nameintegrator = "EulerExplicit";
      else if (iintegrator==1)
        nameintegrator = "TimeStepping";
      else if (iintegrator==2)
        nameintegrator = "ThetaTimeStepping";
      else if (iintegrator==3)
        nameintegrator = "RADAU5";
      else if (iintegrator==4)
        nameintegrator = "DOPRI5";
      else if (iintegrator==5)
        nameintegrator = "LSODE";

      for (int isolver=0; isolver<4; isolver++) {
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
        dss->addGroup(new System("HS", setvalued));
        HydraulicEnvironment::getInstance()->setBasicBulkModulus(2e11);
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
        else if(isolver==3) {
          dss->setConstraintSolver(RootFinding);
          dss->setImpactSolver(RootFinding);
        }
        dss->setgdTol(1e-9);
        dss->init();

        double tEnd=1e-0;
        double dtPlot=1e-3;
        double stepSizeFactor=(setvalued?1.:1e-2);
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


