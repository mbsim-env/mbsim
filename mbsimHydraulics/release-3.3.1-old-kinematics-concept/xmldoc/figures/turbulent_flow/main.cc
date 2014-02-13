#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/rigid_line.h"
#include "mbsimHydraulics/pressure_loss.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimHydraulics;

int main (int argc, char* argv[]) {

  double length=100e-3;
  double diameter=5e-3;

  RigidLine * rigidLine = new RigidLine("l");
  rigidLine->setLength(length);
  rigidLine->setDiameter(diameter);

  HydraulicEnvironment::getInstance()->setBasicBulkModulus(.820e6);
  HydraulicEnvironment::getInstance()->setConstantSpecificMass(850);
  HydraulicEnvironment::getInstance()->setWalterUbbelohdeKinematicViscosity(313.16, 55e-6, 373.16, 10e-6);
  HydraulicEnvironment::getInstance()->setKappa(1.4);
  HydraulicEnvironment::getInstance()->setEnvironmentPressure(1e5);

  for (double T=0; T<=100; T+= 25) {

    HydraulicEnvironment::getInstance()->setTemperature(273.16+T);
    HydraulicEnvironment::getInstance()->initializeFluidData();

    for (double rough=0; rough<=100e-6; rough+=25e-6) {

      TurbulentTubeFlowLinePressureLoss * pl = new TurbulentTubeFlowLinePressureLoss();
      pl->setReferenceDiameter(diameter);
      pl->setHydraulicDiameter(diameter);
      pl->setSurfaceRoughness(rough);

      cerr << "#1: Temperature [degC]" << endl;
      cerr << "#2: kinematic Viscosity nu [mm^2/s]" << endl;
      cerr << "#3: roughness / diameter [-]" << endl;
      cerr << "#4: Q [m^3/s]" << endl;
      cerr << "#5: Q [l/min]" << endl;
      cerr << "#6: Reynolds number [-]" << endl;
      cerr << "#7: lambda [-]" << endl;
      cerr << "#8: pressure loss [bar]" << endl;

      for (double Q=1/6e4; Q<=1000/6e4; Q*=1.01) {
        cerr << " " << T;
        cerr << " " << HydraulicEnvironment::getInstance()->getKinematicViscosity()*1e6;
        cerr << " " << rough/diameter;
        cerr << " " << Q;
        cerr << " " << Q*6e4;
        cerr << " " << (*pl)(Q, rigidLine)*1e-5;
        cerr << endl;
      }

    delete pl;
    }

  }





  return 0;
}

