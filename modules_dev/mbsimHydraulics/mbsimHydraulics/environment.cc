#include "config.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/objectfactory.h"

using namespace std;

namespace MBSim {

  HydraulicEnvironment *HydraulicEnvironment::instance=NULL;
      
  void HydraulicEnvironment::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"environmentPressure");
    setEnvironmentPressure(atof(e->GetText()));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"specificMass");
    if (e->FirstChildElement()->ValueStr()==MBSIMHYDRAULICSNS"constantSpecificMass")
      setConstantSpecificMass(atof(e->FirstChildElement()->GetText()));
    else if (e->FirstChildElement()->ValueStr()==MBSIMHYDRAULICSNS"volumeDependingOnTemperature")
      setVolumeDependingOnTemperature(
          atof(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"dVdT")->GetText()), 
          atof(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"basicSpecificMass")->GetText()), 
          atof(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"basicTemperature")->GetText())
          );
    else if (e->FirstChildElement()->ValueStr()==MBSIMHYDRAULICSNS"specificMassDependingOnTemperature")
      setSpecificMassDependingOnTemperature(
          atof(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"dRhodT")->GetText()), 
          atof(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"basicSpecificMass")->GetText()), 
          atof(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"basicTemperature")->GetText())
          );
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"kinematicViscosity");
    if (e->FirstChildElement()->ValueStr()==MBSIMHYDRAULICSNS"constantKinematicViscosity")
      setConstantKinematicViscosity(atof(e->FirstChildElement()->GetText()));
    else if (e->FirstChildElement()->ValueStr()==MBSIMHYDRAULICSNS"walterUbbeohdeKinematicViscosity") {
      setWalterUbbelohdeKinematicViscosity(
          atof(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"temperature1")->GetText()), 
          atof(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"kinematicViscosity1")->GetText()), 
          atof(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"temperature2")->GetText()), 
          atof(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"kinematicViscosity2")->GetText())
          );
    }
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"basicBulkModulus");
    setBasicBulkModulus(atof(e->GetText()));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"kappa");
    setKappa(atof(e->GetText()));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"fluidTemperature");
    setTemperature(atof(e->GetText()));
    initializeFluidData();
  }

  void HydraulicEnvironment::initializeFluidData() {
    rho=(this->*calcRho)(T);
    nu=(this->*calcNu)(T);
    cout << endl;
    cout << "===============================================" << endl;
    cout << "initializing hydraulic environment at T=" << T << " [degC]" << endl;
    cout << "            with kinematic viscosity nu=" << nu*1e6 << " [mm^2/s]" << endl;
    cout << "                      specific mass rho=" << rho << " [kg/m^3]" << endl;
    cout << "                  dynamic viscosity eta=" << getDynamicViscosity() << " [Pa*s]" << endl;
    cout << "                                  kappa=" << kappa << " [-]" << endl;
    cout << "                      boundary pressure=" << pinf*1e-5 << " [bar]" << endl;
    cout << "===============================================\n\n" << endl;
    cout << endl;
    assert(pinf>0);
    assert(rho>0);
    assert(E0>0);
    assert(kappa>0);
    assert(nu>0);
  }

  void HydraulicEnvironment::setConstantSpecificMass(double rho_) {
    rhoConstant=rho_;
    calcRho = &HydraulicEnvironment::calcConstantSpecificMass;
  }

  void HydraulicEnvironment::setVolumeDependingOnTemperature(double dVdT_, double rho0_, double T0_) {
    dVdT=dVdT_; 
    rho0=rho0_; 
    T0=T0_; 
    calcRho = &HydraulicEnvironment::calcVolumeDependingOnTemperature;
  }

  void HydraulicEnvironment::setSpecificMassDependingOnTemperature(double dRhodT_, double rho0_, double T0_) {
    dRhodT=dRhodT_; 
    rho0=rho0_; 
    T0=T0_; 
    calcRho = &HydraulicEnvironment::calcSpecificMassDependingOnTemperature;
  }

  void HydraulicEnvironment::setConstantKinematicViscosity(double nu_) {
    nuConstant=nu_;
    calcNu = &HydraulicEnvironment::calcConstantKinematicViscosity;
  }

  void HydraulicEnvironment::setWalterUbbelohdeKinematicViscosity(double T1, double nu1, double T2, double nu2) {
    Tm=T1+273.16;
    Wm=log10(log10(nu1*1e6+0.8));  //Umrechnung in cSt
    T2=T2+273.16;
    double W2=log10(log10(nu2*1e6+0.8));
    m=(Wm-W2)/(log10(T2)-log10(Tm));
    calcNu = &HydraulicEnvironment::calcWalterUbbelohdeKinematicViscosity;
  }

  double HydraulicEnvironment::calcWalterUbbelohdeKinematicViscosity(double T) {
    double Tx=T+273.16;
    double Wx=m*(log10(Tm)-log10(Tx))+Wm;
    return (pow(10,pow(10,Wx))-0.8)*1e-6; //Umrechnung zu m^2/s
  }

}
