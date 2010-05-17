/* Copyright (C) 2004-2009 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: schneidm@users.berlios.de
 */

#include "config.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/objectfactory.h"

using namespace std;

namespace MBSimHydraulics {

  HydraulicEnvironment *HydraulicEnvironment::instance=NULL;
      
  void HydraulicEnvironment::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"environmentPressure");
    setEnvironmentPressure(MBSim::Element::getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"specificMass");
    if (e->FirstChildElement()->ValueStr()==MBSIMHYDRAULICSNS"constantSpecificMass")
      setConstantSpecificMass(MBSim::Element::getDouble(e->FirstChildElement()));
    else if (e->FirstChildElement()->ValueStr()==MBSIMHYDRAULICSNS"volumeDependingOnTemperature")
      setVolumeDependingOnTemperature(
          MBSim::Element::getDouble(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"dVdT")), 
          MBSim::Element::getDouble(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"basicSpecificMass")), 
          MBSim::Element::getDouble(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"basicTemperature"))
          );
    else if (e->FirstChildElement()->ValueStr()==MBSIMHYDRAULICSNS"specificMassDependingOnTemperature")
      setSpecificMassDependingOnTemperature(
          MBSim::Element::getDouble(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"dRhodT")), 
          MBSim::Element::getDouble(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"basicSpecificMass")), 
          MBSim::Element::getDouble(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"basicTemperature"))
          );
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"kinematicViscosity");
    if (e->FirstChildElement()->ValueStr()==MBSIMHYDRAULICSNS"constantKinematicViscosity")
      setConstantKinematicViscosity(MBSim::Element::getDouble(e->FirstChildElement()));
    else if (e->FirstChildElement()->ValueStr()==MBSIMHYDRAULICSNS"walterUbbeohdeKinematicViscosity") {
      setWalterUbbelohdeKinematicViscosity(
          MBSim::Element::getDouble(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"temperature1")), 
          MBSim::Element::getDouble(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"kinematicViscosity1")), 
          MBSim::Element::getDouble(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"temperature2")), 
          MBSim::Element::getDouble(e->FirstChildElement()->FirstChildElement(MBSIMHYDRAULICSNS"kinematicViscosity2"))
          );
    }
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"basicBulkModulus");
    setBasicBulkModulus(MBSim::Element::getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"kappa");
    setKappa(MBSim::Element::getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"fluidTemperature");
    setTemperature(MBSim::Element::getDouble(e));
    initializeFluidData();
  }

  void HydraulicEnvironment::initializeFluidData() {
    rho=(this->*calcRho)(T);
    nu=(this->*calcNu)(T);
    cout << endl;
    cout << "===============================================" << endl;
    cout << "initializing hydraulic environment at T=" << T-273.16 << " [degC]" << endl;
    cout << "            with kinematic viscosity nu=" << nu*1e6 << " [mm^2/s]" << endl;
    cout << "                      specific mass rho=" << rho << " [kg/m^3]" << endl;
    cout << "                  dynamic viscosity eta=" << getDynamicViscosity()*1e3 << " [mPa*s]" << endl;
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

  void HydraulicEnvironment::setWalterUbbelohdeKinematicViscosity(double T1, double nu1, double T2_, double nu2) {
    Tm=T1;
    Wm=log10(log10(nu1*1e6+0.8));  //Umrechnung in cSt
    T2=T2_;
    double W2=log10(log10(nu2*1e6+0.8));
    m=(Wm-W2)/(log10(T2)-log10(Tm));
    calcNu = &HydraulicEnvironment::calcWalterUbbelohdeKinematicViscosity;
  }

  double HydraulicEnvironment::calcWalterUbbelohdeKinematicViscosity(double T) {
    double Tx=T;
    double Wx=m*(log10(Tm)-log10(Tx))+Wm;
    return (pow(10,pow(10,Wx))-0.8)*1e-6; //Umrechnung zu m^2/s
  }

}
