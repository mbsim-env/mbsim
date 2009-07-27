#include "config.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/objectfactory.h"
#include "mbsim/utils/eps.h"

using namespace std;

namespace MBSim {

  HydraulicEnvironment *HydraulicEnvironment::instance=NULL;

  void HydraulicEnvironment::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"rho");
    double rho=atof(e->GetText());
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"E");
    double E=atof(e->GetText());
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"nu");
    double nu=atof(e->GetText());
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"kappa");
    double kappa=atof(e->GetText());
    setProperties(rho, E, nu, kappa);
  }

  void HydraulicEnvironment::setProperties(double rho_, double E_, double nu_, double kappa_) {
    rho=rho_;
    E = E_;
    nu = nu_;
    kappa=kappa_;

    f1=pow(1.e5, 1./kappa)*E/kappa;
    f2=-(1.+1./kappa);
  }

  double HydraulicEnvironment::getE(double p, double fracAir) {
    if(p<=0.1) {
      cout << "HydraulicEnvironment: pressure near zero! Continuing anyway, using p=0.1 Pa" << endl;
      p=0.1;
    }
    // Umdruck zur Vorlesung
    // Grundlagen der Oelhydraulik
    // W.Backe
    // H.Murrenhoff
    // 10. Auflage 1994
    // Formel (3-11), S. 103
    return E * ((fracAir<epsroot()) ?
      1. :
      //(1.+fracAir) / (1.+pow((1.e5/p),(1./kappa))*fracAir*E/(kappa*p)));
      // rechentechnisch optimiert:
      (1.+fracAir) / (1. + f1*fracAir*pow(p, f2)));
  }

  double HydraulicEnvironment::getRho(double T_) {
    cout << "Caution! No dependy on temperature considered." << endl;
    return getRho();
  }

  double HydraulicEnvironment::getNu(double T_) {
    cout << "Caution! No dependy on temperature considered." << endl;
    return getNu();
  }

  double HydraulicEnvironment::getEta(double T_) {
    cout << "Caution! No dependy on temperature considered." << endl;
    return getEta(); 
  }

  double HydraulicEnvironment::getTemperature() {
    if (T==0)
      cout << "Caution! No temperature set." << endl;
    return T;
  }

}
