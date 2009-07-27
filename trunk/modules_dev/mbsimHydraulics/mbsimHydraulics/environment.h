#ifndef _MBSIMHYDRAULICS_ENVIRONMENT_H_
#define _MBSIMHYDRAULICS_ENVIRONMENT_H_

#include "mbsimtinyxml/tinyxml-src/tinyxml.h"
#include "mbsim/environment.h"

namespace MBSim {

class HydraulicEnvironment : public Environment {
  protected:
    static HydraulicEnvironment *instance;
    HydraulicEnvironment() : Environment(), rho(0), E(0), nu(0), kappa(0), T(0), f1(0), f2(0) {}
    double rho;
    double E;
    double nu;
    double kappa;
    double T;
    double f1, f2;
  public:
    static HydraulicEnvironment *getInstance() { return instance?instance:(instance=new HydraulicEnvironment); }
    virtual void initializeUsingXML(TiXmlElement *e);

    /*! set the fluid compressibility*/
    void setE(double E_) {E=E_; }
    /*! set the fluid specific mass*/
    void setRho(double rho_) {rho=rho_; }
    /*! set the kinematic viscosity*/
    void setNu(double nu_) {nu=nu_; }
    /*! set the air kappa value*/
    void setKappa(double kappa_) {kappa=kappa_; }
    /*! set all properties*/
    void setProperties(double rho_, double E_, double nu_, double kappa_); 

    /*! get the fluid compressibility depending on node fracAir and node pressure according Walter and Ubbelohde*/
    virtual double getE(double p,double fracAir);
    /*! get the pure fluid compressibility*/
    virtual double getE0() {return E; }
    /*! get the fluid specific mass*/
    virtual double getRho() {return rho; }
    /*! get the fluid specific mass depending on temperature*/
    virtual double getRho(double T_);
    /*! get the kinemaitc viscosity*/
    virtual double getNu() {return nu; }
    /*! get the kinematic visocisty depending on temperature*/
    virtual double getNu(double T_);
    /*! get the dynamic viscosity*/
    virtual double getEta() {return nu*rho; }
    /*! get the dynamic viscosity depending on temperature*/
    virtual double getEta(double T_);
    /*! get the air kappa value*/
    virtual double getKappa() {return kappa; }
    /*! get the temperature of the fluid*/
    virtual double getTemperature();
};

}

#endif
