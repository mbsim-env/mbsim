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

#ifndef _MBSIMHYDRAULICS_ENVIRONMENT_H_
#define _MBSIMHYDRAULICS_ENVIRONMENT_H_

#include "mbsimtinyxml/tinyxml-src/tinyxml.h"
#include "mbsim/environment.h"

namespace MBSimHydraulics {

  /*! HydraulicEnvironment */
  class HydraulicEnvironment : public MBSim::Environment {
    private:
      double rhoConstant, dVdT, rho0, T0, dRhodT;
      double (HydraulicEnvironment::*calcRho)(double T);
      double calcConstantSpecificMass(double T) {return rhoConstant; }
      double calcVolumeDependingOnTemperature(double T) {return rho0*(1./(1.+dVdT*(T-T0))); }
      double calcSpecificMassDependingOnTemperature(double T) {return rho0  + dRhodT*(T- T0 ); }

      double nuConstant, Tm, Wm, T2, m;
      double (HydraulicEnvironment::*calcNu)(double T);
      double calcConstantKinematicViscosity(double T) {return nuConstant; }
      double calcWalterUbbelohdeKinematicViscosity(double T);

      double T;

    protected:
      double E0, kappa, rho, nu;
      double pinf;
      static HydraulicEnvironment * instance;  
      HydraulicEnvironment() : Environment(), rhoConstant(0), dVdT(0), rho0(0), T0(0), dRhodT(0), nuConstant(0), Tm(0), Wm(0), T2(0), m(0), T(0), E0(0), kappa(0), rho(0), nu(0), pinf(0) {}

    public:
      static HydraulicEnvironment * getInstance() {return instance?instance:(instance=new HydraulicEnvironment); }
      virtual void initializeUsingXML(TiXmlElement *e);
      virtual void initializeFluidData();

      /*! set the basic (air-free) bulk modulus*/
      void setBasicBulkModulus(double E0_) {E0=E0_; }
      /*! set the air kappa value*/
      void setKappa(double kappa_) {kappa=kappa_; }

      /*! set the temerature of the fluid*/
      void setTemperature(double T_) {T=T_; }

      /*! set a constant fluid specific mass*/
      void setConstantSpecificMass(double rho_);
      /*! set volume change with temperature*/
      void setVolumeDependingOnTemperature(double dVdT_, double rho0_, double T0_);
      /*! set specific mass change with temperature*/
      void setSpecificMassDependingOnTemperature(double dRhodT_, double rho0_, double T0_);

      /*! set a constant kinematic viscosity */
      void setConstantKinematicViscosity(double nu_);
      /*! set the kinematic viscosity according Walter and Ubbelohde*/
      void setWalterUbbelohdeKinematicViscosity(double T1, double nu1, double T2, double nu2);

      /*! set the boundary pressure*/
      void setEnvironmentPressure(double pinf_) {pinf=pinf_; }
      /*! get the boundary pressure*/
      double getEnvironmentPressure() {return pinf; }

      /*! get the fluid specific mass*/
      double getSpecificMass() {return rho; }
      /*! get the fluid compressibility depending on node fracAir and node pressure according Walter and Ubbelohde*/
      double getBasicBulkModulus() {return E0; }
      /*! get the kinemaitc viscosity*/
      double getKinematicViscosity() {return nu; }
      /*! get the air kappa value*/
      double getKappa() {return kappa; }
      /*! get the dynamic viscosity*/
      double getDynamicViscosity() {return nu*rho; }
      /*! get the fluid temperature*/
      double getTemperature() {return T;}
  };

  /*! OilBulkModulus */
  class OilBulkModulus {
    public:
      OilBulkModulus(const std::string &name, double E0, double pinf, double kappa, double fracAir) {
        ownerName=name;
        factor[0]=E0*(1.+fracAir);
        factor[1]=pow(pinf, 1./kappa) * fracAir * E0 / kappa;
        factor[2]=-(1.+1./kappa);

        // HYSIM[0]=E0;
        // HYSIM[1]=fracAir*pinf;
        // HYSIM[2]=pow(pinf, 1./kappa) * fracAir * E0 / kappa;
        // HYSIM[3]=1./kappa;
      }
      double operator()(const double &p) {
        if(p<=0.1) {
          std::cout << "OilBulkModulus of \"" << ownerName << "\": pressure near zero! Continuing anyway, using p=0.1 Pa" << std::endl;
          return factor[0]/(1.+factor[1]*pow(.1, factor[2]));
        }
        else {
          // Umdruck zur Vorlesung
          // Grundlagen der Oelhydraulik
          // W.Backe
          // H.Murrenhoff
          // 10. Auflage 1994
          // Formel (3-11), S. 103
          const double EBacke= factor[0]/(1.+factor[1]*pow(p, factor[2]));

          // Formel nach HYSIM
          // const double EHysim= HYSIM[0]*pow(1.+HYSIM[1]/p, HYSIM[3]) / (1. + HYSIM[2]/pow(p, 1+HYSIM[3]));
          
          // std::cerr << " " << p << " " << EBacke << " " << EHysim << std::endl;
          
          return EBacke;
        }
      }
    private:
      std::string ownerName;
      double factor[3];
      double HYSIM[4];
  };

}

#endif