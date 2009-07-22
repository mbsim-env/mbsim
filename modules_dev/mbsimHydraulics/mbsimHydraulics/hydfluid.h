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

#ifndef _HYDFLUID_H_
#define _HYDFLUID_H_

namespace MBSim {
  /** \brief A general class for a non-temperature dependent hydraulic fluid.*/
  class HydFluid {
    protected:
      double rho;
      double E;
      double nu;
      double kappa;
      double T;

      double f1, f2;
    public:
      /*! pure Constructor*/
      HydFluid();
      /*! Constructor initialising the fluid data*/
      HydFluid(double rho_, double E_, double nu_, double kappa_);
      /*! Destructor*/
      virtual ~HydFluid() {}

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

