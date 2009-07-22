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

#ifndef _OIL_H_
#define _OIL_H_

#include "mbsimHydraulics/hydfluid.h"

enum OilType {
  CastrolLongLife2TopUp_0W30,
  CastrolLongLife3TopUp_5W30
};

/** \brief Model for (temperature dependent) hydraulic oil*/
class Oil: public MBSim::HydFluid {
  protected:
    double eta;
    /** Parameter fuer Ubbelohde*/
    double m,Tm,Wm;
    bool ubbelohde;
    double rho0,T0,dVdT,dRhodT;
  public:
    /*! Constructor*/
    Oil();
    /*! Constructor and temperature initializer of a databased oiltype.
     * Choose one of type
     *   - CastrolLongLife2TopUp_0W30
     *   - CastrolLongLife3TopUp_5W30*/
    Oil(OilType _type, double _T);
    
    /*! set a databased oiltype*/
    void setType(OilType _type); 
    /*! set fluid parameters according Walter and Ubbelohde*/
    void setUbbelohde(double T1,double nu1, double T2,double nu2);
    /*! set volume change with temperature*/
    void setdVdT(double dVdT_,double rho0_,double T0_);
    /*! set specific mass change with temperature*/
    void setdRhodT(double dRhodT_,double rho0_,double T0_);
    
    /*! initialize the oil at specific temperature*/
    void init(double T_);
    
    /*! get the kinematic viscosity depending on temperature*/
    double getNu(double _T);
    /*! get the dynamic viscosity depending on temperature*/
    double getEta(double _T); 
    /*! get the specific fluid mass depending on temperature*/
    double getRho(double _T);
};

#endif

