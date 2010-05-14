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

#ifndef RIGID_CONTOUR_FUNCTION1S_H
#define RIGID_CONTOUR_FUNCTION1S_H

#include <mbsim/utils/ppolynom.h>
#include <mbsim/utils/contour_functions.h>

namespace MBSim {
  class TabularFunction1_VS;
};

class FuncCrPC : public MBSim::ContourFunction1s {
  public:
    FuncCrPC();
    
    void setYZ(const fmatvec::Mat& YZ, int discretization=1, fmatvec::Vec rYZ=fmatvec::Vec(3, fmatvec::INIT, 0));

    // void init(const double& alpha);
    void enableTabularFit(double tabularFitLength);
    
    fmatvec::Vec operator()(const double& alpha, const void * =NULL) {return (this->*operator_)(alpha); }
    fmatvec::Vec diff1(const double& alpha); // Tangente in C
    fmatvec::Vec diff2(const double& alpha); // 2. Ableitung in C
    fmatvec::Vec computeT(const double& alpha) {return (this->*computeT_)(alpha); }
    fmatvec::Vec computeB(const double& alpha) {return (this->*computeB_)(alpha); }
    fmatvec::Vec computeN(const double& alpha) {return (this->*computeN_)(alpha); }
    double computeCurvature(const double& alpha) {return (this->*computeCurvature_)(alpha); }

    virtual void initializeUsingXML(TiXmlElement * element);
  private:
    fmatvec::Vec Cb;
    MBSim::PPolynom pp_y;
    MBSim::PPolynom pp_z;
    MBSim::TabularFunction1_VS * tab_operator;
    MBSim::TabularFunction1_VS * tab_T;
    MBSim::TabularFunction1_VS * tab_B;
    MBSim::TabularFunction1_VS * tab_N;
    MBSim::TabularFunction1_VS * tab_curvature;

    fmatvec::Vec (FuncCrPC::*operator_)(const double& alpha);
    fmatvec::Vec operatorPPolynom(const double& alpha);
    fmatvec::Vec operatorTabular(const double& alpha);
    fmatvec::Vec (FuncCrPC::*computeT_)(const double& alpha);
    fmatvec::Vec computeTPPolynom(const double& alpha);
    fmatvec::Vec computeTTabular(const double& alpha);
    fmatvec::Vec (FuncCrPC::*computeN_)(const double& alpha);
    fmatvec::Vec computeNPPolynom(const double& alpha);
    fmatvec::Vec computeNTabular(const double& alpha);
    fmatvec::Vec (FuncCrPC::*computeB_)(const double& alpha);
    fmatvec::Vec computeBPPolynom(const double& alpha);
    fmatvec::Vec computeBTabular(const double& alpha);
    double (FuncCrPC::*computeCurvature_)(const double& alpha);
    double computeCurvaturePPolynom(const double& alpha);
    double computeCurvatureTabular(const double& alpha);
 
    double calculateLocalAlpha(const double& alpha);
};  

#endif
