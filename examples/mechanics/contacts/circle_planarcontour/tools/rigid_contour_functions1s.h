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
 * Contact: markus.ms.schneider@gmail.com
 */

#ifndef RIGID_CONTOUR_FUNCTION1S_H
#define RIGID_CONTOUR_FUNCTION1S_H

#include <mbsim/functions/piecewise_polynom_function.h>
#include <mbsim/functions/function.h>

namespace MBSim {
  template<class Ret> class TabularFunction;
};

class FuncCrPC_PlanePolar : public MBSim::Function<fmatvec::Vec3(double)> {
  public:
    FuncCrPC_PlanePolar();
    ~FuncCrPC_PlanePolar();
  
    void setYZ(const fmatvec::Mat& YZ, int discretization=1, fmatvec::Vec rYZ=fmatvec::Vec(3,fmatvec::INIT, 0));

    void init(Element::InitStage stage, const InitConfigSet &config);

    fmatvec::Vec3 operator()(const double& alpha);
    fmatvec::Vec3 parDer(const double& alpha); // Tangente in C
    fmatvec::Vec3 parDerParDer(const double& alpha); // 2. Ableitung in C

  private:
    const fmatvec::Vec3 Cb;
    MBSim::PiecewisePolynomFunction<fmatvec::VecV(double)> * pp_r;

    double alphaSave, salphaSave, calphaSave, rSave, drdalphaSave, d2rdalpha2Save;
    
    void updateData(const double& alpha);
};


#endif
