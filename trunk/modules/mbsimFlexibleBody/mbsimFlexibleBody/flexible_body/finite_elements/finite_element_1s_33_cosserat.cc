/* Copyright (C) 2004-2011 MBSim Development Team
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
 * Contact: thschindler@users.berlios.de
 */

#include<config.h>
#define FMATVEC_NO_INITIALIZATION
#define FMATVEC_NO_BOUNDS_CHECK
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_33_cosserat.h"
#include "mbsim/utils/eps.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FiniteElement1s33Cosserat::FiniteElement1s33Cosserat(double l0_, double rho_,double A_, double E_, double G_, double I1_, double I2_, double I0_, const Vec& g_) : l0(l0_), rho(rho_), A(A_), E(E_), G(G_), I1(I1_), I2(I2_), I0(I0_), g(g_), k10(0.), k20(0.), M(9,INIT,0.), h(9,INIT,0.), X(12,INIT,0.) {}

  FiniteElement1s33Cosserat::~FiniteElement1s33Cosserat() {}

  void FiniteElement1s33Cosserat::setCurlRadius(double R1,double R2) {
    if (fabs(R1)>epsroot()) k10 = 1./R1;
    if (fabs(R2)>epsroot()) k20 = 1./R2;
  }

  void FiniteElement1s33Cosserat::computeM(const Vec& qG) {
    double sbeta = sin(qG(4));
    double sgamma = sin(qG(5));
    double cbeta = cos(qG(4));
    double cgamma = cos(qG(5));

    M(0,0) = 0.5*rho*A*l0;
    M(1,1) = 0.5*rho*A*l0;
    M(2,2) = 0.5*rho*A*l0;
    M(3,3) = rho*l0*(I0*cbeta*cbeta*cgamma*cgamma + I1*cbeta*cbeta*sgamma*sgamma + I2*sbeta*sbeta);
    M(3,4) = rho*l0*(I0*cbeta*cgamma*sgamma - I1*cbeta*sgamma*cgamma);
    M(3,5) = rho*l0*I2*sbeta;
    M(4,4) = rho*l0*(I0*sgamma*sgamma + I1*cgamma*cgamma);
    M(5,5) = rho*l0*I2;
    M(6,6) = 0.5*rho*A*l0;
    M(7,7) = 0.5*rho*A*l0;
    M(8,8) = 0.5*rho*A*l0;
  }

  void FiniteElement1s33Cosserat::computeh(const Vec& qG, const Vec& qGt) {
    /* Cardan angles */
    double salpha = sin(qG(3));
    double sbeta = sin(qG(4));
    double sgamma = sin(qG(5));
    double calpha = cos(qG(3));
    double cbeta = cos(qG(4));
    double cgamma = cos(qG(5));

    Vec tangent(3);
    tangent(0) = cbeta*cgamma;
    tangent(1) = calpha*sgamma+salpha*sbeta*cgamma;
    tangent(2) = salpha*sgamma-calpha*sbeta*cgamma;

    SqrMat dtangentdphi(3);
    dtangentdphi(0,0) = 0.;
    dtangentdphi(0,1) = -sbeta*cgamma;
    dtangentdphi(0,2) = -cbeta*sgamma;

    dtangentdphi(1,0) = -salpha*sgamma+calpha*sbeta*cgamma; 
    dtangentdphi(1,1) = salpha*cbeta*cgamma;
    dtangentdphi(1,2) = calpha*cgamma-salpha*sbeta*sgamma;

    dtangentdphi(2,0) = calpha*sgamma+salpha*sbeta*cgamma;
    dtangentdphi(2,1) = -calpha*cbeta*cgamma;
    dtangentdphi(2,2) = salpha*cgamma+calpha*sbeta*sgamma;

    /* position difference */
    Vec deltax = qG(6,8)-qG(0,2);

    /* Differentiation of 'strain energy' with respect to qG
     * This is a stiff term; hence, we only consider elongation
     */
    Vec SE(9); 
    SE(0) = -tangent(0);
    SE(1) = -tangent(1);
    SE(2) = -tangent(2);
    SE(3,5) = deltax(0)*dtangentdphi.col(0) + deltax(1)*dtangentdphi.col(1) + deltax(2)*dtangentdphi.col(2);
    SE(6) = tangent(0);
    SE(7) = tangent(1);
    SE(8) = tangent(2);
    SE *= E*A*(tangent.T()*deltax-l0)/l0;

    /* Differentiation of 'bending and torsion energy' with respect to qG */
    //Vec BT1(6,INIT,0.);

    //Vec BT2(9,INIT,0.);

    Vec Vel = SE;

    /* Differentation of 'kinetic energy' with respect to phi */
    Vec dTRdphi(3,INIT,0.); // translational part is zero TODO

    /* Differentiation of 'kinetic energy' with respect to phit, phi */
    SqrMat dTRdphitphi(3,INIT,0.); // translational part is zero TODO

    /* Vec of generalized forces */
    h = -Vel;
    h(3,5) += dTRdphi-dTRdphitphi*qGt(3,5);
  }

  const Vec& FiniteElement1s33Cosserat::computeState(const Vec& qG, const Vec& qGt,double s) {
    X(0,2) = qG(0,2) + s*(qG(6,8)-qG(0,2))/l0; // position
    X(6,8) = qGt(0,2) + s*((qGt(6,8)-qGt(0,2))/l0); // velocity

    X(3,5) = qG(3,5); // angles TODO in angle element
    X(9,11) = qGt(3,5); // time differentiated angels TODO in angle element

    return X;
  }

}
