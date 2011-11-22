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
 * Contact: thorsten.schindler@mytum.de
 */

#include<config.h>
#define FMATVEC_NO_INITIALIZATION
#define FMATVEC_NO_BOUNDS_CHECK
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_33_cosserat_rotation.h"
#include "mbsimFlexibleBody/utils/cardan.h"
#include "mbsim/utils/eps.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FiniteElement1s33CosseratRotation::FiniteElement1s33CosseratRotation(double l0_, double E_, double G_, double I1_, double I2_, double I0_, CardanPtr ag_) : l0(l0_), E(E_), G(G_), I1(I1_), I2(I2_), I0(I0_), k10(0.), k20(0.), h(9,INIT,0.), ag(ag_) {}

  FiniteElement1s33CosseratRotation::~FiniteElement1s33CosseratRotation() {}

  void FiniteElement1s33CosseratRotation::setCurlRadius(double R1,double R2) {
    if (fabs(R1)>epsroot()) k10 = 1./R1;
    if (fabs(R2)>epsroot()) k20 = 1./R2;
  }

  void FiniteElement1s33CosseratRotation::computeh(const Vec& qG, const Vec& qGt) {
    /* angles */
    Vec phi = (qG(0,2)+qG(6,8))/2.;
    Vec dphids = (qG(6,8)-qG(0,2))/l0;
    
    Vec tangent = ag->computet(phi);
    Vec normal = ag->computen(phi);
    Vec binormal = ag->computeb(phi);
    
    SqrMat dtangentdphi = ag->computetq(phi);
    SqrMat dnormaldphi = ag->computenq(phi);
    SqrMat dbinormaldphi = ag->computebq(phi);

    /* differentiation of 'bending and torsion energy' with respect to qG */
    double GI0ktilde0 = G*I0*binormal.T()*dnormaldphi*dphids;
    Vec ktilde0_0 = 0.5*dbinormaldphi.T()*dnormaldphi*phi;
    Vec ktilde0_1 = dnormaldphi.T()*binormal/l0;
    Vec ktilde0_2 = 0.5*(ag->computenqt(phi,dphids)).T()*binormal;
    Vec dBTtorsiondqG(9,INIT,0.);
    dBTtorsiondqG(0,2) = ktilde0_0 - ktilde0_1 + ktilde0_2;
    dBTtorsiondqG(6,8) = ktilde0_0 + ktilde0_1 + ktilde0_2;
    dBTtorsiondqG *= GI0ktilde0;

    double EI1ktilde1 = E*I1*(tangent.T()*dbinormaldphi*dphids-k10);
    Vec ktilde1_0 = 0.5*dtangentdphi.T()*dbinormaldphi*phi;
    Vec ktilde1_1 = dbinormaldphi.T()*tangent/l0;
    Vec ktilde1_2 = 0.5*(ag->computebqt(phi,dphids)).T()*tangent;
    Vec dBTbending1dqG(9,INIT,0.);
    dBTbending1dqG(0,2) = ktilde1_0 - ktilde1_1 + ktilde1_2;
    dBTbending1dqG(6,8) = ktilde1_0 + ktilde1_1 + ktilde1_2;
    dBTbending1dqG *= EI1ktilde1;

    double EI2ktilde2 = E*I2*(normal.T()*dtangentdphi*dphids-k20);
    Vec ktilde2_0 = 0.5*dnormaldphi.T()*dtangentdphi*phi;
    Vec ktilde2_1 = dtangentdphi.T()*normal/l0;
    Vec ktilde2_2 = 0.5*(ag->computetqt(phi,dphids)).T()*normal;
    Vec dBTbending2dqG(9,INIT,0.);
    dBTbending2dqG(0,2) = ktilde2_0 - ktilde2_1 + ktilde2_2;
    dBTbending2dqG(6,8) = ktilde2_0 + ktilde2_1 + ktilde2_2;
    dBTbending2dqG *= EI2ktilde2;

    /* generalized forces */
    h = (-l0)*(dBTtorsiondqG+dBTbending1dqG+dBTbending2dqG);
  }

  double FiniteElement1s33CosseratRotation::computeElasticEnergy(const fmatvec::Vec& qG) {
    Vec phi = (qG(0,2)+qG(6,8))/2.;
    Vec dphids = (qG(6,8)-qG(0,2))/l0;

    Vec tangent = ag->computet(phi);
    Vec normal = ag->computen(phi);
    Vec binormal = ag->computeb(phi);

    SqrMat dtangentdphi = ag->computetq(phi);
    SqrMat dnormaldphi = ag->computenq(phi);
    SqrMat dbinormaldphi = ag->computebq(phi);

    return 0.5*l0*(G*I0*pow(binormal.T()*dnormaldphi*dphids,2.)+E*I1*pow(tangent.T()*dbinormaldphi*dphids-k10,2.)+E*I2*pow(normal.T()*dtangentdphi*dphids-k20,2.));
  }

}

