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
    /* differentiation of 'bending and torsion energy' with respect to qG */
    // TODO
    Vec dBTdqG(9,INIT,0.);

    /* generalized forces */
    h = -dBTdqG.copy();
  }

  double FiniteElement1s33CosseratRotation::computeElasticEnergy(const fmatvec::Vec& qG) {
    // TODO bending and torsion energy
    return 0.;
  }

}

