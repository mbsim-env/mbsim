/* Copyright (C) 2004-2012 MBSim Development Team
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
 * Contact: thomas.cebulla@mytum.de
 */

#include<config.h>
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_21_cosserat_rotation.h"
#include "mbsimFlexibleBody/utils/cardan.h"
#include "mbsim/utils/eps.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FiniteElement1s21CosseratRotation::FiniteElement1s21CosseratRotation(double l0_, double E_, double G_, double I1_) : l0(l0_), E(E_), G(G_), I1(I1_), k10(0.), h(4,INIT,0.), X(12,INIT,0.) {}

  FiniteElement1s21CosseratRotation::~FiniteElement1s21CosseratRotation() {}

  void FiniteElement1s21CosseratRotation::setCurlRadius(double R1) {
    if (fabs(R1)>epsroot()) k10 = 1./R1;
  }

  void FiniteElement1s21CosseratRotation::computeh(const Vec& qG, const Vec& qGt) {
	double dgammads = (qG(3)-qG(0))/l0;
	Vec dBTbendingdqG(4,INIT,0.);
	dBTbendingdqG(0) = E*I1*( dgammads - k10  );
	dBTbendingdqG(3) = -E*I1*( dgammads -  k10  );
	h = dBTbendingdqG;
  }

  double FiniteElement1s21CosseratRotation::computeElasticEnergy(const fmatvec::Vec& qG) {
	double dgammads = (qG(3)-qG(0))/l0;
	return 0.5*l0*E*I1*(dgammads - k10);
  }
}

