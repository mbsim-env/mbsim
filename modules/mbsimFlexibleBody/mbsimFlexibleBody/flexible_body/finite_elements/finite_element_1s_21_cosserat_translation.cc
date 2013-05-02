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
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_21_cosserat_translation.h"
#include "mbsimFlexibleBody/utils/cardan.h"
#include "mbsim/utils/eps.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FiniteElement1s21CosseratTranslation::FiniteElement1s21CosseratTranslation(double l0_, double rho_,double A_, double E_, double G_, double I1_, const Vec& g_ ) : l0(l0_), rho(rho_), A(A_), E(E_), G(G_), I1(I1_), g(g_), cEps0D(0.), cEps1D(0.), sigma1(1.), sigma2(1.), M(5,INIT,0.), h(5,INIT,0.), X(12,INIT,0.){}

  FiniteElement1s21CosseratTranslation::~FiniteElement1s21CosseratTranslation() {}

  void FiniteElement1s21CosseratTranslation::setMaterialDamping(double cEps0D_, double cEps1D_) {
    cEps0D = cEps0D_;
    cEps1D = cEps1D_;
  }

  void FiniteElement1s21CosseratTranslation::computeM(const Vec& qG) {
  }

  void FiniteElement1s21CosseratTranslation::computeh(const Vec& qG, const Vec& qGt) {
	//Winkel
	double sgamma = sin(qG(2));
	double cgamma = cos(qG(2));

	// Vectoren
	Vec tangent(2,INIT,0.);
	tangent(0) = cgamma;
	tangent(1) = sgamma;

	Vec normal(2,INIT,0.);
	normal(0) = -sgamma;
	normal(1) = cgamma;

	Vec tangentt(2,INIT,0.);
	tangentt(0) = -sgamma*qGt(2);
	tangentt(1) = cgamma*qGt(2);

	Vec dtangentdgamma(2,INIT,0.);
	dtangentdgamma(0) = -sgamma;
	dtangentdgamma(1) = cgamma;

	Vec dnormaldgamma(2,INIT,0.);
	dnormaldgamma(0) = -cgamma;
	dnormaldgamma(1) = -sgamma;


	// Energieterme
	Vec dSETdqG(5,INIT,0.); // strain
	Vec dSENdqG(5,INIT,0.);
	Vec dVeldqG(5,INIT,0.);

	Vec dVgdqG(5,INIT,0.); // gravitational

	Vec dSDTdqGt(5,INIT,0.);
	Vec dSDNdqGt(5,INIT,0.);
	Vec dSDdqGt(5,INIT,0.);	// dissipation


	/* position and velocity difference */ //DONE
 	Vec deltax = qG(3,4)-qG(0,1);
	Vec deltaxt = qGt(3,4)-qGt(0,1);

	/* differentiation of strain energy */ //DONE
	// strain tangential
	dSETdqG(0,1) = -tangent.copy();
	dSETdqG(2) 	 = deltax.T()*dtangentdgamma;
	dSETdqG(3,4) = tangent.copy();
	dSETdqG 	*= E*A*(tangent.T()*deltax-l0)/l0;

	// strain normal

	dSENdqG(0,1) = -normal.copy();
	dSENdqG(2) 	 = deltax.T()*dnormaldgamma;
	dSENdqG(3,4) = normal.copy();
	dSENdqG 	*= G*sigma1*A*normal.T()*deltax/l0;

	// assemble
	dVeldqG = dSETdqG + dSENdqG;


	/* differentiation of gravitational energy */ //DONE
	dVgdqG(0,1) = -0.5*rho*A*l0*g(0,1);
	dVgdqG(3,4) = -0.5*rho*A*l0*g(0,1);

	/* differentiation of translatory energy */
	// equal zero

	/* differentiation of Rotatory energy */
	// equal zero

	/* differentiation of strain dissipation */ //DONE
	dSDTdqGt(0,1) = -tangent.copy() ;
	dSDTdqGt(2) = -tangent.T()*dtangentdgamma*l0;
	dSDTdqGt(3,4) = tangent.copy();
	dSDTdqGt *= 2.*cEps0D*(deltaxt.T()*tangent/l0 - tangent.T()*tangentt);

	dSDNdqGt(0,1) = -normal.copy();
	dSDNdqGt(2) = -normal.T()*dtangentdgamma*l0;
	dSDNdqGt(3,4) = normal.copy();
	dSDNdqGt *= 2.*cEps1D*(deltaxt.T()*normal/l0 -normal.T()*tangentt);

    dSDdqGt = dSDTdqGt + dSDNdqGt;


    /* generalized forces */
    h = (-dVgdqG-dVeldqG-dSDdqGt);

  }

  double FiniteElement1s21CosseratTranslation::computeKineticEnergy(const fmatvec::Vec& qG, const fmatvec::Vec& qGt) {
	double TT;
	double TR;
	// translatory
	TT = 0.25*rho*A*l0*(pow(nrm2(qGt(0,1)),2.)+pow(nrm2(qGt(3,4)),2.));
	// rotatory
	TR = 0.5*rho*l0*I1*pow(qGt(2),2.);

	return TT+TR;
  }

  double FiniteElement1s21CosseratTranslation::computeGravitationalEnergy(const fmatvec::Vec& qG) {
    return -0.5*rho*A*l0*g.T()*(qG(0,1)+qG(3,4));
  }

  double FiniteElement1s21CosseratTranslation::computeElasticEnergy(const fmatvec::Vec& qG) {
	Vec tangent(2,INIT,0.);
	Vec normal(2,INIT,0.);

	/* position difference */
	Vec deltax = qG(3,4)-qG(0,1);

	/* strain energy */
	return 0.5*(E*A*pow(tangent.T()*deltax-l0,2.) + G*sigma1*A*pow(normal.T()*deltax,2.))/l0;
  }

  const Vec& FiniteElement1s21CosseratTranslation::computeStateTranslation(const Vec& qG, const Vec& qGt,double s) {
    X(0,1) = qG(0,1) + s*(qG(3,4)-qG(0,1))/l0; // position
    X(3,4) = qGt(0,1) + s*((qGt(3,4)-qGt(0,1))/l0); // velocity

    X(5) = qG(2); // angles TODO in angle element or better in FlexibleBody
    X(11) = qGt(2); // time differentiated angels TODO in angle element or better in FlexibleBody

    return X;
  }

  void FiniteElement1s21CosseratTranslation::initM() {
	M(0,0) = 0.5*rho*A*l0;
	M(1,1) = 0.5*rho*A*l0;
	M(2,2) = rho*l0*I1;
	M(3,3) = 0.5*rho*A*l0;
	M(4,4) = 0.5*rho*A*l0;
  }

}

