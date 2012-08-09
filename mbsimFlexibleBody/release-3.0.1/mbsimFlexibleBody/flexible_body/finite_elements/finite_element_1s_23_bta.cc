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
 *          rzander@users.berlios.de
 */

#include <config.h>
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_23_bta.h"
#include "mbsim/utils/eps.h"

using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FiniteElement1s23BTA::FiniteElement1s23BTA(double sl0, double sArho, double sEIyy, double sEIzz, double sItrho, double sGIt, Vec sg) : l0(sl0), Arho(sArho), EIyy(sEIyy), EIzz(sEIzz), Itrho(sItrho), GIt(sGIt), g(sg), depsilon(0.), dTorsional(0.), M(getuSize(),INIT,0.), h(getuSize(),INIT,0.), Dhq(8), Dhqp(8), Damp(8,INIT,0.) {
    l0h2 = l0*l0;
    l0h3 = l0h2*l0;
  }

  void FiniteElement1s23BTA::computeM(const Vec& q) {
    // local coordinates 
    const double &w1 = q(1); const double &b1 = q(2);
    const double &u1 = q(3); const double &g1 = q(4);
    const double &w2 = q(6); const double &b2 = q(7);
    const double &u2 = q(8); const double &g2 = q(9);

    // symmetric mass matrix
    M(0,0) = (l0*(420*Itrho + Arho*(5*Power(b1,2)*l0h2 + 2*Power(b2,2)*l0h2 + 5*Power(g1,2)*l0h2 - 5*g1*g2*l0h2 + 2*Power(g2,2)*l0h2 + 65*g1*l0*u1 - 25*g2*l0*u1 + 290*Power(u1,2) + 19*g1*l0*u2 - 17*g2*l0*u2 + 92*u1*u2 + 38*Power(u2,2) - 25*b2*l0*w1 + 290*Power(w1,2) - 17*b2*l0*w2 + 92*w1*w2 + 38*Power(w2,2) + b1*l0*(-5*b2*l0 + 65*w1 + 19*w2))))/1260.;
    M(0,1) = -(Arho*l0*(15*g1*l0 - 7*g2*l0 + 120*u1 + 27*u2))/420.;
    M(0,2) = -(Arho*l0h2*(5*g1*l0 - 3*g2*l0 + 30*u1 + 12*u2))/840.;
    M(0,3) = (Arho*l0*(15*b1*l0 - 7*b2*l0 + 120*w1 + 27*w2))/420.;
    M(0,4) = (Arho*l0h2*(5*b1*l0 - 3*b2*l0 + 30*w1 + 12*w2))/840.;
    M(0,5) = (l0*(420*Itrho + Arho*(5*Power(b1,2)*l0h2 + 5*Power(b2,2)*l0h2 + 5*Power(g1,2)*l0h2 - 8*g1*g2*l0h2 + 5*Power(g2,2)*l0h2 + 50*g1*l0*u1 - 34*g2*l0*u1 + 140*Power(u1,2) + 34*g1*l0*u2 - 50*g2*l0*u2 + 140*u1*u2 + 140*Power(u2,2) + 2*b1*l0*(-4*b2*l0 + 25*w1 + 17*w2) - 2*b2*l0*(17*w1 + 25*w2) + 140*(Power(w1,2) + w1*w2 + Power(w2,2)))))/2520.;
    M(0,6) = -(Arho*l0*(6*g1*l0 - 7*g2*l0 + 27*u1 + 36*u2))/420.;
    M(0,7) = (Arho*l0h2*(3*g1*l0 - 3*g2*l0 + 14*(u1 + u2)))/840.;
    M(0,8) = (Arho*l0*(6*b1*l0 - 7*b2*l0 + 27*w1 + 36*w2))/420.;
    M(0,9) = -(Arho*l0h2*(3*b1*l0 - 3*b2*l0 + 14*(w1 + w2)))/840.;
    M(1,1) = (13*Arho*l0)/35.;
    M(1,2) = (11*Arho*l0h2)/210.;
    M(1,5) = -(Arho*l0*(7*g1*l0 - 6*g2*l0 + 36*u1 + 27*u2))/420.;
    M(1,6) = (9*Arho*l0)/70.;
    M(1,7) = (-13*Arho*l0h2)/420.;
    M(2,2) = (Arho*l0h3)/105.;
    M(2,5) = -(Arho*l0h2*(3*g1*l0 - 3*g2*l0 + 14*(u1 + u2)))/840.;
    M(2,6) = (13*Arho*l0h2)/420.;
    M(2,7) = -(Arho*l0h3)/140.;
    M(3,3) = (13*Arho*l0)/35.;
    M(3,4) = (11*Arho*l0h2)/210.;
    M(3,5) = (Arho*l0*(7*b1*l0 - 6*b2*l0 + 36*w1 + 27*w2))/420.;
    M(3,6) = 0;
    M(3,7) = 0;
    M(3,8) = (9*Arho*l0)/70.;
    M(3,9) = (-13*Arho*l0h2)/420.;
    M(4,4) = (Arho*l0h3)/105.;
    M(4,5) = (Arho*l0h2*(3*b1*l0 - 3*b2*l0 + 14*(w1 + w2)))/840.;
    M(4,8) = (13*Arho*l0h2)/420.;
    M(4,9) = -(Arho*l0h3)/140.;
    M(5,5) = (l0*(420*Itrho + Arho*(2*Power(b1,2)*l0h2 + 5*Power(b2,2)*l0h2 + 2*Power(g1,2)*l0h2 - 5*g1*g2*l0h2 + 5*Power(g2,2)*l0h2 + 17*g1*l0*u1 - 19*g2*l0*u1 + 38*Power(u1,2) + 25*g1*l0*u2 - 65*g2*l0*u2 + 92*u1*u2 + 290*Power(u2,2) - 19*b2*l0*w1 + 38*Power(w1,2) - 65*b2*l0*w2 + 92*w1*w2 + 290*Power(w2,2) + b1*l0*(-5*b2*l0 + 17*w1 + 25*w2))))/1260.;
    M(5,6) = -(Arho*l0*(7*g1*l0 + 3*(-5*g2*l0 + 9*u1 + 40*u2)))/420.;
    M(5,7) = (Arho*l0h2*(3*g1*l0 - 5*g2*l0 + 12*u1 + 30*u2))/840.;
    M(5,8) = (Arho*l0*(7*b1*l0 + 3*(-5*b2*l0 + 9*w1 + 40*w2)))/420.;
    M(5,9) = -(Arho*l0h2*(3*b1*l0 - 5*b2*l0 + 12*w1 + 30*w2))/840.;
    M(6,6) = (13*Arho*l0)/35.;
    M(6,7) = (-11*Arho*l0h2)/210.;
    M(7,7) = (Arho*l0h3)/105.;
    M(8,8) = (13*Arho*l0)/35.;
    M(8,9) = (-11*Arho*l0h2)/210.;
    M(9,9) = (Arho*l0h3)/105.;
  }

  void FiniteElement1s23BTA::computeh(const Vec& q, const Vec& v) {
    // local coordinates and velocities
    const double &a1 = q(0);
    const double &w1 = q(1); const double &b1 = q(2);
    const double &u1 = q(3); const double &g1 = q(4);
    const double &a2 = q(5);
    const double &w2 = q(6); const double &b2 = q(7);
    const double &u2 = q(8); const double &g2 = q(9);

    const double &a1p = v(0);
    const double &w1p = v(1); const double &b1p = v(2);
    const double &u1p = v(3); const double &g1p = v(4);
    const double &a2p = v(5);
    const double &w2p = v(6); const double &b2p = v(7);
    const double &u2p = v(8); const double &g2p = v(9);

    // right hand side
    h(0) = (((-a1 + a2)*GIt)/l0 - (Arho*l0*(a2p*(5*b2*b2p*l0h2 + 5*g1*g1p*l0h2 - 4*g1p*g2*l0h2 - 4*g1*g2p*l0h2 + 5*g2*g2p*l0h2 + 25*g1p*l0*u1 - 17*g2p*l0*u1 + 25*g1*l0*u1p - 17*g2*l0*u1p + 140*u1*u1p + 17*g1p*l0*u2 - 25*g2p*l0*u2 + 70*u1p*u2 + 17*g1*l0*u2p - 25*g2*l0*u2p + 70*u1*u2p + 140*u2*u2p - 17*b2p*l0*w1 - 17*b2*l0*w1p + 140*w1*w1p - 25*b2p*l0*w2 + 70*w1p*w2 + b1p*l0*(-4*b2*l0 + 25*w1 + 17*w2) - 25*b2*l0*w2p + 70*w1*w2p + 140*w2*w2p + b1*l0*(5*b1p*l0 - 4*b2p*l0 + 25*w1p + 17*w2p)) + a1p*(4*b2*b2p*l0h2 + 10*g1*g1p*l0h2 - 5*g1p*g2*l0h2 - 5*g1*g2p*l0h2 + 4*g2*g2p*l0h2 + 65*g1p*l0*u1 - 25*g2p*l0*u1 + 65*g1*l0*u1p - 25*g2*l0*u1p + 580*u1*u1p + 19*g1p*l0*u2 - 17*g2p*l0*u2 + 92*u1p*u2 + 19*g1*l0*u2p - 17*g2*l0*u2p + 92*u1*u2p + 76*u2*u2p - 25*b2p*l0*w1 - 25*b2*l0*w1p + 580*w1*w1p - 17*b2p*l0*w2 + 92*w1p*w2 + b1p*l0*(-5*b2*l0 + 65*w1 + 19*w2) - 17*b2*l0*w2p + 92*w1*w2p + 76*w2*w2p + b1*l0*(10*b1p*l0 - 5*b2p*l0 + 65*w1p + 19*w2p))))/1260. );
    h(1) = ((-15120*b1*EIzz*l0 - 15120*b2*EIzz*l0 + (65*Power(a1p,2) + 50*a1p*a2p + 17*Power(a2p,2))*Arho*b1*Power(l0,5) - (25*Power(a1p,2) + 34*a1p*a2p + 19*Power(a2p,2))*Arho*b2*Power(l0,5) + 4*(-7560*EIzz*(w1 - w2) + Arho*Power(l0,4)*(Power(a1p,2)*(145*w1 + 23*w2) + a2p*(21*g1p*l0 - 18*g2p*l0 + 108*u1p + 81*u2p + 19*a2p*w1 + 23*a2p*w2) + a1p*(45*g1p*l0 - 21*g2p*l0 + 360*u1p + 81*u2p + 35*a2p*(2*w1 + w2)))))/(2520.*l0h3) );
    h(2) = ((-5040*b2*EIzz*l0 - (5*Power(a1p,2) + 8*a1p*a2p + 5*Power(a2p,2))*Arho*b2*Power(l0,5) + 2*b1*l0*(-5040*EIzz + (5*Power(a1p,2) + 5*a1p*a2p + 2*Power(a2p,2))*Arho*Power(l0,4)) - 15120*EIzz*(w1 - w2) + Arho*Power(l0,4)*(Power(a1p,2)*(65*w1 + 19*w2) + 2*a1p*(15*g1p*l0 - 9*g2p*l0 + 90*u1p + 36*u2p + 25*a2p*w1 + 17*a2p*w2) + a2p*(18*(g1p - g2p)*l0 + 84*u1p + 84*u2p + 17*a2p*w1 + 25*a2p*w2)))/(2520.*l0h2) );
    h(3) = ((-15120*EIyy*(g1*l0 + g2*l0 + 2*u1 - 2*u2) + Arho*Power(l0,4)*(Power(a1p,2)*(65*g1*l0 - 25*g2*l0 + 580*u1 + 92*u2) + a2p*(-84*b1p*l0 + 72*b2p*l0 + a2p*(17*g1*l0 - 19*g2*l0 + 76*u1 + 92*u2) - 108*(4*w1p + 3*w2p)) - 2*a1p*(90*b1p*l0 - 42*b2p*l0 - a2p*(25*g1*l0 - 17*g2*l0 + 70*(2*u1 + u2)) + 18*(40*w1p + 9*w2p))))/(2520.*l0h3) );
    h(4) = ((-5040*EIyy*(2*g1*l0 + g2*l0 + 3*u1 - 3*u2) + Arho*Power(l0,4)*(Power(a1p,2)*(10*g1*l0 - 5*g2*l0 + 65*u1 + 19*u2) - 2*a1p*(15*b1p*l0 - 9*b2p*l0 - a2p*(5*g1*l0 - 4*g2*l0 + 25*u1 + 17*u2) + 90*w1p + 36*w2p) + a2p*(-18*b1p*l0 + 18*b2p*l0 + a2p*(4*g1*l0 - 5*g2*l0 + 17*u1 + 25*u2) - 84*(w1p + w2p))))/(2520.*l0h2) );
    h(5) = (((a1 - a2)*GIt)/l0 - (Arho*l0*(a1p*(5*b2*b2p*l0h2 + 5*g1*g1p*l0h2 - 4*g1p*g2*l0h2 - 4*g1*g2p*l0h2 + 5*g2*g2p*l0h2 + 25*g1p*l0*u1 - 17*g2p*l0*u1 + 25*g1*l0*u1p - 17*g2*l0*u1p + 140*u1*u1p + 17*g1p*l0*u2 - 25*g2p*l0*u2 + 70*u1p*u2 + 17*g1*l0*u2p - 25*g2*l0*u2p + 70*u1*u2p + 140*u2*u2p - 17*b2p*l0*w1 - 17*b2*l0*w1p + 140*w1*w1p - 25*b2p*l0*w2 + 70*w1p*w2 + b1p*l0*(-4*b2*l0 + 25*w1 + 17*w2) - 25*b2*l0*w2p + 70*w1*w2p + 140*w2*w2p + b1*l0*(5*b1p*l0 - 4*b2p*l0 + 25*w1p + 17*w2p)) + a2p*(10*b2*b2p*l0h2 + 4*g1*g1p*l0h2 - 5*g1p*g2*l0h2 - 5*g1*g2p*l0h2 + 10*g2*g2p*l0h2 + 17*g1p*l0*u1 - 19*g2p*l0*u1 + 17*g1*l0*u1p - 19*g2*l0*u1p + 76*u1*u1p + 25*g1p*l0*u2 - 65*g2p*l0*u2 + 92*u1p*u2 + 25*g1*l0*u2p - 65*g2*l0*u2p + 92*u1*u2p + 580*u2*u2p - 19*b2p*l0*w1 - 19*b2*l0*w1p + 76*w1*w1p - 65*b2p*l0*w2 + 92*w1p*w2 + b1p*l0*(-5*b2*l0 + 17*w1 + 25*w2) - 65*b2*l0*w2p + 92*w1*w2p + 580*w2*w2p + b1*l0*(4*b1p*l0 - 5*b2p*l0 + 17*w1p + 25*w2p))))/1260. );
    h(6) = ((15120*b1*EIzz*l0 + 15120*b2*EIzz*l0 + (19*Power(a1p,2) + 34*a1p*a2p + 25*Power(a2p,2))*Arho*b1*Power(l0,5) - (17*Power(a1p,2) + 50*a1p*a2p + 65*Power(a2p,2))*Arho*b2*Power(l0,5) + 4*(7560*EIzz*(w1 - w2) + Arho*Power(l0,4)*(Power(a1p,2)*(23*w1 + 19*w2) + a2p*(21*g1p*l0 - 45*g2p*l0 + 81*u1p + 360*u2p + 23*a2p*w1 + 145*a2p*w2) + a1p*(3*(6*g1p - 7*g2p)*l0 + 81*u1p + 108*u2p + 35*a2p*(w1 + 2*w2)))))/(2520.*l0h3));
    h(7) = (-(5040*b1*EIzz*l0 + 10080*b2*EIzz*l0 + (5*Power(a1p,2) + 8*a1p*a2p + 5*Power(a2p,2))*Arho*b1*Power(l0,5) - 2*(2*Power(a1p,2) + 5*a1p*a2p + 5*Power(a2p,2))*Arho*b2*Power(l0,5) + 15120*EIzz*(w1 - w2) + Arho*Power(l0,4)*(Power(a1p,2)*(25*w1 + 17*w2) + 2*a1p*(9*g1p*l0 - 9*g2p*l0 + 42*u1p + 42*u2p + 17*a2p*w1 + 25*a2p*w2) + a2p*(18*g1p*l0 - 30*g2p*l0 + 72*u1p + 180*u2p + 19*a2p*w1 + 65*a2p*w2)))/(2520.*l0h2));
    h(8) = ((15120*EIyy*(g1*l0 + g2*l0 + 2*u1 - 2*u2) + Arho*Power(l0,4)*(Power(a1p,2)*(19*g1*l0 - 17*g2*l0 + 92*u1 + 76*u2) + a1p*(-72*b1p*l0 + 84*b2p*l0 + 2*a2p*(17*g1*l0 - 25*g2*l0 + 70*(u1 + 2*u2)) - 108*(3*w1p + 4*w2p)) + a2p*(-84*b1p*l0 + 180*b2p*l0 + a2p*(25*g1*l0 - 65*g2*l0 + 92*u1 + 580*u2) - 36*(9*w1p + 40*w2p))))/(2520.*l0h3) );
    h(9) = (-(5040*EIyy*(g1*l0 + 2*g2*l0 + 3*u1 - 3*u2) + Arho*Power(l0,4)*(Power(a1p,2)*(5*g1*l0 - 4*g2*l0 + 25*u1 + 17*u2) - 2*a1p*(9*b1p*l0 - 9*b2p*l0 - a2p*(4*g1*l0 - 5*g2*l0 + 17*u1 + 25*u2) + 42*(w1p + w2p)) + a2p*(-18*b1p*l0 + 30*b2p*l0 + a2p*(5*(g1 - 2*g2)*l0 + 19*u1 + 65*u2) - 36*(2*w1p + 5*w2p))))/(2520.*l0h2) );

    // damping
    h(0) += dTorsional * ( a2p - a1p );
    h(5) += dTorsional * ( a1p - a2p );
  }

  void FiniteElement1s23BTA::computedhdz(const Vec& q, const Vec& v) {
    Vec h0 = h.copy();

    Vec q_tmp = q.copy();
    Vec v_tmp = v.copy();

    /**************** velocity dependent calculations ********************/
    for(int i=0;i<v.size();i++) {  
      double vi = v_tmp(i); // save correct position

      v_tmp(i) += epsroot(); // update with disturbed positions assuming same active links
      computeh(q_tmp,v_tmp);

      Dhqp.col(i) = (h-h0)/epsroot();
      v_tmp(i) = vi;
    }

    /***************** position dependent calculations ********************/
    for(int i=0;i<q.size();i++) { 
      double qi = q_tmp(i); // save correct position

      q_tmp(i) += epsroot(); // update with disturbed positions assuming same active links
      computeh(q_tmp,v_tmp);

      Dhq.col(i) = (h-h0)/epsroot();
      q_tmp(i) = qi;
    }

    /******************* back to initial state **********************/
    computeh(q,v);
  }

  Vec FiniteElement1s23BTA::Tangent(const Vec& q, const double& s) {
    Vec tangent(3,NONINIT);

    const double &a1 = q(0);
    const double &w1 = q(1);  const double &b1 = q(2);
    const double &u1 = q(3);  const double &g1 = q(4);
    const double &a2 = q(5);
    const double &w2 = q(6);  const double &b2 = q(7);
    const double &u2 = q(8);  const double &g2 = q(9);

    tangent(0) = 1.0;
    tangent(1) = -((-(a1/l0) + a2/l0)*(g1*s + u1 - (Power(s,2)*(2*g1*l0 + g2*l0 + 3*u1 - 3*u2))/l0h2 + (Power(s,3)*(g1*l0 + g2*l0 + 2*u1 - 2*u2))/l0h3)*cos((a1*(l0 - s))/l0 + (a2*s)/l0)) + (b1 - (2*s*(2*b1*l0 + b2*l0 + 3*w1 - 3*w2))/l0h2 + (3*Power(s,2)*(b1*l0 + b2*l0 + 2*w1 - 2*w2))/l0h3)*cos((a1*(l0 - s))/l0 + (a2*s)/l0) - (g1 - (2*s*(2*g1*l0 + g2*l0 + 3*u1 - 3*u2))/l0h2 + (3*Power(s,2)*(g1*l0 + g2*l0 + 2*u1 - 2*u2))/l0h3)*sin((a1*(l0 - s))/l0 + (a2*s)/l0) - (-(a1/l0) + a2/l0)*(b1*s + w1 - (Power(s,2)*(2*b1*l0 + b2*l0 + 3*w1 - 3*w2))/l0h2 + (Power(s,3)*(b1*l0 + b2*l0 + 2*w1 - 2*w2))/l0h3)*sin((a1*(l0 - s))/l0 + (a2*s)/l0);
    tangent(2) = (g1 - (2*s*(2*g1*l0 + g2*l0 + 3*u1 - 3*u2))/l0h2 + (3*Power(s,2)*(g1*l0 + g2*l0 + 2*u1 - 2*u2))/l0h3)*cos((a1*(l0 - s))/l0 + (a2*s)/l0) + (-(a1/l0) + a2/l0)*(b1*s + w1 - (Power(s,2)*(2*b1*l0 + b2*l0 + 3*w1 - 3*w2))/l0h2 + (Power(s,3)*(b1*l0 + b2*l0 + 2*w1 - 2*w2))/l0h3)*cos((a1*(l0 - s))/l0 + (a2*s)/l0) - (-(a1/l0) + a2/l0)*(g1*s + u1 - (Power(s,2)*(2*g1*l0 + g2*l0 + 3*u1 - 3*u2))/l0h2 + (Power(s,3)*(g1*l0 + g2*l0 + 2*u1 - 2*u2))/l0h3)*sin((a1*(l0 - s))/l0 + (a2*s)/l0) + (b1 - (2*s*(2*b1*l0 + b2*l0 + 3*w1 - 3*w2))/l0h2 + (3*Power(s,2)*(b1*l0 + b2*l0 + 2*w1 - 2*w2))/l0h3)*sin((a1*(l0 - s))/l0 + (a2*s)/l0);

    tangent /= nrm2(tangent);

    return tangent;
  }

  SqrMat FiniteElement1s23BTA::AWK(const Vec& q, const double& s) {
    SqrMat AWK(3,NONINIT);
    Vec bzT(3,INIT,0.);

    double alpha = q(0) * (1-s/l0) + q(5) * (s/l0);

    bzT(1) = -sin(alpha);
    bzT(2) =  cos(alpha);

    AWK.col(0)  = Tangent(q,s);
    AWK.col(1)  = crossProduct(bzT,AWK.col(0));
    AWK.col(1) /= nrm2(AWK.col(1));
    AWK.col(2)  = crossProduct(AWK.col(0),AWK.col(1));

    return AWK;
  }

  Vec FiniteElement1s23BTA::StateAxis(const Vec& q, const Vec& v,const double& s) {
    Vec X(12); 

    const double &a1 = q(0);
    const double &w1 = q(1); const double &b1 = q(2);
    const double &u1 = q(3); const double &g1 = q(4);
    const double &a2 = q(5);
    const double &w2 = q(6); const double &b2 = q(7);
    const double &u2 = q(8); const double &g2 = q(9);

    const double &a1p = v(0);
    const double &w1p = v(1); const double &b1p = v(2);
    const double &u1p = v(3); const double &g1p = v(4);
    const double &a2p = v(5);
    const double &w2p = v(6); const double &b2p = v(7);
    const double &u2p = v(8); const double &g2p = v(9);

    X(0) = s;
    X(1) = (b1*s + w1 - (Power(s,2)*(2*b1*l0 + b2*l0 + 3*w1 - 3*w2))/Power(l0,2) + (Power(s,3)*(b1*l0 + b2*l0 + 2*w1 - 2*w2))/Power(l0,3))*cos((a1*(l0 - s))/l0 + (a2*s)/l0) - (g1*s + u1 - (Power(s,2)*(2*g1*l0 + g2*l0 + 3*u1 - 3*u2))/Power(l0,2) + (Power(s,3)*(g1*l0 + g2*l0 + 2*u1 - 2*u2))/Power(l0,3))*sin((a1*(l0 - s))/l0 + (a2*s)/l0);
    X(2) = (g1*s + u1 - (Power(s,2)*(2*g1*l0 + g2*l0 + 3*u1 - 3*u2))/Power(l0,2) + (Power(s,3)*(g1*l0 + g2*l0 + 2*u1 - 2*u2))/Power(l0,3))*cos((a1*(l0 - s))/l0 + (a2*s)/l0) + (b1*s + w1 - (Power(s,2)*(2*b1*l0 + b2*l0 + 3*w1 - 3*w2))/Power(l0,2) + (Power(s,3)*(b1*l0 + b2*l0 + 2*w1 - 2*w2))/Power(l0,3))*sin((a1*(l0 - s))/l0 + (a2*s)/l0);
    X(3) = (a1*(l0 - s))/l0 + (a2*s)/l0;
    X(4) = (g1 - (2*s*(2*g1*l0 + g2*l0 + 3*u1 - 3*u2))/Power(l0,2) + (3*Power(s,2)*(g1*l0 + g2*l0 + 2*u1 - 2*u2))/Power(l0,3))*cos((a1*(l0 - s))/l0 + (a2*s)/l0) - (-b1 + (2*s*(2*b1*l0 + b2*l0 + 3*w1 - 3*w2))/Power(l0,2) - (3*Power(s,2)*(b1*l0 + b2*l0 + 2*w1 - 2*w2))/Power(l0,3))*sin((a1*(l0 - s))/l0 + (a2*s)/l0);
    X(5) = (-b1 + (2*s*(2*b1*l0 + b2*l0 + 3*w1 - 3*w2))/Power(l0,2) - (3*Power(s,2)*(b1*l0 + b2*l0 + 2*w1 - 2*w2))/Power(l0,3))*cos((a1*(l0 - s))/l0 + (a2*s)/l0) + (g1 - (2*s*(2*g1*l0 + g2*l0 + 3*u1 - 3*u2))/Power(l0,2) + (3*Power(s,2)*(g1*l0 + g2*l0 + 2*u1 - 2*u2))/Power(l0,3))*sin((a1*(l0 - s))/l0 + (a2*s)/l0);
    X(6) = 0;
    X(7) = -(((a1p*(l0 - s) + a2p*s)*(g1*l0*Power(l0 - s,2)*s + g2*l0*Power(s,2)*(-l0 + s) + Power(l0,3)*u1 - 3*l0*Power(s,2)*u1 + 2*Power(s,3)*u1 + 3*l0*Power(s,2)*u2 - 2*Power(s,3)*u2)*cos((a1*l0 - a1*s + a2*s)/l0))/Power(l0,4)) + (b1p*s + w1p - (Power(s,2)*(2*b1p*l0 + b2p*l0 + 3*w1p - 3*w2p))/Power(l0,2) + (Power(s,3)*(b1p*l0 + b2p*l0 + 2*w1p - 2*w2p))/Power(l0,3))*cos((a1*l0 - a1*s + a2*s)/l0) - (g1p*s + u1p - (Power(s,2)*(2*g1p*l0 + g2p*l0 + 3*u1p - 3*u2p))/Power(l0,2) + (Power(s,3)*(g1p*l0 + g2p*l0 + 2*u1p - 2*u2p))/Power(l0,3))*sin((a1*l0 - a1*s + a2*s)/l0) - ((a1p*(l0 - s) + a2p*s)*(b1*l0*Power(l0 - s,2)*s + b2*l0*Power(s,2)*(-l0 + s) + Power(l0,3)*w1 - 3*l0*Power(s,2)*w1 + 2*Power(s,3)*w1 + 3*l0*Power(s,2)*w2 - 2*Power(s,3)*w2)*sin((a1*l0 - a1*s + a2*s)/l0))/Power(l0,4);
    X(8) = (g1p*s + u1p - (Power(s,2)*(2*g1p*l0 + g2p*l0 + 3*u1p - 3*u2p))/Power(l0,2) + (Power(s,3)*(g1p*l0 + g2p*l0 + 2*u1p - 2*u2p))/Power(l0,3))*cos((a1*l0 - a1*s + a2*s)/l0) + ((a1p*(l0 - s) + a2p*s)*(b1*l0*Power(l0 - s,2)*s + b2*l0*Power(s,2)*(-l0 + s) + Power(l0,3)*w1 - 3*l0*Power(s,2)*w1 + 2*Power(s,3)*w1 + 3*l0*Power(s,2)*w2 - 2*Power(s,3)*w2)*cos((a1*l0 - a1*s + a2*s)/l0))/Power(l0,4) - ((a1p*(l0 - s) + a2p*s)*(g1*l0*Power(l0 - s,2)*s + g2*l0*Power(s,2)*(-l0 + s) + Power(l0,3)*u1 - 3*l0*Power(s,2)*u1 + 2*Power(s,3)*u1 + 3*l0*Power(s,2)*u2 - 2*Power(s,3)*u2)*sin((a1*l0 - a1*s + a2*s)/l0))/Power(l0,4) + (b1p*s + w1p - (Power(s,2)*(2*b1p*l0 + b2p*l0 + 3*w1p - 3*w2p))/Power(l0,2) + (Power(s,3)*(b1p*l0 + b2p*l0 + 2*w1p - 2*w2p))/Power(l0,3))*sin((a1*l0 - a1*s + a2*s)/l0);
    X(9) = (a1p*l0 - a1p*s + a2p*s)/l0;
    X(10) =  -((g1p - (2*s*(2*g1p*l0 + g2p*l0 + 3*u1p - 3*u2p))/Power(l0,2) + (3*Power(s,2)*(g1p*l0 + g2p*l0 + 2*u1p - 2*u2p))/Power(l0,3))*cos((a1*(l0 - s))/l0 + (a2*s)/l0) - (-b1p + (2*s*(2*b1p*l0 + b2p*l0 + 3*w1p - 3*w2p))/Power(l0,2) - (3*Power(s,2)*(b1p*l0 + b2p*l0 + 2*w1p - 2*w2p))/Power(l0,3))*sin((a1*(l0 - s))/l0 + (a2*s)/l0));
    X(11) =  -((-b1p + (2*s*(2*b1p*l0 + b2p*l0 + 3*w1p - 3*w2p))/Power(l0,2) - (3*Power(s,2)*(b1p*l0 + b2p*l0 + 2*w1p - 2*w2p))/Power(l0,3))*cos((a1*(l0 - s))/l0 + (a2*s)/l0) + (g1p - (2*s*(2*g1p*l0 + g2p*l0 + 3*u1p - 3*u2p))/Power(l0,2) + (3*Power(s,2)*(g1p*l0 + g2p*l0 + 2*u1p - 2*u2p))/Power(l0,3))*sin((a1*(l0 - s))/l0 + (a2*s)/l0));

    return X;
  }

  Mat FiniteElement1s23BTA::JGeneralized(const Vec& q,const double& s) {
    Mat J(getuSize(),6-1);

    const double &a1 = q(0);
    const double &w1 = q(1); const double &b1 = q(2);
    const double &u1 = q(3); const double &g1 = q(4);
    const double &a2 = q(5);
    const double &w2 = q(6); const double &b2 = q(7);
    const double &u2 = q(8); const double &g2 = q(9);

    J(0,0) = -(((l0 - s)*(((-l0 + s)*(l0*s*(-(g1*l0) + (g1 + g2)*s) + (-l0 + s)*(l0 + 2*s)*u1) + (3*l0 - 2*s)*Power(s,2)*u2)*cos(a1 + ((-a1 + a2)*s)/l0) + ((-l0 + s)*(l0*s*(-(b1*l0) + (b1 + b2)*s) + (-l0 + s)*(l0 + 2*s)*w1) + (3*l0 - 2*s)*Power(s,2)*w2)*sin(a1 + ((-a1 + a2)*s)/l0)))/Power(l0,4));
    J(0,1) = ((l0 - s)*(((-l0 + s)*(l0*s*(-(b1*l0) + (b1 + b2)*s) + (-l0 + s)*(l0 + 2*s)*w1) + (3*l0 - 2*s)*Power(s,2)*w2)*cos(a1 + ((-a1 + a2)*s)/l0) - ((-l0 + s)*(l0*s*(-(g1*l0) + (g1 + g2)*s) + (-l0 + s)*(l0 + 2*s)*u1) + (3*l0 - 2*s)*Power(s,2)*u2)*sin(a1 + ((-a1 + a2)*s)/l0)))/Power(l0,4);
    J(0,2) = 1 - s/l0;
    J(0,3) = -((l0 - s)*((b1*l0*(l0 - 3*s)*(l0 - s) + s*(b2*l0*(-2*l0 + 3*s) - 6*(l0 - s)*(w1 - w2)))*cos(a1 + ((-a1 + a2)*s)/l0) - (g1*l0*(l0 - 3*s)*(l0 - s) + s*(g2*l0*(-2*l0 + 3*s) - 6*(l0 - s)*(u1 - u2)))*sin(a1 + ((-a1 + a2)*s)/l0)))/Power(l0,4);
    J(0,4) = -((l0 - s)*((g1*l0*(l0 - 3*s)*(l0 - s) + s*(g2*l0*(-2*l0 + 3*s) - 6*(l0 - s)*(u1 - u2)))*cos(a1 + ((-a1 + a2)*s)/l0) + (b1*l0*(l0 - 3*s)*(l0 - s) + s*(b2*l0*(-2*l0 + 3*s) - 6*(l0 - s)*(w1 - w2)))*sin(a1 + ((-a1 + a2)*s)/l0)))/Power(l0,4);
    J(1,0) = (Power(l0 - s,2)*(l0 + 2*s)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
    J(1,1) = (Power(l0 - s,2)*(l0 + 2*s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
    J(1,2) = 0;
    J(1,3) = -(6*s*(-l0 + s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
    J(1,4) = -(6*(l0 - s)*s*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
    J(2,0) = (Power(l0 - s,2)*s*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
    J(2,1) = (Power(l0 - s,2)*s*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
    J(2,2) = 0;
    J(2,3) = -((l0 - 3*s)*(l0 - s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
    J(2,4) = -(-(((l0 - 3*s)*(l0 - s)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2)));
    J(3,0) = -((Power(l0 - s,2)*(l0 + 2*s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3));
    J(3,1) = (Power(l0 - s,2)*(l0 + 2*s)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
    J(3,2) = 0;
    J(3,3) = -(6*s*(-l0 + s)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
    J(3,4) = -(6*s*(-l0 + s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
    J(4,0) = -((Power(l0 - s,2)*s*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2));
    J(4,1) = (Power(l0 - s,2)*s*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
    J(4,2) = 0;
    J(4,3) = -((l0 - 3*s)*(l0 - s)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
    J(4,4) = -((l0 - 3*s)*(l0 - s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
    J(5,0) = -((s*(((-l0 + s)*(l0*s*(-(g1*l0) + (g1 + g2)*s) + (-l0 + s)*(l0 + 2*s)*u1) + (3*l0 - 2*s)*Power(s,2)*u2)*cos(a1 + ((-a1 + a2)*s)/l0) + ((-l0 + s)*(l0*s*(-(b1*l0) + (b1 + b2)*s) + (-l0 + s)*(l0 + 2*s)*w1) + (3*l0 - 2*s)*Power(s,2)*w2)*sin(a1 + ((-a1 + a2)*s)/l0)))/Power(l0,4));
    J(5,1) = (s*(((-l0 + s)*(l0*s*(-(b1*l0) + (b1 + b2)*s) + (-l0 + s)*(l0 + 2*s)*w1) + (3*l0 - 2*s)*Power(s,2)*w2)*cos(a1 + ((-a1 + a2)*s)/l0) - ((-l0 + s)*(l0*s*(-(g1*l0) + (g1 + g2)*s) + (-l0 + s)*(l0 + 2*s)*u1) + (3*l0 - 2*s)*Power(s,2)*u2)*sin(a1 + ((-a1 + a2)*s)/l0)))/Power(l0,4);
    J(5,2) = s/l0;
    J(5,3) = -(s*((b1*l0*(l0 - 3*s)*(l0 - s) + s*(b2*l0*(-2*l0 + 3*s) - 6*(l0 - s)*(w1 - w2)))*cos(a1 + ((-a1 + a2)*s)/l0) - (g1*l0*(l0 - 3*s)*(l0 - s) + s*(g2*l0*(-2*l0 + 3*s) - 6*(l0 - s)*(u1 - u2)))*sin(a1 + ((-a1 + a2)*s)/l0)))/Power(l0,4);
    J(5,4) = -(s*((g1*l0*(l0 - 3*s)*(l0 - s) + s*(g2*l0*(-2*l0 + 3*s) - 6*(l0 - s)*(u1 - u2)))*cos(a1 + ((-a1 + a2)*s)/l0) + (b1*l0*(l0 - 3*s)*(l0 - s) + s*(b2*l0*(-2*l0 + 3*s) - 6*(l0 - s)*(w1 - w2)))*sin(a1 + ((-a1 + a2)*s)/l0)))/Power(l0,4);
    J(6,0) = ((3*l0 - 2*s)*Power(s,2)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
    J(6,1) = ((3*l0 - 2*s)*Power(s,2)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
    J(6,2) = 0;
    J(6,3) = -(6*(l0 - s)*s*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
    J(6,4) = -(6*s*(-l0 + s)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
    J(7,0) = (Power(s,2)*(-l0 + s)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
    J(7,1) = (Power(s,2)*(-l0 + s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
    J(7,2) = 0;
    J(7,3) = -(s*(-2*l0 + 3*s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
    J(7,4) = -((2*l0 - 3*s)*s*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
    J(8,0) = (Power(s,2)*(-3*l0 + 2*s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
    J(8,1) = ((3*l0 - 2*s)*Power(s,2)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
    J(8,2) = 0;
    J(8,3) = -(6*(l0 - s)*s*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
    J(8,4) = -(6*(l0 - s)*s*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,3);
    J(9,0) = ((l0 - s)*Power(s,2)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
    J(9,1) = (Power(s,2)*(-l0 + s)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
    J(9,2) = 0;
    J(9,3) = -(s*(-2*l0 + 3*s)*cos(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);
    J(9,4) = -(s*(-2*l0 + 3*s)*sin(a1 + ((-a1 + a2)*s)/l0))/Power(l0,2);

    return J;
  }

}

