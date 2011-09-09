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

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FiniteElement1s33Cosserat::FiniteElement1s33Cosserat(double l0_, double rho_,double A_, double E_, double G_, double I1_, double I2_, double I0_, 
      const Vec& g_, int currentElement_, bool openStructure_, const Vec& relaxedElement_) : l0(l0_), rho(rho_), A(A_), E(E_), G(G_), I1(I1_), I2(I2_), I0(I0_), g(g_), currentElement(currentElement_), openStructure(openStructure_), relaxedElement(relaxedElement_), M(6,INIT,0.), h(6,INIT,0.), X(12,INIT,0.) {}

  FiniteElement1s33Cosserat::~FiniteElement1s33Cosserat() {}

  void FiniteElement1s33Cosserat::computeM(const Vec& qG ) {
    const double &be = qG(10);
    const double &ga = qG(11);
    M(0,0) = 0.5*rho*A*l0;
    M(1,1) = 0.5*rho*A*l0;
    M(2,2) = 0.5*rho*A*l0;
    M(3,3) = -rho*l0*(-pow(cos(be),2.0)*pow(cos(ga),2.0)*I1-pow(cos(be),2.0)*I2+pow(cos(be),2.0)*I2*pow(cos(ga),2.0)-I0+I0*pow(cos(be),2.0));
    M(3,4) = sin(ga)*rho*l0*cos(be)*cos(ga)*(I1-1.0*I2);
    M(3,5) = 0.1E1*rho*l0*I0*sin(be);
    M(4,3) = sin(ga)*rho*l0*cos(be)*cos(ga)*(I1-1.0*I2);
    M(4,4) = -rho*l0*(-pow(cos(ga),2.0)*I2-I1+I1*pow(cos(ga),2.0));
    M(5,3) = 0.1E1*rho*l0*I0*sin(be);
    M(5,5) = rho*l0*I0;
  }

  void FiniteElement1s33Cosserat::computeh(const Vec& qG, const Vec& qGt) {
    double Ds = l0;
    double A1 = A;
    double A2 = A;

    /*
     * x-> x-direction
     * y-> y-direction
     * z-> z-direction
     * al-> Drehung um lokale x-Achse
     * be-> Drehung um lokale y-Achse
     * ga-> Drehung um lokale z-Achse
     */
    /* DOF's 'previous' element. Index 'p'. */
    const double &xp  = qG(0);
    const double &yp  = qG(1);
    const double &zp  = qG(2);
    const double &alp = qG(3);
    const double &bep = qG(4);
    const double &gap = qG(5);

    /* DOF's element. */
    const double &x   = qG(6);
    const double &y   = qG(7);
    const double &z   = qG(8);
    const double &al  = qG(9);
    const double &be  = qG(10);
    const double &ga  = qG(11);

    /* DOF's 'following' element. Index 'f'. */
    const double &xf  = qG(12);
    const double &yf  = qG(13);
    const double &zf  = qG(14);
    const double &alf = qG(15);
    const double &bef = qG(16);
    const double &gaf = qG(17);

    /* time differentiated DOF's. additional Index 't'. */
    const double &xpt  = qGt(0);
    const double &ypt  = qGt(1);
    const double &zpt  = qGt(2);
    const double &alpt = qGt(3);
    const double &bept = qGt(4);
    const double &gapt = qGt(5);
    const double &xt   = qGt(6);
    const double &yt   = qGt(7);
    const double &zt   = qGt(8);
    const double &alt  = qGt(9);
    const double &bet  = qGt(10);
    const double &gat  = qGt(11);
    const double &xft  = qGt(12);
    const double &yft  = qGt(13);
    const double &zft  = qGt(14);
    const double &alft = qGt(15);
    const double &beft = qGt(16);
    const double &gaft = qGt(17);

    /* DOF's at relaxed stage during initialization. Additional Index '_0'. */
    const double &alp_0 = relaxedElement(3);
    const double &bep_0 = relaxedElement(4);
    const double &gap_0 = relaxedElement(5);

    const double &al_0 = relaxedElement(9);
    const double &be_0 = relaxedElement(10);
    const double &ga_0 = relaxedElement(11);
    const double &alf_0 = relaxedElement(15);
    const double &bef_0 = relaxedElement(16);
    const double &gaf_0 = relaxedElement(17);

    /*****************************************************************************************************************************/

    /* Partial derivations of 'strain and shearing energy', 'bending and torsion energy' with respect to qG */

    /* Differentiation of 'strain and shearing energy' with respect to qG */
    Vec SE1(6,INIT,0.);
    SE1(0) = 0.1E1*(cos(bep)*cos(gap)*(x-xp)/Ds+(cos(alp)*sin(gap)+sin(alp)*sin(bep)*cos(gap))*(y-yp)/Ds+(sin(alp)*sin(gap)-cos(alp)*sin(bep)*cos(gap))*( z-zp)/Ds)*G*A1*cos(bep)*cos(gap)-0.1E1*(-cos(bep)*sin(gap)*(x-xp)/Ds+(cos(alp)* cos(gap)-sin(alp)*sin(bep)*sin(gap))*(y-yp)/Ds+(sin(alp)*cos(gap)+cos(alp)*sin( bep)*sin(gap))*(z-zp)/Ds)*G*A2*cos(bep)*sin(gap)+0.1E1*(sin(bep)*(x-xp)/Ds-sin(alp)*cos(bep)*(y-yp)/Ds+cos(alp)*cos(bep)*(z-zp)/Ds-1.0)*E*A*sin(bep);
    SE1(1) = 0.1E1*(cos(bep)*cos(gap)*(x-xp)/Ds+(cos(alp)*sin(gap)+sin(alp)*sin(bep)*cos(gap))*(y-yp)/Ds+(sin(alp)*sin(gap)-cos(alp)*sin(bep)*cos(gap))*(z-zp)/Ds)*G*A1*(cos(alp)*sin(gap)+sin(alp)*sin(bep)*cos(gap))+0.1E1*(-cos(bep)* sin(gap)*(x-xp)/Ds+(cos(alp)*cos(gap)-sin(alp)*sin(bep)*sin(gap))*(y-yp)/Ds+( sin(alp)*cos(gap)+cos(alp)*sin(bep)*sin(gap))*(z-zp)/Ds)*G*A2*(cos(alp)*cos(gap)-sin(alp)*sin(bep)*sin(gap))-0.1E1*(sin(bep)*(x-xp)/Ds-sin(alp)*cos(bep)*(y-yp)/Ds+cos(alp)*cos(bep)*(z-zp)/Ds-1.0)*E*A*sin(alp)*cos(bep);
    SE1(2) = 0.1E1*(cos(bep)*cos(gap)*(x-xp)/Ds+(cos(alp)*sin(gap)+sin(alp)*sin(bep)*cos(gap))*(y-yp)/Ds+(sin(alp)*sin(gap)-cos(alp)*sin(bep)*cos(gap))*(z-zp)/Ds)*G*A1*(sin(alp)*sin(gap)-cos(alp)*sin(bep)*cos(gap))+0.1E1*(-cos(bep)*sin(gap)*(x-xp)/Ds+(cos(alp)*cos(gap)-sin(alp)*sin(bep)*sin(gap))*(y-yp)/Ds+( sin(alp)*cos(gap)+cos(alp)*sin(bep)*sin(gap))*(z-zp)/Ds)*G*A2*(sin(alp)*cos(gap)+cos(alp)*sin(bep)*sin(gap))+0.1E1*(sin(bep)*(x-xp)/Ds-sin(alp)*cos(bep)*(y-yp)/Ds+cos(alp)*cos(bep)*(z-zp)/Ds-1.0)*E*A*cos(alp)*cos(bep);

    Vec SE2(6,INIT,0.);
    SE2(0) = -0.1E1*(cos(be)*cos(ga)*(xf-x)/Ds+(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(yf-y)/Ds+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zf-z)/Ds) *G*A1*cos(be)*cos(ga)+0.1E1*(-cos(be)*sin(ga)*(xf-x)/Ds+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yf-y)/Ds+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zf-z)/ Ds)*G*A2*cos(be)*sin(ga)-0.1E1*(sin(be)*(xf-x)/Ds-sin(al)*cos(be)*(yf-y)/Ds+cos (al)*cos(be)*(zf-z)/Ds-1.0)*E*A*sin(be);
    SE2(1) = -0.1E1*(cos(be)*cos(ga)*(xf-x)/Ds+(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(yf-y)/Ds+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zf-z)/Ds) *G*A1*(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))-0.1E1*(-cos(be)*sin(ga)*(xf-x)/ Ds+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yf-y)/Ds+(sin(al)*cos(ga)+cos(al) *sin(be)*sin(ga))*(zf-z)/Ds)*G*A2*(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))+ 0.1E1*(sin(be)*(xf-x)/Ds-sin(al)*cos(be)*(yf-y)/Ds+cos(al)*cos(be)*(zf-z)/Ds -1.0)*E*A*sin(al)*cos(be);
    SE2(2) = -0.1E1*(cos(be)*cos(ga)*(xf-x)/Ds+(cos(al)*sin(ga)+sin(al)* sin(be)*cos(ga))*(yf-y)/Ds+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zf-z)/Ds) *G*A1*(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))-0.1E1*(-cos(be)*sin(ga)*(xf-x)/ Ds+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yf-y)/Ds+(sin(al)*cos(ga)+cos(al) *sin(be)*sin(ga))*(zf-z)/Ds)*G*A2*(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga)) -0.1E1*(sin(be)*(xf-x)/Ds-sin(al)*cos(be)*(yf-y)/Ds+cos(al)*cos(be)*(zf-z)/Ds -1.0)*E*A*cos(al)*cos(be);

    double MapleGenVar1 = 0.1E1*(cos(be)*cos(ga)*(xf-x)/Ds+(cos(al)*sin(ga)+sin(al)* sin(be)*cos(ga))*(yf-y)/Ds+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zf-z)/Ds) *Ds*G*A1*((-sin(al)*sin(ga)+cos(al)*sin(be)*cos(ga))*(yf-y)/Ds+(cos(al)*sin(ga) +sin(al)*sin(be)*cos(ga))*(zf-z)/Ds); 
    double MapleGenVar2 = 0.1E1*(-cos(be)*sin(ga)*(xf-x)/Ds+(cos(al)*cos(ga)-sin(al) *sin(be)*sin(ga))*(yf-y)/Ds+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zf-z)/Ds)*Ds*G*A2*((-sin(al)*cos(ga)-cos(al)*sin(be)*sin(ga))*(yf-y)/Ds+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(zf-z)/Ds)+0.1E1*(sin(be)*(xf-x)/Ds-sin(al)*cos(be)* (yf-y)/Ds+cos(al)*cos(be)*(zf-z)/Ds-1.0)*Ds*E*A*(-cos(al)*cos(be)*(yf-y)/Ds-sin (al)*cos(be)*(zf-z)/Ds);
    SE2(3) = MapleGenVar1+MapleGenVar2;

    MapleGenVar1 = 0.1E1*(cos(be)*cos(ga)*(xf-x)/Ds+(cos(al)*sin(ga)+sin(al)* sin(be)*cos(ga))*(yf-y)/Ds+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zf-z)/Ds) *Ds*G*A1*(-sin(be)*cos(ga)*(xf-x)/Ds+sin(al)*cos(be)*cos(ga)*(yf-y)/Ds-cos(al)*cos(be)*cos(ga)*(zf-z)/Ds);
    MapleGenVar2 = 0.1E1*(-cos(be)*sin(ga)*(xf-x)/Ds+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yf-y)/Ds+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zf-z)/Ds)*Ds*G*A2*(sin(be)*sin(ga)*(xf-x)/Ds-sin(al)*cos(be)*sin(ga)*(yf-y)/Ds+cos(al)* cos(be)*sin(ga)*(zf-z)/Ds)+0.1E1*(sin(be)*(xf-x)/Ds-sin(al)*cos(be)*(yf-y)/Ds+ cos(al)*cos(be)*(zf-z)/Ds-1.0)*Ds*E*A*(cos(be)*(xf-x)/Ds+sin(al)*sin(be)*(yf-y) /Ds-cos(al)*sin(be)*(zf-z)/Ds);
    SE2(4) = MapleGenVar1+MapleGenVar2;

    MapleGenVar1 = 0.1E1*(cos(be)*cos(ga)*(xf-x)/Ds+(cos(al)*sin(ga)+sin(al)* sin(be)*cos(ga))*(yf-y)/Ds+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zf-z)/Ds) *Ds*G*A1*(-cos(be)*sin(ga)*(xf-x)/Ds+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))* (yf-y)/Ds+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zf-z)/Ds);
    MapleGenVar2 = 0.1E1*(-cos(be)*sin(ga)*(xf-x)/Ds+(cos(al)*cos(ga)-sin(al) *sin(be)*sin(ga))*(yf-y)/Ds+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zf-z)/Ds)*Ds*G*A2*(-cos(be)*cos(ga)*(xf-x)/Ds+(-cos(al)*sin(ga)-sin(al)*sin(be)*cos(ga))*(yf-y)/Ds+(-sin(al)*sin(ga)+cos(al)*sin(be)*cos(ga))*(zf-z)/Ds);
    SE2(5) = MapleGenVar1+MapleGenVar2;

    /* Differentiation of 'bending and torsion energy' with respect to qG */
    Vec BT1(6,INIT,0.);
    BT1(3) = 0.5*((be-bep)/Ds*sin(ga/2.0+gap/2.0)+cos(be/2.0+bep/2.0)*cos( ga/2.0+gap/2.0)*(al-alp)/Ds)*E*I1*cos(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)-0.5*( cos(ga/2.0+gap/2.0)*(be-bep)/Ds-sin(ga/2.0+gap/2.0)*(al-alp)/Ds*cos(be/2.0+bep/2.0))*E*I2*sin(ga/2.0+gap/2.0)*cos(be/2.0+bep/2.0); 
    BT1(4) = 0.5*((be-bep)/Ds*sin(ga/2.0+gap/2.0)+cos(be/2.0+bep/2.0)*cos( ga/2.0+gap/2.0)*(al-alp)/Ds)*Ds*E*I1*(1/Ds*sin(ga/2.0+gap/2.0)-sin(be/2.0+bep/ 2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/Ds/2.0)+0.5*(cos(ga/2.0+gap/2.0)*(be-bep)/Ds- sin(ga/2.0+gap/2.0)*(al-alp)/Ds*cos(be/2.0+bep/2.0))*Ds*E*I2*(cos(ga/2.0+gap/ 2.0)/Ds+sin(ga/2.0+gap/2.0)*(al-alp)/Ds*sin(be/2.0+bep/2.0)/2.0);
    BT1(5) = 0.5*((be-bep)/Ds*sin(ga/2.0+gap/2.0)+cos(be/2.0+bep/2.0)*cos( ga/2.0+gap/2.0)*(al-alp)/Ds)*Ds*E*I1*(cos(ga/2.0+gap/2.0)*(be-bep)/Ds/2.0-sin( ga/2.0+gap/2.0)*(al-alp)/Ds*cos(be/2.0+bep/2.0)/2.0)+0.5*(cos(ga/2.0+gap/2.0)*( be-bep)/Ds-sin(ga/2.0+gap/2.0)*(al-alp)/Ds*cos(be/2.0+bep/2.0))*Ds*E*I2*(-(be- bep)/Ds*sin(ga/2.0+gap/2.0)/2.0-cos(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/Ds/2.0);

    Vec BT1_0(6,INIT,0.);
    BT1_0(3) = 0.5*((be_0-bep_0)/Ds*sin(ga_0/2.0+gap_0/2.0)+cos(be_0/2.0+bep_0/2.0)*cos( ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/Ds)*E*I1*cos(be_0/2.0+bep_0/2.0)*cos(ga_0/2.0+gap_0/2.0)-0.5*(cos(ga_0/2.0+gap_0/2.0)*(be_0-bep_0)/Ds-sin(ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/Ds*cos(be_0/2.0+bep_0/ 2.0))*E*I2*sin(ga_0/2.0+gap_0/2.0)*cos(be_0/2.0+bep_0/2.0);
    BT1_0(4) = 0.5*((be_0-bep_0)/Ds*sin(ga_0/2.0+gap_0/2.0)+ cos(be_0/2.0+bep_0/2.0)*cos(ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/Ds)*Ds*E*I1*(1/Ds*sin(ga_0/2.0+gap_0/2.0)- sin(be_0/2.0+bep_0/2.0)*cos(ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/Ds/2.0)+0.5*(cos(ga_0/2.0+gap_0/2.0)*(be_0-bep_0)/Ds-sin(ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/Ds*cos(be_0/2.0+bep_0/2.0))*Ds*E*I2*(cos(ga_0/2.0+gap_0/2.0)/Ds+ sin(ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/Ds*sin(be_0/2.0+bep_0/2.0)/2.0);
    BT1_0(5) = 0.5*((be_0-bep_0)/Ds*sin(ga_0/2.0+gap_0/2.0)+ cos(be_0/2.0+bep_0/2.0)*cos(ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/Ds)*Ds*E*I1*(cos(ga_0/2.0+gap_0/2.0)*(be_0-bep_0)/Ds/2.0- sin(ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/Ds*cos(be_0/2.0+bep_0/2.0)/2.0)+ 0.5*(cos(ga_0/2.0+gap_0/2.0)*(be_0-bep_0)/Ds-sin(ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/Ds*cos(be_0/2.0+ bep_0/2.0))*Ds*E*I2*(-(be_0-bep_0)/Ds*sin(ga_0/2.0+gap_0/2.0)/2.0-cos(be_0/2.0+bep_0/2.0)*cos(ga_0/2.0+gap_0/2.0)*(al_0-alp_0)/Ds/2.0);

    Vec BT2(6,INIT,0.);
    BT2(3) = -0.5*((bef-be)/Ds*sin(gaf/2.0+ga/2.0)+cos(bef/2.0+be/2.0)*cos (gaf/2.0+ga/2.0)*(alf-al)/Ds)*E*I1*cos(bef/2.0+be/2.0)*cos(gaf/2.0+ga/2.0)+0.5* (cos(gaf/2.0+ga/2.0)*(bef-be)/Ds-sin(gaf/2.0+ga/2.0)*(alf-al)/Ds*cos(bef/2.0+be /2.0))*E*I2*sin(gaf/2.0+ga/2.0)*cos(bef/2.0+be/2.0)-0.5*((alf-al)/Ds*sin(bef/ 2.0+be/2.0)+(gaf-ga)/Ds)*G*I0*sin(bef/2.0+be/2.0);
    BT2(4) = 0.5*((bef-be)/Ds*sin(gaf/2.0+ga/2.0)+cos(bef/2.0+be/2.0)*cos( gaf/2.0+ga/2.0)*(alf-al)/Ds)*Ds*E*I1*(-1/Ds*sin(gaf/2.0+ga/2.0)-sin(bef/2.0+be/ 2.0)*cos(gaf/2.0+ga/2.0)*(alf-al)/Ds/2.0)+0.5*(cos(gaf/2.0+ga/2.0)*(bef-be)/Ds- sin(gaf/2.0+ga/2.0)*(alf-al)/Ds*cos(bef/2.0+be/2.0))*Ds*E*I2*(-cos(gaf/2.0+ga/ 2.0)/Ds+sin(gaf/2.0+ga/2.0)*(alf-al)/Ds*sin(bef/2.0+be/2.0)/2.0)+0.25*((alf-al) /Ds*sin(bef/2.0+be/2.0)+(gaf-ga)/Ds)*G*I0*(alf-al)*cos(bef/2.0+be/2.0);
    BT2(5) = 0.5*((bef-be)/Ds*sin(gaf/2.0+ga/2.0)+cos(bef/2.0+be/2.0)*cos( gaf/2.0+ga/2.0)*(alf-al)/Ds)*Ds*E*I1*(cos(gaf/2.0+ga/2.0)*(bef-be)/Ds/2.0-sin( gaf/2.0+ga/2.0)*(alf-al)/Ds*cos(bef/2.0+be/2.0)/2.0)+0.5*(cos(gaf/2.0+ga/2.0)*( bef-be)/Ds-sin(gaf/2.0+ga/2.0)*(alf-al)/Ds*cos(bef/2.0+be/2.0))*Ds*E*I2*(-(bef- be)/Ds*sin(gaf/2.0+ga/2.0)/2.0-cos(bef/2.0+be/2.0)*cos(gaf/2.0+ga/2.0)*(alf-al) /Ds/2.0)-0.5*((alf-al)/Ds*sin(bef/2.0+be/2.0)+(gaf-ga)/Ds)*G*I0;

    Vec BT2_0(6,INIT,0.);
    BT2_0(3) = -0.5*((bef_0-be_0)/Ds*sin(gaf_0/2.0+ga_0/2.0)+cos(bef_0/2.0+be_0/2.0)*cos (gaf_0/2.0+ga_0/2.0)*(alf_0-al_0)/Ds)*E*I1*cos(bef_0/2.0+be_0/2.0)*cos(gaf_0/2.0+ga_0/2.0)+0.5*(cos(gaf_0/2.0+ga_0/2.0)*(bef_0-be_0)/Ds-sin(gaf_0/2.0+ga_0/2.0)*(alf_0-al_0)/Ds*cos(bef_0/2.0+be_0/2.0))*E*I2*sin(gaf_0/2.0+ga_0/2.0)*cos(bef_0/2.0+be_0/2.0)-0.5*((alf_0-al_0)/Ds*sin(bef_0/ 2.0+be_0/2.0)+(gaf_0-ga_0)/Ds)*G*I0*sin(bef_0/2.0+be_0/2.0);
    BT2_0(4) = 0.5*((bef_0-be_0)/Ds*sin(gaf_0/2.0+ga_0/2.0)+cos(bef_0/2.0+be_0/2.0)*cos( gaf_0/2.0+ga_0/2.0)*(alf_0-al_0)/Ds)*Ds*E*I1*(-1/Ds*sin(gaf_0/2.0+ga_0/2.0)-sin(bef_0/2.0+be_0/ 2.0)*cos(gaf_0/2.0+ga_0/2.0)*(alf_0-al_0)/Ds/2.0)+0.5*(cos(gaf_0/2.0+ga_0/2.0)*(bef_0-be_0)/Ds-sin(gaf_0/2.0+ga_0/2.0)*(alf_0-al_0)/Ds*cos(bef_0/2.0+be_0/2.0))*Ds*E*I2*(-cos(gaf_0/2.0+ga_0/ 2.0)/Ds+sin(gaf_0/2.0+ga_0/2.0)*(alf_0-al_0)/Ds*sin(bef_0/2.0+be_0/2.0)/2.0)+0.25*((alf_0-al_0) /Ds*sin(bef_0/2.0+be_0/2.0)+(gaf_0-ga_0)/Ds)*G*I0*(alf_0-al_0)*cos(bef_0/2.0+be_0/2.0);
    BT2_0(5) = 0.5*((bef_0-be_0)/Ds*sin(gaf_0/2.0+ga_0/2.0)+cos(bef_0/2.0+be_0/2.0)*cos( gaf_0/2.0+ga_0/2.0)*(alf_0-al_0)/Ds)*Ds*E*I1*(cos(gaf_0/2.0+ga_0/2.0)*(bef_0-be_0)/Ds/2.0-sin( gaf_0/2.0+ga_0/2.0)*(alf_0-al_0)/Ds*cos(bef_0/2.0+be_0/2.0)/2.0)+0.5*(cos(gaf_0/2.0+ga_0/2.0)*( bef_0-be_0)/Ds-sin(gaf_0/2.0+ga_0/2.0)*(alf_0-al_0)/Ds*cos(bef_0/2.0+be_0/2.0))*Ds*E*I2*(-(bef_0- be_0)/Ds*sin(gaf_0/2.0+ga_0/2.0)/2.0-cos(bef_0/2.0+be/2.0)*cos(gaf_0/2.0+ga_0/2.0)*(alf_0-al_0) /Ds/2.0)-0.5*((alf_0-al)/Ds*sin(bef_0/2.0+be_0/2.0)+(gaf_0-ga_0)/Ds)*G*I0;

    Vec V = SE1 + SE2 + BT1 + BT2 - BT1_0 - BT2_0;
    /*****************************************************************************************************************************/

    /*****************************************************************************************************************************/

    /* Partial derivations of dissipative strain and shearing energy with respect to qGt */

    Vec dissSE1(6,INIT,0.);
    MapleGenVar1 = 0.1E1*(-sin(bep)*bept*cos(gap)*(x-xp)/Ds-cos(bep)*sin(gap)*gapt*(x-xp)/Ds+cos(bep)*cos(gap)*(xt-xpt)/Ds+(-sin(alp)*alpt*sin(gap)+cos(alp) *cos(gap)*gapt+cos(alp)*alpt*sin(bep)*cos(gap)+sin(alp)*cos(bep)*bept*cos(gap)-sin(alp)*sin(bep)*sin(gap)*gapt)*(y-yp)/Ds+(cos(alp)*sin(gap)+sin(alp)*sin(bep)*cos(gap))*(yt-ypt)/Ds+(cos(alp)*alpt*sin(gap)+sin(alp)*cos(gap)*gapt+sin(alp)* alpt*sin(bep)*cos(gap)-cos(alp)*cos(bep)*bept*cos(gap)+cos(alp)*sin(bep)*sin(gap)*gapt)*(z-zp)/Ds+(sin(alp)*sin(gap)-cos(alp)*sin(bep)*cos(gap))*(zt-zpt)/Ds)*G*A1*cos(bep)*cos(gap);
    double MapleGenVar3 = -0.1E1*(sin(bep)*bept*sin(gap)*(x-xp)/Ds-cos(bep)*cos(gap)*gapt*(x-xp)/Ds-cos(bep)*sin(gap)*(xt-xpt)/Ds+(-sin(alp)*alpt*cos(gap)-cos(alp) *sin(gap)*gapt-cos(alp)*alpt*sin(bep)*sin(gap)-sin(alp)*cos(bep)*bept*sin(gap)- sin(alp)*sin(bep)*cos(gap)*gapt)*(y-yp)/Ds+(cos(alp)*cos(gap)-sin(alp)*sin(bep) *sin(gap))*(yt-ypt)/Ds+(cos(alp)*alpt*cos(gap)-sin(alp)*sin(gap)*gapt-sin(alp)* alpt*sin(bep)*sin(gap)+cos(alp)*cos(bep)*bept*sin(gap)+cos(alp)*sin(bep)*cos(gap)*gapt)*(z-zp)/Ds+(sin(alp)*cos(gap)+cos(alp)*sin(bep)*sin(gap))*(zt-zpt)/Ds)*G*A2*cos(bep)*sin(gap);
    double MapleGenVar4 = 0.1E1*(cos(bept)*bept*(xt-xpt)/Ds+sin(bept)*(xt-xpt)/Ds- cos(alpt)*alpt*cos(bept)*(yt-ypt)/Ds+sin(alpt)*sin(bept)*bept*(yt-ypt)/Ds-sin( alpt)*cos(bept)*(yt-ypt)/Ds-sin(alpt)*alpt*cos(bept)*(zt-zpt)/Ds-cos(alpt)*sin( bept)*bept*(zt-zpt)/Ds+cos(alpt)*cos(bept)*(zt-zpt)/Ds)*Ds*E*A*(cos(bept)*bept/ Ds+sin(bept)/Ds);
    MapleGenVar2 = MapleGenVar3+MapleGenVar4;
    dissSE1(0) = MapleGenVar1+MapleGenVar2;

    MapleGenVar2 = -0.1E1*sin(bep)*bept*cos(gap)*(x-xp)/Ds-0.1E1*cos(bep)*sin (gap)*gapt*(x-xp)/Ds+0.1E1*cos(bep)*cos(gap)*(xt-xpt)/Ds+0.1E1*(-sin(alp)*alpt* sin(gap)+cos(alp)*cos(gap)*gapt+cos(alp)*alpt*sin(bep)*cos(gap)+sin(alp)*cos( bep)*bept*cos(gap)-sin(alp)*sin(bep)*sin(gap)*gapt)*(y-yp)/Ds+0.1E1*(cos(alp)*sin(gap)+sin(alp)*sin(bep)*cos(gap))*(yt-ypt)/Ds+0.1E1*(cos(alp)*alpt*sin(gap)+ sin(alp)*cos(gap)*gapt+sin(alp)*alpt*sin(bep)*cos(gap)-cos(alp)*cos(bep)*bept* cos(gap)+cos(alp)*sin(bep)*sin(gap)*gapt)*(z-zp)/Ds+0.1E1*(sin(alp)*sin(gap)-cos(alp)*sin(bep)*cos(gap))*(zt-zpt)/Ds;
    MapleGenVar3 = G*A1*(cos(alp)*sin(gap)+sin(alp)*sin(bep)*cos(gap));
    MapleGenVar1 = MapleGenVar2*MapleGenVar3;
    MapleGenVar4 = 0.1E1*sin(bep)*bept*sin(gap)*(x-xp)/Ds-0.1E1*cos(bep)*cos( gap)*gapt*(x-xp)/Ds-0.1E1*cos(bep)*sin(gap)*(xt-xpt)/Ds+0.1E1*(-sin(alp)*alpt* cos(gap)-cos(alp)*sin(gap)*gapt-cos(alp)*alpt*sin(bep)*sin(gap)-sin(alp)*cos( bep)*bept*sin(gap)-sin(alp)*sin(bep)*cos(gap)*gapt)*(y-yp)/Ds+0.1E1*(cos(alp)* cos(gap)-sin(alp)*sin(bep)*sin(gap))*(yt-ypt)/Ds+0.1E1*(cos(alp)*alpt*cos(gap)- sin(alp)*sin(gap)*gapt-sin(alp)*alpt*sin(bep)*sin(gap)+cos(alp)*cos(bep)*bept* sin(gap)+cos(alp)*sin(bep)*cos(gap)*gapt)*(z-zp)/Ds+0.1E1*(sin(alp)*cos(gap)+cos(alp)*sin(bep)*sin(gap))*(zt-zpt)/Ds;
    double MapleGenVar5 = G*A2*(cos(alp)*cos(gap)-sin(alp)*sin(bep)*sin(gap));
    MapleGenVar3 = MapleGenVar4*MapleGenVar5;
    MapleGenVar4 = 0.1E1*(cos(bept)*bept*(xt-xpt)/Ds+sin(bept)*(xt-xpt)/Ds- cos(alpt)*alpt*cos(bept)*(yt-ypt)/Ds+sin(alpt)*sin(bept)*bept*(yt-ypt)/Ds-sin( alpt)*cos(bept)*(yt-ypt)/Ds-sin(alpt)*alpt*cos(bept)*(zt-zpt)/Ds-cos(alpt)*sin( bept)*bept*(zt-zpt)/Ds+cos(alpt)*cos(bept)*(zt-zpt)/Ds)*Ds*E*A*(-cos(alpt)*alpt *cos(bept)/Ds+sin(alpt)*sin(bept)*bept/Ds-sin(alpt)*cos(bept)/Ds);
    MapleGenVar2 = MapleGenVar3+MapleGenVar4;
    dissSE1(1) = MapleGenVar1+MapleGenVar2;

    MapleGenVar2 = -0.1E1*sin(bep)*bept*cos(gap)*(x-xp)/Ds-0.1E1*cos(bep)*sin (gap)*gapt*(x-xp)/Ds+0.1E1*cos(bep)*cos(gap)*(xt-xpt)/Ds+0.1E1*(-sin(alp)*alpt* sin(gap)+cos(alp)*cos(gap)*gapt+cos(alp)*alpt*sin(bep)*cos(gap)+sin(alp)*cos( bep)*bept*cos(gap)-sin(alp)*sin(bep)*sin(gap)*gapt)*(y-yp)/Ds+0.1E1*(cos(alp)* sin(gap)+sin(alp)*sin(bep)*cos(gap))*(yt-ypt)/Ds+0.1E1*(cos(alp)*alpt*sin(gap)+ sin(alp)*cos(gap)*gapt+sin(alp)*alpt*sin(bep)*cos(gap)-cos(alp)*cos(bep)*bept* cos(gap)+cos(alp)*sin(bep)*sin(gap)*gapt)*(z-zp)/Ds+0.1E1*(sin(alp)*sin(gap)-cos(alp)*sin(bep)*cos(gap))*(zt-zpt)/Ds;
    MapleGenVar3 = G*A1*(sin(alp)*sin(gap)-cos(alp)*sin(bep)*cos(gap));
    MapleGenVar1 = MapleGenVar2*MapleGenVar3;
    MapleGenVar4 = 0.1E1*sin(bep)*bept*sin(gap)*(x-xp)/Ds-0.1E1*cos(bep)*cos( gap)*gapt*(x-xp)/Ds-0.1E1*cos(bep)*sin(gap)*(xt-xpt)/Ds+0.1E1*(-sin(alp)*alpt* cos(gap)-cos(alp)*sin(gap)*gapt-cos(alp)*alpt*sin(bep)*sin(gap)-sin(alp)*cos( bep)*bept*sin(gap)-sin(alp)*sin(bep)*cos(gap)*gapt)*(y-yp)/Ds+0.1E1*(cos(alp)* cos(gap)-sin(alp)*sin(bep)*sin(gap))*(yt-ypt)/Ds+0.1E1*(cos(alp)*alpt*cos(gap)- sin(alp)*sin(gap)*gapt-sin(alp)*alpt*sin(bep)*sin(gap)+cos(alp)*cos(bep)*bept* sin(gap)+cos(alp)*sin(bep)*cos(gap)*gapt)*(z-zp)/Ds+0.1E1*(sin(alp)*cos(gap)+ cos(alp)*sin(bep)*sin(gap))*(zt-zpt)/Ds; MapleGenVar5 = G*A2*(sin(alp)*cos(gap)+cos(alp)*sin(bep)*sin(gap));
    MapleGenVar3 = MapleGenVar4*MapleGenVar5;
    MapleGenVar4 = 0.1E1*(cos(bept)*bept*(xt-xpt)/Ds+sin(bept)*(xt-xpt)/Ds-cos(alpt)*alpt*cos(bept)*(yt-ypt)/Ds+sin(alpt)*sin(bept)*bept*(yt-ypt)/Ds-sin( alpt)*cos(bept)*(yt-ypt)/Ds-sin(alpt)*alpt*cos(bept)*(zt-zpt)/Ds-cos(alpt)*sin( bept)*bept*(zt-zpt)/Ds+cos(alpt)*cos(bept)*(zt-zpt)/Ds)*Ds*E*A*(-sin(alpt)*alpt *cos(bept)/Ds-cos(alpt)*sin(bept)*bept/Ds+cos(alpt)*cos(bept)/Ds);
    MapleGenVar2 = MapleGenVar3+MapleGenVar4;
    dissSE1(2) = MapleGenVar1+MapleGenVar2;

    Vec dissSE2(6,INIT,0.);
    MapleGenVar1 = -0.1E1*(-sin(be)*bet*cos(ga)*(xf-x)/Ds-cos(be)*sin(ga)*gat*(xf-x)/Ds+cos(be)*cos(ga)*(xft-xt)/Ds+(-sin(al)*alt*sin(ga)+cos(al)*cos(ga)* gat+cos(al)*alt*sin(be)*cos(ga)+sin(al)*cos(be)*bet*cos(ga)-sin(al)*sin(be)*sin (ga)*gat)*(yf-y)/Ds+(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(yft-yt)/Ds+(cos(al)*alt*sin(ga)+sin(al)*cos(ga)*gat+sin(al)*alt*sin(be)*cos(ga)-cos(al)*cos(be) *bet*cos(ga)+cos(al)*sin(be)*sin(ga)*gat)*(zf-z)/Ds+(sin(al)*sin(ga)-cos(al)* sin(be)*cos(ga))*(zft-zt)/Ds)*G*A1*cos(be)*cos(ga);
    MapleGenVar3 = 0.1E1*(sin(be)*bet*sin(ga)*(xf-x)/Ds-cos(be)*cos(ga)*gat*(xf-x)/Ds-cos(be)*sin(ga)*(xft-xt)/Ds+(-sin(al)*alt*cos(ga)-cos(al)*sin(ga)*gat- cos(al)*alt*sin(be)*sin(ga)-sin(al)*cos(be)*bet*sin(ga)-sin(al)*sin(be)*cos(ga) *gat)*(yf-y)/Ds+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yft-yt)/Ds+(cos(al)* alt*cos(ga)-sin(al)*sin(ga)*gat-sin(al)*alt*sin(be)*sin(ga)+cos(al)*cos(be)*bet*sin(ga)+cos(al)*sin(be)*cos(ga)*gat)*(zf-z)/Ds+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zft-zt)/Ds)*G*A2*cos(be)*sin(ga);
    MapleGenVar4 = -0.1E1*(cos(be)*bet*(xf-x)/Ds+sin(be)*(xft-xt)/Ds-cos(al)* alt*cos(be)*(yf-y)/Ds+sin(al)*sin(be)*bet*(yft-y)/Ds-sin(al)*cos(be)*(yft-y)/Ds -sin(al)*alt*cos(be)*(zft-zt)/Ds-cos(al)*sin(be)*bet*(zft-zt)/Ds+cos(al)*cos(be)*(zft-zt)/Ds)*E*A*sin(be);
    MapleGenVar2 = MapleGenVar3+MapleGenVar4;
    dissSE2(0) = MapleGenVar1+MapleGenVar2;

    MapleGenVar1 = -0.1E1*(-sin(be)*bet*cos(ga)*(xf-x)/Ds-cos(be)*sin(ga)*gat *(xf-x)/Ds+cos(be)*cos(ga)*(xft-xt)/Ds+(-sin(al)*alt*sin(ga)+cos(al)*cos(ga)* gat+cos(al)*alt*sin(be)*cos(ga)+sin(al)*cos(be)*bet*cos(ga)-sin(al)*sin(be)*sin (ga)*gat)*(yf-y)/Ds+(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(yft-yt)/Ds+(cos( al)*alt*sin(ga)+sin(al)*cos(ga)*gat+sin(al)*alt*sin(be)*cos(ga)-cos(al)*cos(be) *bet*cos(ga)+cos(al)*sin(be)*sin(ga)*gat)*(zf-z)/Ds+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zft-zt)/Ds)*G*A1*(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga));
    MapleGenVar2 = -0.1E1*(sin(be)*bet*sin(ga)*(xf-x)/Ds-cos(be)*cos(ga)*gat* (xf-x)/Ds-cos(be)*sin(ga)*(xft-xt)/Ds+(-sin(al)*alt*cos(ga)-cos(al)*sin(ga)*gat -cos(al)*alt*sin(be)*sin(ga)-sin(al)*cos(be)*bet*sin(ga)-sin(al)*sin(be)*cos(ga)*gat)*(yf-y)/Ds+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yft-yt)/Ds+(cos(al) *alt*cos(ga)-sin(al)*sin(ga)*gat-sin(al)*alt*sin(be)*sin(ga)+cos(al)*cos(be)*bet*sin(ga)+cos(al)*sin(be)*cos(ga)*gat)*(zf-z)/Ds+(sin(al)*cos(ga)+cos(al)*sin (be)*sin(ga))*(zft-zt)/Ds)*G*A2*(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga));
    dissSE2(1) = MapleGenVar1+MapleGenVar2;

    MapleGenVar1 = -0.1E1*(-sin(be)*bet*cos(ga)*(xf-x)/Ds-cos(be)*sin(ga)*gat *(xf-x)/Ds+cos(be)*cos(ga)*(xft-xt)/Ds+(-sin(al)*alt*sin(ga)+cos(al)*cos(ga)* gat+cos(al)*alt*sin(be)*cos(ga)+sin(al)*cos(be)*bet*cos(ga)-sin(al)*sin(be)*sin (ga)*gat)*(yf-y)/Ds+(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(yft-yt)/Ds+(cos( al)*alt*sin(ga)+sin(al)*cos(ga)*gat+sin(al)*alt*sin(be)*cos(ga)-cos(al)*cos(be) *bet*cos(ga)+cos(al)*sin(be)*sin(ga)*gat)*(zf-z)/Ds+(sin(al)*sin(ga)-cos(al)* sin(be)*cos(ga))*(zft-zt)/Ds)*G*A1*(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga));
    MapleGenVar3 = -0.1E1*(sin(be)*bet*sin(ga)*(xf-x)/Ds-cos(be)*cos(ga)*gat* (xf-x)/Ds-cos(be)*sin(ga)*(xft-xt)/Ds+(-sin(al)*alt*cos(ga)-cos(al)*sin(ga)*gat -cos(al)*alt*sin(be)*sin(ga)-sin(al)*cos(be)*bet*sin(ga)-sin(al)*sin(be)*cos(ga)*gat)*(yf-y)/Ds+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yft-yt)/Ds+(cos(al) *alt*cos(ga)-sin(al)*sin(ga)*gat-sin(al)*alt*sin(be)*sin(ga)+cos(al)*cos(be)* bet*sin(ga)+cos(al)*sin(be)*cos(ga)*gat)*(zf-z)/Ds+(sin(al)*cos(ga)+cos(al)*sin (be)*sin(ga))*(zft-zt)/Ds)*G*A2*(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga));
    MapleGenVar4 = 0.1E1*(cos(be)*bet*(xf-x)/Ds+sin(be)*(xft-xt)/Ds-cos(al)* alt*cos(be)*(yf-y)/Ds+sin(al)*sin(be)*bet*(yft-y)/Ds-sin(al)*cos(be)*(yft-y)/Ds -sin(al)*alt*cos(be)*(zft-zt)/Ds-cos(al)*sin(be)*bet*(zft-zt)/Ds+cos(al)*cos(be)*(zft-zt)/Ds)*Ds*E*A*(sin(al)*alt*cos(be)/Ds+cos(al)*sin(be)*bet/Ds-cos(al)* cos(be)/Ds);
    MapleGenVar2 = MapleGenVar3+MapleGenVar4;
    dissSE2(2) = MapleGenVar1+MapleGenVar2;

    MapleGenVar2 = 0.1E1*Ds*(-sin(be)*bet*cos(ga)*(xf-x)/Ds-cos(be)*sin(ga)* gat*(xf-x)/Ds+cos(be)*cos(ga)*(xft-xt)/Ds+(-sin(al)*alt*sin(ga)+cos(al)*cos(ga) *gat+cos(al)*alt*sin(be)*cos(ga)+sin(al)*cos(be)*bet*cos(ga)-sin(al)*sin(be)* sin(ga)*gat)*(yf-y)/Ds+(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(yft-yt)/Ds+( cos(al)*alt*sin(ga)+sin(al)*cos(ga)*gat+sin(al)*alt*sin(be)*cos(ga)-cos(al)*cos (be)*bet*cos(ga)+cos(al)*sin(be)*sin(ga)*gat)*(zf-z)/Ds+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zft-zt)/Ds);
    MapleGenVar3 = G*A1*((-sin(al)*sin(ga)+cos(al)*sin(be)*cos(ga))*(yf-y)/Ds +(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(zf-z)/Ds);
    MapleGenVar1 = MapleGenVar2*MapleGenVar3;
    MapleGenVar4 = 0.1E1*Ds*(sin(be)*bet*sin(ga)*(xf-x)/Ds-cos(be)*cos(ga)* gat*(xf-x)/Ds-cos(be)*sin(ga)*(xft-xt)/Ds+(-sin(al)*alt*cos(ga)-cos(al)*sin(ga) *gat-cos(al)*alt*sin(be)*sin(ga)-sin(al)*cos(be)*bet*sin(ga)-sin(al)*sin(be)* cos(ga)*gat)*(yf-y)/Ds+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yft-yt)/Ds+( cos(al)*alt*cos(ga)-sin(al)*sin(ga)*gat-sin(al)*alt*sin(be)*sin(ga)+cos(al)*cos (be)*bet*sin(ga)+cos(al)*sin(be)*cos(ga)*gat)*(zf-z)/Ds+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zft-zt)/Ds);
    MapleGenVar5 = G*A2*((-sin(al)*cos(ga)-cos(al)*sin(be)*sin(ga))*(yf-y)/Ds +(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(zf-z)/Ds);
    MapleGenVar3 = MapleGenVar4*MapleGenVar5;
    MapleGenVar4 = 0.1E1*(cos(be)*bet*(xf-x)/Ds+sin(be)*(xft-xt)/Ds-cos(al)* alt*cos(be)*(yf-y)/Ds+sin(al)*sin(be)*bet*(yft-y)/Ds-sin(al)*cos(be)*(yft-y)/Ds -sin(al)*alt*cos(be)*(zft-zt)/Ds-cos(al)*sin(be)*bet*(zft-zt)/Ds+cos(al)*cos(be)*(zft-zt)/Ds)*Ds*E*A*(-cos(al)*cos(be)*(yf-y)/Ds-sin(al)*cos(be)*(zft-zt)/Ds);
    MapleGenVar2 = MapleGenVar3+MapleGenVar4;
    dissSE2(3) = MapleGenVar1+MapleGenVar2;

    MapleGenVar2 = 0.1E1*Ds*(-sin(be)*bet*cos(ga)*(xf-x)/Ds-cos(be)*sin(ga)* gat*(xf-x)/Ds+cos(be)*cos(ga)*(xft-xt)/Ds+(-sin(al)*alt*sin(ga)+cos(al)*cos(ga) *gat+cos(al)*alt*sin(be)*cos(ga)+sin(al)*cos(be)*bet*cos(ga)-sin(al)*sin(be)* sin(ga)*gat)*(yf-y)/Ds+(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(yft-yt)/Ds+( cos(al)*alt*sin(ga)+sin(al)*cos(ga)*gat+sin(al)*alt*sin(be)*cos(ga)-cos(al)*cos(be)*bet*cos(ga)+cos(al)*sin(be)*sin(ga)*gat)*(zf-z)/Ds+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zft-zt)/Ds);
    MapleGenVar3 = G*A1*(-sin(be)*cos(ga)*(xf-x)/Ds+sin(al)*cos(be)*cos(ga)*(yf-y)/Ds-cos(al)*cos(be)*cos(ga)*(zf-z)/Ds);
    MapleGenVar1 = MapleGenVar2*MapleGenVar3;
    MapleGenVar4 = 0.1E1*Ds*(sin(be)*bet*sin(ga)*(xf-x)/Ds-cos(be)*cos(ga)* gat*(xf-x)/Ds-cos(be)*sin(ga)*(xft-xt)/Ds+(-sin(al)*alt*cos(ga)-cos(al)*sin(ga) *gat-cos(al)*alt*sin(be)*sin(ga)-sin(al)*cos(be)*bet*sin(ga)-sin(al)*sin(be)*cos(ga)*gat)*(yf-y)/Ds+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yft-yt)/Ds+(cos(al)*alt*cos(ga)-sin(al)*sin(ga)*gat-sin(al)*alt*sin(be)*sin(ga)+cos(al)*cos(be)*bet*sin(ga)+cos(al)*sin(be)*cos(ga)*gat)*(zf-z)/Ds+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zft-zt)/Ds);
    MapleGenVar5 = G*A2*(sin(be)*sin(ga)*(xf-x)/Ds-sin(al)*cos(be)*sin(ga)*(yf-y)/Ds+cos(al)*cos(be)*sin(ga)*(zf-z)/Ds);
    MapleGenVar3 = MapleGenVar4*MapleGenVar5;
    MapleGenVar4 = 0.1E1*(cos(be)*bet*(xf-x)/Ds+sin(be)*(xft-xt)/Ds-cos(al)* alt*cos(be)*(yf-y)/Ds+sin(al)*sin(be)*bet*(yft-y)/Ds-sin(al)*cos(be)*(yft-y)/Ds-sin(al)*alt*cos(be)*(zft-zt)/Ds-cos(al)*sin(be)*bet*(zft-zt)/Ds+cos(al)*cos(be)*(zft-zt)/Ds)*Ds*E*A*(cos(be)*(xf-x)/Ds+sin(al)*sin(be)*(yft-y)/Ds-cos(al)*sin(be)*(zft-zt)/Ds);
    MapleGenVar2 = MapleGenVar3+MapleGenVar4;
    dissSE2(4) = MapleGenVar1+MapleGenVar2;

    MapleGenVar2 = 0.1E1*Ds*(-sin(be)*bet*cos(ga)*(xf-x)/Ds-cos(be)*sin(ga)* gat*(xf-x)/Ds+cos(be)*cos(ga)*(xft-xt)/Ds+(-sin(al)*alt*sin(ga)+cos(al)*cos(ga) *gat+cos(al)*alt*sin(be)*cos(ga)+sin(al)*cos(be)*bet*cos(ga)-sin(al)*sin(be)* sin(ga)*gat)*(yf-y)/Ds+(cos(al)*sin(ga)+sin(al)*sin(be)*cos(ga))*(yft-yt)/Ds+( cos(al)*alt*sin(ga)+sin(al)*cos(ga)*gat+sin(al)*alt*sin(be)*cos(ga)-cos(al)*cos (be)*bet*cos(ga)+cos(al)*sin(be)*sin(ga)*gat)*(zf-z)/Ds+(sin(al)*sin(ga)-cos(al)*sin(be)*cos(ga))*(zft-zt)/Ds);
    MapleGenVar3 = G*A1*(-cos(be)*sin(ga)*(xf-x)/Ds+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yf-y)/Ds+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zf-z)/Ds);
    MapleGenVar1 = MapleGenVar2*MapleGenVar3;
    MapleGenVar3 = 0.1E1*Ds*(sin(be)*bet*sin(ga)*(xf-x)/Ds-cos(be)*cos(ga)*gat*(xf-x)/Ds-cos(be)*sin(ga)*(xft-xt)/Ds+(-sin(al)*alt*cos(ga)-cos(al)*sin(ga)*gat-cos(al)*alt*sin(be)*sin(ga)-sin(al)*cos(be)*bet*sin(ga)-sin(al)*sin(be)*cos(ga)*gat)*(yf-y)/Ds+(cos(al)*cos(ga)-sin(al)*sin(be)*sin(ga))*(yft-yt)/Ds+(cos(al)*alt*cos(ga)-sin(al)*sin(ga)*gat-sin(al)*alt*sin(be)*sin(ga)+cos(al)*cos(be)*bet*sin(ga)+cos(al)*sin(be)*cos(ga)*gat)*(zf-z)/Ds+(sin(al)*cos(ga)+cos(al)*sin(be)*sin(ga))*(zft-zt)/Ds);
    MapleGenVar4 = G*A2*(-cos(be)*cos(ga)*(xf-x)/Ds+(-cos(al)*sin(ga)-sin(al)*sin(be)*cos(ga))*(yf-y)/Ds+(-sin(al)*sin(ga)+cos(al)*sin(be)*cos(ga))*(zf-z)/Ds);
    MapleGenVar2 = MapleGenVar3*MapleGenVar4;
    dissSE2(5) = MapleGenVar1+MapleGenVar2;

    Vec dissBT1(6,INIT,0.);
    MapleGenVar1 = 0.5*((bet-bept)*sin(ga/2.0+gap/2.0)/Ds+(be-bep)*cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)/Ds-sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/Ds-cos(be/2.0+bep/2.0)*sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)/Ds+cos(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)*(alt-alpt)/Ds)*E*I1*cos(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0);
    MapleGenVar2 = -0.5*(-sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(be-bep)/Ds+cos(ga/2.0+gap/2.0)*(bet-bept)/Ds-cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)*cos(be/2.0+bep/2.0)/Ds-sin(ga/2.0+gap/2.0)*(alt-alpt)*cos(be/2.0+bep/2.0)/Ds+sin(ga/2.0+gap/2.0)*(al-alp)*sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)/Ds)*E*I2*sin(ga/2.0+gap/2.0)*cos(be/2.0+bep/2.0)+0.5*((alt-alpt)*sin(be/2.0+bep/2.0)/Ds+(al-alp)*cos(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)/Ds+(gat-gapt)/Ds)*G*I0*sin(be/2.0+bep/2.0);
    dissBT1(3) = MapleGenVar1+MapleGenVar2;

    MapleGenVar1 = 0.5*((bet-bept)*sin(ga/2.0+gap/2.0)/Ds+(be-bep)*cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)/Ds-sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/Ds-cos(be/2.0+bep/2.0)*sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)/Ds+cos(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)*(alt-alpt)/Ds)*Ds*E*I1*(sin(ga/2.0+gap/2.0)/Ds-sin(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/Ds/2.0);
    MapleGenVar2 = 0.5*(-sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(be-bep)/Ds+cos(ga/2.0+gap/2.0)*(bet-bept)/Ds-cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)*cos(be/2.0+bep/2.0)/Ds-sin(ga/2.0+gap/2.0)*(alt-alpt)*cos(be/2.0+bep/2.0)/Ds+sin(ga/2.0+gap/2.0)*(al-alp)*sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)/Ds)*Ds*E*I2*(cos(ga/2.0+gap/2.0)/Ds+sin(ga/2.0+gap/2.0)*(al-alp)*sin(be/2.0+bep/2.0)/Ds/2.0)+0.25*((alt-alpt)*sin(be/2.0+bep/2.0)/Ds+(al-alp)*cos(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)/Ds+(gat-gapt)/Ds)*G*I0*(al-alp)*cos(be/2.0+bep/2.0);
    dissBT1(4) = MapleGenVar1+MapleGenVar2;

    MapleGenVar1 = 0.5*((bet-bept)*sin(ga/2.0+gap/2.0)/Ds+(be-bep)*cos(ga/2.0 +gap/2.0)*(gat/2.0+gapt/2.0)/Ds-sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)*cos(ga/ 2.0+gap/2.0)*(al-alp)/Ds-cos(be/2.0+bep/2.0)*sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/ 2.0)*(al-alp)/Ds+cos(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)*(alt-alpt)/Ds)*Ds*E*I1 *((be-bep)*cos(ga/2.0+gap/2.0)/Ds/2.0-cos(be/2.0+bep/2.0)*sin(ga/2.0+gap/2.0)*( al-alp)/Ds/2.0);
    MapleGenVar2 = 0.5*(-sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(be-bep)/Ds+ cos(ga/2.0+gap/2.0)*(bet-bept)/Ds-cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al- alp)*cos(be/2.0+bep/2.0)/Ds-sin(ga/2.0+gap/2.0)*(alt-alpt)*cos(be/2.0+bep/2.0)/ Ds+sin(ga/2.0+gap/2.0)*(al-alp)*sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)/Ds)*Ds*E *I2*(-sin(ga/2.0+gap/2.0)*(be-bep)/Ds/2.0-cos(ga/2.0+gap/2.0)*(al-alp)*cos(be/ 2.0+bep/2.0)/Ds/2.0)+0.5*((alt-alpt)*sin(be/2.0+bep/2.0)/Ds+(al-alp)*cos(be/2.0 +bep/2.0)*(bet/2.0+bept/2.0)/Ds+(gat-gapt)/Ds)*G*I0;
    dissBT1(5) = MapleGenVar1+MapleGenVar2;

    Vec dissBT2(6,INIT,0.);
    MapleGenVar2 = -cos(bef/2.0+be/2.0)*cos(gaf/2.0+ga/2.0)/Ds*(0.25*(bet- bept)*sin(ga/2.0+gap/2.0)/Ds+0.25*(be-bep)*cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/ 2.0)/Ds-0.25*sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/Ds-0.25*cos(be/2.0+bep/2.0)*sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)/ Ds+0.25*cos(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)*(alt-alpt)/Ds)*E*I1;
    MapleGenVar4 = 0.25*((beft-bet)*sin(gaf/2.0+ga/2.0)/Ds+(bef-be)*cos(gaf/ 2.0+ga/2.0)*(gaft/2.0+gat/2.0)/Ds-sin(bef/2.0+be/2.0)*(beft/2.0+bet/2.0)*cos( gaf/2.0+ga/2.0)*(alf-al)/Ds-cos(bef/2.0+be/2.0)*sin(gaf/2.0+ga/2.0)*(gaft/2.0+ gat/2.0)*(alf-al)/Ds+cos(bef/2.0+be/2.0)*cos(gaf/2.0+ga/2.0)*(alft-alt)/Ds)*cos (be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)/Ds*E*I1;
    MapleGenVar5 = sin(gaf/2.0+ga/2.0)*cos(bef/2.0+be/2.0)/Ds*(-0.25*sin(ga/ 2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(be-bep)/Ds+0.25*cos(ga/2.0+gap/2.0)*(bet-bept) /Ds-0.25*cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)*cos(be/2.0+bep/2.0)/Ds -0.25*sin(ga/2.0+gap/2.0)*(alt-alpt)*cos(be/2.0+bep/2.0)/Ds+0.25*sin(ga/2.0+gap /2.0)*(al-alp)*sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)/Ds)*E*I2;
    MapleGenVar3 = MapleGenVar4+MapleGenVar5;
    MapleGenVar1 = MapleGenVar2+MapleGenVar3;
    MapleGenVar2 = MapleGenVar1-0.25*(-sin(gaf/2.0+ga/2.0)*(gaft/2.0+gat/2.0)*(bef-be)/Ds+cos(gaf/2.0+ga/2.0)*(beft-bet)/Ds-cos(gaf/2.0+ga/2.0)*(gaft/2.0+ gat/2.0)*(alf-al)*cos(bef/2.0+be/2.0)/Ds-sin(gaf/2.0+ga/2.0)*(alft-alt)*cos(bef /2.0+be/2.0)/Ds+sin(gaf/2.0+ga/2.0)*(alf-al)*sin(bef/2.0+be/2.0)*(beft/2.0+bet/ 2.0)/Ds)*sin(ga/2.0+gap/2.0)*cos(be/2.0+bep/2.0)/Ds*E*I2; dissBT2(3) = MapleGenVar2-sin(bef/2.0+be/2.0)/Ds*(0.25*(alt-alpt)*sin( be/2.0+bep/2.0)/Ds+0.25*(al-alp)*cos(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)/Ds+0.25 *(gat-gapt)/Ds)*G*I0+0.25*((alft-alt)*sin(bef/2.0+be/2.0)/Ds+(alf-al)*cos(bef/ 2.0+be/2.0)*(beft/2.0+bet/2.0)/Ds+(gaft-gat)/Ds)*sin(be/2.0+bep/2.0)/Ds*G*I0; MapleGenVar2 = (-sin(gaf/2.0+ga/2.0)/Ds-sin(bef/2.0+be/2.0)*cos(gaf/2.0+ ga/2.0)*(alf-al)/Ds/2.0)*(0.25*(bet-bept)*sin(ga/2.0+gap/2.0)/Ds+0.25*(be-bep)* cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)/Ds-0.25*sin(be/2.0+bep/2.0)*(bet/2.0+ bept/2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/Ds-0.25*cos(be/2.0+bep/2.0)*sin(ga/2.0+ gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)/Ds+0.25*cos(be/2.0+bep/2.0)*cos(ga/2.0+gap /2.0)*(alt-alpt)/Ds)*E*I1;
    MapleGenVar4 = ((beft-bet)*sin(gaf/2.0+ga/2.0)/Ds+(bef-be)*cos(gaf/2.0+ga/2.0)*(gaft/2.0+gat/2.0)/Ds-sin(bef/2.0+be/2.0)*(beft/2.0+bet/2.0)*cos(gaf/2.0+ ga/2.0)*(alf-al)/Ds-cos(bef/2.0+be/2.0)*sin(gaf/2.0+ga/2.0)*(gaft/2.0+gat/2.0)* (alf-al)/Ds+cos(bef/2.0+be/2.0)*cos(gaf/2.0+ga/2.0)*(alft-alt)/Ds)*(0.25*sin(ga /2.0+gap/2.0)/Ds-0.125*sin(be/2.0+bep/2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/Ds)*E* I1;
    MapleGenVar5 = (-cos(gaf/2.0+ga/2.0)/Ds+sin(gaf/2.0+ga/2.0)*(alf-al)*sin(bef/2.0+be/2.0)/Ds/2.0)*(-0.25*sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(be-bep)/Ds+0.25*cos(ga/2.0+gap/2.0)*(bet-bept)/Ds-0.25*cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)*cos(be/2.0+bep/2.0)/Ds-0.25*sin(ga/2.0+gap/2.0)*(alt-alpt)*cos(be/2.0+bep/2.0)/Ds+0.25*sin(ga/2.0+gap/2.0)*(al-alp)*sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)/Ds)*E*I2;
    MapleGenVar3 = MapleGenVar4+MapleGenVar5;
    MapleGenVar1 = MapleGenVar2+MapleGenVar3;
    MapleGenVar2 = MapleGenVar1+(-sin(gaf/2.0+ga/2.0)*(gaft/2.0+gat/2.0)*(bef -be)/Ds+cos(gaf/2.0+ga/2.0)*(beft-bet)/Ds-cos(gaf/2.0+ga/2.0)*(gaft/2.0+gat/2.0)*(alf-al)*cos(bef/2.0+be/2.0)/Ds-sin(gaf/2.0+ga/2.0)*(alft-alt)*cos(bef/2.0+be /2.0)/Ds+sin(gaf/2.0+ga/2.0)*(alf-al)*sin(bef/2.0+be/2.0)*(beft/2.0+bet/2.0)/Ds)*(0.25*cos(ga/2.0+gap/2.0)/Ds+0.125*sin(ga/2.0+gap/2.0)*(al-alp)*sin(be/2.0+ bep/2.0)/Ds)*E*I2;
    dissBT2(4) = MapleGenVar2+(alf-al)*cos(bef/2.0+be/2.0)/Ds*(0.25*(alt- alpt)*sin(be/2.0+bep/2.0)/Ds+0.25*(al-alp)*cos(be/2.0+bep/2.0)*(bet/2.0+bept/ 2.0)/Ds+0.25*(gat-gapt)/Ds)*G*I0/2.0+0.125*((alft-alt)*sin(bef/2.0+be/2.0)/Ds+( alf-al)*cos(bef/2.0+be/2.0)*(beft/2.0+bet/2.0)/Ds+(gaft-gat)/Ds)*(al-alp)*cos( be/2.0+bep/2.0)/Ds*G*I0;

    MapleGenVar2 = ((bef-be)*cos(gaf/2.0+ga/2.0)/Ds/2.0-cos(bef/2.0+be/2.0)* sin(gaf/2.0+ga/2.0)*(alf-al)/Ds/2.0)*(0.25*(bet-bept)*sin(ga/2.0+gap/2.0)/Ds+ 0.25*(be-bep)*cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)/Ds-0.25*sin(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)*cos(ga/2.0+gap/2.0)*(al-alp)/Ds-0.25*cos(be/2.0+bep/2.0)* sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)/Ds+0.25*cos(be/2.0+bep/2.0)*cos (ga/2.0+gap/2.0)*(alt-alpt)/Ds)*E*I1;
    MapleGenVar4 = ((beft-bet)*sin(gaf/2.0+ga/2.0)/Ds+(bef-be)*cos(gaf/2.0+ga /2.0)*(gaft/2.0+gat/2.0)/Ds-sin(bef/2.0+be/2.0)*(beft/2.0+bet/2.0)*cos(gaf/2.0+ ga/2.0)*(alf-al)/Ds-cos(bef/2.0+be/2.0)*sin(gaf/2.0+ga/2.0)*(gaft/2.0+gat/2.0)* (alf-al)/Ds+cos(bef/2.0+be/2.0)*cos(gaf/2.0+ga/2.0)*(alft-alt)/Ds)*(0.125*(be- bep)*cos(ga/2.0+gap/2.0)/Ds-0.125*cos(be/2.0+bep/2.0)*sin(ga/2.0+gap/2.0)*(al- alp)/Ds)*E*I1;
    MapleGenVar5 = (-sin(gaf/2.0+ga/2.0)*(bef-be)/Ds/2.0-cos(gaf/2.0+ga/2.0)* (alf-al)*cos(bef/2.0+be/2.0)/Ds/2.0)*(-0.25*sin(ga/2.0+gap/2.0)*(gat/2.0+gapt/ 2.0)*(be-bep)/Ds+0.25*cos(ga/2.0+gap/2.0)*(bet-bept)/Ds-0.25*cos(ga/2.0+gap/2.0)*(gat/2.0+gapt/2.0)*(al-alp)*cos(be/2.0+bep/2.0)/Ds-0.25*sin(ga/2.0+gap/2.0)*( alt-alpt)*cos(be/2.0+bep/2.0)/Ds+0.25*sin(ga/2.0+gap/2.0)*(al-alp)*sin(be/2.0+ bep/2.0)*(bet/2.0+bept/2.0)/Ds)*E*I2;
    MapleGenVar3 = MapleGenVar4+MapleGenVar5;
    MapleGenVar1 = MapleGenVar2+MapleGenVar3;
    MapleGenVar2 = MapleGenVar1+(-sin(gaf/2.0+ga/2.0)*(gaft/2.0+gat/2.0)*(bef -be)/Ds+cos(gaf/2.0+ga/2.0)*(beft-bet)/Ds-cos(gaf/2.0+ga/2.0)*(gaft/2.0+gat/2.0)*(alf-al)*cos(bef/2.0+be/2.0)/Ds-sin(gaf/2.0+ga/2.0)*(alft-alt)*cos(bef/2.0+be /2.0)/Ds+sin(gaf/2.0+ga/2.0)*(alf-al)*sin(bef/2.0+be/2.0)*(beft/2.0+bet/2.0)/Ds)*(-0.125*sin(ga/2.0+gap/2.0)*(be-bep)/Ds-0.125*cos(ga/2.0+gap/2.0)*(al-alp)* cos(be/2.0+bep/2.0)/Ds)*E*I2;
    dissBT2(5) = MapleGenVar2-1/Ds*(0.25*(alt-alpt)*sin(be/2.0+bep/2.0)/Ds +0.25*(al-alp)*cos(be/2.0+bep/2.0)*(bet/2.0+bept/2.0)/Ds+0.25*(gat-gapt)/Ds)*G* I0+0.25*((alft-alt)*sin(bef/2.0+be/2.0)/Ds+(alf-al)*cos(bef/2.0+be/2.0)*(beft/ 2.0+bet/2.0)/Ds+(gaft-gat)/Ds)/Ds*G*I0; 

    Vec D = dissBT1 + dissBT1 + dissSE1 + dissSE2;
    /*****************************************************************************************************************************/

    /*****************************************************************************************************************************/

    /* Differentation of kinetic energy with respect to qG */

    Vec dTdqG(6,INIT,0.);

    dTdqG(4) = rho*Ds*alt*(-1.0*I1*sin(be)*pow(cos(ga),2.0)*cos(be)*alt-1.0 *I1*sin(be)*cos(ga)*sin(ga)*bet-1.0*I2*sin(be)*cos(be)*alt+I2*sin(be)*cos(be)* alt*pow(cos(ga),2.0)+I2*sin(be)*sin(ga)*cos(ga)*bet+I0*cos(be)*sin(be)*alt+I0* cos(be)*gat);
    dTdqG(5) = -1.0*rho*Ds*(I1*pow(cos(be),2.0)*cos(ga)*alt*alt*sin(ga)-2.0 *I1*cos(be)*pow(cos(ga),2.0)*alt*bet+I1*bet*cos(be)*alt-1.0*cos(ga)*bet*bet*I1* sin(ga)-1.0*I2*pow(cos(be),2.0)*cos(ga)*alt*alt*sin(ga)-1.0*I2*bet*cos(be)*alt+ 2.0*I2*cos(be)*pow(cos(ga),2.0)*alt*bet+cos(ga)*bet*bet*I2*sin(ga));
    /*****************************************************************************************************************************/

    /*****************************************************************************************************************************/

    /* Differentiation of kinetic energy with respect to qGt, qG */
    SqrMat dTdqGtqG(6,INIT,0.);

    dTdqGtqG(3,4)  = rho*Ds*(-2.0*sin(be)*pow(cos(ga),2.0)*alt*I1*cos(be)-1.0* I1*sin(be)*cos(ga)*sin(ga)*bet-2.0*sin(be)*alt*I2*cos(be)+2.0*sin(be)*alt*I2* cos(be)*pow(cos(ga),2.0)+I2*sin(be)*sin(ga)*cos(ga)*bet+2.0*cos(be)*alt*I0*sin( be)+I0*cos(be)*gat);
    dTdqGtqG(3,5) = rho*Ds*(-2.0*sin(be)*pow(cos(ga),2.0)*alt*I1*cos(be)-1.0* I1*sin(be)*cos(ga)*sin(ga)*bet-2.0*sin(be)*alt*I2*cos(be)+2.0*sin(be)*alt*I2* cos(be)*pow(cos(ga),2.0)+I2*sin(be)*sin(ga)*cos(ga)*bet+2.0*cos(be)*alt*I0*sin( be)+I0*cos(be)*gat);

    dTdqGtqG(4,4) = -1.0*sin(be)*cos(ga)*alt*rho*Ds*sin(ga)*(I1-1.0*I2);
    dTdqGtqG(4,5) = -1.0*sin(be)*cos(ga)*alt*rho*Ds*sin(ga)*(I1-1.0*I2);

    dTdqGtqG(5,4) = 0.1E1*rho*Ds*cos(be)*alt*I0;
    /*****************************************************************************************************************************/

    /* Vec of generalized forces */
    h = V - D - dTdqG + dTdqGtqG*qGt(6,11);
  }

  double FiniteElement1s33Cosserat::computeKineticEnergy(const Vec& qG, const Vec& qGt) {
    int k;
    if(openStructure == true && currentElement == 0)
      k = 0;
    else
      k = 3;

    return rho * A * l0 * (0.25 * pow(nrm2(qGt(6 + k, 8 + k)), 2) + 0.5 * (pow( (I1 * (cos(qG(7 + k)) * cos(qG(8 + k)) * qGt(6 + k) + sin(qG(8 + k)) * qGt(7 + k))), 2) + pow((I2 * (-cos(qG(7 + k)) * sin(qG(8 + k)) * qGt(6 + k) + cos(qG(8 + k) * qGt(7 + k)))), 2) + pow((I0 * (qGt(8 + k) + sin(qG(7 + k)) * qGt(6 + k))), 2)));
  }

  double FiniteElement1s33Cosserat::computeGravitationalEnergy(const Vec& qG) {
    int j;
    if(currentElement == 0 && openStructure == true)
      j = 0;
    else
      j = 3;
    return -rho * A * g(1) * qG(5 + j);
  }

  const Vec& FiniteElement1s33Cosserat::computeState(const Vec& qG, const Vec& qGt,double x) {
    int j;
    if(currentElement == 0 && openStructure == true)
      j = 0;
    else
      j = 3;

    X(0, 2) = qG(3 + j, 5 + j) + x * (qG(9 + j, 11 + j) - qG(3 + j, 5 + j)) / l0; // position
    X(6, 8) = qGt(3 + j, 5 + j) + x * ((qGt(9 + j, 11 + j) - qGt(3 + j, 5 + j)) / l0); // velocity

    if(x >= l0/2.) {
      X(3, 5) = qG(6 + j, 8 + j) + (x - 0.5 * l0) * (qG(12 + j, 14 + j) - qG(6 + j, 8 + j)) / l0; // angles
      X(9, 11) = qGt(6 + j, 8 + j) + (x - 0.5 * l0) * (qGt(12 + j, 14 + j) - qGt(6 + j, 8 + j)) / l0; // time differentiated angels
    }
    else if(x < l0/2.) {
      X(3, 5) = qG(j, 2 + j) + (x + 0.5 * l0) * (qG(6 + j, 8 + j) - qG(j, 2 + j)) / l0; // angles
      X(9, 11) = qGt(j, 2 + j) + (x + 0.5 * l0) * (qGt(6 + j, 8 + j) - qGt(j, 2 + j)) / l0; // time differentiated angels
    }

    return X;
  }

  Vec FiniteElement1s33Cosserat::computeData(const Vec& qG,const Vec& qGt) {
    Vec Data(6);
    Data(0,2) = qG(0,2);
    Data(3,5) = qG(3,5);

    return Data.copy();
  }

}
