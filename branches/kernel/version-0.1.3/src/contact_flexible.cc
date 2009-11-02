/* Copyright (C) 2004-2006  Martin Förg

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
 * Contact:
 *   mfoerg@users.berlios.de
 *
 */
#include<config.h>
#define FMATVEC_NO_BOUNDS_CHECK
#include "contact_flexible.h"
#include "multi_body_system.h"
#include "eps.h"

namespace MBSim {

  ContactFlexible::ContactFlexible(const string &name) : Contact(name,false), DeleteDIB_c(true), DeleteDIB_d(true), DeleteDIB_V(true), gdT_grenz(0.1), flag_c(false) {
    V_fun = new FuncConst(Vec(1,INIT,0.));
    c_fun = new FuncConst(Vec(1,INIT,0.));
    d_fun = new FuncConst(Vec(1,INIT,0.));
    active = false;
  }

  ContactFlexible::ContactFlexible(const ContactFlexible *master,const string &name_) : Contact(master,name_), DeleteDIB_c(false), DeleteDIB_d(false), DeleteDIB_V(false) {
    V_fun = master->V_fun;
    c_fun = master->c_fun;
    d_fun = master->d_fun;
    flag_c = master->flag_c;
    gdT_grenz = master->gdT_grenz;
    active = false;
  }

  ContactFlexible::~ContactFlexible() {
    if(DeleteDIB_c) delete c_fun;
    if(DeleteDIB_d) delete d_fun; 
    if(DeleteDIB_V) delete V_fun; 
  }

  void ContactFlexible::setStiffness(double c_) {
    if(DeleteDIB_c) delete c_fun; 
    c_fun = new FuncLinear(Vec(1,INIT,-c_),Vec(1,INIT,0.)); 
    DeleteDIB_c = true;
    if (DeleteDIB_V) delete V_fun; 
    V_fun = new FuncQuadratic(Vec(1,INIT,0.5*c_),Vec(1,INIT,0.),Vec(1,INIT,0.)); 
    flag_c = true;
    DeleteDIB_V = true;
  }
  void ContactFlexible::setStiffnessFunction(DataInterfaceBase *c_fun_, bool DeleteDIB_c_) {
    if(DeleteDIB_c) delete c_fun; 
    c_fun = c_fun_;
    DeleteDIB_c = DeleteDIB_c_;
    flag_c = true;
  }

  void ContactFlexible::setPotential(DataInterfaceBase *V_fun_, bool DeleteDIB_V_) {
    if(DeleteDIB_V) delete V_fun; 
    V_fun = V_fun_;
    DeleteDIB_V = DeleteDIB_V_;
  }

  void ContactFlexible::setDamping(double d_) {
    if(DeleteDIB_d) delete d_fun; 
    d_fun = new FuncLinear(Vec(1,INIT,-d_),Vec(1,INIT,0.));
    DeleteDIB_d = true;
  }


  void ContactFlexible::setDampingFunction(DataInterfaceBase *d_fun_, bool DeleteDIB_d_) {
    if(DeleteDIB_d) delete d_fun;
    d_fun = d_fun_;
    DeleteDIB_d = DeleteDIB_d_;
  }

  void ContactFlexible::init() {
    Contact::init();
    load.clear();
    for(int i=0; i<2 ; i++) {
      load.push_back(Vec(6));
      WF[i] >> load[i](Index(0,2));
      WM[i] >> load[i](Index(3,5));
    }
  }

  void ContactFlexible::updateKinetics(double t) {
    static const double eps = epsroot();
    if(flag_c)
      if(gd(0)<0) la(0) = ((*c_fun)(g(0)))(0) + ((*d_fun)(gd(0)))(0); // normal direction
      else la(0) = ((*c_fun)(g(0)))(0);
    else
      if(gd(0)<0) la(0) = (((*V_fun)(g(0)))(0)-((*V_fun)(g(0)+eps))(0))/eps + ((*d_fun)(gd(0)))(0); // normal direction
      else la(0) = (((*V_fun)(g(0)))(0)-((*V_fun)(g(0)+eps))(0))/eps;


      if(nFric == 1) { // tangential directions
	if(fabs(gd(1)) < gdT_grenz) la(1) = -la(0)*((*mue_fun)(gdT_grenz))(0)*gd(1)/gdT_grenz;
	else la(1) = gd(1)>0?-la(0)*((*mue_fun)(gd(1)))(0):la(0)*((*mue_fun)(-gd(1)))(0);
      }
      else if(nFric == 2) {
	double norm_gdT = nrm2(gd(1,2));
	if(norm_gdT < gdT_grenz) la(1,2) = gd(1,2)*(-la(0)*((*mue_fun)(gdT_grenz))(0)/gdT_grenz);
	else la(1,2) = gd(1,2)*(-la(0)*((*mue_fun)(norm_gdT))(0)/norm_gdT);
      }

      WF[0] = getContourPointData(0).Wn*la(0) + getContourPointData(0).Wt*la(iT);
      WF[1] = -WF[0];
  }

  double ContactFlexible::computePotentialEnergy() {
    if(g(0)<0.) return ((*V_fun)(g(0)))(0);
    else return 0.;
  }

}
