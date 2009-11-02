/* Copyright (C) 2006  Mathias Bachmayer
 
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
 *
 * Contact:
 *   mbachmayer@gmx.de
 *
 */ 
#include <config.h>
#include "mectosignal.h"
#include "body_rigid.h"
#include "body_rigid_constrained.h"

Vec PosInterface::operator()(double t)  { 
  return trans(JT)*port->getWrOP();
}
Vec RotVelInterface::operator()(double t)  { 
  return trans(JR)*port->getWomegaP();
}
AngularInterface::AngularInterface(Body *body_) {
  body=body_;
  index=-1;
  BodyRigid* b11=dynamic_cast<BodyRigid*>(body);
  BodyRigidConstrainedVel* b12=dynamic_cast<BodyRigidConstrainedVel*>(body); 
  BodyRigidConstrainedAcc* b13=dynamic_cast<BodyRigidConstrainedAcc*>(body); 
  BodyRigid* b21=dynamic_cast<BodyRigid*>(body);
  if(b11||b12||b13) index=0; 
  if(b21) index=2;
  if(index==-1) {
    cout << "AngularInterface: Error: AngularInterface for this Type of Body not possible." << endl; 
    throw 50;
  }
}
Vec AngularInterface::operator()(double t)  { 
  return Vec(1,INIT,body->getq()(index));
}  

