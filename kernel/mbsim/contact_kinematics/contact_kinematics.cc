/* Copyright (C) 2004-2020 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h> 
#include "mbsim/contact_kinematics/contact_kinematics.h"
#include "mbsim/contours/contour.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/functions/function.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematics::updateg(vector<SingleContact> &contact) {
    search();
    for(int i=0; i<maxNumContacts; i++)
      updateg(contact[i],i);
  }

  void ContactKinematics::updatewb(SingleContact &contact, int i) {

    const Vec3 u2 = contact.getContourFrame(1)->evalOrientation().col(1);
    const Vec3 v2 = contact.getContourFrame(1)->getOrientation().col(2);
    const Mat3x2 R2 = contour[1]->evalWR(contact.getContourFrame(1)->getZeta());
    const Mat3x2 U2 = contour[1]->evalWU(contact.getContourFrame(1)->getZeta());
    const Mat3x2 V2 = contour[1]->evalWV(contact.getContourFrame(1)->getZeta());
    const Vec3 paruPart2 = contour[1]->evalParWuPart(contact.getContourFrame(1)->getZeta());
    const Vec3 parvPart2 = contour[1]->evalParWvPart(contact.getContourFrame(1)->getZeta());
    const Mat3x2 parWvCParZeta2 = contour[1]->evalParWvCParZeta(contact.getContourFrame(1)->getZeta());

    const Vec3 u1 = contact.getContourFrame(0)->evalOrientation().col(1);
    const Vec3 v1 = contact.getContourFrame(0)->getOrientation().col(2);
    const Vec3 n1 = contact.getContourFrame(0)->evalOrientation().col(0);
    const Mat3x2 R1 = contour[0]->evalWR(contact.getContourFrame(0)->getZeta());
    const Mat3x2 U1 = contour[0]->evalWU(contact.getContourFrame(0)->getZeta());
    const Mat3x2 V1 = contour[0]->evalWV(contact.getContourFrame(0)->getZeta());
    const Mat3x2 N1 = contour[0]->evalWN(contact.getContourFrame(0)->getZeta());
    const Vec3 paruPart1 = contour[0]->evalParWuPart(contact.getContourFrame(0)->getZeta());
    const Vec3 parvPart1 = contour[0]->evalParWvPart(contact.getContourFrame(0)->getZeta());
    const Vec3 parnPart1 = contour[0]->evalParWnPart(contact.getContourFrame(0)->getZeta());
    const Mat3x2 parWvCParZeta1 = contour[0]->evalParWvCParZeta(contact.getContourFrame(0)->getZeta());

    const Vec3 vC2 = contact.getContourFrame(1)->evalVelocity();
    const Vec3 vC1 = contact.getContourFrame(0)->evalVelocity();

    SqrMat A(4,NONINIT);
    A.set(RangeV(0,0),RangeV(0,1), -u1.T()*R1);
    A.set(RangeV(0,0),RangeV(2,3), u1.T()*R2);
    A.set(RangeV(1,1),RangeV(0,1), -v1.T()*R1);
    A.set(RangeV(1,1),RangeV(2,3), v1.T()*R2);
    A.set(RangeV(2,2),RangeV(0,1), u2.T()*N1);
    A.set(RangeV(2,2),RangeV(2,3), n1.T()*U2);
    A.set(RangeV(3,3),RangeV(0,1), v2.T()*N1);
    A.set(RangeV(3,3),RangeV(2,3), n1.T()*V2);

    Vec b(4,NONINIT);
    b(0) = -u1.T()*(vC2-vC1);
    b(1) = -v1.T()*(vC2-vC1);
    b(2) = -u2.T()*parnPart1-n1.T()*paruPart2;
    b(3) = -v2.T()*parnPart1-n1.T()*parvPart2;
    Vec zetad =  slvLU(A,b);
    Vec zetad1 = zetad(RangeV(0,1));
    Vec zetad2 = zetad(RangeV(2,3));

    if(contact.isNormalForceLawSetValued())
      contact.getwb(false)(0) += (N1*zetad1+parnPart1).T()*(vC2-vC1)+n1.T()*(parWvCParZeta2*zetad2-parWvCParZeta1*zetad1);
    if(contact.isTangentialForceLawSetValuedAndActive()) {
      contact.getwb(false)(contact.isNormalForceLawSetValued()) += (U1*zetad1+paruPart1).T()*(vC2-vC1)+u1.T()*(parWvCParZeta2*zetad2-parWvCParZeta1*zetad1);
      if(contact.getFrictionDirections()>1)
	contact.getwb(false)(contact.isNormalForceLawSetValued()+1) += (V1*zetad1+parvPart1).T()*(vC2-vC1)+v1.T()*(parWvCParZeta2*zetad2-parWvCParZeta1*zetad1);
    }
  }

  vector<double> ContactKinematics::searchPossibleContactPoints(Function<double(double)> *func, double eta, const vector<double> &nodes, double tol) {
    vector<double> zetai;
    for(size_t j=0; j<nodes.size()-1; j++) {
      eta = nodes[j];
      double fa = (*func)(eta);
      eta = nodes[j+1];
      double fb = (*func)(eta);
      if(fa*fb < 0)
	zetai.push_back((nodes[j]+nodes[j+1])/2.);
      else if(fabs(fa) < tol)
	zetai.push_back(nodes[j]);
      else if(fabs(fb) < tol)
	zetai.push_back(nodes[j+1]);
    }
    return zetai;
  }

  vector<double> ContactKinematics::searchPossibleContactPoints(Function<Vec(Vec)> *func, int i, Vec &zeta, const vector<double> &nodes, double tol) {
    vector<double> zetai;
    for(size_t j=0; j<nodes.size()-1; j++) {
      zeta(i) = nodes[j];
      double fa = (*func)(zeta)(i);
      zeta(i) = nodes[j+1];
      double fb = (*func)(zeta)(i);
      if(fa*fb < 0)
	zetai.push_back((nodes[j]+nodes[j+1])/2.);
      else if(fabs(fa) < tol)
	zetai.push_back(nodes[j]);
      else if(fabs(fb) < tol)
	zetai.push_back(nodes[j+1]);
    }
    return zetai;
  }

}
