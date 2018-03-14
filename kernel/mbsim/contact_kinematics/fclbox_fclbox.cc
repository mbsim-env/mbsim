/* Copyright (C) 2004-2018 MBSim Development Team
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
#include "fclbox_fclbox.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/fcl_box.h"
#include "fcl/narrowphase/distance.h"

using namespace fmatvec;
using namespace std;
using namespace fcl;

namespace MBSim {

  Vector3d Vec3ToVector3d(const Vec3 &x) {
    Vector3d y;
    for(int i=0; i<3; i++)
      y(i) = x(i);
    return y;
  }

  Matrix3d SqrMat3ToMatrix3d(const SqrMat3 &A) {
    Matrix3d B;
    for(int i=0; i<3; i++)
      for(int j=0; j<3; j++)
        B(i,j) = A(i,j);
    return B;
  }

  Vec3 Vector3dToVec3(const Vector3d &x) {
    Vec3 y;
    for(int i=0; i<3; i++)
      y(i) = x(i);
    return y;
  }

  SqrMat3 Matrix3dToSqrMat3(const Matrix3d &A) {
    SqrMat3 B;
    for(int i=0; i<3; i++)
      for(int j=0; j<3; j++)
        B(i,j) = A(i,j);
    return B;
  }

  void ContactKinematicsFCLBoxFCLBox::assignContours(const vector<Contour*> &contour) {
    ibox0 = 0; ibox1 = 1;
    box0 = static_cast<FCLBox*>(contour[0]);
    box1 = static_cast<FCLBox*>(contour[1]);
    obj0 = shared_ptr<CollisionObject<double> >(new CollisionObject<double>(box0->getCollisionGeometry()));
    obj1 = shared_ptr<CollisionObject<double> >(new CollisionObject<double>(box1->getCollisionGeometry()));
  }

  bool ContactKinematicsFCLBoxFCLBox::updateg(vector<SingleContact> &contact) {
    obj0->setTranslation(Vec3ToVector3d(box0->getFrame()->evalPosition()));
    obj0->setRotation(SqrMat3ToMatrix3d(box0->getFrame()->getOrientation()));
    obj1->setTranslation(Vec3ToVector3d(box1->getFrame()->evalPosition()));
    obj1->setRotation(SqrMat3ToMatrix3d(box1->getFrame()->getOrientation()));

    CollisionRequest<double> request(maxNumContacts,true);
    CollisionResult<double> result;
    collide<double>(obj0.get(), obj1.get(), request, result);
//    cout << result.isCollision() << endl;
//    cout << result.numContacts() << endl;
    if(result.isCollision()) {
//      cout << "numContacts = " << result.numContacts() << endl;
      for(size_t i=0; i<result.numContacts(); i++) {
      Vec3 n = Vector3dToVec3(result.getContact(i).normal);
      Vec3 r = Vector3dToVec3(result.getContact(i).pos);
      double g = -result.getContact(i).penetration_depth;
      Vec3 t;
      t(0) = 1;
      Vec3 t1, t2;
      if(fabs(t.T()*n) > 0.9) {
        t(0) = 0;
        t(1) = 1;
      }
      t1 = crossProduct(n,t);
      t2 = crossProduct(n,t1);
      contact[i].getGeneralizedRelativePosition(false)(0) = g;
      contact[i].getContourFrame(ibox0)->getOrientation(false).set(0, n);
      contact[i].getContourFrame(ibox0)->getOrientation(false).set(1, t1);
      contact[i].getContourFrame(ibox0)->getOrientation(false).set(2, t2);
      contact[i].getContourFrame(ibox1)->getOrientation(false).set(0, -n);
      contact[i].getContourFrame(ibox1)->getOrientation(false).set(1, -t1);
      contact[i].getContourFrame(ibox1)->getOrientation(false).set(2, t2);
      contact[i].getContourFrame(ibox0)->setPosition(r + n*g/2.);
      contact[i].getContourFrame(ibox1)->setPosition(r - n*g/2.);
      }
      for(size_t i=result.numContacts(); i<maxNumContacts; i++) {
        contact[i].getGeneralizedRelativePosition(false)(0) = 1;
        contact[i].getContourFrame(ibox0)->setPosition(box0->getFrame()->getPosition());
        contact[i].getContourFrame(ibox0)->setOrientation(box0->getFrame()->getOrientation());
        contact[i].getContourFrame(ibox1)->setPosition(box1->getFrame()->getPosition());
        contact[i].getContourFrame(ibox1)->setOrientation(box1->getFrame()->getOrientation());
      }
      return true;
    }
    else {
//      DistanceRequest<double> request;
//      DistanceResult<double> result;
//      distance<double>(obj0.get(), obj1.get(), request, result);
//      g = result.min_distance;
      for(size_t i=0; i<maxNumContacts; i++) {
      contact[i].getGeneralizedRelativePosition(false)(0) = 1;
      contact[i].getContourFrame(ibox0)->setPosition(box0->getFrame()->getPosition());
      contact[i].getContourFrame(ibox0)->setOrientation(box0->getFrame()->getOrientation());
      contact[i].getContourFrame(ibox1)->setPosition(box1->getFrame()->getPosition());
      contact[i].getContourFrame(ibox1)->setOrientation(box1->getFrame()->getOrientation());
      }
      return false;
    }
  }

  void ContactKinematicsFCLBoxFCLBox::updateg(double &g, std::vector<ContourFrame*> &cFrame, int index) {
    obj0->setTranslation(Vec3ToVector3d(box0->getFrame()->evalPosition()));
    obj0->setRotation(SqrMat3ToMatrix3d(box0->getFrame()->getOrientation()));
    obj1->setTranslation(Vec3ToVector3d(box1->getFrame()->evalPosition()));
    obj1->setRotation(SqrMat3ToMatrix3d(box1->getFrame()->getOrientation()));

    CollisionRequest<double> request(1,true);
    CollisionResult<double> result;
    collide<double>(obj0.get(), obj1.get(), request, result);
    cout << result.isCollision() << endl;
    if(result.isCollision()) {
      Vec3 n = Vector3dToVec3(result.getContact(0).normal);
      Vec3 r = Vector3dToVec3(result.getContact(0).pos);
      g = -result.getContact(0).penetration_depth;
      Vec3 t;
      t(0) = 1;
      Vec3 t1, t2;
      if(fabs(t.T()*n) > 0.9) {
        t(0) = 0;
        t(1) = 1;
      }
      t1 = crossProduct(n,t);
      t2 = crossProduct(n,t1);
      cFrame[ibox0]->getOrientation(false).set(0, n);
      cFrame[ibox0]->getOrientation(false).set(1, t1);
      cFrame[ibox0]->getOrientation(false).set(2, t2);
      cFrame[ibox1]->getOrientation(false).set(0, -n);
      cFrame[ibox0]->getOrientation(false).set(1, -t1);
      cFrame[ibox0]->getOrientation(false).set(2, t2);
      cFrame[ibox0]->setPosition(r + n*g/2.);
      cFrame[ibox1]->setPosition(r - n*g/2.);
    }
    else {
//      DistanceRequest<double> request;
//      DistanceResult<double> result;
//      distance<double>(obj0.get(), obj1.get(), request, result);
//      g = result.min_distance;
      g = 1;
      cFrame[ibox0]->setPosition(box0->getFrame()->getPosition());
      cFrame[ibox0]->setOrientation(box0->getFrame()->getOrientation());
      cFrame[ibox1]->setPosition(box1->getFrame()->getPosition());
      cFrame[ibox1]->setOrientation(box1->getFrame()->getOrientation());
    }

  }

}
