/* Copyright (C) 2004-2015 MBSim Development Team
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
 *          rzander@users.berlios.de
 */

#include "cylindersolid_cylinderhollow_ehd.h"
#include "mbsim/contour.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/rotarymatrices.h"
#include <mbsim/mbsim_event.h>

using namespace fmatvec;
using namespace MBSim;
using namespace std;

namespace MBSimEHD {

  void ContactKinematicsEHDInterface::updateg(Vec &g, ContourPointData *cpData, int index) {
    throw MBSimError("updateg-interface is not sufficient for EHD-contact. Use updateKinematics!");
  }

  void ContactKinematicsCylinderSolidCylinderHollowEHD::assignContours(const vector<Contour*> &contour) {
    if (not (dynamic_cast<Frustum*>(contour[0]) and dynamic_cast<Frustum*>(contour[1]))) {
      throw MBSimError("Must be two frustums!");
    }

    Vec2 rs0 = static_cast<Frustum*>(contour[0])->getRadii();
    Vec2 rs1 = static_cast<Frustum*>(contour[1])->getRadii();

    if (rs0(0) < rs1(0)) {
      isolid = 0;
      ihollow = 1;
    }
    else {
      isolid = 1;
      ihollow = 0;
    }

    solid = static_cast<Frustum*>(contour[isolid]);
    hollow = static_cast<Frustum*>(contour[ihollow]);

    // Check the sizes of the radii (must be the same to be a cylinder and save them
    Vec2 rs = solid->getRadii();
    if (rs(1) - rs(0) > macheps()) {
      throw MBSimError("The two radii of the solid must match to be cylinder!");
    }
    else {
      rSolid = rs(0);
    }

    Vec2 rh = hollow->getRadii();
    if (rh(1) - rh(0) > macheps()) {
      throw MBSimError("The two radii of the hollow must match to be cylinder!");
    }
    else {
      rHollow = rh(0);
    }

    if (rSolid >= rHollow) {
      throw MBSimError("Radius of the solid must be smaller than radius of the hollow!");
    }

    //initialize values
    Vec2 h;
    h(1) = rHollow; //is h2 in [1]
    for(int i = 0; i < numberOfPotentialContactPoints; i++) {
      heightsPos.push_back(h);
      dheightsdyPos.push_back(Vec2());
    }

  }

  void ContactKinematicsCylinderSolidCylinderHollowEHD::updateKinematics(const vector<SingleContact> & contacts) {
    // See [1], Bild 4.4 but here: y-direction is height direction of cylinder and z-direction (of F-coordinate system) is the azimuthal direction

    IrC1C2 = solid->getFrame()->getPosition() - hollow->getFrame()->getPosition();
    IvC1C2 = solid->getFrame()->getVelocity() - hollow->getFrame()->getVelocity();
    omegaRel = solid->getFrame()->getAngularVelocity() - hollow->getFrame()->getAngularVelocity();

    if(nrm2(IrC1C2) > rHollow - rSolid) {
      throw MBSimError("Penetration active");
    }

    Vec3 r; //position of node in F-coordinate system

    int posCounter = 0;
    for (int i = 0; i < numberOfPotentialContactPoints; i++) {
      ContourPointData * cpData = contacts[i].getcpData();
      double y = pos(posCounter++);
      double z = pos(posCounter++);

      // Orientation
      double phi =  - y / rHollow;
      SqrMat3 AKF = BasicRotAIKy(phi);
      SqrMat3 AIF = hollow->getFrameOfReference()->getOrientation() * AKF;
      cpData[ihollow].getFrameOfReference().getOrientation().set(0, - AIF.col(0));
      cpData[ihollow].getFrameOfReference().getOrientation().set(1, hollow->getFrameOfReference()->getOrientation().col(1)); //TODO holds not for frustum, only cylinder
      cpData[ihollow].getFrameOfReference().getOrientation().set(2, crossProduct(cpData[ihollow].getFrameOfReference().getOrientation().col(0), cpData[ihollow].getFrameOfReference().getOrientation().col(1)));

      cpData[isolid].getFrameOfReference().getOrientation().set(0, -cpData[ihollow].getFrameOfReference().getOrientation().col(0));
      cpData[isolid].getFrameOfReference().getOrientation().set(1, -cpData[ihollow].getFrameOfReference().getOrientation().col(1));
      cpData[isolid].getFrameOfReference().getOrientation().set(2, cpData[ihollow].getFrameOfReference().getOrientation().col(2));

      //Position
      Vec3 e = AIF.T() * IrC1C2;

      // Here the full computation of the Thickness and the velocities should follow to store it once and then just ask for it again
      double er = e(0);
      double et = e(2);
      double r1 = sqrt(pow(rSolid, 2) - pow(et, 2));
      heightsPos[i](0) = er + r1;//is h1 in [1] (h2 is constant)
      dheightsdyPos[i](0) = et * heightsPos[i](0) / (rHollow * r1);
      //TODO: Add the velocities information as well and save it to a vector or a set!!

      r(0) = heightsPos[i](0);
      r(1) = z;

      cpData[isolid].getFrameOfReference().getPosition() = hollow->getFrameOfReference()->getPosition() + AIF * r;

      r(0) = heightsPos[i](1);
      cpData[ihollow].getFrameOfReference().getPosition() = hollow->getFrameOfReference()->getPosition() + AIF * r;
    }
  }

  void ContactKinematicsCylinderSolidCylinderHollowEHD::updatewb(Vec &wb, const Vec &g, ContourPointData *cpData) {
//    const Vec3 KrPC1 = circlehol->getFrame()->getOrientation().T() * (cpData[icirclehol].getFrameOfReference().getPosition() - circlehol->getFrame()->getPosition());
//    const double zeta1 = (KrPC1(1) > 0) ? acos(KrPC1(0) / nrm2(KrPC1)) : 2. * M_PI - acos(KrPC1(0) / nrm2(KrPC1));
//    const double sa1 = sin(zeta1);
//    const double ca1 = cos(zeta1);
//    const double r1 = circlehol->getRadius();
//    Vec3 Ks1(NONINIT);
//    Ks1(0) = -r1 * sa1;
//    Ks1(1) = r1 * ca1;
//    Ks1(2) = 0;
//    Vec3 Kt1(NONINIT);
//    Kt1(0) = 0;
//    Kt1(1) = 0;
//    Kt1(2) = 1;
//    const Vec3 s1 = circlehol->getFrame()->getOrientation() * Ks1;
//    const Vec3 t1 = circlehol->getFrame()->getOrientation() * Kt1;
//    Vec3 n1 = crossProduct(s1, t1);
//    n1 /= nrm2(n1);
//    const Vec3 u1 = s1 / nrm2(s1);
//    const Vec3 &R1 = s1;
//    Vec3 KN1(NONINIT);
//    KN1(0) = -sa1;
//    KN1(1) = ca1;
//    KN1(2) = 0;
//    const Vec3 N1 = circlehol->getFrame()->getOrientation() * KN1;
//    Vec3 KU1(NONINIT);
//    KU1(0) = -ca1;
//    KU1(1) = -sa1;
//    KU1(2) = 0;
//    const Vec3 U1 = circlehol->getFrame()->getOrientation() * KU1;
//
//    const Vec3 KrPC2 = circlesol->getFrame()->getOrientation().T() * (cpData[icirclesol].getFrameOfReference().getPosition() - circlesol->getFrame()->getPosition());
//    const double zeta2 = (KrPC2(1) > 0) ? acos(KrPC2(0) / nrm2(KrPC2)) : 2. * M_PI - acos(KrPC2(0) / nrm2(KrPC2));
//    const double sa2 = sin(zeta2);
//    const double ca2 = cos(zeta2);
//    const double r2 = circlesol->getRadius();
//    Vec3 Ks2(NONINIT);
//    Ks2(0) = -r2 * sa2;
//    Ks2(1) = r2 * ca2;
//    Ks2(2) = 0;
//    Vec3 Kt2(NONINIT);
//    Kt2(0) = 0;
//    Kt2(1) = 0;
//    Kt2(2) = 1;
//    const Vec3 s2 = circlesol->getFrame()->getOrientation() * Ks2;
//    const Vec3 t2 = circlesol->getFrame()->getOrientation() * Kt2;
//    Vec3 n2 = -crossProduct(s2, t2);
//    n2 /= nrm2(n2);
//    const Vec3 u2 = s2 / nrm2(s2);
//    const Vec3 v2 = crossProduct(n2, u2);
//    const Vec3 &R2 = s2;
//    Vec3 KU2(NONINIT);
//    KU2(0) = -ca2;
//    KU2(1) = -sa2;
//    KU2(2) = 0;
//    const Vec3 U2 = circlesol->getFrame()->getOrientation() * KU2;
//
//    const Vec3 vC1 = cpData[icirclehol].getFrameOfReference().getVelocity();
//    const Vec3 vC2 = cpData[icirclesol].getFrameOfReference().getVelocity();
//    const Vec3 Om1 = cpData[icirclehol].getFrameOfReference().getAngularVelocity();
//    const Vec3 Om2 = cpData[icirclesol].getFrameOfReference().getAngularVelocity();
//
//    SqrMat A(2, NONINIT); // TODO: change to FSqrMat
//    A(0, 0) = -(u1.T() * R1);
//    A(0, 1) = u1.T() * R2;
//    A(1, 0) = u2.T() * N1;
//    A(1, 1) = n1.T() * U2;
//    Vec b(2, NONINIT);
//    b(0) = -(u1.T() * (vC2 - vC1));
//    b(1) = -(v2.T() * (Om2 - Om1));
//    const Vec zetad = slvLU(A, b);
//
//    const Mat3x3 tOm1 = tilde(Om1);
//    const Mat3x3 tOm2 = tilde(Om2);
//
//    wb(0) += ((vC2 - vC1).T() * N1 - n1.T() * tOm1 * R1) * zetad(0) + n1.T() * tOm2 * R2 * zetad(1) - n1.T() * tOm1 * (vC2 - vC1);
//    if (wb.size() > 1)
//      wb(1) += ((vC2 - vC1).T() * U1 - u1.T() * tOm1 * R1) * zetad(0) + u1.T() * tOm2 * R2 * zetad(1) - u1.T() * tOm1 * (vC2 - vC1);
//  }
  }

  void ContactKinematicsCylinderSolidCylinderHollowEHD::Thickness(const fmatvec::VecV & x, const int & e, const int & g, double & h1, double & h2, double & h1dy, double & h2dy) {

    double y = x(0);
    if (dimLess) {
      y = y * xrF;
    }

    //compute eccentricity
    Eccentricity(y);

    // Compute distances h1 and h2
    h1 = er + r1;
    h2 = rHollow;

    // Compute derivatives of h1 and h2 with respect to y
    h1dy = et * h1 / (rHollow * r1);
    h2dy = 0;

    // Check if the film thickness is smaller than zero, meaning
    // if there is a penetration of the contacting bodies
    if (h2 < h1)
      throw MBSimError("Film thickness is smaller than zero, i.e. h < 0.");

    if (dimLess) {
      h1 = h1 / hrF;
      h2 = h2 / hrF;
      h1dy = h1dy * xrF / hrF;
      h2dy = h2dy * xrF / hrF;
    }
  }

  void ContactKinematicsCylinderSolidCylinderHollowEHD::Velocities(const fmatvec::VecV & x, const int & e, const int & g, double & u1, double & u2, double & v1, double & v2, double & v1dy, double & v2dy) {

    double y = x(0);
    if (dimLess) {
      y = y * xrF;
    }

    double phi;
    SqrMat2 AFK;
    AngleCoordSys(y, phi, AFK);

    Eccentricity(y);

    double h2;
    double void1;
    Thickness(x, void1, void1, void1, h2, void1, void1);

    // Compute derivative of second line from rotation matrix AFK
    RowVec2 AFKdphi2;
    AFKdphi2(0) = -cos(phi);
    AFKdphi2(1) = -sin(phi);

    // Compute velocities on journal surface
    // Note: K coincides with I for fixed bearing shell (phi2 = 0)
    double omega1 = nrm2(solid->getFrameOfReference()->getAngularVelocity());
    double omega2 = nrm2(hollow->getFrameOfReference()->getAngularVelocity());
    fmatvec::Vec2 IuS1;
    IuS1(0) = solid->getFrameOfReference()->getVelocity()(0);
    IuS1(1) = solid->getFrameOfReference()->getVelocity()(2);
    fmatvec::Vec2 IuS2;
    IuS2(0) = hollow->getFrameOfReference()->getVelocity()(0);
    IuS2(1) = hollow->getFrameOfReference()->getVelocity()(2);
    u1 = AFK.row(0) * IuS1 + omega1 * et;
    v1 = AFK.row(1) * IuS1 + omega1 * r1;

    // Compute velocities on inner bearing shell surface
    u2 = AFK.row(0) * IuS2;
    v2 = AFK.row(1) * IuS2 + omega2 * h2;

    // Compute derivative of tangential velocities
    v1dy = 1 / rHollow * (AFKdphi2 * IuS1 + omega1 * et * er / r1);
    v2dy = 1 / rHollow * AFKdphi2 * IuS2;

    if (dimLess) {
      v1dy = v1dy * xrF;
      v2dy = v2dy * xrF;
    }
  }

  void ContactKinematicsCylinderSolidCylinderHollowEHD::Normalvector(const fmatvec::VecV & x, fmatvec::Vec3 & n, fmatvec::Mat3x2 & t) {
    double y = x(0);
    if (dimLess) {
      y = y * xrF;
    }

    double phi;
    SqrMat2 AFK;
    AngleCoordSys(y, phi, AFK);
    n(0) = cos(phi);
    n(1) = sin(phi);

    t(0, 0) = -sin(phi);
    t(1, 0) = cos(phi);
    t(2, 1) = 1;
  }

  void ContactKinematicsCylinderSolidCylinderHollowEHD::Eccentricity(const double & y) {
    // Get rotation matrix
    double phi;
    SqrMat2 AFK;
    AngleCoordSys(y, phi, AFK);

    // Compute eccentricity in coordinate system K
    // Note: K coincides with I for fixed bearing shell (phi2 = 0)
    fmatvec::Vec2 IxS1;      //TODO: is IxS1 the center position?!
    fmatvec::Vec2 IxS2;
    IxS1(0) = solid->getFrameOfReference()->getPosition()(0);
    IxS1(1) = solid->getFrameOfReference()->getPosition()(2);
    IxS2(0) = hollow->getFrameOfReference()->getPosition()(0);
    IxS2(1) = hollow->getFrameOfReference()->getPosition()(2);

    Vec2 Ke(IxS1 - IxS2);

    // Transform eccentricity into coordinate system F
    er = AFK.row(0) * Ke;
//    cout << IxS1 << IxS2 << endl;
    et = AFK.row(1) * Ke;

    // Compute auxiliary length variable
    r1 = sqrt(pow(rSolid, 2) - pow(et, 2));
  }

  void ContactKinematicsCylinderSolidCylinderHollowEHD::AngleCoordSys(const double & y, double & phi, fmatvec::SqrMat2 & AFK) {

// Define mapping
    phi = y / rHollow;

// Build rotation matrix
    AFK(0, 0) = cos(phi);
    AFK(0, 1) = sin(phi);
    AFK(1, 0) = -sin(phi);
    AFK(1, 1) = cos(phi);
  }

  fmatvec::Vec3 ContactKinematicsCylinderSolidCylinderHollowEHD::getWrD() {
    return IrC1C2;
  }
  fmatvec::Vec3 ContactKinematicsCylinderSolidCylinderHollowEHD::getWrDdot() {
    return IvC1C2;
  }
  fmatvec::Vec3 ContactKinematicsCylinderSolidCylinderHollowEHD::getomegaRel() {
    return omegaRel;
  }
  Frustum * ContactKinematicsCylinderSolidCylinderHollowEHD::getcirclesol() {
    return solid;
  }
  Frustum * ContactKinematicsCylinderSolidCylinderHollowEHD::getcirclehol() {
    return hollow;
  }
}

