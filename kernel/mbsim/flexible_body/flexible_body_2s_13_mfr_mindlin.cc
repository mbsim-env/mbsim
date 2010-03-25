/* Copyright (C) 2004-2009 MBSim Development Team
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

#define FMATVEC_DEEP_COPY
#define FMATVEC_NO_INITIALIZATION
#define FMATVEC_NO_BOUNDS_CHECK

#include "mbsim/flexible_body/flexible_body_2s_13_mfr_mindlin.h"
#include "mbsim/contours/nurbs_disk_2s_mfr_mindlin.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/utils/eps.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/nurbsdisk.h"
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Mat condenseMatrixRows_cd(Mat A, Index I) {
    Mat B(A.rows() - (I.end() - I.start() + 1), A.cols());
    Index upperPart(0, I.start() - 1);
    Index lowerPartA(I.end() + 1, A.rows() - 1);
    Index lowerPartB(I.start(), B.rows() - 1);
    Index AllCols(0, A.cols() - 1);

    B(upperPart, AllCols) = A(upperPart, AllCols); // upper
    B(lowerPartB, AllCols) = A(lowerPartA, AllCols); // lower
    return B;
  }

  Mat condenseMatrixCols_cd(Mat A, Index I) {
    Mat B(A.rows(), A.cols() - (I.end() - I.start() + 1));
    Index leftPart(0, I.start() - 1);
    Index rightPartA(I.end() + 1, A.cols() - 1);
    Index rightPartB(I.start(), B.cols() - 1);
    Index AllRows(0, A.rows() - 1);

    B(AllRows, leftPart) = A(AllRows, leftPart); // upper
    B(AllRows, rightPartB) = A(AllRows, rightPartA); // lower
    return B;
  }

  SymMat condenseMatrix_cd(SymMat A, Index I) {
    // build size of result matrix
    SymMat B(A.size() - (I.end() - I.start() + 1));
    Index upperPart(0, I.start() - 1);
    Index lowerPartA(I.end() + 1, A.size() - 1);
    Index lowerPartB(I.start(), B.size() - 1);

    // assemble result matrix
    B(upperPart) << A(upperPart); // upper left
    B(upperPart, lowerPartB) << A(upperPart, lowerPartA); // upper right
    B(lowerPartB) << A(lowerPartA); // lower right
    return B;
  }

  double ArcTan_cd(double x, double y) {
    double phi;
    phi = atan2(y, x);

    if (phi < 0.)
      phi += 2 * M_PI;
    if (phi >= 2 * M_PI) // NECESSARY ? (TS)
      phi -= 2 * M_PI;
    return phi;
  }

  FlexibleBody2s13MFRMindlin::FlexibleBody2s13MFRMindlin(const string &name) : FlexibleBodyContinuum<Vec> (name), Elements(0), NodeDofs(3), RefDofs(6), E(0.), nu(0.), rho(0.), d(3, INIT, 0.), Ri(0), Ra(0), dr(0), dj(0), m0(0), J0(3,INIT,0.), degV(3), degU(3), drawDegree(0), currentElement(0), nr(0), nj(0), Nodes(0), Dofs(0), LType(innerring), A(3, EYE), G(3, EYE) {
#ifdef HAVE_NURBS
    contour = new NurbsDisk2sMFRMindlin("SurfaceContour");
    Body::addContour(contour);
#else
    cout << "WARNING (FlexibleBody2s13MFRMindlin::FlexibleBody2s13MFRMindlin): No NURBS library installed!" << endl;
#endif

    // frame in axis
    Vec s(2, fmatvec::INIT, 0.);
    addFrame("COG", s);
  }

  void FlexibleBody2s13MFRMindlin::updateh(double t) {
    // update positions and velocities
    qext = Jext * q;
    uext = Jext * u;

    h = -K * q;
    hObject = -K * q;
  }

  void FlexibleBody2s13MFRMindlin::updateM(double t) {
    SymMat Mext(Dofs, INIT, 0.);
    Vec qf = qext(RefDofs,Dofs-1).copy();

    /*M_RR is constant*/

    /*M_RTheta*/
    Vec u(3,INIT,0.);
    u = N_compl * qf;

    SqrMat u_tyl(3,INIT,0.);
    u_tyl(0,0) = 0;
    u_tyl(0,1) = -(u(2)+R_compl(2));
    u_tyl(0,2) =  (u(1)+R_compl(1));

    u_tyl(1,0) = (u(2)+R_compl(2));
    u_tyl(1,1) = 0;
    u_tyl(1,2) = -(u(0)+R_compl(0));

    u_tyl(2,0) = -(u(1)+R_compl(1));
    u_tyl(2,1) =  (u(0)+R_compl(0));
    u_tyl(2,2) = 0;

    SqrMat M_RTheta = -A*u_tyl*G;

    /*M_RF*/
    Mat M_RF = A*N_compl;

    /*M_ThetaTheta*/
    SymMat I(3,INIT,0.);

    //TODO: proof signes
    I(0,0) =  R_ij(1,1)+R_ij(2,2) +   2*(NR_ij[1][1]+NR_ij[2][2]) *qf + qf.T() * (N_ij[1][1]+N_ij[2][2]) * qf;
    I(0,1) =-(R_ij(0,1)           +     (NR_ij[1][0]+NR_ij[0][1]) *qf + qf.T() * (N_ij[1][0]+N_ij[0][1]) * qf);
    I(0,2) = (R_ij(0,2)           +     (NR_ij[2][0]+NR_ij[0][2]) *qf + qf.T() * (N_ij[2][0]+N_ij[0][2]) * qf);
    I(1,1) =  R_ij(2,2)+R_ij(0,0) +   2*(NR_ij[2][2]+NR_ij[0][0]) *qf + qf.T() * (N_ij[2][2]+N_ij[0][0]) * qf;
    I(1,2) =-(R_ij(1,2)           +     (NR_ij[2][1]+NR_ij[1][2]) *qf + qf.T() * (N_ij[2][1]+N_ij[1][2]) * qf);
    I(2,2) =  R_ij(1,1)+R_ij(0,0) +   2*(NR_ij[1][1]+NR_ij[0][0]) *qf + qf.T() * (N_ij[1][1]+N_ij[0][0]) * qf;

    Mat M_ThetaTheta = G.T() * (I+J0) * G; //TODO: SymMat was not possible ...

    /*M_ThetaF*/
    Mat qN(3,Dofs-RefDofs,INIT,0.);

    //TODO: proof whether N_ij[i][j].T() == N_ij[j][i]
    qN(0,0,0,Dofs-RefDofs-1) = NR_ij[1][2]-NR_ij[2][1] + qf.T()*(N_ij[1][2]-N_ij[2][1]);
    qN(1,0,1,Dofs-RefDofs-1) = NR_ij[2][0]-NR_ij[0][2] + qf.T()*(N_ij[2][0]-N_ij[0][2]);
    qN(2,0,2,Dofs-RefDofs-1) = NR_ij[0][1]-NR_ij[1][0] + qf.T()*(N_ij[0][1]-N_ij[1][0]);

    Mat M_ThetaF = G.T()*qN;

    /*sort into Mext*/
    Mext(0,3,2,5) = M_RTheta;

    Mext(0,RefDofs,2,Dofs-1) = M_RF;

    for(int i=3; i<RefDofs; i++)//Because M_ThetaTheta is symmetric
      for(int j=i; j<RefDofs; j++)
        Mext(i,j) = M_ThetaTheta(i-3,j-3);

    Mext(3,RefDofs,5,Dofs-1) = M_ThetaF;

    M = (MConst + condenseMatrix_cd(Mext, ILocked)).copy();

    /*TESTING*/
	//cout << "Time:" << t << endl;
	//cout << "M: " << M << endl;
	//cout << "M_old-M: " << M_old-M << endl;
	//cout << "Mext: " << Mext << endl;
	//cout << "MConst:"  << MConst << endl;
	//cout << "MConst-M: " << MConst-M << endl;
	//M_old = M.copy();
	/*END-TESTING*/

    // LU-decomposition of M
    //LLM = facLL(M); //TODO: why this?, but it doesn't work anyway ...
  }

  void FlexibleBody2s13MFRMindlin::updatedhdz(double t) {
    updateh(t);
    for (int i = 0; i < dhdq.cols(); i++)
      for (int j = 0; j < dhdq.rows(); j++)
        dhdq(i, j) = -K(i, j);
  }

  void FlexibleBody2s13MFRMindlin::updateStateDependentVariables(double t) {
    FlexibleBodyContinuum<Vec>::updateStateDependentVariables(t);

    updateAG();

#ifdef HAVE_NURBS
    contour->computeSurface();
    contour->computeSurfaceVelocities();
    contour->computeSurfaceJacobiansOfTranslation();
#endif
  }

  void FlexibleBody2s13MFRMindlin::BuildElements() {
    for (int i = 0; i < Elements; i++) {
      //  ^ phi
      //  |
      //  |   4--------3
      //  |   |        |
      //  |   1--------2
      //  |
      //  | --------------> r
      // radial and azimuthal coordinates of the FE [ElementalNodes(r1,phi1,r2,phi2)]
      // r1 and phi1 are defined with node 1, r2 and phi2 with node 3
      ElementalNodes[i](0, 1) << NodeCoordinates.row(ElementNodeList(i, 0)).T(); // node 1
      ElementalNodes[i](2, 3) << NodeCoordinates.row(ElementNodeList(i, 2)).T(); // node 3

      if (ElementalNodes[i](3) <= ElementalNodes[i](1)) //phi2 < phi1
        { // ring closure
        ElementalNodes[i](3) += 2 * M_PI;
        }

      //TODO: needed?
      // mapping node dof position (w, a, b) from global vector to element vector
      // ref, node 1, node 2, node 3, node 4
      qElement[i](0, RefDofs - 1) << qext(0, RefDofs - 1);
      qElement[i](RefDofs, RefDofs + NodeDofs - 1) << qext(RefDofs + ElementNodeList(i, 0) * NodeDofs, RefDofs + (ElementNodeList(i, 0) + 1) * NodeDofs - 1);
      qElement[i](RefDofs + NodeDofs, RefDofs + 2 * NodeDofs - 1) << qext(RefDofs + ElementNodeList(i, 1) * NodeDofs, RefDofs + (ElementNodeList(i, 1) + 1) * NodeDofs - 1);
      qElement[i](RefDofs + 2 * NodeDofs, RefDofs + 3 * NodeDofs - 1) << qext(RefDofs + ElementNodeList(i, 2) * NodeDofs, RefDofs + (ElementNodeList(i, 2) + 1) * NodeDofs - 1);
      qElement[i](RefDofs + 3 * NodeDofs, RefDofs + 4 * NodeDofs - 1) << qext(RefDofs + ElementNodeList(i, 3) * NodeDofs, RefDofs + (ElementNodeList(i, 3) + 1) * NodeDofs - 1);

      // mapping node dof velocity from global vector to element vector
      // ref, node 1, node 2, node 3, node 4
      uElement[i](0, RefDofs - 1) << uext(0, RefDofs - 1);
      uElement[i](RefDofs, RefDofs + NodeDofs - 1) << uext(RefDofs + ElementNodeList(i, 0) * NodeDofs, RefDofs + (ElementNodeList(i, 0) + 1) * NodeDofs - 1);
      uElement[i](RefDofs + NodeDofs, RefDofs + 2 * NodeDofs - 1) << uext(RefDofs + ElementNodeList(i, 1) * NodeDofs, RefDofs + (ElementNodeList(i, 1) + 1) * NodeDofs - 1);
      uElement[i](RefDofs + 2 * NodeDofs, RefDofs + 3 * NodeDofs - 1) << uext(RefDofs + ElementNodeList(i, 2) * NodeDofs, RefDofs + (ElementNodeList(i, 2) + 1) * NodeDofs - 1);
      uElement[i](RefDofs + 3 * NodeDofs, RefDofs + 4 * NodeDofs - 1) << uext(RefDofs + ElementNodeList(i, 3) * NodeDofs, RefDofs + (ElementNodeList(i, 3) + 1) * NodeDofs - 1);
      }
  }

  void FlexibleBody2s13MFRMindlin::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff, Frame *frame) {
    if (cp.getContourParameterType() == CONTINUUM)
      { // frame on continuum
#ifdef HAVE_NURBS
      contour->updateKinematicsForFrame(cp, ff);
#endif
      }
    else if (cp.getContourParameterType() == NODE)
      { // frame on node
      const int &node = cp.getNodeNumber();

      if (ff == position || ff == position_cosy || ff == all)
        {
        Vec r_ref(3, INIT, 0.);
        r_ref(0) = qext(RefDofs + node*NodeDofs+1) * computeThickness(NodeCoordinates(node,0))/2 + NodeCoordinates(node,0);
        r_ref(1) = qext(RefDofs + node*NodeDofs+2) * computeThickness(NodeCoordinates(node,0))/2;
        r_ref(2) = qext(RefDofs + node*NodeDofs)   + computeThickness(NodeCoordinates(node,0))/2;

        r_ref = A * TransformationMatrix(NodeCoordinates(node, 1)) * r_ref;
        r_ref += qext(0, 2);
        cp.getFrameOfReference().setPosition(frameOfReference->getPosition() + frameOfReference->getOrientation() * r_ref);
        }

      if (ff == firstTangent || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all)
        throw new MBSimError("ERROR(FlexibleBody2s13MFRMindlin::updateKinematicsForFrame): Not implemented!");
      if (ff == normal || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all)
        throw new MBSimError("ERROR(FlexibleBody2s13MFRMindlin::updateKinematicsForFrame): Not implemented!");
      if (ff == secondTangent || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all)
        throw new MBSimError("ERROR(FlexibleBody2s13MFRMindlin::updateKinematicsForFrame): Not implemented!");

      if (ff == velocity || ff == velocities || ff == velocity_cosy || ff == velocities_cosy || ff == all)
        {

        Vec u(3, INIT, 0.);
        u(0) = computeThickness(NodeCoordinates(node, 0)) / 2 * uext(RefDofs + node * NodeDofs + 1);
        u(1) = computeThickness(NodeCoordinates(node, 0)) / 2 * uext(RefDofs + node * NodeDofs + 2);
        u(2) = uext(RefDofs + node * NodeDofs);

        Vec r_ref(3, INIT, 0.);
        r_ref(0) = qext(RefDofs + node*NodeDofs+1) * computeThickness(NodeCoordinates(node,0))/2 + NodeCoordinates(node,0);
        r_ref(1) = qext(RefDofs + node*NodeDofs+2) * computeThickness(NodeCoordinates(node,0))/2;
        r_ref(2) = qext(RefDofs + node*NodeDofs)   + computeThickness(NodeCoordinates(node,0))/2;

        r_ref    = TransformationMatrix(NodeCoordinates(node,1))*r_ref;

        SqrMat r_tyl(3, INIT, 0.); //TODO: proof especially signes
        r_tyl(0, 0) = 0;
        r_tyl(0, 1) = -r_ref(2);
        r_tyl(0, 2) =  r_ref(1);

        r_tyl(1, 0) = r_ref(2);
        r_tyl(1, 1) = 0;
        r_tyl(1, 2) = -r_ref(0);

        r_tyl(2, 0) = -r_ref(1);
        r_tyl(2, 1) =  r_ref(0);
        r_tyl(2, 2) = 0;

        Vec u_ref(3, INIT, 0.);
        u_ref = A * (-r_tyl * G * uext(3, 5) + TransformationMatrix(NodeCoordinates(node,1))*u);
        u_ref += uext(0, 2);

        cp.getFrameOfReference().setVelocity(frameOfReference->getOrientation() * u_ref);
        }

      if (ff == angularVelocity || ff == velocities || ff == velocities_cosy || ff == all)
        {

        Vec w_loc(3, INIT, 0.);
        w_loc(0) = uext(RefDofs + node * NodeDofs + 1);
        w_loc(1) = uext(RefDofs + node * NodeDofs + 2);

        Vec w_ref(3, INIT, 0.);
        w_ref = G * uext(3, 5) + A * TransformationMatrix(NodeCoordinates(node, 1)) * w_loc;

        cp.getFrameOfReference().setAngularVelocity(frameOfReference->getOrientation() * w_ref);
        }
      }
    else
      throw new MBSimError("ERROR(FlexibleBody2s13MFRMindlin::updateKinematicsForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    if (frame != 0) { // frame should be linked to contour point data
      frame->setPosition(cp.getFrameOfReference().getPosition());
      frame->setOrientation(cp.getFrameOfReference().getOrientation());
      frame->setVelocity(cp.getFrameOfReference().getVelocity());
      frame->setAngularVelocity(cp.getFrameOfReference().getAngularVelocity());
      }
  }

  void FlexibleBody2s13MFRMindlin::updateJacobiansForFrame(ContourPointData &cp, Frame *frame) {

    if (cp.getContourParameterType() == CONTINUUM) { // force on continuum
      Vec alpha = cp.getLagrangeParameterPosition();

      if (nrm2(alpha) < epsroot()) { // center of gravity

        Mat Jacext_trans(3, Dofs, INIT, 0.),
        	Jacext_rot(3, Dofs, INIT, 0.);

        Jacext_trans(0, 0, 2, 2) = SqrMat(3, EYE);
        Jacext_rot(0, 3, 2, 5) = G;

        //condensation
        Mat Jacobian_trans = condenseMatrixCols_cd(Jacext_trans, ILocked);
        Mat Jacobian_rot = condenseMatrixCols_cd(Jacext_rot, ILocked);

        // transformation
        cp.getFrameOfReference().setJacobianOfTranslation(frameOfReference->getOrientation() * Jacobian_trans);
        cp.getFrameOfReference().setJacobianOfRotation(frameOfReference->getOrientation() * Jacobian_rot);

        }
      else { // on the disk
        contour->updateJacobiansForFrame(cp);
        }
      }

    else if (cp.getContourParameterType() == NODE) { // force on node
      int Node = cp.getNodeNumber();

      /* Jacobian of element */
      Mat Jactmp_trans(3, RefDofs + NodeDofs, INIT, 0.),
    	  Jactmp_rot(3, RefDofs + NodeDofs, INIT, 0.); // Initializing Ref + 1 Node

      // translational DOFs
      Jactmp_trans(0, 0, 2, 2) = SqrMat(3, EYE); // ref
      //Jactmp_rot(0,0,2,2) = Mat(3,INIT,0.);

      // rotational DOFs
      SqrMat dAdalpha(3, INIT, 0.), dAdbeta(3, INIT, 0.), dAdgamma(3, INIT, 0.);

      dAdalpha(0, 0) = 0;
      dAdalpha(0, 1) = 0;
      dAdalpha(0, 2) = 0;
      dAdalpha(1, 0) = cos(qext(3)) * sin(qext(4)) * cos(qext(5)) - sin(qext(3)) * sin(qext(5));
      dAdalpha(1, 1) = -cos(qext(3)) * sin(qext(4)) * sin(qext(5)) - sin(qext(3)) * cos(qext(5));
      dAdalpha(1, 2) = -cos(qext(3)) * cos(qext(4));
      dAdalpha(2, 0) = cos(qext(3)) * sin(qext(5)) + sin(qext(3)) * sin(qext(4)) * cos(qext(5));
      dAdalpha(2, 1) = cos(qext(3)) * cos(qext(5)) - sin(qext(3)) * sin(qext(4)) * sin(qext(5));
      dAdalpha(2, 2) = -sin(qext(3)) * cos(qext(4));

      dAdbeta(0, 0) = -sin(qext(4)) * cos(qext(5));
      dAdbeta(0, 1) = sin(qext(4)) * sin(qext(5));
      dAdbeta(0, 2) = cos(qext(4));
      dAdbeta(1, 0) = sin(qext(3)) * cos(qext(4)) * cos(qext(5));
      dAdbeta(1, 1) = -sin(qext(3)) * cos(qext(4)) * sin(qext(5));
      dAdbeta(1, 2) = sin(qext(3)) * sin(qext(4));
      dAdbeta(2, 0) = -cos(qext(3)) * cos(qext(4)) * cos(qext(5));
      dAdbeta(2, 1) = cos(qext(3)) * cos(qext(4)) * sin(qext(5));
      dAdbeta(2, 2) = -cos(qext(3)) * sin(qext(4));

      dAdgamma(0, 0) = -cos(qext(4)) * sin(qext(5));
      dAdgamma(0, 1) = -cos(qext(4)) * cos(qext(5));
      dAdgamma(0, 2) = 0;
      dAdgamma(1, 0) = cos(qext(3)) * cos(qext(5)) - sin(qext(3)) * sin(qext(4)) * sin(qext(5));
      dAdgamma(1, 1) = -cos(qext(3)) * sin(qext(5)) - sin(qext(3)) * sin(qext(4)) * cos(qext(5));
      dAdgamma(1, 2) = 0;
      dAdgamma(2, 0) = cos(qext(3)) * sin(qext(4)) * sin(qext(5)) + sin(qext(3)) * cos(qext(5));
      dAdgamma(2, 1) = cos(qext(3)) * sin(qext(4)) * cos(qext(5)) - sin(qext(3)) * sin(qext(5));
      dAdgamma(2, 2) = 0;

      Vec r_tmp(3, INIT, 0.);
      r_tmp(0) = computeThickness(NodeCoordinates(Node, 0)) / 2 * qext(RefDofs + Node * NodeDofs + 1);
      r_tmp(1) = computeThickness(NodeCoordinates(Node, 0)) / 2 * qext(RefDofs + Node * NodeDofs + 2);
      r_tmp(2) = qext(RefDofs + Node * NodeDofs);

      r_tmp = TransformationMatrix(NodeCoordinates(Node, 1)) * r_tmp;

      Jactmp_trans(0, 3, 2, 3) = dAdalpha * r_tmp;
      Jactmp_trans(0, 4, 2, 4) = dAdbeta * r_tmp;
      Jactmp_trans(0, 5, 2, 5) = dAdgamma * r_tmp;

      Jactmp_rot(0, RefDofs, 2, RefDofs + 2) = G;

      //elastic DOFs
      SqrMat u_tmp(3, INIT, 0.);
      u_tmp(0, 1) = computeThickness(NodeCoordinates(Node, 0)) / 2;
      u_tmp(1, 2) = computeThickness(NodeCoordinates(Node, 0)) / 2;
      u_tmp(2, 0) = 1;

      Jactmp_trans(0, RefDofs, 2, RefDofs + 2) = A * TransformationMatrix(NodeCoordinates(Node, 1)) * u_tmp;
      Jactmp_rot(0, RefDofs, 2, RefDofs + 2) = Jactmp_trans(0, RefDofs, 2, RefDofs + 2);

      /* sort in the Jacobian of the disc disk */
      // reference dofs (translational and rotational)
      Mat Jacext_trans(3, Dofs, INIT, 0.),
    	  Jacext_rot(3, Dofs, INIT, 0.);

      Jacext_trans(0, 0, 2, RefDofs - 1) = Jactmp_trans(0, 0, 2, RefDofs - 1);
      Jacext_rot  (0, 0, 2, RefDofs - 1) = Jactmp_rot(0, 0, 2, RefDofs - 1);

      // elastic dofs
      Jacext_trans(0, RefDofs + Node * NodeDofs, 2, RefDofs + Node * NodeDofs + 2) = Jactmp_trans(0, RefDofs, 2, RefDofs + 2);
      Jacext_rot  (0, RefDofs + Node * NodeDofs, 2, RefDofs + Node * NodeDofs + 2) = Jactmp_rot  (0, RefDofs, 2, RefDofs + 2);

      //condesnation
      Mat Jacobian_trans = condenseMatrixCols_cd(Jacext_trans, ILocked);
      Mat Jacobian_rot   = condenseMatrixCols_cd(Jacext_rot, ILocked);

      // transformation
      cp.getFrameOfReference().setJacobianOfTranslation(frameOfReference->getOrientation() * Jacobian_trans);
      cp.getFrameOfReference().setJacobianOfRotation   (frameOfReference->getOrientation() * Jacobian_rot);

      }
    else
      throw new MBSimError("ERROR(FlexibleBody2s13MFRMindlin::updateJacobiansForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    // cp.getFrameOfReference().setGyroscopicAccelerationOfTranslation(TODO)
    // cp.getFrameOfReference().setGyroscopicAccelerationOfRotation(TODO)

    if (frame != 0) { // frame should be linked to contour point data
      frame->setJacobianOfTranslation(cp.getFrameOfReference().getJacobianOfTranslation());
      frame->setJacobianOfRotation(cp.getFrameOfReference().getJacobianOfRotation());
      frame->setGyroscopicAccelerationOfTranslation(cp.getFrameOfReference().getGyroscopicAccelerationOfTranslation());
      frame->setGyroscopicAccelerationOfRotation(cp.getFrameOfReference().getGyroscopicAccelerationOfRotation());
      }
  }

  void FlexibleBody2s13MFRMindlin::init(InitStage stage) {
    if (stage == resize) {
      FlexibleBodyContinuum<Vec>::init(stage);
      assert(nr>0); // at least on radial row
      assert(nj>1); // at least two azimuthal elements

      for (int i = 0; i < Elements; i++) {
        discretization.push_back(new FiniteElement2s13MFRMindlin(E, nu, rho, d(0), d(1), d(2)));
        qElement.push_back(Vec(discretization[0]->getqSize(), INIT, 0.)); //TODO: discretization[i]?
        uElement.push_back(Vec(discretization[0]->getuSize(), INIT, 0.));
        ElementalNodes.push_back(Vec(4, INIT, 0.));
        }

      // condensation
      switch (LType) {
        case innerring: // 0: innerring
          ILocked = Index(RefDofs, RefDofs + NodeDofs * nj - 1);
          Jext = Mat(Dofs, qSize, INIT, 0.);
          Jext(0, 0, RefDofs - 1, RefDofs - 1) << DiagMat(RefDofs, INIT, 1.);
          Jext(RefDofs + NodeDofs * nj, RefDofs, Dofs - 1, qSize - 1) << DiagMat(qSize - RefDofs, INIT, 1.);
          break;

        case outerring: // 1: outerring
          ILocked = Index(qSize, Dofs - 1);
          Jext = Mat(Dofs, qSize, INIT, 0.);
          Jext(0, 0, qSize - 1, qSize - 1) << DiagMat(qSize, INIT, 1.);
          break;
        }

      dr = (Ra - Ri) / nr;
      dj = 2 * M_PI / nj;

      NodeCoordinates = Mat(Nodes, 2);
      ElementNodeList.resize(Elements, 4);

      // mapping nodes - node coordinates - elements 
      for (int i = 0; i <= nr; i++)
        {
        for (int j = 0; j < nj; j++)
          {
          // node number increases azimuthally from the inner to the outer ring
          NodeCoordinates(j + i * nj, 0) = Ri + dr * i;
          NodeCoordinates(j + i * nj, 1) = 0. + dj * j;

          // element number increases azimuthally from the inner to the outer ring
          if (i < nr && j < nj - 1) {
            ElementNodeList(j + i * nj, 0) = j + i * nj; // elementnode 1
            ElementNodeList(j + i * nj, 1) = j + i * nj + nj; // elementnode 2
            ElementNodeList(j + i * nj, 2) = j + 1 + i * nj + nj; // elementnode 3
            ElementNodeList(j + i * nj, 3) = j + 1 + i * nj; // elementnode 4
            }
          else if (i < nr && j == nj - 1) { // ring closure
            ElementNodeList(j + i * nj, 0) = j + i * nj; // elementnode 1
            ElementNodeList(j + i * nj, 1) = j + i * nj + nj; // elementnode 2
            ElementNodeList(j + i * nj, 2) = j + 1 + i * nj; // elementnode 3
            ElementNodeList(j + i * nj, 3) = j + 1 + i * nj - nj; // elementnode 4
            }
          }
        }

#ifdef HAVE_NURBS
      // borders of contour parametrisation 
      // beginning 
      Vec alphaS(2);
      alphaS(0) = Ri; // radius
      alphaS(1) = 0.; // angle

      // end 
      Vec alphaE(2);
      alphaE(0) = Ra; // radius
      alphaE(1) = 2 * M_PI; // angle

      contour->setAlphaStart(alphaS);
      contour->setAlphaEnd(alphaE);
#endif

      qext = Jext * q0;
      uext = Jext * u0;

      initMatrices(); // calculate constant stiffness matrix and the constant parts of the mass-matrix

      }
    if (stage == MBSim::plot) {
      updatePlotFeatures(parent);

      if (getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
#ifdef HAVE_NURBS
        if (getPlotFeature(openMBV) == enabled) {
          OpenMBV::NurbsDisk *Diskbody = new OpenMBV::NurbsDisk;

          drawDegree = 30 / nj;
          Diskbody->setStaticColor(0.3);
          Diskbody->setMinimalColorValue(0.);
          Diskbody->setMaximalColorValue(1.);
          Diskbody->setDrawDegree(drawDegree);
          Diskbody->setRadii(Ri, Ra);

          float *openmbvUVec = new float[nj + 1 + 2 * degU];
          float *openmbvVVec = new float[nr + 2 + degV];
          for (int i = 0; i < nj + 1 + 2 * degU; i++)
            openmbvUVec[i] = contour->getUVector()(i);
          for (int i = 0; i < nr + 1 + degV + 1; i++)
            openmbvVVec[i] = contour->getVVector()(i);

          Diskbody->setKnotVecAzimuthal(openmbvUVec);
          Diskbody->setKnotVecRadial(openmbvVVec);

          Diskbody->setElementNumberRadial(nr);
          Diskbody->setElementNumberAzimuthal(nj);

          Diskbody->setInterpolationDegreeRadial(degV);
          Diskbody->setInterpolationDegreeAzimuthal(degU);
          openMBVBody = Diskbody;
          }
#endif
#endif
        FlexibleBodyContinuum<Vec>::init(stage);
        }
      }
    else
      FlexibleBodyContinuum<Vec>::init(stage);

#ifdef HAVE_NURBS
    contour->initContourFromBody(stage); // initialize contour
#endif
  }

  void FlexibleBody2s13MFRMindlin::plot(double t, double dt) {
    if (getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
#ifdef HAVE_NURBS
      if (getPlotFeature(openMBV) == enabled && openMBVBody) {
        vector<double> data;
        data.push_back(t); //time

        for (int i = 0; i < nr + 1; i++) {
          for (int j = 0; j < nj + degU; j++) {
            data.push_back(contour->getControlPoints(j, i)(0)); //global x-coordinate
            data.push_back(contour->getControlPoints(j, i)(1)); //global y-coordinate
            data.push_back(contour->getControlPoints(j, i)(2)); //global z-coordinate
            }
          }

        ContourPointData cp;
        cp.getLagrangeParameterPosition() = Vec(2, NONINIT);

        //inner ring
        for (int i = 0; i < nj; i++) {
          for (int j = 0; j < drawDegree; j++) {
            cp.getLagrangeParameterPosition()(0) = Ri;
            cp.getLagrangeParameterPosition()(1) = 2 * M_PI * (i * drawDegree + j) / (nj * drawDegree);
            contour->updateKinematicsForFrame(cp, position);
            Vec pos = cp.getFrameOfReference().getPosition();

            data.push_back(pos(0)); //global x-coordinate
            data.push_back(pos(1)); //global y-coordinate
            data.push_back(pos(2)); //global z-coordinate
            }
          }

        //outer Ring
        for (int i = 0; i < nj; i++) {
          for (int j = 0; j < drawDegree; j++) {
            cp.getLagrangeParameterPosition()(0) = Ra;
            cp.getLagrangeParameterPosition()(1) = 2 * M_PI * (i * drawDegree + j) / (nj * drawDegree);
            contour->updateKinematicsForFrame(cp, position);
            Vec pos = cp.getFrameOfReference().getPosition();

            data.push_back(pos(0)); //global x-coordinate
            data.push_back(pos(1)); //global y-coordinate
            data.push_back(pos(2)); //global z-coordinate
            }
          }

        //center of gravity
        cp.getLagrangeParameterPosition()(0) = 0.;
        cp.getLagrangeParameterPosition()(1) = 0.;
        contour->updateKinematicsForFrame(cp, position_cosy); // kinematics of the center of gravity of the disk (TODO frame feature)

        data.push_back(cp.getFrameOfReference().getPosition()(0) - (cp.getFrameOfReference().getOrientation()*A)(0, 2) * d(0) * 0.5); //global x-coordinate
        data.push_back(cp.getFrameOfReference().getPosition()(1) - (cp.getFrameOfReference().getOrientation()*A)(1, 2) * d(0) * 0.5); //global y-coordinate
        data.push_back(cp.getFrameOfReference().getPosition()(2) - (cp.getFrameOfReference().getOrientation()*A)(2, 2) * d(0) * 0.5); //global z-coordinate

        for (int i = 0; i < 3; i++)
          for (int j = 0; j < 3; j++)
            data.push_back((cp.getFrameOfReference().getOrientation()*A)(i, j));

        ((OpenMBV::NurbsDisk*) openMBVBody)->append(data);
        }
#endif
#endif
      }
    FlexibleBodyContinuum<Vec>::plot(t, dt);
    cout << endl;
  }

  void FlexibleBody2s13MFRMindlin::setNumberElements(int nr_, int nj_) {
    nr = nr_;
    nj = nj_;
    degV = min(degV, nr); // radial adaptation of spline degree to have correct knot vector
    degU = min(degU, nj); // azimuthal adaptation of spline degree to have correct knot vector
    Elements = nr * nj;
    Nodes = (nr + 1) * nj;

    Dofs = RefDofs + Nodes * NodeDofs;

    qSize = Dofs - NodeDofs * nj; // missing one node row because of bearing
    uSize[0] = qSize;
    uSize[1] = qSize; // TODO

    qext = Vec(Dofs);
    uext = Vec(Dofs);

    q0.resize(qSize);
    u0.resize(uSize[0]);
  }

  Vec FlexibleBody2s13MFRMindlin::transformCW(const Vec& WrPoint) {
    Vec CrPoint(WrPoint.size());

    CrPoint = WrPoint - getFrameOfReference()->getPosition();
    CrPoint = inv(getFrameOfReference()->getOrientation()) * CrPoint;  //now_ position in reference frame

    CrPoint -= q(0,2);
    CrPoint = inv(A) * CrPoint; //now position in moving frame of reference

    const double &xt = CrPoint(0);
    const double &yt = CrPoint(1);

    CrPoint(0) = sqrt(xt * xt + yt * yt);
    CrPoint(1) = ArcTan_cd(xt, yt);
    CrPoint(2) = WrPoint(2);

    return CrPoint;
  }

  void FlexibleBody2s13MFRMindlin::BuildElement(const Vec &s) {
    assert(Ri <= s(0)); // is the input on the disk?
    assert(Ra >= s(0));

    currentElement = int((s(0) - Ri) / dr) * nj + int(s(1) / dj); // which element is involved?
  }

  void FlexibleBody2s13MFRMindlin::initMatrices() {
    BuildElements();
    updateAG();

    // element loop, to get all (constant) Matrices of the elements
    for (int i = 0; i < Elements; i++)
      static_cast<FiniteElement2s13MFRMindlin*> (discretization[i])->computeConstantSystemMatrices(ElementalNodes[i]);

    M_old = SymMat(qSize,INIT,0.);
    computeStiffnessMatrix();
    computeConstantMassMatrixParts();
    updateM(0); //TODO: okay, that t=0?
  }

  void FlexibleBody2s13MFRMindlin::computeStiffnessMatrix() {
    SymMat Kext(Dofs - RefDofs, INIT, 0.);
    int ElementNodes = 4;

    //for Testing ...
    SymMat K_old = Kext.copy();

    // element loop
    for (int element = 0; element < Elements; element++) {
      SymMat ElK = static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->getK();

      for (int node = 0; node < ElementNodes; node++) {
        for (int conode = node; conode < ElementNodes; conode++) {
          //coupling of the node with itself
          int GlobalNode = ElementNodeList(element, node) * NodeDofs;
          int localNode = node * NodeDofs;
          int GlobalCouplingNode = ElementNodeList(element, conode) * NodeDofs;
          int localCouplingNode = conode * NodeDofs;

          for (int i = 0; i < NodeDofs; i++) {
            if (node == conode)//a part on the diagonal is added, so just write the upper triangle-part
              {
              for (int j = i; j < NodeDofs; j++) {
                Kext(GlobalNode + i, GlobalCouplingNode + j) += ElK(localNode + i, localCouplingNode + j);
                }//j
              }
            else //non-diagonal parts
              {
              for (int j = 0; j < NodeDofs; j++) {
                Kext(GlobalNode + i, GlobalCouplingNode + j) += ElK(localNode + i, localCouplingNode + j);
                }//j
              }
            }//i
          /******TESTING**********/
          //proof, wether the sort in of the element-matrix parts is correct
          //cout << "Element-Nummer: " << element << endl;
          //cout.precision(8);
          //cout.setf(ios::scientific);
          //cout << "Element-Matrix:" << Kplatte << endl;
          //cout << "Knoten:        " << node   << "  globale Nummer:" << GlobalNode << endl;
          //cout << "Koplungsknoten:" << conode << "  globale Nummer"  << GlobalCouplingNode << endl;
          //cout << "   ";
          //for(int j=0; j<Kext.size(); j++)
          //{
          //  if(j<10) cout << "       " << j << "        ";
          //  else     cout << "       " << j << "       ";
          //}

          //for(int k=0; k<Kext.size(); k++)
          //{
          //  //Zeilennummer
          //  if (k<10) cout << endl <<  k << "  ";
          //  else      cout << endl <<  k << " ";

          //  for(int j=0; j<Kext.size(); j++){
          //    if     (Kext(k,j)-K_old(k,j)>0)         cout << "  " << Kext(k,j)-K_old(k,j);
          //    else if(!(int)fabs(Kext(k,j)-K_old(k,j)))   cout << "        0       ";
          //    else if(Kext(k,j)-K_old(k,j)<0)         cout << " "  <<Kext(k,j)-K_old(k,j);
          //  }
          //}

          //cout << endl;

          //K_old = Kext.copy();
          /*****END-TESTING*******/
          }//conode
        }//node
      }//el

    SymMat Kgebl(Dofs, INIT, 0.);//TODO: muss das sein, kann man das nicht bei updatedhdz anpassen?

    for (int i = 0; i < Dofs - RefDofs; i++)
      for (int j = i; j < Dofs - RefDofs; j++)
        Kgebl(RefDofs + i, RefDofs + j) = Kext(i, j); //TODO: proof, watch out: symmetric matrices!

    K = condenseMatrix_cd(Kgebl, ILocked);

    /*****TESTING********/
    //get values to compare with ANSYS
    //cout << "KEL: " << KEl << endl;

    //Vec F_test(K.size()-2,INIT, 0.);

    //F_test(nj*(nr-1)*3) = -10000;

    //SymMat Ktmp(K.size()-2,INIT,0.);
    //for(int i=0;i<Ktmp.size();i++)
    //  for(int j=i;j<Ktmp.size();j++)
    //    Ktmp(i,j) = K(i+2,j+2);

    //cout << "Steifigkeitsmatrix: " << Ktmp << endl;
    //cout << "Kraftlastvektor: " << F_test << endl;

    //Vec q_test = slvLL(Ktmp, F_test);

    /*Ausgabe als Matrix für maxima*/
    //cout << "qf=matrix([" ;
    //for(int i=0; i<q_test.size(); i++)
    //  cout << q_test(i) << ",";

    //cout << "]);" << endl;

    /*Ausgabe der Knotendurchsenkungen in x-Richtung (y=0)*/
    //for(int i=0; i<nr; i++)
    //  cout << q_test(i*nj*3) << endl;

    /*******END TESTING*****************/
  }

  void FlexibleBody2s13MFRMindlin::computeConstantMassMatrixParts() {
    SymMat M_RR(3, INIT, 0.);
    N_compl = Mat(3, Dofs - RefDofs, INIT, 0.);
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        {
        N_ij[i][j]  = SqrMat(Dofs-RefDofs, INIT, 0.);
        NR_ij[i][j] = RowVec(Dofs-RefDofs, INIT, 0.);
        }
    R_compl = Vec(3, INIT, 0.);
    R_ij = SymMat(3, INIT, 0.);

    double ElementNodes = 4;

    // element loop
    for (int element = 0; element < Elements; element++)
      {
      /*get the Element-matrices*/
      //M_RR, R_compl, R_ij can be assembled directly
      M_RR += static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->getM_RR();
      R_compl += static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->getR_compl();
      R_ij += static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->getR_ij();

      //the other matrices have to be assembled later on
      Mat ElN_compl = static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->getN_compl();
      SqrMat ElN_ij[3][3];
      RowVec ElNR_ij[3][3];
      for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
          {
          ElN_ij[i][j]  = static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->getN_ij(i,j);
          ElNR_ij[i][j] = static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->getNR_ij(i,j);
          }
      /**********************/

      /*Assembly*/
      //Assembly of N_compl (just the cols have to be assembled (there is no coupling node), but for every dimension)
      for (int node = 0; node < ElementNodes; node++)//for every node
         {
         int GlobalNode = ElementNodeList(element, node) * NodeDofs;
         int localNode = node * NodeDofs;

         for (int dim = 0; dim < 3; dim++)
           {
           for (int l=0; l<NodeDofs; l++)
             {
               N_compl(dim,GlobalNode+l) += ElN_compl(dim,localNode+l);
             }//l
           }//dim
         }//node

      //Assembly of the X_ij-matrices
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          for (int node = 0; node < ElementNodes; node++)//for every node...
            {
              int GlobalNode = ElementNodeList(element, node) * NodeDofs;
              int localNode = node * NodeDofs;

              //Assembly of NR_ij (just the cols have to be assembled (there is no coupling node))
              for (int l=0; l<NodeDofs; l++) {
                NR_ij[i][j](GlobalNode+l) += ElNR_ij[i][j](localNode+l);
              }//l

            for (int conode = node; conode < ElementNodes; conode++)//...and its coupling node
              {
              int GlobalCouplingNode = ElementNodeList(element, conode) * NodeDofs;
              int localCouplingNode = conode * NodeDofs;

              //Assembly of N_ij (both (rows and cols) have to be assembled)
              for (int k = 0; k < NodeDofs; k++)
                {
                for (int l=0; l < NodeDofs; l++)
                  {
                  N_ij[i][j](GlobalNode+k,GlobalCouplingNode+l) += ElN_ij[i][j](localNode+k,localCouplingNode+l);
                  }//l
                }//k
              }//conode
            }//node
          }//j
        }//i
      }//el

    SymMat Mext(Dofs, INIT, 0.);

    //M_RR (coupling of the translational DOFs)
    for (int i = 0; i < 3; i++)
    	Mext(i, i) = m0 + M_RR(i, i); //just diagonal entries!

    //M_FF (coupling of the elastic DOFs)
    for (int i = 0; i < 3; i++)
    	for (int k = RefDofs; k < Dofs; k++)
    		for (int l = k; l < Dofs; l++)
    			Mext(k, l) += N_ij[i][i](k-RefDofs,l-RefDofs);

    // condensation
    MConst = condenseMatrix_cd(Mext, ILocked);

  }

  void FlexibleBody2s13MFRMindlin::updateAG() {
    double sinalpha = sin(q(3)), cosalpha = cos(q(3)), sinbeta = sin(q(4)), cosbeta = cos(q(4)), singamma = sin(q(5)), cosgamma = cos(q(5));

    A(0, 0) = cosbeta * cosgamma;
    A(0, 1) = -cosbeta * singamma;
    A(0, 2) = sinbeta;

    A(1, 0) = cosalpha * singamma + sinalpha * sinbeta * cosgamma;
    A(1, 1) = cosalpha * cosgamma - sinalpha * sinbeta * singamma;
    A(1, 2) = -sinalpha * cosbeta;

    A(2, 0) = sinalpha * singamma - cosalpha * sinbeta * cosgamma;
    A(2, 1) = sinalpha * cosgamma + cosalpha * sinbeta * singamma;
    A(2, 2) = cosalpha * cosbeta;

    G(0,0) = 1;
    //G(0,1) = 0;
    G(0, 2) = sinbeta;

    //G(1,0) = 0;
    G(1, 1) = cosalpha;
    G(1, 2) = -sinalpha * cosbeta;

    //G(2,0) = 0;
    G(2, 1) = sinalpha;
    G(2, 2) = cosalpha * cosbeta;

    /*TESTING*/
    //cout << "A" << endl << A << endl;
    //cout << "G" << endl << G << endl;
    /*END-TESTING*/
    }

  SqrMat FlexibleBody2s13MFRMindlin::TransformationMatrix(const double &phi) {
    SqrMat TransMat(3, EYE);

    TransMat(0, 0) = cos(phi);
    TransMat(0, 1) = -sin(phi);
    //TransMat(0,2) = 0;

    TransMat(1, 0) = sin(phi);
    TransMat(1, 1) = cos(phi);
    //TransMat(1,2) = 0;

    //TransMat(2,0) = 0;
    //TransMat(2,1) = 0;
    TransMat(2,2) = 1;

    return TransMat;
    }

  double FlexibleBody2s13MFRMindlin::computeThickness(const double &r_) {
    return d(0) + d(1) * r_ + d(2) * r_ * r_; // quadratic parameterization
    }

  }

