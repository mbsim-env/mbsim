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

#include "mbsimFlexibleBody/flexible_body/flexible_body_2s_13_mfr_mindlin.h"
#include "mbsimFlexibleBody/contours/nurbs_disk_2s.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/utils/eps.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/nurbsdisk.h"
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FlexibleBody2s13MFRMindlin::FlexibleBody2s13MFRMindlin(const string &name) :
    FlexibleBody2s13(name) {
      RefDofs = 6;
    }

  void FlexibleBody2s13MFRMindlin::updateM(double t) {
    SymMat Mext(Dofs, INIT, 0.);
    Vec qf = qext(RefDofs, Dofs - 1).copy();

    /*M_RR is constant*/

    /*M_RTheta*/
    Vec u(3, INIT, 0.);
    u = (*N_compl) * qf + (*R_compl);

    SqrMat u_tyl(3, INIT, 0.);
    u_tyl(0, 0) = 0;
    u_tyl(0, 1) = -u(2);
    u_tyl(0, 2) = u(1);

    u_tyl(1, 0) = u(2);
    u_tyl(1, 1) = 0;
    u_tyl(1, 2) = -u(0);

    u_tyl(2, 0) = -u(1);
    u_tyl(2, 1) = u(0);
    u_tyl(2, 2) = 0;

    SqrMat M_RTheta = -A * u_tyl * G;

    /*M_RF*/
    Mat M_RF = A * (*N_compl);

    /*M_ThetaTheta*/
    SymMat I(3, INIT, 0.);

    //TODO: proof signes
    I(0, 0) = (*R_ij)(1, 1) + (*R_ij)(2, 2) + 2 * ((*NR_ij[1][1]) + (*NR_ij[2][2])) * qf + qf.T() * (((*N_ij[1][1]) + (*N_ij[2][2])) * qf);
    I(0, 1) = -((*R_ij)(0, 1) + ((*NR_ij[1][0]) + (*NR_ij[0][1])) * qf + qf.T() * ((*N_ij[1][0]) + (*N_ij[0][1])) * qf);
    I(0, 2) = -((*R_ij)(0, 2) + ((*NR_ij[2][0]) + (*NR_ij[0][2])) * qf + qf.T() * ((*N_ij[2][0]) + (*N_ij[0][2])) * qf);
    I(1, 1) = (*R_ij)(2, 2) + (*R_ij)(0, 0) + 2 * ((*NR_ij[2][2]) + (*NR_ij[0][0])) * qf + qf.T() * ((*N_ij[2][2]) + (*N_ij[0][0])) * qf;
    I(1, 2) = -((*R_ij)(1, 2) + ((*NR_ij[2][1]) + (*NR_ij[1][2])) * qf + qf.T() * ((*N_ij[2][1]) + (*N_ij[1][2])) * qf);
    I(2, 2) = (*R_ij)(1, 1) + (*R_ij)(0, 0) + 2 * ((*NR_ij[1][1]) + (*NR_ij[0][0])) * qf + qf.T() * ((*N_ij[1][1]) + (*N_ij[0][0])) * qf;

    Mat M_ThetaTheta = G.T() * (I + J0) * G; //TODO: SymMat was not possible ...

    /*M_ThetaF*/
    Mat qN(3, Dofs - RefDofs, INIT, 0.);

    //Mark: N_ij[i][j].T() == N_ij[j][i]
    qN(0, 0, 0, Dofs - RefDofs - 1) = (*NR_ij[1][2]) - (*NR_ij[2][1]) + qf.T() * ((*N_ij[1][2]) - (*N_ij[2][1]));
    qN(1, 0, 1, Dofs - RefDofs - 1) = (*NR_ij[2][0]) - (*NR_ij[0][2]) + qf.T() * ((*N_ij[2][0]) - (*N_ij[0][2]));
    qN(2, 0, 2, Dofs - RefDofs - 1) = (*NR_ij[0][1]) - (*NR_ij[1][0]) + qf.T() * ((*N_ij[0][1]) - (*N_ij[1][0]));

    Mat M_ThetaF = G.T() * qN;

    /*M_FF is constant*/

    /*sort into Mext*/
    Mext(0, 3, 2, 5) = M_RTheta;

    Mext(0, RefDofs, 2, Dofs - 1) = M_RF;

    for (int i = 3; i < RefDofs; i++)//Because M_ThetaTheta is symmetric just sort in a triangular
      for (int j = i; j < RefDofs; j++)
        Mext(i, j) = M_ThetaTheta(i - 3, j - 3);

    Mext(3, RefDofs, 5, Dofs - 1) = M_ThetaF;

    Mext += MConst;

    /*TESING
      for (int i = 0; i< Dofs; i++)
      for (int j = RefDofs; j<Dofs; j++)
      {
      if (i==j)
      {
      Mext(i,j) = 1;
      }
      else
      {
      Mext(i,j) = 0;
      }
      }

      for (int i=0; i<K.size(); i++)
      for(int j = i; j<K.size(); j++)
      {
      if(i==j)
      K(i,j) = 0;
      else
      K(i,j) = 0;
      }


      TESTING-END*/

    M = condenseMatrix(Mext, ILocked).copy();

    ofstream file("M.txt");
    file << M << endl;
    file.close();


    //stringstream filename;
    //filename << "LLM" << t << ".txt";
    //ofstream file(filename.str().c_str());
    //file << LLM << endl;
    //file.close();

    //cout << eigval(M) << endl;
    //inv(M);

    /*TESTING*/
    //cout << "Time:" << t << endl;
    //cout << "M: " << M << endl;
    //cout << "M_old-M: " << M_old-M << endl;
    //cout << "Mext: " << Mext << endl;
    //cout << "MConst:"  << MConst << endl;
    //cout << "MConst-M: " << MConst-M << endl;
    //M_old = M.copy();
    /*END-TESTING*/

    /*NATURAL HARMONICS*/
    stringstream filename;
    time_t sekunde = time(NULL);
    filename << "Invertation" << nr << "x" << nj;
    tm *uhr = localtime(&sekunde);
    //cout << "Maple-Output M um " << uhr->tm_min << ":" << uhr->tm_sec << endl;
    MapleOutput(M, "M", filename.str());
    //uhr = localtime(&sekunde);
    //cout << "Maple-Output K um " << uhr->tm_min << ":" << uhr->tm_sec << endl;
    MapleOutput(K, "K", filename.str());
    //Mat H = inv(M)*K;
    //SqrMat Hsqr(H.rows());
    //cout << "Spalten" <<  H.cols() << endl;
    //cout << "Reihen " <<  H.rows() << endl;
    //bool symm = true;
    //for(int i = 0; i< H.rows(); i++)
    // 	for(int j=0; j< H.cols(); j++)
    // 	{
    //		Hsqr(i,j) = H(i,j);
    //		if(H(i,j) > H(j,i) or H(i,j) < H(j,i))
    //			symm = false;
    // 	}
    //if (symm)
    //	cout << "symmetrisch" << endl;
    //else
    //	cout << "nicht symmetrisch" << endl;

    //Vec eigenval = eigval(Hsqr);
    //cout << "natural harmonics: " << eigenval << endl;
    //Mat sortedeigval = bubbleSort(eigenval);

    //file = ofstream("M.txt");
    //for (int i = 0; i < sortedeigval.rows(); i++)
    //	file << sortedeigval(i) << endl;
    //file.close();
    throw new MBSimError("Natural Harmonics of flexible_body_2s13_mfr_mindlin were computed -> exit now ...");
    /*END NATURAL HARMONICS*/

    // LU-decomposition of M
    //LLM = facLL(M); //TODO: why this?, but it doesn't work anyway ...
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
      //qElement[i](0, RefDofs - 1) << qext(0, RefDofs - 1);
      //qElement[i](RefDofs, RefDofs + NodeDofs - 1) << qext(RefDofs + ElementNodeList(i, 0) * NodeDofs, RefDofs + (ElementNodeList(i, 0) + 1) * NodeDofs - 1);
      //qElement[i](RefDofs + NodeDofs, RefDofs + 2 * NodeDofs - 1) << qext(RefDofs + ElementNodeList(i, 1) * NodeDofs, RefDofs + (ElementNodeList(i, 1) + 1) * NodeDofs - 1);
      //qElement[i](RefDofs + 2 * NodeDofs, RefDofs + 3 * NodeDofs - 1) << qext(RefDofs + ElementNodeList(i, 2) * NodeDofs, RefDofs + (ElementNodeList(i, 2) + 1) * NodeDofs - 1);
      //qElement[i](RefDofs + 3 * NodeDofs, RefDofs + 4 * NodeDofs - 1) << qext(RefDofs + ElementNodeList(i, 3) * NodeDofs, RefDofs + (ElementNodeList(i, 3) + 1) * NodeDofs - 1);

      // mapping node dof velocity from global vector to element vector
      // ref, node 1, node 2, node 3, node 4
      //uElement[i](0, RefDofs - 1) << uext(0, RefDofs - 1);
      //uElement[i](RefDofs, RefDofs + NodeDofs - 1) << uext(RefDofs + ElementNodeList(i, 0) * NodeDofs, RefDofs + (ElementNodeList(i, 0) + 1) * NodeDofs - 1);
      //uElement[i](RefDofs + NodeDofs, RefDofs + 2 * NodeDofs - 1) << uext(RefDofs + ElementNodeList(i, 1) * NodeDofs, RefDofs + (ElementNodeList(i, 1) + 1) * NodeDofs - 1);
      //uElement[i](RefDofs + 2 * NodeDofs, RefDofs + 3 * NodeDofs - 1) << uext(RefDofs + ElementNodeList(i, 2) * NodeDofs, RefDofs + (ElementNodeList(i, 2) + 1) * NodeDofs - 1);
      //uElement[i](RefDofs + 3 * NodeDofs, RefDofs + 4 * NodeDofs - 1) << uext(RefDofs + ElementNodeList(i, 3) * NodeDofs, RefDofs + (ElementNodeList(i, 3) + 1) * NodeDofs - 1);
    }
  }

  void FlexibleBody2s13MFRMindlin::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff, Frame *frame) {
    if (cp.getContourParameterType() == CONTINUUM) { // frame on continuum
#ifdef HAVE_NURBS
      contour->updateKinematicsForFrame(cp, ff);
#endif
    }
    else if (cp.getContourParameterType() == NODE) { // frame on node
      const int &node = cp.getNodeNumber();

      if (ff == position || ff == position_cosy || ff == all) {
        Vec r_ref(3, INIT, 0.);
        r_ref(0) = qext(RefDofs + node * NodeDofs + 1) * computeThickness(NodeCoordinates(node, 0)) / 2 + NodeCoordinates(node, 0);
        r_ref(1) = -qext(RefDofs + node * NodeDofs + 2) * computeThickness(NodeCoordinates(node, 0)) / 2;
        r_ref(2) = qext(RefDofs + node * NodeDofs) + computeThickness(NodeCoordinates(node, 0)) / 2;

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

      if (ff == velocity || ff == velocities || ff == velocity_cosy || ff == velocities_cosy || ff == all) {

        Vec u(3, INIT, 0.);
        u(0) = computeThickness(NodeCoordinates(node, 0)) / 2 * uext(RefDofs + node * NodeDofs + 1);
        u(1) = -computeThickness(NodeCoordinates(node, 0)) / 2 * uext(RefDofs + node * NodeDofs + 2);
        u(2) = uext(RefDofs + node * NodeDofs);

        Vec r_ref(3, INIT, 0.);
        r_ref(0) = qext(RefDofs + node * NodeDofs + 1) * computeThickness(NodeCoordinates(node, 0)) / 2 + NodeCoordinates(node, 0);
        r_ref(1) = -qext(RefDofs + node * NodeDofs + 2) * computeThickness(NodeCoordinates(node, 0)) / 2;
        r_ref(2) = qext(RefDofs + node * NodeDofs) + computeThickness(NodeCoordinates(node, 0)) / 2;

        r_ref = TransformationMatrix(NodeCoordinates(node, 1)) * r_ref;

        SqrMat r_tyl(3, INIT, 0.); //TODO: proof especially signes
        r_tyl(0, 0) = 0;
        r_tyl(0, 1) = -r_ref(2);
        r_tyl(0, 2) = r_ref(1);

        r_tyl(1, 0) = r_ref(2);
        r_tyl(1, 1) = 0;
        r_tyl(1, 2) = -r_ref(0);

        r_tyl(2, 0) = -r_ref(1);
        r_tyl(2, 1) = r_ref(0);
        r_tyl(2, 2) = 0;

        Vec u_ref(3, INIT, 0.);
        u_ref = A * (-r_tyl * G * uext(3, 5) + TransformationMatrix(NodeCoordinates(node, 1)) * u);
        u_ref += uext(0, 2);

        cp.getFrameOfReference().setVelocity(frameOfReference->getOrientation() * u_ref);
      }

      if (ff == angularVelocity || ff == velocities || ff == velocities_cosy || ff == all) {

        Vec w_loc(3, INIT, 0.);
        w_loc(0) = uext(RefDofs + node * NodeDofs + 1);
        w_loc(1) = uext(RefDofs + node * NodeDofs + 2);

        Vec w_ref(3, INIT, 0.);
        w_ref = A * (G * uext(3, 5) + TransformationMatrix(NodeCoordinates(node, 1)) * w_loc);

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

        Mat Jacext_trans(3, Dofs, INIT, 0.), Jacext_rot(3, Dofs, INIT, 0.);

        Jacext_trans(0, 0, 2, 2) = SqrMat(3, EYE);
        Jacext_rot(0, 3, 2, 5) = A * G;

        //condensation
        Mat Jacobian_trans = condenseMatrixCols(Jacext_trans, ILocked);
        Mat Jacobian_rot = condenseMatrixCols(Jacext_rot, ILocked);

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
      Mat Jactmp_trans(3, RefDofs + NodeDofs, INIT, 0.), Jactmp_rot(3, RefDofs + NodeDofs, INIT, 0.); // Initializing Ref + 1 Node

      // translational DOFs (d/dR)
      Jactmp_trans(0, 0, 2, 2) = SqrMat(3, EYE); // ref
      //Jactmp_rot(0,0,2,2) = Mat(3,INIT,0.);

      // rotational DOFs (d/dTheta)
      SqrMat dAdalpha(3, INIT, 0.), dAdbeta(3, INIT, 0.), dAdgamma(3, INIT, 0.);

      double const &alpha = qext(3), &beta = qext(4), &gamma = qext(5);
      dAdalpha(0, 0) = 0;
      dAdalpha(0, 1) = 0;
      dAdalpha(0, 2) = 0;
      dAdalpha(1, 0) = -sin(alpha) * sin(gamma) + cos(alpha) * sin(beta) * cos(gamma);
      dAdalpha(1, 1) = -sin(alpha) * cos(gamma) - cos(alpha) * sin(beta) * sin(gamma);
      dAdalpha(1, 2) = -cos(alpha) * cos(beta);
      dAdalpha(2, 0) = cos(alpha) * sin(gamma) + sin(alpha) * sin(beta) * cos(gamma);
      dAdalpha(2, 1) = cos(alpha) * cos(gamma) - sin(alpha) * sin(beta) * sin(gamma);
      dAdalpha(2, 2) = -sin(alpha) * cos(beta);

      dAdbeta(0, 0) = -sin(beta) * cos(gamma);
      dAdbeta(0, 1) = sin(beta) * sin(gamma);
      dAdbeta(0, 2) = cos(beta);
      dAdbeta(1, 0) = sin(alpha) * cos(beta) * cos(gamma);
      dAdbeta(1, 1) = -sin(alpha) * cos(beta) * sin(gamma);
      dAdbeta(1, 2) = sin(alpha) * sin(beta);
      dAdbeta(2, 0) = -cos(alpha) * cos(beta) * cos(gamma);
      dAdbeta(2, 1) = cos(alpha) * cos(beta) * sin(gamma);
      dAdbeta(2, 2) = -cos(alpha) * sin(beta);

      dAdgamma(0, 0) = -cos(beta) * sin(gamma);
      dAdgamma(0, 1) = -cos(beta) * cos(gamma);
      dAdgamma(0, 2) = 0;
      dAdgamma(1, 0) = cos(alpha) * cos(gamma) - sin(alpha) * sin(beta) * sin(gamma);
      dAdgamma(1, 1) = -cos(alpha) * sin(gamma) - sin(alpha) * sin(beta) * cos(gamma);
      dAdgamma(1, 2) = 0;
      dAdgamma(2, 0) = sin(alpha) * cos(gamma) + cos(alpha) * sin(beta) * sin(gamma);
      dAdgamma(2, 1) = -sin(alpha) * sin(gamma) + cos(alpha) * sin(beta) * cos(gamma);
      dAdgamma(2, 2) = 0;

      Vec r_tmp(3, INIT, 0.);
      r_tmp(0) = NodeCoordinates(Node, 0) + computeThickness(NodeCoordinates(Node, 0)) / 2 * qext(RefDofs + Node * NodeDofs + 1);
      r_tmp(1) = -computeThickness(NodeCoordinates(Node, 0)) / 2 * qext(RefDofs + Node * NodeDofs + 2);
      r_tmp(2) = qext(RefDofs + Node * NodeDofs);

      r_tmp = TransformationMatrix(NodeCoordinates(Node, 1)) * r_tmp;

      Jactmp_trans(0, 3, 2, 3) = dAdalpha * r_tmp;
      Jactmp_trans(0, 4, 2, 4) = dAdbeta * r_tmp;
      Jactmp_trans(0, 5, 2, 5) = dAdgamma * r_tmp;

      Jactmp_rot(0, 3, 2, 5) = A * G;

      //elastic DOFs
      SqrMat u_tmp(3, INIT, 0.);
      u_tmp(0, 1) = computeThickness(NodeCoordinates(Node, 0)) / 2;
      u_tmp(1, 2) = -computeThickness(NodeCoordinates(Node, 0)) / 2;
      u_tmp(2, 0) = 1;

      Jactmp_trans(0, RefDofs, 2, RefDofs + 2) = A * TransformationMatrix(NodeCoordinates(Node, 1)) * u_tmp;

      //rotation
      SqrMat Z_tmp(3, INIT, 0);
      Z_tmp(0, 2) = -1;
      Z_tmp(1, 1) = 1;
      Jactmp_rot(0, RefDofs, 2, RefDofs + 2) = A * TransformationMatrix(NodeCoordinates(Node, 1)) * Z_tmp;

      /* sort in the Jacobian of the disc disk */
      // reference dofs (translational and rotational)
      Mat Jacext_trans(3, Dofs, INIT, 0.), Jacext_rot(3, Dofs, INIT, 0.);

      Jacext_trans(0, 0, 2, RefDofs - 1) = Jactmp_trans(0, 0, 2, RefDofs - 1);
      Jacext_rot(0, 0, 2, RefDofs - 1) = Jactmp_rot(0, 0, 2, RefDofs - 1);

      // elastic dofs
      Jacext_trans(0, RefDofs + Node * NodeDofs, 2, RefDofs + Node * NodeDofs + 2) = Jactmp_trans(0, RefDofs, 2, RefDofs + 2);
      Jacext_rot(0, RefDofs + Node * NodeDofs, 2, RefDofs + Node * NodeDofs + 2) = Jactmp_rot(0, RefDofs, 2, RefDofs + 2);

      //condesnation
      Mat Jacobian_trans = condenseMatrixCols(Jacext_trans, ILocked);
      Mat Jacobian_rot = condenseMatrixCols(Jacext_rot, ILocked);

      // transformation
      cp.getFrameOfReference().setJacobianOfTranslation(frameOfReference->getOrientation() * Jacobian_trans);
      cp.getFrameOfReference().setJacobianOfRotation(frameOfReference->getOrientation() * Jacobian_rot);

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
      for (int i = 0; i <= nr; i++) {
        for (int j = 0; j < nj; j++) {
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

      for (int i = 0; i < Elements; i++) {
        ElementalNodes.push_back(Vec(4, INIT, 0.));
      }

      BuildElements();

      for (int i = 0; i < Elements; i++) {
        discretization.push_back(new FiniteElement2s13MFRMindlin(E, nu, rho, d(0), d(1), d(2), ElementalNodes[i]));
        qElement.push_back(Vec(discretization[0]->getqSize(), INIT, 0.)); //TODO: discretization[i]?
        uElement.push_back(Vec(discretization[0]->getuSize(), INIT, 0.));
      }

      cout << "Alle Elemente Initialisiert ... " << endl;

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

          Diskbody->setKnotVecAzimuthal(contour->getUVector());
          Diskbody->setKnotVecRadial(contour->getVVector());

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

  Vec FlexibleBody2s13MFRMindlin::transformCW(const Vec& WrPoint) {
    Vec CrPoint = WrPoint.copy();

    CrPoint -= q(0,2);
    CrPoint = A.T()*CrPoint; //now position in moving frame of reference

    const double xt = CrPoint(0);
    const double yt = CrPoint(1);

    CrPoint(0) = sqrt(xt * xt + yt * yt);
    CrPoint(1) = ArcTan(xt, yt);

    return CrPoint;
  }

  void FlexibleBody2s13MFRMindlin::initMatrices() {
    BuildElements();
    updateAG();

    MConst = SymMat(Dofs,INIT,0.);
    computeStiffnessMatrix();
    computeConstantMassMatrixParts();
    updateM(0); //TODO: okay, that t=0?
  }

  void FlexibleBody2s13MFRMindlin::computeStiffnessMatrix() {
    SymMat Kext(Dofs - RefDofs, INIT, 0.);
    int ElementNodes = 4;

    // element loop
    for (int element = 0; element < Elements; element++) {
      static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->computeStiffnesMatrix();
      SymMat ElK = static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->getK();

      for (int node = 0; node < ElementNodes; node++) {
        for (int conode = node; conode < ElementNodes; conode++) {
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
            else //non-diagonal parts -> all entries have to be added
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
      static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->freeK();
    }//element

    SymMat Kgebl(Dofs, INIT, 0.);//TODO: muss das sein, kann man das nicht bei updatedhdz anpassen?

    for (int i = 0; i < Dofs - RefDofs; i++)
      for (int j = i; j < Dofs - RefDofs; j++)
        Kgebl(RefDofs + i, RefDofs + j) = Kext(i, j); //TODO: proof, watch out: symmetric matrices!

    K = condenseMatrix(Kgebl, ILocked);

    /*****TESTING********
    //get values to compare with ANSYS
    //cout << "KEL: " << KEl << endl;

    Vec F_test(K.size()-RefDofs,INIT, 0.);

    F_test(nj*(nr-1)*3) = 1e10;

    SymMat Ktmp(K.size()-RefDofs,INIT,0.);
    for(int i=0;i<Ktmp.size();i++)
    for(int j=i;j<Ktmp.size();j++)
    Ktmp(i,j) = K(i+RefDofs,j+RefDofs);

    //cout << "Steifigkeitsmatrix: " << Ktmp << endl;
    //cout << "Kraftlastvektor: " << F_test << endl;

    Vec q_test = slvLL(Ktmp, F_test);
    Vec u_mbsim(12,INIT,0.);
    //first: positive x-axis
    u_mbsim(0) = q_test(0);
    u_mbsim(1) = q_test(nr/2*nj*3);
    u_mbsim(2) = q_test((nr-1)*nj*3);
    //second: positive y-axis
    u_mbsim(3) = q_test(nj/4*3);
    u_mbsim(4) = q_test(nr/2*nj/4*3);
    u_mbsim(5) = q_test((nr+1)*nj/4*3);
    //third: negative x-axis
    u_mbsim(6) = q_test(nj/2*3);
    u_mbsim(7) = q_test(nr/2*nj/2*3);
    u_mbsim(8) = q_test((nr+1)*nj/2*3);
    //fourth: negative y-axis
    u_mbsim(9)  = q_test(3*nj/4*3);
    u_mbsim(10) = q_test(nr/2*3*nj/4*3);
    u_mbsim(11) = q_test((nr+1)*3*nj/4*3);

    //the twelve displacements of the Ansys example:
    Vec u_ansys("[0.10837E-15; 16.590; 50.111; -0.18542E-04; -0.85147; -2.4926;   0.0000; -0.17509; -0.31493; -0.18542E-04; -0.85147; -2.4926 ]");

    Vec err = u_ansys - u_mbsim;

    cout << "Ansys liefert: "<< u_ansys << endl;
    cout << "MBSim liefert: "<< u_mbsim << endl;

    int pos = 11;
    double maxerr = err(pos);

    for(int i=0; i<11; i++)
    if(abs(err(i))>abs(maxerr))
    {
    maxerr = err(i);
    pos = i;
    }

    cout << "positon of maximal error: " << pos << endl;

    //Ausgabe als Matrix für maxima
    //cout << "qf=matrix([" ;
    //for(int i=0; i<q_test.size(); i++)
    //  cout << q_test(i) << ",";

    //cout << "]);" << endl;

    //usgabe der Knotendurchsenkungen entlag der x-Achse (y=0)
    //for(int i=0; i<nr; i++)
    //  cout << q_test(i*nj*3) << endl;

    //Ausgabe für Gnuplot
    ofstream file;
    stringstream stringstream;
    stringstream << nr << "-nr" << "MBSim3D.txt";
    file.open(stringstream.str().c_str() ,ios::app);
    file << nj << "  ";
    file << maxerr << endl;
    file.close();

    throw new MBSimError("Stopped because of testing...");

    ******END TESTING*****************/
  }

  void FlexibleBody2s13MFRMindlin::computeConstantMassMatrixParts() {
    double ElementNodes = 4;

    /*M_RR*/
    //cout << "computeConstantMassMatrixParts() --- M_RR " << endl;
    SymMat* M_RR = new SymMat(3, INIT, 0.);
    for (int element = 0; element < Elements; element++) {
      static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->computeM_RR();
      *M_RR += static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->getM_RR(); //M_RR can be assembled directly
      static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->freeM_RR();
    }

    for (int i = 0; i < 3; i++)
      MConst(i, i) = m0 + (*M_RR)(i, i); //just diagonal entries!
    free(M_RR);
    /*M_RR end*/

    /*N_compl*/
    //cout << "computeConstantMassMatrixParts() --- N_compl " << endl;
    N_compl = new Mat(3, Dofs - RefDofs, INIT, 0.);
    // element loop
    Mat* ElN_compl = new Mat(3, NodeDofs * 4, INIT, 0.);
    for (int element = 0; element < Elements; element++) {
      static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->computeN_compl();
      Mat ElN_compl = static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->getN_compl();
      //Assembly of N_compl (just the cols have to be assembled (there is no coupling node), but for every dimension)
      for (int node = 0; node < ElementNodes; node++)//for every node
      {
        int GlobalNode = ElementNodeList(element, node) * NodeDofs;
        int localNode = node * NodeDofs;

        for (int dim = 0; dim < 3; dim++) {
          for (int l = 0; l < NodeDofs; l++) {
            (*N_compl)(dim, GlobalNode + l) += ElN_compl(dim, localNode + l);
          }//l
        }//dim
      }//node
      static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->freeN_compl();
    }//el
    free(ElN_compl);
    /*N_compl end*/

    /*N_ij*/
    //cout << "computeConstantMassMatrixParts() --- N_ij " << endl;
    SqrMat* ElN_ij = new SqrMat(NodeDofs * 4, INIT, 0.);
    for (int i = 0; i < 3; i++) {
      //cout << "i " << i << endl;
      for (int j = 0; j < 3; j++) {
        //cout << "j " << j << endl;
        N_ij[i][j] = new SqrMat(Dofs - RefDofs, INIT, 0.);
        for (int element = 0; element < Elements; element++) {
          static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->computeN_ij(i, j);
          *ElN_ij = static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->getN_ij(i, j);

          for (int node = 0; node < ElementNodes; node++)//for every node...
          {
            int GlobalNode = ElementNodeList(element, node) * NodeDofs;
            int localNode = node * NodeDofs;

            for (int conode = node; conode < ElementNodes; conode++)//...and its coupling node
            {
              int GlobalCouplingNode = ElementNodeList(element, conode) * NodeDofs;
              int localCouplingNode = conode * NodeDofs;

              //Assembly of N_ij (both (rows and cols) have to be assembled)
              for (int k = 0; k < NodeDofs; k++) {
                for (int l = 0; l < NodeDofs; l++) {
                  (*(N_ij[i][j]))(GlobalNode + k, GlobalCouplingNode + l) += (*ElN_ij)(localNode + k, localCouplingNode + l);
                }//l
              }//k
            }//conode
          }//node
          static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->freeN_ij(i, j);
        }//element
      }//j
    }//i


    //M_FF (coupling of the elastic DOFs)
    for (int i = 0; i < 3; i++)
      for (int k = RefDofs; k < Dofs; k++)
        for (int l = k; l < Dofs; l++)
          MConst(k, l) += (*(N_ij[i][i]))(k - RefDofs, l - RefDofs);
    /*N_ij end*/

    /*NR_ij*/
    //cout << "computeConstantMassMatrixParts() --- NR_ij " << endl;
    RowVec* ElNR_ij = new RowVec(NodeDofs * 4, INIT, 0.);
    for (int i = 0; i < 3; i++) {
      //cout << "i " << i << endl;
      for (int j = 0; j < 3; j++) {
        //cout << "j " << j << endl;
        NR_ij[i][j] = new RowVec(Dofs - RefDofs, INIT, 0.);
        for (int element = 0; element < Elements; element++) {
          static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->computeNR_ij(i, j);
          *ElNR_ij = static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->getNR_ij(i, j);
          for (int node = 0; node < ElementNodes; node++)//for every node...
          {
            int GlobalNode = ElementNodeList(element, node) * NodeDofs;
            int localNode = node * NodeDofs;

            //Assembly of NR_ij (just the cols have to be assembled (there is no coupling node))
            for (int l = 0; l < NodeDofs; l++) {
              (*(NR_ij[i][j]))(GlobalNode + l) += (*ElNR_ij)(localNode + l);
            }//l
          }//node
          static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->freeNR_ij(i, j);
        }//element
      }//j
    }//i
    free(ElNR_ij);
    /*NR_ij end*/

    /*R_compl*/
    //cout << "computeConstantMassMatrixParts() --- R_compl " << endl;
    R_compl = new Vec(3, INIT, 0.);
    for (int element = 0; element < Elements; element++) {
      static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->computeR_compl();
      *R_compl += static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->getR_compl();
      static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->freeR_compl();
    }//element
    /*R_compl end*/

    /*R_ij*/
    //cout << "computeConstantMassMatrixParts() --- R_compl " << endl;
    R_ij = new SymMat(3, INIT, 0.);
    for (int element = 0; element < Elements; element++) {
      static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->computeR_ij();
      *R_ij += static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->getR_ij();
      static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->freeR_ij();
    }//element
    /*R_ij end*/

    //  // element loop
    //  for (int element = 0; element < Elements; element++) {
    //    cout << "computeConstantMassMatrixParts() --- Element Nr.  " << element << endl;
    //    /*get the Element-matrices*/
    //    //M_RR, R_compl, R_ij can be assembled directly
    //    R_compl += static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->getR_compl();
    //    R_ij += static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->getR_ij();
    //
    //    //the other matrices have to be assembled later on
    //    Mat ElN_compl = static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->getN_compl();
    //    SqrMat ElN_ij[3][3];
    //    RowVec ElNR_ij[3][3];
    //    for (int i = 0; i < 3; i++)
    //      for (int j = 0; j < 3; j++) {
    //        ElN_ij[i][j] = static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->getN_ij(i, j);
    //        ElNR_ij[i][j] = static_cast<FiniteElement2s13MFRMindlin*> (discretization[element])->getNR_ij(i, j);
    //      }
    //    /**********************/
    //
    //    /*Assembly*/
    //    //Assembly of N_compl (just the cols have to be assembled (there is no coupling node), but for every dimension)
    //    for (int node = 0; node < ElementNodes; node++)//for every node
    //    {
    //      int GlobalNode = ElementNodeList(element, node) * NodeDofs;
    //      int localNode = node * NodeDofs;
    //
    //      for (int dim = 0; dim < 3; dim++) {
    //        for (int l = 0; l < NodeDofs; l++) {
    //          N_compl(dim, GlobalNode + l) += ElN_compl(dim, localNode + l);
    //        }//l
    //      }//dim
    //    }//node
    //
    //    //Assembly of the X_ij-matrices
    //    for (int i = 0; i < 3; i++) {
    //      for (int j = 0; j < 3; j++) {
    //        for (int node = 0; node < ElementNodes; node++)//for every node...
    //        {
    //          int GlobalNode = ElementNodeList(element, node) * NodeDofs;
    //          int localNode = node * NodeDofs;
    //
    //          //Assembly of NR_ij (just the cols have to be assembled (there is no coupling node))
    //          for (int l = 0; l < NodeDofs; l++) {
    //            NR_ij[i][j](GlobalNode + l) += ElNR_ij[i][j](localNode + l);
    //          }//l
    //
    //          for (int conode = node; conode < ElementNodes; conode++)//...and its coupling node
    //          {
    //            int GlobalCouplingNode = ElementNodeList(element, conode) * NodeDofs;
    //            int localCouplingNode = conode * NodeDofs;
    //
    //            //Assembly of N_ij (both (rows and cols) have to be assembled)
    //            for (int k = 0; k < NodeDofs; k++) {
    //              for (int l = 0; l < NodeDofs; l++) {
    //                N_ij[i][j](GlobalNode + k, GlobalCouplingNode + l) += ElN_ij[i][j](localNode + k, localCouplingNode + l);
    //              }//l
    //            }//k
    //          }//conode
    //        }//node
    //      }//j
    //    }//i
    //  }//el

    //cout << "computeConstantMassMatrixParts() --- Assembly beendet " << endl;

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

    G(0, 0) = cosbeta * cosgamma;
    G(0, 1) = singamma;
    G(0, 2) = 0;

    G(1, 0) = -cosbeta * singamma;
    G(1, 1) = cosgamma;
    G(1, 2) = 0;

    G(2, 0) = sinbeta;
    G(2, 1) = 0;
    G(2, 2) = 1;
  }

}

