/* Copyright (C) 2004-2010 MBSim Development Team
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

  FlexibleBody2s13MFRMindlin::FlexibleBody2s13MFRMindlin(const string &name) : FlexibleBody2s13(name), N_compl(0), R_compl(0), R_ij(0) {
    RefDofs = 6;
    for(int i=0;i<3;i++) 
      for(int j=0;j<3;j++) {
        N_ij[i][j]=0;
        NR_ij[i][j]=0;
      }
  }

  FlexibleBody2s13MFRMindlin::~FlexibleBody2s13MFRMindlin() {
    delete N_compl;
    for(int i=0;i<3;i++) 
      for(int j=0;j<3;j++) {
        delete N_ij[i][j];
        delete NR_ij[i][j];
      }
    delete R_compl;
    delete R_ij;
  }

  void FlexibleBody2s13MFRMindlin::updateM(double t) {
    SymMat Mext = MConst.copy(); // copy constant mass matrix parts
    Vec qf(Dofs-RefDofs,INIT,0.);
    // Vec qf = qext(RefDofs,Dofs-1).copy(); TODO

    /* M_RR is constant */
    /* M_RTheta */
    Vec u_bar = (*N_compl)*qf+(*R_compl);
    SqrMat M_RTheta = (-A)*tilde(u_bar)*G;

    /* M_RF */
    Mat M_RF = A*(*N_compl);

    /* M_ThetaTheta */
    SymMat I(3,INIT,0.);

    I(0,0) = (*R_ij)(1,1)+(*R_ij)(2,2)+2*((*NR_ij[1][1])+(*NR_ij[2][2]))*qf+qf.T()*((*N_ij[1][1])+(*N_ij[2][2]))*qf;
    I(0,1) = -((*R_ij)(1,0)+((*NR_ij[1][0])+(*NR_ij[0][1]))*qf+qf.T()*(*N_ij[1][0])*qf);
    I(0,2) = -((*R_ij)(2,0)+((*NR_ij[2][0])+(*NR_ij[0][2]))*qf+qf.T()*(*N_ij[2][0])*qf);
    I(1,1) = (*R_ij)(2,2)+(*R_ij)(0,0)+2*((*NR_ij[2][2])+(*NR_ij[0][0]))*qf+qf.T()*((*N_ij[2][2])+(*N_ij[0][0]))*qf;
    I(1,2) = -((*R_ij)(2,1)+((*NR_ij[2][1])+(*NR_ij[1][2]))*qf+qf.T()*(*N_ij[2][1])*qf);
    I(2,2) = (*R_ij)(1,1)+(*R_ij)(0,0)+2*((*NR_ij[1][1])+(*NR_ij[0][0]))*qf+qf.T()*((*N_ij[1][1])+(*N_ij[0][0]))*qf;

    Mat M_ThetaTheta = G.T()*(I+J0)*G;

    /* M_ThetaF */
    Mat qN(3,Dofs-RefDofs);

    qN(0,0,0,Dofs-RefDofs-1) = (*NR_ij[1][2])-(*NR_ij[2][1])+qf.T()*((*N_ij[1][2])-(*N_ij[2][1]));
    qN(1,0,1,Dofs-RefDofs-1) = (*NR_ij[2][0])-(*NR_ij[0][2])+qf.T()*((*N_ij[2][0])-(*N_ij[0][2]));
    qN(2,0,2,Dofs-RefDofs-1) = (*NR_ij[0][1])-(*NR_ij[1][0])+qf.T()*((*N_ij[0][1])-(*N_ij[1][0]));

    Mat M_ThetaF = G.T()*qN;

    /* M_FF is constant */

    /* sort into Mext */
    Mext(0,3,2,5) += M_RTheta;
    Mext(0,RefDofs,2,Dofs-1) += M_RF;

    for(int i=3;i<RefDofs;i++)
      for(int j=i;j<RefDofs;j++)
        Mext(i,j)+=M_ThetaTheta(i-3,j-3);

    Mext(3,RefDofs,5,Dofs-1)+=M_ThetaF;

    M = condenseMatrix(Mext,ILocked).copy();

    ofstream file_M("M.txt");
    file_M << M << endl;
    file_M << eigval(M) << endl;
    file_M.close();

    /* EIGENFREQUENCIES */
    SqrMat H = static_cast<SqrMat>(inv(M)*K);
    ofstream file_eigval("EigVal.txt");
    file_eigval << eigval(H) << endl;
    file_eigval.close();

    // LU-decomposition of M
    LLM = facLL(M); 
  }

  void FlexibleBody2s13MFRMindlin::BuildElements() {
    for(int i=0;i<Elements;i++) {
      //  ^ phi
      //  |
      //  |   4--------3
      //  |   |        |
      //  |   1--------2
      //  |
      //  | --------------> r
      // radial and azimuthal coordinates of the FE [ElementalNodes(r1,phi1,r2,phi2)]
      // r1 and phi1 are defined with node 1, r2 and phi2 with node 3
      ElementalNodes[i](0,1) << NodeCoordinates.row(ElementNodeList(i,0)).T(); // node 1
      ElementalNodes[i](2,3) << NodeCoordinates.row(ElementNodeList(i,2)).T(); // node 3

      if(ElementalNodes[i](3) <= ElementalNodes[i](1)) { // ring closure
        ElementalNodes[i](3) += 2*M_PI;
      }
    }
  }

  void FlexibleBody2s13MFRMindlin::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff, Frame *frame) {
    if(cp.getContourParameterType() == CONTINUUM) { // frame on continuum
#ifdef HAVE_NURBS
      contour->updateKinematicsForFrame(cp,ff);
#endif
    }
    else if(cp.getContourParameterType() == NODE) { // frame on node
      const int &node = cp.getNodeNumber();

      if(ff == position || ff == position_cosy || ff == all) {
        Vec r_ref(3,NONINIT);
        r_ref(0) = qext(RefDofs+node*NodeDofs+1)*computeThickness(NodeCoordinates(node,0))/2.+NodeCoordinates(node,0);
        r_ref(1) = -qext(RefDofs+node*NodeDofs+2)*computeThickness(NodeCoordinates(node,0))/2.;
        r_ref(2) = qext(RefDofs+node*NodeDofs)+computeThickness(NodeCoordinates(node,0))/2.;

        r_ref = A*TransformationMatrix(NodeCoordinates(node,1))*r_ref;
        r_ref += qext(0,2);
        cp.getFrameOfReference().setPosition(frameOfReference->getPosition()+frameOfReference->getOrientation()*r_ref);
      }

      if(ff == firstTangent || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all) throw new MBSimError("ERROR(FlexibleBody2s13MFRMindlin::updateKinematicsForFrame): Not implemented!");
      if(ff == normal || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all) throw new MBSimError("ERROR(FlexibleBody2s13MFRMindlin::updateKinematicsForFrame): Not implemented!");
      if(ff == secondTangent || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy || ff == all) throw new MBSimError("ERROR(FlexibleBody2s13MFRMindlin::updateKinematicsForFrame): Not implemented!");

      if(ff == velocity || ff == velocities || ff == velocity_cosy || ff == velocities_cosy || ff == all) {
        Vec u_ref_1(3,NONINIT);
        u_ref_1(0) = computeThickness(NodeCoordinates(node,0))/2.*uext(RefDofs+node*NodeDofs+1);
        u_ref_1(1) = -computeThickness(NodeCoordinates(node,0))/2.*uext(RefDofs+node*NodeDofs+2);
        u_ref_1(2) = uext(RefDofs+node*NodeDofs);

        Vec r_ref(3,NONINIT);
        r_ref(0) = qext(RefDofs+node*NodeDofs+1)*computeThickness(NodeCoordinates(node,0))/2.+NodeCoordinates(node,0);
        r_ref(1) = -qext(RefDofs+node*NodeDofs+2)*computeThickness(NodeCoordinates(node,0))/2.;
        r_ref(2) = qext(RefDofs+node*NodeDofs)+computeThickness(NodeCoordinates(node,0))/2.;

        r_ref = TransformationMatrix(NodeCoordinates(node,1))*r_ref;

        Vec u_ref_2 = A*(-tilde(r_ref)*G*uext(3,5)+TransformationMatrix(NodeCoordinates(node,1))*u_ref_1);
        u_ref_2 += uext(0,2);

        cp.getFrameOfReference().setVelocity(frameOfReference->getOrientation()*u_ref_2);
      }

      if(ff == angularVelocity || ff == velocities || ff == velocities_cosy || ff == all) {
        Vec w_ref_1(3,INIT,0.);
        w_ref_1(0) = -uext(RefDofs+node*NodeDofs+2);
        w_ref_1(1) = uext(RefDofs+node*NodeDofs+1);

        Vec w_ref_2 = A*(G*uext(3,5)+TransformationMatrix(NodeCoordinates(node,1))*w_ref_1);

        cp.getFrameOfReference().setAngularVelocity(frameOfReference->getOrientation()*w_ref_2);
      }
    }
    else throw new MBSimError("ERROR(FlexibleBody2s13MFRMindlin::updateKinematicsForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    if(frame!=0) { // frame should be linked to contour point data
      frame->setPosition       (cp.getFrameOfReference().getPosition());
      frame->setOrientation    (cp.getFrameOfReference().getOrientation());
      frame->setVelocity       (cp.getFrameOfReference().getVelocity());
      frame->setAngularVelocity(cp.getFrameOfReference().getAngularVelocity());
    }
  }

  void FlexibleBody2s13MFRMindlin::updateJacobiansForFrame(ContourPointData &cp, Frame *frame) {
    if(cp.getContourParameterType() == CONTINUUM) { // force on continuum
      Vec alpha = cp.getLagrangeParameterPosition();

      if(nrm2(alpha) < epsroot()) { // center of gravity
        Mat Jacext_trans(3,Dofs,INIT,0.), Jacext_rot(3,Dofs,INIT,0.);

        Jacext_trans(0,0,2,2) = SqrMat(3,EYE);
        Jacext_rot(0,3,2,5) = A*G;

        // condensation
        Mat Jacobian_trans = condenseMatrixCols(Jacext_trans,ILocked);
        Mat Jacobian_rot = condenseMatrixCols(Jacext_rot,ILocked);

        // transformation
        cp.getFrameOfReference().setJacobianOfTranslation(frameOfReference->getOrientation() * Jacobian_trans);
        cp.getFrameOfReference().setJacobianOfRotation(frameOfReference->getOrientation() * Jacobian_rot);
      }
      else { // on the disk
        contour->updateJacobiansForFrame(cp);
      }
    }

    else if(cp.getContourParameterType() == NODE) { // force on node
      int Node = cp.getNodeNumber();

      // Jacobian of element
      Mat Jactmp_trans(3,RefDofs+NodeDofs,INIT,0.), Jactmp_rot(3,RefDofs+NodeDofs,INIT,0.); // initializing Ref + 1 Node

      // translational DOFs (d/dR)
      Jactmp_trans(0,0,2,2) = SqrMat(3,EYE); // ref

      // rotational DOFs (d/dTheta)
      SqrMat dAdalpha(3,NONINIT), dAdbeta(3,NONINIT), dAdgamma(3,NONINIT);

      double const &alpha = qext(3);
      double const &beta = qext(4);
      double const &gamma = qext(5);

      dAdalpha(0,0) = 0.;
      dAdalpha(0,1) = 0.;
      dAdalpha(0,2) = 0.;
      dAdalpha(1,0) = -sin(alpha)*sin(gamma)+cos(alpha)*sin(beta)*cos(gamma);
      dAdalpha(1,1) = -sin(alpha)*cos(gamma)-cos(alpha)*sin(beta)*sin(gamma);
      dAdalpha(1,2) = -cos(alpha)*cos(beta);
      dAdalpha(2,0) = cos(alpha)*sin(gamma)+sin(alpha)*sin(beta)*cos(gamma);
      dAdalpha(2,1) = cos(alpha)*cos(gamma)-sin(alpha)*sin(beta)*sin(gamma);
      dAdalpha(2,2) = -sin(alpha)*cos(beta);

      dAdbeta(0,0) = -sin(beta)*cos(gamma);
      dAdbeta(0,1) = sin(beta)*sin(gamma);
      dAdbeta(0,2) = cos(beta);
      dAdbeta(1,0) = sin(alpha)*cos(beta)*cos(gamma);
      dAdbeta(1,1) = -sin(alpha)*cos(beta)*sin(gamma);
      dAdbeta(1,2) = sin(alpha)*sin(beta);
      dAdbeta(2,0) = -cos(alpha)*cos(beta)*cos(gamma);
      dAdbeta(2,1) = cos(alpha)*cos(beta)*sin(gamma);
      dAdbeta(2,2) = -cos(alpha)*sin(beta);

      dAdgamma(0,0) = -cos(beta)*sin(gamma);
      dAdgamma(0,1) = -cos(beta)*cos(gamma);
      dAdgamma(0,2) = 0.;
      dAdgamma(1,0) = cos(alpha)*cos(gamma)-sin(alpha)*sin(beta)*sin(gamma);
      dAdgamma(1,1) = -cos(alpha)*sin(gamma)-sin(alpha)*sin(beta)*cos(gamma);
      dAdgamma(1,2) = 0.;
      dAdgamma(2,0) = sin(alpha)*cos(gamma)+cos(alpha)*sin(beta)*sin(gamma);
      dAdgamma(2,1) = -sin(alpha)*sin(gamma)+cos(alpha)*sin(beta)*cos(gamma);
      dAdgamma(2,2) = 0.;

      Vec r_tmp(3,NONINIT);
      r_tmp(0) = NodeCoordinates(Node,0)+computeThickness(NodeCoordinates(Node,0))/2.*qext(RefDofs+Node*NodeDofs+1);
      r_tmp(1) = -computeThickness(NodeCoordinates(Node,0))/2.*qext(RefDofs+Node*NodeDofs+2);
      r_tmp(2) = qext(RefDofs+Node*NodeDofs);

      r_tmp = TransformationMatrix(NodeCoordinates(Node,1))*r_tmp;

      Jactmp_trans(0,3,2,3) = dAdalpha*r_tmp;
      Jactmp_trans(0,4,2,4) = dAdbeta*r_tmp;
      Jactmp_trans(0,5,2,5) = dAdgamma*r_tmp;

      Jactmp_rot(0,3,2,5) = A*G;

      // elastic DOFs
      // translation
      SqrMat u_tmp(3,INIT,0.);
      u_tmp(0,1) = computeThickness(NodeCoordinates(Node,0))/2.;
      u_tmp(1,2) = -computeThickness(NodeCoordinates(Node,0))/2.;
      u_tmp(2,0) = 1.;

      Jactmp_trans(0,RefDofs,2,RefDofs+2) = A*TransformationMatrix(NodeCoordinates(Node,1))*u_tmp;

      // rotation
      SqrMat Z_tmp(3,INIT,0.);
      Z_tmp(0,2) = -1;
      Z_tmp(1,1) = 1;
      Jactmp_rot(0,RefDofs,2,RefDofs+2) = A*TransformationMatrix(NodeCoordinates(Node,1))*Z_tmp;

      // sort in the Jacobian of the disc disk
      // reference dofs 
      Mat Jacext_trans(3,Dofs,INIT,0.), Jacext_rot(3,Dofs,INIT,0.);

      Jacext_trans(0,0,2,RefDofs-1) = Jactmp_trans(0,0,2,RefDofs-1);
      Jacext_rot(0,0,2,RefDofs-1) = Jactmp_rot(0,0,2,RefDofs-1);

      // elastic dofs
      Jacext_trans(0,RefDofs+Node*NodeDofs,2,RefDofs+Node*NodeDofs+2) = Jactmp_trans(0,RefDofs,2,RefDofs+2);
      Jacext_rot(0,RefDofs+Node*NodeDofs,2,RefDofs+Node*NodeDofs+2) = Jactmp_rot(0,RefDofs,2,RefDofs+2);

      // condensation
      Mat Jacobian_trans = condenseMatrixCols(Jacext_trans,ILocked);
      Mat Jacobian_rot = condenseMatrixCols(Jacext_rot,ILocked);

      // transformation
      cp.getFrameOfReference().setJacobianOfTranslation(frameOfReference->getOrientation() * Jacobian_trans);
      cp.getFrameOfReference().setJacobianOfRotation(frameOfReference->getOrientation() * Jacobian_rot);

    }
    else throw new MBSimError("ERROR(FlexibleBody2s13MFRMindlin::updateJacobiansForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    // cp.getFrameOfReference().setGyroscopicAccelerationOfTranslation(TODO)
    // cp.getFrameOfReference().setGyroscopicAccelerationOfRotation(TODO)

    if(frame!=0) { // frame should be linked to contour point data
      frame->setJacobianOfTranslation(cp.getFrameOfReference().getJacobianOfTranslation());
      frame->setJacobianOfRotation   (cp.getFrameOfReference().getJacobianOfRotation());
      frame->setGyroscopicAccelerationOfTranslation(cp.getFrameOfReference().getGyroscopicAccelerationOfTranslation());
      frame->setGyroscopicAccelerationOfRotation   (cp.getFrameOfReference().getGyroscopicAccelerationOfRotation());
    }
  }

  void FlexibleBody2s13MFRMindlin::init(InitStage stage) {
    if(stage == resize) {
      FlexibleBodyContinuum<Vec>::init(stage);
      assert(nr>0); // at least on radial row
      assert(nj>1); // at least two azimuthal elements

      // condensation
      switch(LType) {
        case innerring: // 0: innerring
          ILocked = Index(RefDofs,RefDofs+NodeDofs*nj-1);
          Jext = Mat(Dofs,qSize,INIT,0.);
          Jext(0,0,RefDofs-1,RefDofs-1) << DiagMat(RefDofs,INIT,1.);
          Jext(RefDofs+NodeDofs*nj,RefDofs,Dofs-1,qSize-1) << DiagMat(qSize-RefDofs,INIT,1.);
          break;

        case outerring: // 1: outerring
          ILocked = Index(qSize,Dofs-1);
          Jext = Mat(Dofs,qSize,INIT,0.);
          Jext(0,0,qSize-1,qSize-1) << DiagMat(qSize,INIT,1.);
          break;
      }

      dr = (Ra-Ri)/nr;
      dj = 2*M_PI/nj;

      NodeCoordinates = Mat(Nodes,2);
      ElementNodeList.resize(Elements,4);

      // mapping nodes - node coordinates - elements
      for(int i=0;i<=nr;i++) {
        for(int j=0;j<nj;j++) {
          // node number increases azimuthally from the inner to the outer ring
          NodeCoordinates(j+i*nj,0) = Ri+dr*i;
          NodeCoordinates(j+i*nj,1) = 0.+dj*j;

          // element number increases azimuthally from the inner to the outer ring
          if(i<nr && j<nj-1) { 
            ElementNodeList(j+i*nj,0) = j     + i    *nj; // elementnode 1
            ElementNodeList(j+i*nj,1) = j     + (i+1)*nj; // elementnode 2
            ElementNodeList(j+i*nj,2) = (j+1) + (i+1)*nj; // elementnode 3
            ElementNodeList(j+i*nj,3) = (j+1) +  i   *nj; // elementnode 4
          }
          else if(i<nr && j==nj-1) { // ring closure 
            ElementNodeList(j+i*nj,0) = j     + i    *nj; // elementnode 1
            ElementNodeList(j+i*nj,1) = j     + (i+1)*nj; // elementnode 2
            ElementNodeList(j+i*nj,2) = 0     + (i+1)*nj; // elementnode 3
            ElementNodeList(j+i*nj,3) = 0     +  i   *nj; // elementnode 4
          }
        }
      }

      for(int i=0; i<Elements;i++) {
        ElementalNodes.push_back(Vec(4,INIT,0.));
      }

      BuildElements();

      for(int i=0;i<Elements;i++) {
        discretization.push_back(new FiniteElement2s13MFRMindlin(E,nu,rho,d(0),d(1),d(2),ElementalNodes[i]));
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
      alphaE(1) = 2*M_PI; // angle

      contour->setAlphaStart(alphaS);
      contour->setAlphaEnd(alphaE);
#endif

      qext = Jext * q0;
      uext = Jext * u0;

      initMatrices(); // calculate constant stiffness matrix and the constant parts of the mass-matrix

    }
    if(stage==MBSim::plot) {
      updatePlotFeatures(parent);

      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
#ifdef HAVE_NURBS
        if (getPlotFeature(openMBV)==enabled) {
          OpenMBV::NurbsDisk *Diskbody = new OpenMBV::NurbsDisk;

          drawDegree = 30/nj;
          Diskbody->setStaticColor(0.3);
          Diskbody->setMinimalColorValue(0.);
          Diskbody->setMaximalColorValue(1.);
          Diskbody->setDrawDegree(drawDegree);
          Diskbody->setRadii(Ri,Ra);

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
    CrPoint = A.T()*CrPoint; // position in moving frame of reference

    const double xt = CrPoint(0);
    const double yt = CrPoint(1);

    CrPoint(0) = sqrt(xt*xt + yt*yt);
    CrPoint(1) = ArcTan(xt,yt);

    return CrPoint;
  }

  void FlexibleBody2s13MFRMindlin::initMatrices() {
    updateAG();

    computeStiffnessMatrix();
    computeConstantMassMatrixParts();
    updateM(0); 
  }

  void FlexibleBody2s13MFRMindlin::updateAG() {
    double sinalpha = sin(q(3));
    double cosalpha = cos(q(3));
    double sinbeta = sin(q(4));
    double cosbeta = cos(q(4));
    double singamma = sin(q(5));
    double cosgamma = cos(q(5));

    A(0,0) = cosbeta * cosgamma;
    A(0,1) = -cosbeta * singamma;
    A(0,2) = sinbeta;

    A(1,0) = cosalpha * singamma + sinalpha * sinbeta * cosgamma;
    A(1,1) = cosalpha * cosgamma - sinalpha * sinbeta * singamma;
    A(1,2) = -sinalpha * cosbeta;

    A(2,0) = sinalpha * singamma - cosalpha * sinbeta * cosgamma;
    A(2,1) = sinalpha * cosgamma + cosalpha * sinbeta * singamma;
    A(2,2) = cosalpha * cosbeta;

    G(0,0) = cosbeta * cosgamma;
    G(0,1) = singamma;
    G(0,2) = 0.;

    G(1,0) = -cosbeta * singamma;
    G(1,1) = cosgamma;
    G(1,2) = 0.;

    G(2,0) = sinbeta;
    G(2,1) = 0.;
    G(2,2) = 1.;
  }

  void FlexibleBody2s13MFRMindlin::computeStiffnessMatrix() {
    SymMat Kext(Dofs,INIT,0.); 
    int ElementNodes = 4;

    // element loop
    for(int element=0;element< Elements;element++) {
      static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->computeStiffnesMatrix();
      SymMat ElK = static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->getK();

      for(int node=0;node<ElementNodes;node++) { 
        Index Ikges(RefDofs+ElementNodeList(element,node)*NodeDofs,RefDofs+(ElementNodeList(element,node)+1)*NodeDofs-1);
        Index Ikelement(node*NodeDofs,(node+1)*NodeDofs-1);

        Kext(Ikges) += ElK(Ikelement); // diagonal
        for(int n=node+1;n<ElementNodes;n++) // secondary diagonals
          Kext(Ikges,Index(RefDofs+ElementNodeList(element,n)*NodeDofs,RefDofs+(ElementNodeList(element,n)+1)*NodeDofs-1)) += ElK(Ikelement,Index(n*NodeDofs,(n+1)*NodeDofs-1));
      }
      static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->freeK();
    }

    // condensation
    K = condenseMatrix(Kext,ILocked);

    /* STATIC TEST */
    Index Iall(RefDofs,K.size()-1);

    // load
    Vec F_test(K.size()-RefDofs,INIT,0.);
    F_test((nr-1)*nj*3) = 1e10;

    // displacements in MBSim    
    Vec q_test = slvLL(K(Iall),F_test);
    Vec u_mbsim(12,NONINIT);
    // first: positive x-axis
    u_mbsim(0) = q_test(0);
    u_mbsim(1) = q_test(nr/2*nj*3);
    u_mbsim(2) = q_test((nr-1)*nj*3);
    // second: positive y-axis
    u_mbsim(3) = q_test(nj/4*3);
    u_mbsim(4) = q_test(nr/2*nj*3+nj/4*3); 
    u_mbsim(5) = q_test((nr-1)*nj*3+nj/4*3); 
    // third: negative x-axis
    u_mbsim(6) = q_test(nj/2*3);
    u_mbsim(7) = q_test(nr/2*nj*3+nj/2*3);
    u_mbsim(8) = q_test((nr-1)*nj*3+nj/2*3);
    // fourth: negative y-axis
    u_mbsim(9)  = q_test(3*nj/4*3);
    u_mbsim(10) = q_test(nr/2*nj*3+3*nj/4*3); 
    u_mbsim(11) = q_test((nr-1)*nj*3+3*nj/4*3);

    // displacements in ANSYS
    Vec u_ansys("[0.10837E-15; 16.590; 50.111; -0.18542E-04; -0.85147; -2.4926; 0.0000; -0.17509; -0.31493; -0.18542E-04; -0.85147; -2.4926 ]");

    // error
    double maxerr = nrmInf(u_ansys-u_mbsim);

    // output
    ofstream file_static;
    file_static.open("Static.txt");
    file_static << "error=" << maxerr << endl;
    file_static << "static_mbsim=matrix([" ;
    for(int i=0;i<u_mbsim.size();i++)
      file_static << u_mbsim(i) << ",";
    file_static << "]);" << endl;
    file_static.close();
  }

  void FlexibleBody2s13MFRMindlin::computeConstantMassMatrixParts() {
    MConst = SymMat(Dofs,INIT,0.);
    double ElementNodes = 4;

    /* M_RR */
    for(int i=0;i<3;i++) {
      MConst(i,i) += m0;
      for(int element=0;element<Elements;element++) {
        static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->computeM_RR();
        MConst(i,i) += static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->getM_RR()(i,i);
        static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->freeM_RR();
      }
    }

    /* N_compl */
    N_compl = new Mat(3,Dofs-RefDofs,INIT,0.);
    //for(int element=0;element<Elements;element++) { // TODO
    //  static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->computeN_compl();
    //  Mat ElN_compl = static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->getN_compl();
    //  Index IRefTrans(0,2);
    //  for(int node=0;node<ElementNodes;node++) {
    //    Index Ikges(ElementNodeList(element,node)*NodeDofs,(ElementNodeList(element,node)+1)*NodeDofs-1);
    //    Index Ikelement(node*NodeDofs,(node+1)*NodeDofs-1);
    //    (*N_compl)(IRefTrans,Ikges) += ElN_compl(IRefTrans,Ikelement);
    //  }
    //  static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->freeN_compl();
    //}

    /* N_ij */
    for(int i=0;i<3;i++) {
      for(int j=0;j<3;j++) {
        N_ij[i][j] = new SqrMat(Dofs-RefDofs,INIT,0.);
        for(int element=0;element<Elements;element++) {
          static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->computeN_ij(i,j);
          SqrMat ElN_ij = static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->getN_ij(i,j);

          for(int node=0;node<ElementNodes;node++) { 
            Index Ikges(ElementNodeList(element,node)*NodeDofs,(ElementNodeList(element,node)+1)*NodeDofs-1);
            Index Ikelement(node*NodeDofs,(node+1)*NodeDofs-1);

            (*(N_ij[i][j]))(Ikges) += ElN_ij(Ikelement); // diagonal
            for(int n=node+1;n<ElementNodes;n++) // secondary diagonals
              (*(N_ij[i][j]))(Ikges,Index(ElementNodeList(element,n)*NodeDofs,(ElementNodeList(element,n)+1)*NodeDofs-1)) += ElN_ij(Ikelement,Index(n*NodeDofs,(n+1)*NodeDofs-1));
          }
          static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->freeN_ij(i,j);
        }
      }
    }

    /* M_FF */
    for(int i=0;i<3;i++)
      for(int k=RefDofs;k<Dofs;k++)
        for(int l=k;l<Dofs;l++)
          MConst(k,l) += (*(N_ij[i][i]))(k-RefDofs,l-RefDofs);

    /* NR_ij */
    for(int i=0;i<3;i++) {
      for(int j=0;j<3;j++) {
        NR_ij[i][j] = new RowVec(Dofs-RefDofs,INIT,0.);
        //for(int element=0;element<Elements;element++) { TODO
        //  static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->computeNR_ij(i,j);
        //  RowVec ElNR_ij = static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->getNR_ij(i,j);
        //  for(int node=0;node<ElementNodes;node++) {
        //    Index Ikges(ElementNodeList(element,node)*NodeDofs,(ElementNodeList(element,node)+1)*NodeDofs-1);
        //    Index Ikelement(node*NodeDofs,(node+1)*NodeDofs-1);
        //    (*(NR_ij[i][j]))(Ikges) += ElNR_ij(Ikelement);
        //  }
        //  static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->freeNR_ij(i,j);
        //}
      }
    }

    /* R_compl */
    R_compl = new Vec(3,INIT,0.);
    for(int element=0;element<Elements;element++) {
      static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->computeR_compl();
      *R_compl += static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->getR_compl();
      static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->freeR_compl();
    }

    /* R_ij */
    R_ij = new SymMat(3,INIT,0.);
    for(int element=0;element<Elements;element++) {
      static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->computeR_ij();
      *R_ij+=static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->getR_ij();
      static_cast<FiniteElement2s13MFRMindlin*>(discretization[element])->freeR_ij();
    }

  }
}

