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
 * Contact: rzander@users.berlios.de
 *          thschindler@users.berlios.de
 */

#include <config.h>
#include <mbsim/flexible_body/flexible_body_1s_21_rcm.h>
#include <mbsim/flexible_body/finite_elements/finite_element_1s_21_rcm.h>

#include <mbsim/frame.h>
#include <mbsim/multi_body_system.h>
#include <mbsim/contour.h>

#define FMATVEC_DEEP_COPY

#ifdef HAVE_AMVIS
#include "elastic1s21rcm.h"
using namespace AMVis;
#endif

namespace MBSim {

  FlexibleBody1s21RCM::FlexibleBody1s21RCM(const string &name, bool openStructure_) :FlexibleBody1s(name), L(0), l0(0), E(0), A(0), I(0), rho(0), rc(0), dm(0), dl(0), openStructure(openStructure_), implicit(false), initialized(false), alphaRelax0(-99999.99999), alphaRelax(alphaRelax0)
#ifdef HAVE_AMVIS
                                                                                     ,
                                                                                     AMVisRadius(0), AMVisBreadth(0), AMVisHeight(0)
#endif

                                                                                     { 
                                                                                       contourR = new Contour1sFlexible("R");
                                                                                       contourL = new Contour1sFlexible("L");
                                                                                       ContourPointData cpTmp;
                                                                                       FlexibleBody::addContour(contourR);
                                                                                       FlexibleBody::addContour(contourL);
                                                                                     }

  void FlexibleBody1s21RCM::GlobalMatrixContribution(int n) {
    int j = 5 * n;

    if ( n < Elements - 1 || openStructure==true) {
      M(Index(j,j+7))   += discretization[n]->getMassMatrix();
      h(j,j+7)          += discretization[n]->getGeneralizedForceVector();

      //if(implicit) {
      //  Dhq (j,j,j+7,j+7) += discretization[n]->getJacobianForImplicitIntegrationRegardingPosition;
      //  Dhqp(j,j,j+7,j+7) += discretization[n]->getJacobianForImplicitIntegrationRegardingVelocity;
      //}
    } 
    else { // ring closure at finite element (end,1) with angle difference 2*M_PI
      M(Index(j,j+4))               += discretization[n]->getMassMatrix()(Index(0,4));
      M(Index(j,j+4),Index(0,2))    += discretization[n]->getMassMatrix()(Index(0,4),Index(5,7));
      M(Index(0,2))                 += discretization[n]->getMassMatrix()(Index(5,7));

      h(j,j+4)                      += discretization[n]->getGeneralizedForceVector()(0,4);
      h(0,  2)                      += discretization[n]->getGeneralizedForceVector()(5,7);

      //if(implicit) {
      //  Dhq (j,j,j+4,j+4) += discretization[n]->getJacobianForImplicitIntegrationRegardingPosition(0,0,4,4);
      //  Dhq (j,0,j+4,  2) += discretization[n]->getJacobianForImplicitIntegrationRegardingPosition(0,5,4,7);
      //  Dhq (0,j,  2,j+4) += discretization[n]->getJacobianForImplicitIntegrationRegardingPosition(5,0,7,4);
      //  Dhq (0,0,  2,  2) += discretization[n]->getJacobianForImplicitIntegrationRegardingPosition(5,5,7,7);

      //  Dhqp(j,j,j+4,j+4) += discretization[n]->getJacobianForImplicitIntegrationRegardingVelocity(0,0,4,4);
      //  Dhqp(j,0,j+4,  2) += discretization[n]->getJacobianForImplicitIntegrationRegardingVelocity(0,5,4,7);
      //  Dhqp(0,j,  2,j+4) += discretization[n]->getJacobianForImplicitIntegrationRegardingVelocity(5,0,7,4);
      //  Dhqp(0,0,  2,  2) += discretization[n]->getJacobianForImplicitIntegrationRegardingVelocity(5,5,7,7);
      //}
    }
  }

  Mat FlexibleBody1s21RCM::computeJacobianMatrix(const ContourPointData &S_) {
    Index All(0,3-1);
    Mat Jacobian(qSize,3,INIT,0.0);

    if(S_.type == CONTINUUM) { // force on continuum
      double s = S_.alpha(0); // globale contour parameter
      double sLokal = BuildElement(s);
      Mat Jtmp = dynamic_cast<FiniteElement1s21RCM*>(discretization[CurrentElement])->JGeneralized(qElement[CurrentElement],sLokal);
      if(CurrentElement<Elements-1 || openStructure) {
        Index Dofs(5*CurrentElement,5*CurrentElement+7);
        Jacobian(Dofs,All) = Jtmp;
      }
      else { // ringstructure
        Jacobian(Index(5*CurrentElement,5*CurrentElement+4),All) = Jtmp(Index(0,4),All);
        Jacobian(Index(               0,                 2),All) = Jtmp(Index(5,7),All);
      }

    }
    else if(S_.type == NODE) { // force on node
      int node = S_.ID;
      Index Dofs(5*node,5*node+2);
      Jacobian(Dofs,All) << DiagMat(3,INIT,1.0);
    }
    return Jacobian;
  }
  
  Frame* FlexibleBody1s21RCM::computeKinematicsForFrame(const ContourPointData &S_) {
    for(unsigned int i=0; i<port.size(); i++) {
      if(S_Frame[i].type == CONTINUUM) { // force on continuum
        const double     &s = S_Frame[i].alpha(0); // globale contour parameter
        double sLokal = BuildElement(s);
        Vec Z = dynamic_cast<FiniteElement1s21RCM*>(discretization[CurrentElement])->StateBalken(qElement[CurrentElement],uElement[CurrentElement],sLokal);
        port[i]->setWrOP(frameParent->getPosition() + frameParent->getOrientation()*JT*Z(0,1));
        port[i]->setWvP(frameParent->getOrientation()*JT*Z(3,4));
        port[i]->setWomegaP(frameParent->getOrientation()*JR*Z(5,5));
      }
      else if(S_Frame[i].type == NODE) { // force on node
        const int &node = S_Frame[i].ID;
        port[i]->setWrOP(frameParent->getPosition() + frameParent->getOrientation()*JT*q(5*node+0,5*node+1));
        port[i]->setWvP(frameParent->getOrientation()*JT*u(5*node+0,5*node+1));
        port[i]->setWomegaP(frameParent->getOrientation()*JR*u(5*node+2,5*node+2));
      }
    }
  }

  void FlexibleBody1s21RCM::init() {
    FlexibleBody::init();

    initialized = true;

    SqrMat AWC(3,INIT,0.0);
    AWC(Index(0,2),Index(0,1)) = JT;
    AWC(Index(0,2),Index(2,2)) = JR;
    contourR->setAWC(AWC);
    contourL->setAWC(AWC);

    Vec contourRBinormal(3,INIT,0.0); contourRBinormal(2) = 1.0;
    Vec contourLBinormal = - contourRBinormal;

    contourR->setCb(contourRBinormal);
    contourL->setCb(contourLBinormal);

    contourR->setAlphaStart(0); contourR->setAlphaEnd(L);
    contourL->setAlphaStart(0); contourL->setAlphaEnd(L);
    if(userContourNodes.size()==0) {
      Vec contourNodes(Elements+1);
      for(int i=0;i<=Elements;i++)
        contourNodes(i) = L/Elements * i; // search area for each finite element contact search
      contourR->setNodes(contourNodes);
      contourL->setNodes(contourNodes);
    }
    else {
      contourR->setNodes(userContourNodes);
      contourL->setNodes(userContourNodes);
    }

    l0 = L/Elements;
    Vec g = trans(JT)*mbs->getGrav();
    for(int i=0;i<Elements;i++) {
      qElement.push_back(Vec(8,INIT,0.));
      uElement.push_back(Vec(8,INIT,0.));
      discretization.push_back(new FiniteElement1s21RCM(l0, A*rho, E*A, E*I, g));
      if(rc != 0) dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->setCurlRadius(rc);
      dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->setMaterialDamping(dm);
      dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->setLehrDamping(dl);
    }
    if(alphaRelax != alphaRelax0) initRelaxed(alphaRelax);

#ifdef HAVE_AMVIS
    if(boolAMVis) {
      ElasticBody1s21RCM *RCMbody = new ElasticBody1s21RCM(fullName,Elements,openStructure,1,boolAMVisBinary);
      RCMbody->setElementLength(l0);

      float amvisJT[3][2], amvisJR[3];
      for(int i=0;i<3;i++) {
        for(int j=0;j<2;j++) amvisJT[i][j] = JT(i,j);
        amvisJR[i] = JR(i,0);
      }
      RCMbody->setJacobians(amvisJT,amvisJR);
      RCMbody->setInitialTranslation(frameParent->getPosition()(0),frameParent->getPosition()(1),frameParent->getPosition()(2));
      RCMbody->setCylinder(AMVisRadius);
      RCMbody->setCuboid(AMVisBreadth,AMVisHeight);
      RCMbody->setColor(AMVisColor);

      bodyAMVis = RCMbody;
    } 
#endif
  }

  double FlexibleBody1s21RCM::computePotentialEnergy () {
    double V = 0.0;
    for(int i=0;i<Elements;i++) {
      BuildElement(i);
      V += dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->computeV(qElement[i]);
    }
    return V;
  }

  void FlexibleBody1s21RCM::updateKinematics(double t) {
    BuildElements();
    sTangent = -l0;
    FlexibleBody::updateKinematics(t);
  }

  void FlexibleBody1s21RCM::setNumberElements(int n) {
    Elements = n;
    if(openStructure) qSize = 5*n+3; 
    else qSize = 5*n;
    uSize[0] = qSize;
    q0.resize(qSize);
    u0.resize(uSize[0]);
  }

  void FlexibleBody1s21RCM::setCurlRadius(double r) {
    rc = r;
    if(initialized) 
      for(int i=0;i<Elements;i++) 
        dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->setCurlRadius(rc);
  }

  void FlexibleBody1s21RCM::setMaterialDamping(double d) {
    dm = d;
    if(initialized) 
      for(int i=0;i<Elements;i++) dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->setMaterialDamping(dm);
  }

  void FlexibleBody1s21RCM::setLehrDamping(double d) {
    dl = d;
    if(initialized) 
      for(int i=0;i<Elements;i++) dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->setLehrDamping(dl);
  }

  void FlexibleBody1s21RCM::updateJh_internal(double t) {
    if(!implicit) { 
      implicit = true;
      for(int i=0;i<Elements;i++) dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->Implicit(implicit);
      Dhq.resize(uSize,qSize);
      Dhqp.resize(uSize,uSize);
      updateh(t);
    }
    Mat Jh = mbs->getJh()(Iu,Index(0,mbs->getzSize()-1));
    Jh(Index(0,uSize-1),Index(    0,qSize      -1)) << Dhq;
    Jh(Index(0,uSize-1),Index(qSize,qSize+uSize-1)) << Dhqp;
  }

  //-----------------------------------------------------------------------------------
  void FlexibleBody1s21RCM::BuildElements() {
    for(int i=0;i<Elements;i++) {

      int n = 5 * i ;

      if(i<Elements-1 || openStructure==true) {
        // Standard-Elemente
        qElement[i] << q (n,n+7);
        uElement[i] << u(n,n+7);
      }
      else { // i == Elements-1 und Ringschluss
        qElement[i](0,4) << q (n,n+4);
        uElement[i](0,4) << u(n,n+4);
        qElement[i](5,7) << q (0,  2);
        if(qElement[i](2)-q(2)>0.0) 
          qElement[i](7)   += 2*M_PI;
        else
          qElement[i](7)   -= 2*M_PI;
        uElement[i](5,7) << u(0,  2);
      } 
    }
  }

  double FlexibleBody1s21RCM::BuildElement(const double& sGlobal) 
  {
    double remainder = fmod(sGlobal,L);
    if(sGlobal<0.) remainder += L; // project into periodic structure 

    CurrentElement = int(remainder/l0);   
    double sLokal = remainder - (0.5 + CurrentElement) * l0; // Lagrange-Parameter of the affected FE with sLocal==0 in the middle of the FE and sGlobal==0 at the beginning of the beam

    if(CurrentElement >= Elements) { // contact solver computes to large sGlobal at the end of the entire beam
      if(openStructure) { 
        CurrentElement =  Elements-1;
        sLokal += l0;
      }
      else
        CurrentElement -= Elements;
    }
    return sLokal;
  }

  Vec FlexibleBody1s21RCM::computeState(const double &s) {
    Vec X(12);

    double sLokal = BuildElement(s);
    Vec Xlokal = dynamic_cast<FiniteElement1s21RCM*>(discretization[CurrentElement])->StateBalken(qElement[CurrentElement],uElement[CurrentElement],sLokal);

    // Lagen 
    X(Index(0, 2)) = WrON00 +                  JT*Xlokal(Index(0,1)) ;
    X(Index(3, 5)) =          static_cast<Vec>(JR*Xlokal(        2 ));

    // Geschwindigkeiten
    X(Index(6, 8)) =                           JT*Xlokal(Index(3,4)) ;
    X(Index(9,11)) =          static_cast<Vec>(JR*Xlokal(        5 ));

    return X;
  }

  //----------------------------------------------------------------------
  Frame* FlexibleBody1s21RCM::computeFrame(const ContourPointData &S_) {
    if(S_.alpha(0) != sTangent) { // update necessary
      sTangent      = S_.alpha(0); // nur CONTINUUM implementiert TODO: nodes
      double sLokal = BuildElement(sTangent);
      Vec X = dynamic_cast<FiniteElement1s21RCM*>(discretization[CurrentElement])->StateBalken(qElement[CurrentElement],uElement[CurrentElement],sLokal); // x,y,phi und xp,yp,phip
      double phi = X(2);

      Vec tangente(2); tangente(0) =     cos(phi); tangente(1) =    sin(phi);
      Vec normale (2); normale (0) = -tangente(1); normale (1) = tangente(0);

      Wn     =          JT * normale;
      Wt     =          JT * tangente;
      WrOC   = WrON00 + JT * X(0,1);
      WvC    =          JT * X(3,4);
      Womega =          JR.col(0) * X(5);
    }
    return Wt.copy();
  }


  //-----------------------------------------------------------------------------------
  void FlexibleBody1s21RCM::initRelaxed(double alpha) {
    Vec q0Dummy(q0.size(),INIT,0.0);
    if(!initialized) {
      alphaRelax = alpha;
    } else {
      if(openStructure) {
        Vec direction(2);
        direction(0) = cos(alpha);
        direction(1) = sin(alpha);

        for(int i=0;i<=Elements;i++) {
          q0Dummy(5*i+0,5*i+1) = direction*L/Elements*i;
          q0Dummy(5*i+2)        = alpha;
        }
      } else {
        double R  = L/(2*M_PI);
        double a_ = sqrt(R*R + (l0*l0)/16.) - R;

        for(int i=0;i<Elements;i++) {
          double alpha_ = i*(2*M_PI)/Elements;
          q0Dummy(5*i+0) = R*cos(alpha_);
          q0Dummy(5*i+1) = R*sin(alpha_);
          q0Dummy(5*i+2) = alpha_ + M_PI/2.;
          q0Dummy(5*i+3) = a_;
          q0Dummy(5*i+4) = a_;
        }
      }
      setq0(q0Dummy);
      setu0(Vec(q0Dummy.size(),INIT,0.0));
    }
  }

}

