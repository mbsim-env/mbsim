/* Copyright (C) 2005-2006  Roland Zander

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
 *   rzander@users.berlios.de
 *
 */
#include <config.h>
#include "finite_element_1s_21_rcm.h"
#include "body_flexible_1s_21_rcm.h"
#include "port.h"
#include "multi_body_system.h"
#include "contact_flexible.h"
#include "contact_rigid.h"
#include "contour.h"

#define FMATVEC_DEEP_COPY

#ifdef HAVE_AMVIS
#include "elastic1s21rcm.h"
using namespace AMVis;
#endif

namespace MBSim {

  BodyFlexible1s21RCM::BodyFlexible1s21RCM(const string &name, bool openStructure_) :BodyFlexible1s(name), L(0), l0(0), E(0), A(0), I(0), rho(0), rc(0), dm(0), dl(0), openStructure(openStructure_), implicit(false), WrON00(3), WrON0(3), initialized(false), alphaRelax0(-99999.99999), alphaRelax(alphaRelax0), Wt(3), Wn(3), WrOC(3), WvC(3)
#ifdef HAVE_AMVIS
										     ,
										     AMVisRadius(0), AMVisBreadth(0), AMVisHeight(0)
#endif

  { 
    contourR = new Contour1sFlexible("R");
    contourL = new Contour1sFlexible("L");
    ContourPointData cpTmp;
    BodyFlexible::addContour(contourR,cpTmp,false);
    BodyFlexible::addContour(contourL,cpTmp,false);
  }

  void BodyFlexible1s21RCM::init() {
    BodyFlexible::init();

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

    contourR->setAlphaStart(0);  contourR->setAlphaEnd(L);
    contourL->setAlphaStart(0);  contourL->setAlphaEnd(L);
    if(userContourNodes.size()==0) {
      Vec contourNodes(Elements+1);
      for(int i=0;i<=Elements;i++)
	contourNodes(i) = L/Elements * i; // jedes FE hat eigenen Suchbereich fuer Kontaktstellensuche
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
      if(rc != 0) dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->setCurleRadius(rc);
      dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->setMaterialDamping(dm);
      dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->setLehrDamping(dl);
    }
    // balken = new FiniteElement1s21RCM(l0, A*rho, E*A, E*I, g);

    if(alphaRelax != alphaRelax0) initRelaxed(alphaRelax);

#ifdef HAVE_AMVIS

    // wenn ein file fuer AMVis geschrieben werden soll
    if(boolAMVis) {
      ElasticBody1s21RCM *RCMbody = new ElasticBody1s21RCM(fullName,Elements,openStructure,1,boolAMVisBinary);
      RCMbody->setElementLength(l0);

      float amvisJT[3][2], amvisJR[3];
      for(int i=0;i<3;i++) {
	for(int j=0;j<2;j++) amvisJT[i][j] = JT(i,j);
	amvisJR[i] = JR(i,0);
      }
      RCMbody->setJacobians(amvisJT,amvisJR);
      RCMbody->setInitialTranslation(WrON00(0),WrON00(1),WrON00(2));
      RCMbody->setCylinder(AMVisRadius);
      RCMbody->setCuboid(AMVisBreadth,AMVisHeight);
      RCMbody->setColor(AMVisColor);

      bodyAMVis = RCMbody;
    } 
#endif
  }

  void BodyFlexible1s21RCM::setNumberElements(int n) {
    Elements = n;
    if(openStructure) {
      qSize = 5*n+3;
    } else {
      qSize = 5*n;
    }
    uSize = qSize;
    q0.resize(qSize);
    u0.resize(uSize);
  }

  void BodyFlexible1s21RCM::setCurleRadius(double r) {rc = r;if(initialized) for(int i=0;i<Elements;i++) dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->setCurleRadius(rc);}
  void BodyFlexible1s21RCM::setMaterialDamping(double d) {dm = d;if(initialized) for(int i=0;i<Elements;i++) dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->setMaterialDamping(dm);}
  void BodyFlexible1s21RCM::setLehrDamping(double d)     {dl = d;if(initialized) for(int i=0;i<Elements;i++) dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->setLehrDamping(dl);}


  void BodyFlexible1s21RCM::plotParameters() {
    parafile << "BodyFlexible1s21RCM\n---------------------------\n"  << endl;
    parafile << "# Elements = " << Elements << endl;
    parafile << "#   E = " << E << endl;
    parafile << "#   A = " << A << endl;
    parafile << "#   I = " << I << endl;
    parafile << "# rho = " << rho << endl;
    parafile << "#   L = " << L << endl;
    parafile << "#  dm = " << dm << endl;
    parafile << "#  dl = " << dl << endl;

    parafile << "\n# JT:\n"      << JT   << endl;
    parafile << "\n# JR:\n"      << JR   << endl;

    if(port.size()>0) parafile << "\nports:" <<endl;
    for(unsigned int i=0; i<port.size(); i++) { 
      parafile << "# s: (port:  name= "<< port[i]->getName()<<",  ID= "<<port[i]->getID()<<") = ";
      if(S_Port[i].type==CONTINUUM) parafile << S_Port[i].alpha(0) << endl;
      if(S_Port[i].type==NODE     ) parafile << S_Port[i].ID*L/Elements    << endl;
    }
    //    parafile << "\n# contours:" <<endl;
    //    for(int i=0; i<contour.size(); i++) {
    //      parafile << "# J: (contour:  name= "<< contour[i]->getName()<<",  ID= "<<contour[i]->getID()<<")"<< endl;
    //      if(contour[i]->getType() == point) parafile << J[contour[i]->getID()]<<endl;
    //    }

  }


  //-----------------------------------------------------------------------------------

  void BodyFlexible1s21RCM::updateKinematics(double t) {
    BuildElements();
    sTangent = -l0;

    WrON0 = WrON00 + JT*q(Index(0,1));
    updatePorts(t);
    //updateContours(t);
  }

  void BodyFlexible1s21RCM::updatePorts(double t) {
    for(unsigned int i=0; i<port.size(); i++) {
      //    if(S_Port[i](0) == -1) // ForceElement on continuum
      if(S_Port[i].type == CONTINUUM) { // ForceElement on continuum
	const double     &s = S_Port[i].alpha(0);// globaler KontParameter
	double sLokal = BuildElement(s);
	Vec Z = dynamic_cast<FiniteElement1s21RCM*>(discretization[CurrentElement])->StateBalken(qElement[CurrentElement],uElement[CurrentElement],sLokal);
	port[i]->setWrOP(WrON00 + JT * Z(0,1));//q(5*node+0,5*node+1));
	port[i]->setWvP (         JT * Z(3,4));//u(5*node+0,5*node+1));
	port[i]->setWomegaP(      JR * Z(5,5));//u(5*node+2,5*node+2));
      }
      //    else                   // ForceElement on node
      else if(S_Port[i].type == NODE) {                  // ForceElement on node
	const int &node = S_Port[i].ID;
	port[i]->setWrOP(WrON00 + JT * q(5*node+0,5*node+1));//WrOS + WrSP[i]);
	port[i]->setWvP (         JT * u(5*node+0,5*node+1));//WvS + crossProduct(WomegaK, WrSP[i]));
	port[i]->setWomegaP(      JR * u(5*node+2,5*node+2));
      }
    }
  }

  Mat BodyFlexible1s21RCM::computeJacobianMatrix(const ContourPointData &S_) {
    Index All(0,3-1);
    Mat Jacobian(qSize,3,INIT,0.0);

    // ForceElement on continuum
    if(S_.type == CONTINUUM)
    {
      double s = S_.alpha(0); // globaler KontParameter
      double sLokal = BuildElement(s);
      Mat Jtmp = dynamic_cast<FiniteElement1s21RCM*>(discretization[CurrentElement])->JGeneralized(qElement[CurrentElement],sLokal);
      if(CurrentElement<Elements-1 || openStructure) {
	Index Dofs(5*CurrentElement,5*CurrentElement+7);
	Jacobian(Dofs,All) = Jtmp;
      }
      else { // Ringschluss
	Jacobian(Index(5*CurrentElement,5*CurrentElement+4),All) = Jtmp(Index(0,4),All);
	Jacobian(Index(               0,                 2),All) = Jtmp(Index(5,7),All);
      }

    }
    // ForceElement on node
    else if(S_.type == NODE)
    {
      int node = S_.ID;
      Index Dofs(5*node,5*node+2);
      Jacobian(Dofs,All) << DiagMat(3,INIT,1.0);
    }
    return Jacobian;
  }

  Mat BodyFlexible1s21RCM::computeJp(const ContourPointData &S_) {
    Index All(0,3-1);
    Mat Jp(qSize,3,INIT,0.0);

    // ForceElement on continuum
    if(S_.type == CONTINUUM)
    {
      double s  = S_.alpha(0) ; // globaler KontParameter
      double sp = 0;
      if(S_.alphap.size()>0) sp = S_.alphap(0); // globale  KontGeschwindigkeit
      double sLokal = BuildElement(s);

      Mat Jtmp = dynamic_cast<FiniteElement1s21RCM*>(discretization[CurrentElement])->JpGeneralized(qElement[CurrentElement],uElement[CurrentElement],sLokal,sp);

      if(CurrentElement<Elements-1 || openStructure) {
	Index Dofs(5*CurrentElement,5*CurrentElement+7);
	Jp(Dofs,All) = Jtmp;
      }
      else { // Ringschluss
	Jp(Index(5*CurrentElement,5*CurrentElement+4),All) = Jtmp(Index(0,4),All);
	Jp(Index(               0,                 2),All) = Jtmp(Index(5,7),All);
      }

    }

    /// cout << "BodyFlexible1s21RCM::computeJp" << endl;
    /// cout << "   BodyFlexible1s21RCM::Jp = " << trans(Jp) <<  endl;
    /// cout << "          BodyFlexible::Jp = " << trans(BodyFlexible::computeJp(S_)) <<  endl;

    //    // ForceElement on node
    //    else if(S_.type == NODE)
    //    {
    //      int node = S_.ID;
    //      Index Dofs(5*node,5*node+2);
    //      Jp(Dofs,All) << DiagMat(3,INIT,0.0);
    //    }
    return Jp;
  }


  Mat BodyFlexible1s21RCM::computeK    (const ContourPointData &S_) {
    double s  = S_.alpha(0);
    double sLokal = BuildElement(s);
    return JR*dynamic_cast<FiniteElement1s21RCM*>(discretization[CurrentElement])->Kcurvature(qElement[CurrentElement],sLokal);
  } 
  Mat BodyFlexible1s21RCM::computeKp   (const ContourPointData &S_){
    double s  = S_.alpha(0);
    double sp = 0;
    if(S_.alphap.size()>0) sp = S_.alphap(0); // globale  KontGeschwindigkeit
    double sLokal = BuildElement(s);
    return JR*dynamic_cast<FiniteElement1s21RCM*>(discretization[CurrentElement])->Kpcurvature(qElement[CurrentElement],uElement[CurrentElement],sLokal,sp);
  }

  Mat BodyFlexible1s21RCM::computeDrDs (const ContourPointData &S_) {
    double s  = S_.alpha(0);
    double sLokal = BuildElement(s);
    Vec DrDs = dynamic_cast<FiniteElement1s21RCM*>(discretization[CurrentElement])->DrDs(qElement[CurrentElement],sLokal);
    return JT*DrDs;
  }

  Mat BodyFlexible1s21RCM::computeDrDsp(const ContourPointData &S_) {
    double s  = S_.alpha(0);
    double sp = 0;
    if(S_.alphap.size()>0) sp = S_.alphap(0); // globale  KontGeschwindigkeit
    double sLokal = BuildElement(s);
    Vec DrDsp = dynamic_cast<FiniteElement1s21RCM*>(discretization[CurrentElement])->DrDsp(qElement[CurrentElement],uElement[CurrentElement],sLokal,sp);
    /// cout << "---------------------------" << endl;
    /// cout << "DrDsp.analytisch = " << trans(JT*DrDsp) << endl;
    /// cout << "DrDsp.numerisch  = " << trans(BodyFlexible::computeDrDsp(S_)) << endl;
    return JT*DrDsp;
  }

  void BodyFlexible1s21RCM::GlobalMatrixContribution(int n) {
    int j = 5 * n;

    if ( n < Elements - 1 || openStructure==true) {
      // * Matrizen berechnen
      M(Index(j,j+7)) +=  discretization[n]->getMassMatrix();
      h(j,j+7)       += discretization[n]->getGeneralizedForceVector();

      //if(implicit) {
      //  Dhq (j,j,j+7,j+7) += discretization[n]->getJacobianForImplicitIntegrationRegardingPosition;
      //  Dhqp(j,j,j+7,j+7) += discretization[n]->getJacobianForImplicitIntegrationRegardingVelocity;
      //}
    } else {
      // * Matrizen berechnen

      // Ringschluss durch Element (nEnde,1), Achtung!!! Winkelunterschied: 2*pi;
      M(Index(j,j+4)) +=  discretization[n]->getMassMatrix()(Index(0,4));
      M(Index(j,j+4),Index(0,2)) += discretization[n]->getMassMatrix()(Index(0,4),Index(5,7));
      // M(0,j,  2,j+4) += discretization[n]->getM()(5,0,7,4); Symmetrie
      M(Index(0,2)) += discretization[n]->getMassMatrix()(Index(5,7));

      h(j,j+4)       += discretization[n]->getGeneralizedForceVector()(0,4);
      h(0,  2)       += discretization[n]->getGeneralizedForceVector()(5,7);

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

  void BodyFlexible1s21RCM::updateJh_internal(double t) {
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
  void BodyFlexible1s21RCM::BuildElements() {
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

  double BodyFlexible1s21RCM::BuildElement(const double& sGlobal) 
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


  //  double BodyFlexible1s21RCM::BuildElement(const double& sGlobal) { 	/* removed because of static and and because I CARE !!
  //	static double sGlobalOld = -1.0;				           BodyFlexible1s33RCM::BuildElement used instead */
  //	static double sLokal = 0;                                       // HR 04.02.2009
  //
  //	if (sGlobal != sGlobalOld ) {
  //	  sGlobalOld = sGlobal;
  //
  //	  double remainder = sGlobal;
  //	  if(!openStructure) {
  //		remainder = fmod(sGlobal,L);
  //		if(sGlobal<0.) remainder += L; // project into periodic structure 
  //	  }
  ////  double remainder = fmod(sGlobal,L);
  ////	  if(remainder<0.0) remainder += L;
  //	  CurrentElement = int(remainder/l0);
  //	  sLokal = remainder - ( 0.5 + CurrentElement ) * l0;
  //
  //	  if(CurrentElement >= Elements) {
  //		if(openStructure) { CurrentElement =  Elements-1; sLokal += l0;} /*somehow buggy, but who cares?!?*/
  //		else              { CurrentElement -= Elements;}                         /* start at the beginning again  */
  //	  }
  //	  else if( CurrentElement <0 )   {
  //		if(openStructure) { CurrentElement =  0;           sLokal -= l0;} /*somehow buggy, but who cares?!?*/
  //		else              { CurrentElement += Elements;}                         /* start at theqElement[i]in  */
  //	  }
  //	}
  //	return sLokal;
  //  }

  void BodyFlexible1s21RCM::addPort(const string &name, const int &node) {
    Port *port = new Port(name);
    ContourPointData cpTemp;
    cpTemp.type = NODE;
    cpTemp.ID = node;
    addPort(port,cpTemp);
  }

  Vec BodyFlexible1s21RCM::computeState(const double &s) {
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
  Mat BodyFlexible1s21RCM::computeWt  (const ContourPointData &S_) {
    if(S_.alpha(0) != sTangent) { //update noetig
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

  Vec BodyFlexible1s21RCM::computeWn  (const ContourPointData &S_) {
    if(S_.alpha(0) != sTangent) computeWt(S_);
    return Wn.copy();
  }

  Vec BodyFlexible1s21RCM::computeWrOC(const ContourPointData &S_) {
    if(S_.alpha(0) != sTangent) computeWt(S_);
    return WrOC.copy();
  }

  Vec BodyFlexible1s21RCM::computeWvC (const ContourPointData &S_) {
    if(S_.alpha(0) != sTangent) computeWt(S_);
    return WvC.copy();
  }
  Vec BodyFlexible1s21RCM::computeWomega(const ContourPointData &S_) {
    if(S_.alpha(0) != sTangent) computeWt(S_);
    return Womega.copy();
  }

  //-----------------------------------------------------------------------------------
  double BodyFlexible1s21RCM::computePotentialEnergy () {
    double V = 0.0;
    for(int i=0;i<Elements;i++) {
      BuildElement(i);
      V += dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->computeV(qElement[i]);
    }
    return V;
  }

  //-----------------------------------------------------------------------------------
  void BodyFlexible1s21RCM::initRelaxed(double alpha) {
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