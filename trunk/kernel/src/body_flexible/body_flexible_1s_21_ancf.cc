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
#include "finite_element_1s_21_ancf.h"
#include "body_flexible_1s_21_ancf.h"
#include "port.h"
#include "multi_body_system.h"
#include "contact.h"
#include "contour.h"

#define FMATVEC_DEEP_COPY

namespace MBSim {

  const int BodyFlexible1s21ANCF::NodeDOFs      = 4;
  const int BodyFlexible1s21ANCF::ElementalDOFs = 2*NodeDOFs;

  BodyFlexible1s21ANCF::BodyFlexible1s21ANCF(const string &name, bool openStructure_) 
    :BodyFlexible1s(name), 
    L(0), E(0), A(0), I(0), rho(0), openStructure(openStructure_), 
    implicit(false), qElement(8), uElement(8), 
    WrON00(3), WrON0(3), 
    alphaRelax0(-99999.99999), alphaRelax(alphaRelax0), initialized(false),
    Wt(3), Wn(3), WrOC(3), WvC(3) { 
	  contourR = new Contour1sFlexible("R");
	  contourL = new Contour1sFlexible("L");
	  ContourPointData cpTmp;
	  BodyFlexible::addContour(contourR,cpTmp,false);
	  BodyFlexible::addContour(contourL,cpTmp,false);
    }


  void BodyFlexible1s21ANCF::init() {
    BodyFlexible1s::init();
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

    double l0 = L/Elements;
    Vec g = trans(JT)*mbs->getGrav();
    balken = new FiniteElement1s21ANCF(l0, A*rho, E*A, E*I, g);

    initM();

    if(alphaRelax != alphaRelax0) initRelaxed(alphaRelax);

    //   if(rc != 0)
    //     balken->setCurleRadius(rc);
    //   balken->setMaterialDamping(dm);
    //   balken->setLehrDamping(dl);
  }

  void BodyFlexible1s21ANCF::setNumberElements(int n) {
    Elements = n;
    if(openStructure) {
      qSize = NodeDOFs * (n+1);
    } else {
      qSize = NodeDOFs *  n   ;
    }
    uSize = qSize;
    q0.resize(qSize);
    u0.resize(uSize);
  }
  //-----------------------------------------------------------------------------------

  void BodyFlexible1s21ANCF::updateKinematics(double t) {
    sTangent = -balken->l0;

    WrON0 = WrON00 + JT*q(Index(0,1));
    updatePorts(t);
    //updateContours(t);
  }


  void BodyFlexible1s21ANCF::updatePorts(double t) {
    for(int i=0; i<port.size(); i++) {
      if(S_Port[i].type == CONTINUUM) {      // ForceElement on continuum
	const double &s = S_Port[i].alpha(0);// globaler KontParameter
	double sLokal = BuildElement(s);
	Vec Z = balken->StateBalken(qElement,uElement,sLokal);
	port[i]->setWrOP(WrON00 + JT * Z(0,1));//q(5*node+0,5*node+1));
	port[i]->setWvP (         JT * Z(3,4));//u(5*node+0,5*node+1));
	port[i]->setWomegaP(      JR * Z(5,5));//u(5*node+2,5*node+2));
      }
      else if(S_Port[i].type == NODE) { // ForceElement on node
	const int &node = S_Port[i].ID;
	port[i]->setWrOP(WrON00 + JT * q(NodeDOFs*node+0,NodeDOFs*node+1));//WrOS + WrSP[i]);
	port[i]->setWvP (         JT * u(NodeDOFs*node+0,NodeDOFs*node+1));//WvS + crossProduct(WomegaK, WrSP[i]));
	Vec a  = q(NodeDOFs*node+2,NodeDOFs*node+3); // Winkelgeschwindigkeit bestimmen: tangente an struktur
	double a_ = nrm2(a);
	Vec t(2); t(0) = - a(1)/a_; t(1) = a(0)/a_; 
	Vec w(1); w(0) = trans(t) * u(NodeDOFs*node+2,NodeDOFs*node+3); // Winkelgeschwindigkeit bestimmen
	port[i]->setWomegaP(      JR * w);
      }
    }
  }

  Mat BodyFlexible1s21ANCF::computeJacobianMatrix(const ContourPointData &data) {
    static Index All(0,3-1);
    Mat Jacobian(qSize,3);

    if(data.type == CONTINUUM) {
      const double &s = data.alpha(0);// globaler KontParameter
      double   sLokal = BuildElement(s);
      Mat Jtmp = balken->JGeneralized(qElement,sLokal);

      if(CurrentElement<Elements-1 || openStructure) {
	Index Dofs(NodeDOFs*CurrentElement,NodeDOFs*CurrentElement+ElementalDOFs-1);
	Jacobian(Dofs,All) = Jtmp;
      }
      else { // Ringschluss
	Jacobian(Index(NodeDOFs*CurrentElement,NodeDOFs*CurrentElement+NodeDOFs-1),All) = Jtmp(Index(       0,NodeDOFs-1),All);
	Jacobian(Index(                      0,                        NodeDOFs-1),All) = Jtmp(Index(NodeDOFs,NodeDOFs-1),All);
      }
    } else if(data.type == NODE) {                 // ForceElement on node
      const int &node = data.ID;
      Jacobian(NodeDOFs*node+0,0) = 1.0;
      Jacobian(NodeDOFs*node+1,1) = 1.0;
      double bufferABS = sqrt( q(NodeDOFs*node+2)*q(NodeDOFs*node+2) + q(NodeDOFs*node+3)*q(NodeDOFs*node+3) );
      Jacobian(NodeDOFs*node+2,2) = - q(NodeDOFs*node+3)/bufferABS;
      Jacobian(NodeDOFs*node+3,2) =   q(NodeDOFs*node+2)/bufferABS;
    }

    return Jacobian;
  }

  void BodyFlexible1s21ANCF::updateh(double t) {
    h.init(0.0);

    //   if(implicit) {
    //     Dhq .init(0.0);
    //     Dhqp.init(0.0);
    //   }

    // Alle Elemente ausser (n-1,0)
    for (int i = 0; i < Elements ; i++ ) {

      // * Koordinaten Sortieren
      BuildElement(i);
      int j = NodeDOFs * i;

      balken->berechneh(qElement, uElement);

      if ( i < Elements - 1 || openStructure==true) {
	// * Matrizen berechnen
	//      Index I = Index(j,j+ElementalDOFs-1);
	h(Index(j,j+ElementalDOFs-1)) += balken->h;

	/*	if(implicit) {
		Dhq (I) += balken->Dhq; //(j,j,j+7,j+7)
		Dhqp(I) += balken->Dhqp;//(j,j,j+7,j+7)
		}*/
      } else {
	h(j,j+NodeDOFs-1)       += balken->h(       0,     NodeDOFs-1);
	h(0,  NodeDOFs-1)       += balken->h(NodeDOFs,ElementalDOFs-1);

	/*	if(implicit) {
		Dhq (j,j,j+4,j+4) += balken->Dhq(0,0,4,4);
		Dhq (j,0,j+4,  2) += balken->Dhq(0,5,4,7);
		Dhq (0,j,  2,j+4) += balken->Dhq(5,0,7,4);
		Dhq (0,0,  2,  2) += balken->Dhq(5,5,7,7);

		Dhqp(j,j,j+4,j+4) += balken->Dhqp(0,0,4,4);
		Dhqp(j,0,j+4,  2) += balken->Dhqp(0,5,4,7);
		Dhqp(0,j,  2,j+4) += balken->Dhqp(5,0,7,4);
		Dhqp(0,0,  2,  2) += balken->Dhqp(5,5,7,7);
		}*/
      }
    }

    sumUpForceElements(t);
  }

  void BodyFlexible1s21ANCF::initM() {
    balken->berechneM();

    M.init(0.0);

    // Alle Elemente ausser (n-1,0)
    for (int i = 0; i < Elements ; i++ ) {
      int j = NodeDOFs * i;
      if ( i < Elements - 1 || openStructure==true)
	M(Index(j,j+ElementalDOFs-1)) += balken->M;
      else {
	M(Index(j,j+NodeDOFs-1))                     += balken->M(Index(0,NodeDOFs-1));
	M(Index(j,j+NodeDOFs-1),Index(0,NodeDOFs-1)) += balken->M(Index(0,NodeDOFs-1),Index(NodeDOFs,ElementalDOFs-1));
	// Symmetrie !!!
	M(Index(0,NodeDOFs-1)) += balken->M(Index(NodeDOFs,ElementalDOFs-1));
      }
    }

    LLM << facLL(M);

  }

  //-----------------------------------------------------------------------------------
  void BodyFlexible1s21ANCF::BuildElement(const int& ENumber) {
    CurrentElement = ENumber;
    static int n;
    n = NodeDOFs * ENumber ;

    if  ( ENumber < Elements-1  || openStructure==true) {
      // Standard-Elemente
      qElement << q(n,n+ElementalDOFs-1);
      uElement << u(n,n+ElementalDOFs-1);
    } else if (ENumber == Elements-1) {
      // Ringschluss durch Einbeziehnung des Ersten Referenzpunktes als zweiten Knoten
      qElement(0,NodeDOFs-1) << q(n,n+NodeDOFs-1);
      uElement(0,NodeDOFs-1) << u(n,n+NodeDOFs-1);
      qElement(NodeDOFs,ElementalDOFs-1) << q(0,  NodeDOFs-1);
      uElement(NodeDOFs,ElementalDOFs-1) << u(0,  NodeDOFs-1);
    } else {
      qElement.init(0.0);uElement.init(0.0);
      cout << "\nKein Element " <<ENumber<< " vorhanden. Nur 0, 1 ...  " <<Elements-1<< " Elemente definiert!\n";
      throw(1);
    }
  }

  double BodyFlexible1s21ANCF::BuildElement(const double& sGlobal) {
    double sLokal = 0;
    int Element = 0;
    while( (Element+1)*balken->l0 < sGlobal)
      Element++;
    sLokal = sGlobal - Element * balken->l0;

    if(Element >= Elements) {
      if(openStructure) { Element =  Elements-1; sLokal += balken->l0;} /*somehow buggy, but who cares?!?*/
      else              { Element -= Elements;}                         /* start at the beginning again  */
    }

    BuildElement(Element);
    return sLokal;
  }

  void BodyFlexible1s21ANCF::addPort(const string &name, const int &node) {
    Port *port = new Port(name);
    ContourPointData cpTemp;
    cpTemp.type = NODE;
    cpTemp.ID = node;
    addPort(port,cpTemp);
  }

  // void BodyFlexible1s21ANCF::addPort(const string &name, const double &s) {
  //   Port *port = new Port(name);
  //   ContourPointData cpTemp;
  //   cpTemp.type  = CONTINUUM;
  //   cpTemp.alpha = Vec(1,INIT,s);
  //   addPort(port,cpTemp);
  // }

  //----------------------------------------------------------------------
  Mat BodyFlexible1s21ANCF::computeWt  (const ContourPointData &S_) {
    if(S_.alpha(0) != sTangent) { //update noetig
      sTangent = S_.alpha(0); // TODO : nodes
      double sLokal = BuildElement(sTangent);

      // Tangente berechnen
      Vec tangente = balken->Tangente(qElement,sLokal);
      Vec normale (2); normale (0) = -tangente(1); normale (1) = tangente(0);

      Vec X = balken->StateBalken(qElement,uElement,sLokal); // x,y und xp,yp

      Wn     =          JT * normale;
      Wt     =          JT * tangente;
      WrOC   = WrON00 + JT * X(0,1);
      WvC    =          JT * X(3,4);
      Womega =          JR.col(0) * X(5);
    }
    return Wt.copy();
  }

  Vec BodyFlexible1s21ANCF::computeWn  (const ContourPointData &S_) {
    if(S_.alpha(0) != sTangent) computeWt(S_);
    return Wn.copy();
  }

  Vec BodyFlexible1s21ANCF::computeWrOC(const ContourPointData &S_) {
    if(S_.alpha(0) != sTangent) computeWt(S_);
    return WrOC.copy();
  }

  Vec BodyFlexible1s21ANCF::computeWvC (const ContourPointData &S_) {
    if(S_.alpha(0) != sTangent) computeWt(S_);
    return WvC.copy();
  }

  Vec BodyFlexible1s21ANCF::computeWomega(const ContourPointData &S_) {
    if(S_.alpha(0) != sTangent) computeWt(S_);
    return Womega.copy();
  }

  ////----------------------------------------------------------------------
  //Vec BodyFlexible1s21ANCF::computeWt  (const double &s) {
  //    static ContourPointData cdTemp;
  //    cdTemp.alpha = Vec(1,INIT,s);
  //    return computeWt(cdTemp).col(0);
  //}
  //
  //Vec BodyFlexible1s21ANCF::computeWn  (const double &s) {
  //    static ContourPointData cdTemp;
  //    cdTemp.alpha = Vec(1,INIT,s);
  //    return computeWn(cdTemp);
  //}
  //
  //Vec BodyFlexible1s21ANCF::computeWrOC(const double &s) {
  //    static ContourPointData cdTemp;
  //    cdTemp.alpha = Vec(1,INIT,s);
  //    return computeWrOC(cdTemp);
  //}
  //
  //Vec BodyFlexible1s21ANCF::computeWvC (const double &s) {
  //    static ContourPointData cdTemp;
  //    cdTemp.alpha = Vec(1,INIT,s);
  //    return computeWvC(cdTemp);
  //}
  //
  //Vec BodyFlexible1s21ANCF::computeWomega(const double &s) {
  //    static ContourPointData cdTemp;
  //    cdTemp.alpha = Vec(1,INIT,s);
  //    return computeWomega(cdTemp);
  //}

  //-----------------------------------------------------------------------------------
  void BodyFlexible1s21ANCF::initRelaxed(double alpha) {
    if(!initialized) {
      alphaRelax = alpha;
    }
    else
    {
      if(openStructure) {
	Vec direction(2);
	direction(0) = cos(alpha);
	direction(1) = sin(alpha);

	for(int i=0;i<=Elements;i++) {
	  q0(NodeDOFs*i+0,NodeDOFs*i+1) = direction*L/Elements*i;
	  q0(NodeDOFs*i+2,NodeDOFs*i+3) = direction;
	}
      }
      else {
	double R  = L/(2*M_PI);

	for(int i=0;i<Elements;i++) {
	  double alpha_ = i*(2*M_PI)/Elements;
	  q0(NodeDOFs*i+0) = R*cos(alpha_);
	  q0(NodeDOFs*i+1) = R*sin(alpha_);
	  q0(NodeDOFs*i+2) = cos(alpha_ + M_PI/2.);
	  q0(NodeDOFs*i+3) = sin(alpha_ + M_PI/2.);
	}
      }
    }
  }
  
}
