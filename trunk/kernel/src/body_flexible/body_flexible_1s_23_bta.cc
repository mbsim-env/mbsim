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
#include "finite_element_1s_23_bta.h"
#include "body_flexible_1s_23_bta.h"
#include "port.h"
#include "multi_body_system.h"
#include "contact_flexible.h"
#include "contact_rigid.h"
#include "contour.h"

#ifdef HAVE_AMVIS
#include "elastic1sbta.h"
using namespace AMVis;
#endif

namespace MBSim {

  const int BodyFlexible1s23BTA::nodalDOFs = FiniteElement1s23BTA::nodalDOFs;

  BodyFlexible1s23BTA::BodyFlexible1s23BTA(const string &name) :BodyFlexible1s(name), 
  L(0), E(0), A(0), Iyy(0), Izz(0), rho(0), rc(0), dm(0), dl(0),
  implicit(false), WrON00(3), WrON0(3),
  WrOC(3), WvC(3), Womega(3) { 

    contourCyl = new CylinderFlexible("cylinder");
    ContourPointData cpTmp;
    BodyFlexible::addContour(contourCyl,cpTmp,false);
  }

  void BodyFlexible1s23BTA::init() {
    BodyFlexible1s::init();
    assert(0<Elements);

    double l0 = L/Elements;
    Vec g = trans(JR)*mbs->getGrav();

    for(int i=0; i<Elements; i++) {
      //    FiniteElement1s23BTA(double sl0, double sArho, double sEIyy, double sEIzz, double sItrho, double sGIt, Vec sg)
      element.push_back(  FiniteElement1s23BTA(l0, A*rho, E*Iyy, E*Izz, It*rho, G*It, g )  );

      element[i].setMaterialDamping(dm);
      element[i].setLehrDamping(dl);
    }


    contourCyl->setAlphaStart(0);  contourCyl->setAlphaEnd(L);

    if(userContourNodes.size()==0) {
      Vec contourNodes(Elements+1);
      for(int i=0;i<=Elements;i++)
	contourNodes(i) = L/Elements * i; // jedes FE hat eigenen Suchbereich fuer Kontaktstellensuche
      contourCyl->setNodes(contourNodes);
    }
    else {
      contourCyl->setNodes(userContourNodes);
    }
#ifdef HAVE_AMVIS
    // wenn ein file fuer AMVis geschrieben werden soll
    if(boolAMVis) {
      ElasticBody1sBTA *BTAbody = new ElasticBody1sBTA(fullName,Elements,1,boolAMVisBinary);
      BTAbody->setElementLength(element[0].l0);
      BTAbody->setRadius(static_cast<CylinderFlexible*>(contourCyl)->getRadius());

      float amvisJ[3][3];
      for(int i=0;i<3;i++)
	for(int j=0;j<3;j++) amvisJ[i][j] = JR(i,j);
      BTAbody->setJacobian(amvisJ);
      BTAbody->setInitialTranslation(WrON00(0),WrON00(1),WrON00(2));

      bodyAMVis = BTAbody;
    }
#endif 
  }

  void BodyFlexible1s23BTA::setNumberElements(int n) {
    Elements = n;
    qSize = nodalDOFs*( Elements + 1 ); 

    uSize = qSize;
    q0.resize(qSize);
    u0.resize(uSize);
  }

  void BodyFlexible1s23BTA::plotParameters() {
    parafile << "BodyFlexible1s23BTA"  << endl;
    parafile << "# Elements = "  << Elements << endl;
    parafile << "#   L = "  << L << endl;
    parafile << "#   E = "  << E << endl;
    parafile << "#   G = "  << G << endl;
    parafile << "# rho = "  << rho << endl;
    parafile << "#   A = "  << A << endl;
    parafile << "# Iyy = "  << Iyy << endl;
    parafile << "# Izz = "  << Izz << endl;
    parafile << "#  It = "  << It << endl;

    parafile << "\n# JT:\n"      << JT   << endl;
    parafile << "\n# JR:\n"      << JR   << endl;

    if(port.size()>0) parafile << "\nports:" <<endl;
    for(int i=0; i<port.size(); i++) { 
      parafile << "# s: (port:  name= "<< port[i]->getName()<<",  ID= "<<port[i]->getID()<<") = ";
      if(S_Port[i].type==CONTINUUM) parafile << S_Port[i].alpha(0) << endl;
      if(S_Port[i].type==NODE     ) parafile << S_Port[i].ID*L/Elements  << endl;
    }
    //    parafile << "\n# contours:" <<endl;
    //    for(int i=0; i<contour.size(); i++) {
    //      parafile << "# J: (contour:  name= "<< contour[i]->getName()<<",  ID= "<<contour[i]->getID()<<")"<< endl;
    //      if(contour[i]->getType() == point) parafile << J[contour[i]->getID()]<<endl;
    //    }


  }
  //-----------------------------------------------------------------------------------

  void BodyFlexible1s23BTA::updateKinematics(double t) {
    sTangent = -element[0].l0;

    //  WrON0 = WrON00 + JT*q(Index(0,1));
    updatePorts(t);
    //updateContours(t);
  }

  void BodyFlexible1s23BTA::updatePorts(double t) {
    for(int i=0; i<port.size(); i++) {
      if(S_Port[i].type == CONTINUUM) // ForceElement on continuum
      {
	const double &s = S_Port[i].alpha(0);
	double sElement = BuildElement( s );

	Vec Z = element[CurrentElement].StateAxis(qElement,uElement,sElement);
	Z(0) = s;
	port[i]->setWrOP(WrON00 + JR * Z( 0,2));
	//	    port[i]->setWaP (         JR * Z( 3,5));
	port[i]->setWvP (         JR * Z( 6,8));
	port[i]->setWomegaP(      JR * Z(9,11));

      }
      // 	else                   // ForceElement on node
      // 	{
      // 	    int node = int(S_Port[i](0));
      // 	    port[i]->setWrOP(WrON00 + JT * q(5*node+0,5*node+1));
      // 	    port[i]->setWvP (         JT * u(5*node+0,5*node+1));
      // 	    port[i]->setWomegaP(      JR * u(5*node+2,5*node+2));
      // 	}
    }
  }

  Mat BodyFlexible1s23BTA::computeJacobianMatrix(const ContourPointData &S_) {
    //     cout << "BodyFlexible1s23BTA::updatePorts" << endl;
    static Index All(0,5-1);
    Mat Jacobian(qSize,5);

    // ForceElement on continuum
    if(S_.type == CONTINUUM)
    {
      const double &s = S_.alpha(0);
      double sElement = BuildElement( s );

      Jacobian(activeElement,All) = element[CurrentElement].JGeneralized(qElement,sElement);
    }
    if(S_.type == NODE)
    {
      ContourPointData Stmp;
      Stmp.alpha = Vec(1,INIT,S_.ID*L/Elements);
      Jacobian = computeJacobianMatrix(Stmp);
    } 
    return Jacobian;
  }

  void BodyFlexible1s23BTA::updateh(double t) {
    static int i,j;

    M.init(0.0);
    h.init(0.0);

    if(implicit) {
      Dhq .init(0.0);
      Dhqp.init(0.0);
    }

    // Alle Elemente ausser (n-1,0)
    for ( i = 0; i < Elements ; i++ ) {
      BuildElement(i);
      element[i].update(qElement,uElement);
      M(activeElement) += element[i].M;
      h(activeElement) += element[i].h;

      //       if(implicit) {
      // 	  Dhq (j,j,j+7,j+7) += element[i].Dhq;
      // 	  Dhqp(j,j,j+7,j+7) += element[i].Dhqp;
      //       }
    }

    LLM = facLL(M);

    sumUpForceElements(t);
  }

  //-----------------------------------------------------------------------------------
  void BodyFlexible1s23BTA::BuildElement(const int& ENumber) {
    // Grenzen testen
    assert(ENumber >= 0       );
    assert(ENumber <  Elements);

    CurrentElement = ENumber;

    activeElement = Index( nodalDOFs*CurrentElement , nodalDOFs*(CurrentElement+2) -1 );
    qElement = q(activeElement);
    uElement = u(activeElement);
  }

  double BodyFlexible1s23BTA::BuildElement(const double& sGlobal) {
    CurrentElement = 0;
    double lCummulated = 0.0;
    while(lCummulated <= sGlobal && CurrentElement < Elements) lCummulated += element[CurrentElement++].l0;
    CurrentElement--; // eins zu weit gerannt...

    double sLokal = sGlobal - ( lCummulated - element[CurrentElement].l0 );
    BuildElement(CurrentElement);

    return sLokal;
  }


  void BodyFlexible1s23BTA::addPort(const string &name, const int &node) {
    Port *port = new Port(name);
    ContourPointData cpTemp;
    //  cpTemp.type = NODE;
    cpTemp.type = CONTINUUM;
    cpTemp.ID = node;
    cpTemp.alpha = Vec(1,INIT,node*L/Elements);
    addPort(port,cpTemp);
  }

  //----------------------------------------------------------------------
  Mat BodyFlexible1s23BTA::computeWt  (const ContourPointData &S_) {
    const double &s = S_.alpha(0);
    double sElement = BuildElement(s);
    Vec Tangent = element[CurrentElement].Tangent(qElement,sElement);
    Wt = JR * Tangent;
    return Wt.copy();
  }

  SqrMat BodyFlexible1s23BTA::computeAWK (const ContourPointData &S_) {
    const double &s = S_.alpha(0);
    double sElement = BuildElement(s);
    SqrMat AWK = element[CurrentElement].AWK(qElement,sElement);
    return SqrMat(JR * AWK);
  }

  Vec BodyFlexible1s23BTA::computeWrOC(const ContourPointData &S_) {
    if (S_.alpha(0) != sTangent) 
    {
      sTangent = S_.alpha(0);
      double sElement = BuildElement(sTangent);
      Vec Z = element[CurrentElement].StateAxis(qElement,uElement,sElement);
      Z(0) = sTangent;
      WrOC   = JR * Z(0,2);
      WvC    = JR * Z(6,8);
      Womega = JR * Z(9,11);
    }
    return WrOC.copy();
  }

  Vec BodyFlexible1s23BTA::computeWvC (const ContourPointData &S_) {
    if (S_.alpha(0) != sTangent) 
      computeWrOC(S_);
    return WvC.copy();
  }

  Vec BodyFlexible1s23BTA::computeWomega (const ContourPointData &S_) {
    if (S_.alpha(0) != sTangent) 
      computeWrOC(S_);
    return Womega.copy();
  }

}
