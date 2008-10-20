/* Copyright (C) 2005-2008  Roland Zander

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
#include<config.h>
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

  BodyFlexible1s23BTA::BodyFlexible1s23BTA(const string &name) :BodyFlexible1s(name), 
  L(0), l0(0.), E(0), A(0), Iyy(0), Izz(0), rho(0), rc(0), implicit(false), WrON00(3), WrOC(3), WvC(3), Womega(3) 
  { 
	contourCyl = new CylinderFlexible("cylinder");
	ContourPointData cpTmp;
	BodyFlexible::addContour(contourCyl,cpTmp,false);
  }

  void BodyFlexible1s23BTA::init() {
	BodyFlexible1s::init();
	assert(0<Elements);

	l0 = L/Elements;
	Vec g = trans(JR)*mbs->getGrav();

	for(int i=0; i<Elements; i++) {
	  discretization.push_back(new FiniteElement1s23BTA(l0, A*rho, E*Iyy, E*Izz, It*rho, G*It, g ));
	  qElement.push_back(Vec(discretization[0]->getSizeOfPositions(),INIT,0.));
	  uElement.push_back(Vec(discretization[0]->getSizeOfVelocities(),INIT,0.));
	  dynamic_cast<FiniteElement1s23BTA*>(discretization[i])->setTorsionalDamping(dTorsional);
	}

	contourCyl->setAlphaStart(0);  contourCyl->setAlphaEnd(L);

	if(userContourNodes.size()==0) {
	  Vec contourNodes(Elements+1);
	  for(int i=0;i<=Elements;i++) contourNodes(i) = L/Elements * i; 
	  contourCyl->setNodes(contourNodes);
	}
	else contourCyl->setNodes(userContourNodes);
	
#ifdef HAVE_AMVIS
	if(boolAMVis) {
	  ElasticBody1sBTA *BTAbody = new ElasticBody1sBTA(fullName,Elements,1,boolAMVisBinary);
	  BTAbody->setElementLength(l0);
	  BTAbody->setRadius(static_cast<CylinderFlexible*>(contourCyl)->getRadius());
	  BTAbody->setColor(AMVisColor);

	  float amvisJ[3][3];
	  for(int i=0;i<3;i++) for(int j=0;j<3;j++) amvisJ[i][j] = JR(i,j);
	  BTAbody->setJacobian(amvisJ);
	  BTAbody->setInitialTranslation(WrON00(0),WrON00(1),WrON00(2));

	  bodyAMVis = BTAbody;
	}
#endif 
  }

  void BodyFlexible1s23BTA::setNumberElements(int n) {
	Elements = n;
	qSize = 5*( Elements + 1 ); 

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
	for(unsigned int i=0; i<port.size(); i++) { 
	  parafile << "# s: (port:  name= "<< port[i]->getName()<<",  ID= "<<port[i]->getID()<<") = ";
	  if(S_Port[i].type==CONTINUUM) parafile << S_Port[i].alpha(0) << endl;
	  if(S_Port[i].type==NODE     ) parafile << S_Port[i].ID*L/Elements  << endl;
	}
  }
  //-----------------------------------------------------------------------------------

  void BodyFlexible1s23BTA::updateKinematics(double t) {
	BuildElements();
	sTangent = -l0;
	updatePorts(t);
  }

  void BodyFlexible1s23BTA::updatePorts(double t) {
	for(unsigned int i=0; i<port.size(); i++) {
	  if(S_Port[i].type == CONTINUUM) // force on continuum
	  {
		const double &s = S_Port[i].alpha(0);
		double sElement = BuildElement( s );

		Vec Z = dynamic_cast<FiniteElement1s23BTA*>(discretization[CurrentElement])->StateAxis(qElement[CurrentElement],uElement[CurrentElement],sElement);
		Z(0) = s;
		port[i]->setWrOP(WrON00 + JR * Z( 0,2));
		port[i]->setWvP (         JR * Z( 6,8));
		port[i]->setWomegaP(      JR * Z(9,11));
	  }
	}
  }

  Mat BodyFlexible1s23BTA::computeJacobianMatrix(const ContourPointData &S_) {
	Index All(0,5-1);
	Mat Jacobian(qSize,5);

	if(S_.type == CONTINUUM) { // force on continuum
	  const double &s = S_.alpha(0);
	  double sElement = BuildElement( s );

	  Jacobian(activeElement,All) = dynamic_cast<FiniteElement1s23BTA*>(discretization[CurrentElement])->JGeneralized(qElement[CurrentElement],sElement);
	}
	if(S_.type == NODE)
	{
	  ContourPointData Stmp;
	  Stmp.alpha = Vec(1,INIT,S_.ID*L/Elements);
	  Jacobian = computeJacobianMatrix(Stmp);
	} 
	return Jacobian;
  }

  void BodyFlexible1s23BTA::GlobalMatrixContribution(int n) {
	activeElement = Index( discretization[n]->getSizeOfVelocities()/2*n , discretization[n]->getSizeOfVelocities()/2*(n+2) -1 );
	M(activeElement) += discretization[n]->getMassMatrix();
	h(activeElement) += discretization[n]->getGeneralizedForceVector();

	// if(implicit) {
	//   Dhq (j,j,j+7,j+7) += discretization[n]->getJacobianForImplicitIntegrationRegardingPosition();
	// 	Dhqp(j,j,j+7,j+7) += discretization[n]->getJacobianForImplicitIntegrationRegardingVelocity();
	// }
  }

  //-----------------------------------------------------------------------------------
  void BodyFlexible1s23BTA::BuildElements() {
	for(int i=0;i<Elements;i++) {
	activeElement = Index( discretization[i]->getSizeOfVelocities()/2*i , discretization[i]->getSizeOfVelocities()/2*(i+2) -1 );
	qElement[i] = q(activeElement);
	uElement[i] = u(activeElement);
	}
  }

  double BodyFlexible1s23BTA::BuildElement(const double& sGlobal) {
    CurrentElement = int(sGlobal/l0);   
    double sLokal = sGlobal - CurrentElement * l0; // Lagrange-Parameter of the affected FE 

    if(CurrentElement >= Elements) { // contact solver computes to large sGlobal at the end of the entire beam
      CurrentElement =  Elements-1;
      sLokal += l0;
    }

    activeElement = Index( discretization[CurrentElement]->getSizeOfVelocities()/2*CurrentElement, discretization[CurrentElement]->getSizeOfVelocities()/2*(CurrentElement+2) -1 );

    return sLokal;
  }

  void BodyFlexible1s23BTA::addPort(const string &name, const int &node) {
	Port *port = new Port(name);
	ContourPointData cpTemp;
	cpTemp.type = CONTINUUM;
	cpTemp.ID = node;
	cpTemp.alpha = Vec(1,INIT,node*L/Elements);
	addPort(port,cpTemp);
  }

  Mat BodyFlexible1s23BTA::computeWt  (const ContourPointData &S_) {
	const double &s = S_.alpha(0);
	double sElement = BuildElement(s);
	Vec Tangent = dynamic_cast<FiniteElement1s23BTA*>(discretization[CurrentElement])->Tangent(qElement[CurrentElement],sElement);
	Wt = JR * Tangent;
	return Wt.copy();
  }

  SqrMat BodyFlexible1s23BTA::computeAWK (const ContourPointData &S_) {
	const double &s = S_.alpha(0);
	double sElement = BuildElement(s);
	SqrMat AWK = dynamic_cast<FiniteElement1s23BTA*>(discretization[CurrentElement])->AWK(qElement[CurrentElement],sElement);
	return SqrMat(JR * AWK);
  }

  Vec BodyFlexible1s23BTA::computeWrOC(const ContourPointData &S_) {
	if (S_.alpha(0) != sTangent) 
	{
	  sTangent = S_.alpha(0);
	  double sElement = BuildElement(sTangent);
	  Vec Z = dynamic_cast<FiniteElement1s23BTA*>(discretization[CurrentElement])->StateAxis(qElement[CurrentElement],uElement[CurrentElement],sElement);
	  Z(0) = sTangent;
	  WrOC   = WrON00 + JR * Z(0,2);
	  WvC    =          JR * Z(6,8);
	  Womega =          JR * Z(9,11);
	}
	return WrOC.copy();
  }

  Vec BodyFlexible1s23BTA::computeWvC (const ContourPointData &S_) {
	if (S_.alpha(0) != sTangent) computeWrOC(S_);
	return WvC.copy();
  }

  Vec BodyFlexible1s23BTA::computeWomega (const ContourPointData &S_) {
	if (S_.alpha(0) != sTangent) 
	  computeWrOC(S_);
	return Womega.copy();
  }

}
