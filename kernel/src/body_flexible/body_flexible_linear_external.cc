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
#include "body_flexible_linear_external.h"
#include "superelement_linear_external.h"
#include "port.h"
#include "contour.h"
#include "multi_body_system.h"

namespace MBSim {

  BodyFlexibleLinearExternal::BodyFlexibleLinearExternal(const string &name) 
	:BodyFlexible(name),
	nContours(0), WrON00(Vec(3)) {
	  JT << DiagMat(3,INIT,1.0);
	  discretization.push_back(new SuperElementLinearExternal());
	  qElement.push_back(Vec(0));
	  uElement.push_back(Vec(0));
	}

  void BodyFlexibleLinearExternal::GlobalMatrixContribution(int i) {
    M += discretization[i]->getMassMatrix();
    h += discretization[i]->getGeneralizedForceVector();
  }

  void BodyFlexibleLinearExternal::setProportionalDamping(const double &a, const double &b) {
	static_cast<SuperElementLinearExternal*>(discretization[0])->setProportionalDamping(a,b);
  }
  ContourPointData BodyFlexibleLinearExternal::addInterface(const Mat &J, const Vec &r) {
	return static_cast<SuperElementLinearExternal*>(discretization[0])->addInterface(J,r);
  }

  void BodyFlexibleLinearExternal::updateKinematics(double t) {
    if(qElement[0]!=q) qElement[0] >> q;
    if(uElement[0]!=u) uElement[0] >> u;

	updatePorts(t); 
	updateContours(t); 
  }

  void BodyFlexibleLinearExternal::updatePorts(double t) {
	for(unsigned int i=0; i<port.size(); i++) {
	  ContourPointData cp;
	  cp.ID = port[i]->getID();
	  Mat J = static_cast<SuperElementLinearExternal*>(discretization[0])->computeJacobianOfMinimalRepresentationRegardingPhysics(qElement[0],cp);
//~~      if( J.rows() ) {
		port[i]->setWrOP(WrON00  + JT *  static_cast<SuperElementLinearExternal*>(discretization[0])->computeTranslation(qElement[0],cp)            );
		port[i]->setWvP (          JT *  static_cast<SuperElementLinearExternal*>(discretization[0])->computeTranslationalVelocity(qElement[0],uElement[0],cp));
//~~	  }
	}
  }

  void BodyFlexibleLinearExternal::updateContours(double t) {
    Vec WrHS(3);
    RHitSphere = 0;

    for(unsigned int i=0; i<contour.size(); i++)
	  if(contourType[i].type==NODE) {
	  ContourPointData cp;
	  cp.ID = contour[i]->getID();
	  Mat J = static_cast<SuperElementLinearExternal*>(discretization[0])->computeJacobianOfMinimalRepresentationRegardingPhysics(qElement[0],cp);
//~~      if( J.rows() ) { // echte, keine Interpolationscontour  // TODO: nutze constContourPosition aus BodyFlexible
		contour[i]->setWrOP(WrON00  + JT *  static_cast<SuperElementLinearExternal*>(discretization[0])->computeTranslation(qElement[0],cp)            );
		contour[i]->setWvP (          JT *  static_cast<SuperElementLinearExternal*>(discretization[0])->computeTranslationalVelocity (qElement[0],uElement[0],cp));
//// 	Vec rContour = trans(J[contour[i]->getID()]) * q ;
//// 	Vec uContour = trans(J[contour[i]->getID()]) * u ;
//// 
//// 	Vec WrOP = WrON00  + JT * (KrP[contour[i]->getID()] + rContour);
//// 
//// 	contour[i]->setWrOP(      WrOP   );
//// 	contour[i]->setWvP (JT * uContour);
//// 
	WrHS += contour[i]->getWrOP();
	double R = nrm2(WrOHitSphere-contour[i]->getWrOP());
	if( R > RHitSphere)
	  RHitSphere = R;	
//~~      }
    }
    WrOHitSphere =  WrHS/nContours;
    //     cout << name << ": midpoint HitSphere " << trans(WrOHitSphere) << "Radius " << RHitSphere << endl;
  }

  Vec BodyFlexibleLinearExternal::computeWrOC(const ContourPointData& CP) {
	Vec WrOC;
	if(CP.type == NODE)
	  WrOC = (WrON00  + JT *  static_cast<SuperElementLinearExternal*>(discretization[0])->computeTranslation(qElement[0],CP) ); // JT * (KrP[CP.ID] + trans(J[CP.ID]) * q) ) ;
	else if(CP.type == EXTINTERPOL) {
	  Vec Temp(3);
	  for (unsigned int i=0;i<CP.iPoints.size();i++) {
		ContourPointData cpTemp; cpTemp.type=NODE;
		cpTemp.ID = CP.iPoints[i]->getID();
//		Temp += CP.iWeights(i) * ( KrP[ID] + trans(J[ID]) * q );
		Temp += CP.iWeights(i) * (computeWrOC(cpTemp));
	  }
	  WrOC = (Temp) ;
	}
	return WrOC;
  }
 
  Mat BodyFlexibleLinearExternal::computeJacobianMatrix(const ContourPointData& CP) {
	Mat Jreturn;
	if(CP.type == NODE) {
	  Jreturn = discretization[0]->computeJacobianOfMinimalRepresentationRegardingPhysics(qElement[0],CP);
	} else if(CP.type == EXTINTERPOL) {
	  for(unsigned int i=0;i<(CP.iPoints).size();i++) {
		ContourPointData cpTemp; cpTemp.type=NODE;
		cpTemp.ID = CP.iPoints[i]->getID();
		if(!Jreturn.rows()) Jreturn  = CP.iWeights(i) * ( computeJacobianMatrix (cpTemp) );
		else                Jreturn += CP.iWeights(i) * ( computeJacobianMatrix (cpTemp) );
	  }
	}
	return Jreturn;    
  }

  Vec BodyFlexibleLinearExternal::computeWvC (const ContourPointData& CP) {
	Vec WvC;
	if(CP.type == NODE) {
	  WvC = JT *  static_cast<SuperElementLinearExternal*>(discretization[0])->computeTranslationalVelocity(qElement[0],uElement[0],CP) ;
	}
   	else if(CP.type == EXTINTERPOL) {
	  for (unsigned int i=0;i<CP.iPoints.size();i++) {
		ContourPointData cpTemp; cpTemp.type=NODE;
		cpTemp.ID = CP.iPoints[i]->getID();
		if(!WvC.size()) WvC  = CP.iWeights(i) * ( computeWvC (CP) );
		else            WvC += CP.iWeights(i) * ( computeWvC (CP) );
	  }
	}
	return WvC;
  }

  Vec BodyFlexibleLinearExternal::computeWomega(const ContourPointData& CP) {
    //     return (contour[ CP.ID ]->getWomegaC() );
    return Vec(3);
  }

  void BodyFlexibleLinearExternal::init() {
    BodyFlexible::init();
	for(unsigned int i=0;i<discretization.size();i++)
	  static_cast<SuperElementLinearExternal*>(discretization[i])->init();
    // Groessen von Massen- und Steifigkeitsmatrix vergleichen
    M = discretization[0]->getMassMatrix();
    LLM = facLL(M);

    Mat Jh = mbs->getJh()(Iu,Index(0,mbs->getzSize()-1));
    Jh(Index(0,uSize-1),Index(    0,qSize      -1)) << discretization[0]->getJacobianForImplicitIntegrationRegardingPosition();
    Jh(Index(0,uSize-1),Index(qSize,qSize+uSize-1)) << discretization[0]->getJacobianForImplicitIntegrationRegardingVelocity();
  }

  void BodyFlexibleLinearExternal::setMassMatrix(const SymMat &mat) {
	static_cast<SuperElementLinearExternal*>(discretization[0])->setM(mat);
	uSize = discretization[0]->getSizeOfVelocities();
  }
  void BodyFlexibleLinearExternal::readMassMatrix(const string &massfilename) {
	fstream datafile(massfilename.c_str(),ios::in);
	if (!datafile.is_open()) {
	  cout << "File " << massfilename << " containing massmatrix not found." << endl;
	  throw 1;
	}
	Mat MTemp;
	datafile >> MTemp;
	setMassMatrix((SymMat)(MTemp));
	datafile.close();
  }

  void BodyFlexibleLinearExternal::setStiffnessMatrix(const SqrMat &mat) {
	static_cast<SuperElementLinearExternal*>(discretization[0])->setK(mat);
	qSize = discretization[0]->getSizeOfPositions();
  }
  void BodyFlexibleLinearExternal::readStiffnessMatrix(const string &stiffnessfilename) {
    fstream datafile(stiffnessfilename.c_str(),ios::in);
    if (!datafile.is_open()) {
      cout << "File " << stiffnessfilename << " containing stiffnessmatrix not found." << endl;
      throw 1;
    }
    SqrMat KTemp;
    datafile >> KTemp;
    setStiffnessMatrix( KTemp );
    datafile.close();
  }

  //----------------------------------------------------------------------------------------

  void BodyFlexibleLinearExternal::addPort(const string &name, const Mat &J_, const Vec &r_) {
	ContourPointData cp = addInterface(J_,r_);
    Port *port_ = new Port(name);
    port_->setID(cp.ID); // Stelle, an der die Jacobi steht
    BodyFlexible::addPort(port_, cp );
  }

  void BodyFlexibleLinearExternal::addPort(const string &name, const string &jacobifile) {
    ContourPointData cp = addInterface(jacobifile);
    Port *port_ = new Port(name);
    port_->setID(cp.ID); // Stelle, an der die Jacobi steht
    BodyFlexible::addPort(port_, cp );
  }

  void BodyFlexibleLinearExternal::addContour(Contour *contour_, const Mat &J_, const Vec &r_) {
	ContourPointData cp = addInterface(J_,r_);
	contourType.push_back(cp);
    contour_->setID(cp.ID); // Stelle, an der die Jacobi steht
    BodyFlexible::addContour(contour_, cp );
    nContours++;
  }
  void BodyFlexibleLinearExternal::addContour(Contour *contour_, const string &jacobifile) {
	ContourPointData cp = addInterface(jacobifile);
	contourType.push_back(cp);
    contour_->setID(cp.ID); // Stelle, an der die Jacobi steht
    BodyFlexible::addContour(contour_, cp );
    nContours++;
  }

  void BodyFlexibleLinearExternal::addContourInterpolation(ContourInterpolation *contour_) {
    contour_->setID(contourType.size()); // Stelle, an der die Jacobi steht
    ContourPointData cpData;
    cpData.type=EXTINTERPOL;
	contourType.push_back(cpData);
    BodyFlexible::addContour(contour_,cpData,false); 
  }


  ContourPointData BodyFlexibleLinearExternal::addInterface(const string &jacobifile) {

    fstream datafile(jacobifile.c_str(),ios::in);
    if (!datafile.is_open()) {
      cout << "File " << jacobifile << " containing Jacobimatrix not found." << endl;
      throw 1;
    }

    //      cout << "reading from " << jacobifile << endl;

    Mat JTemp;
    datafile >> JTemp;

    //      cout << "Jacobian: " << JTemp << endl;

    char buffer[100]; // fmatvec-read method does not read end of line
    datafile.getline(buffer,100);

    Mat KrPTemp;
    datafile >> KrPTemp;

    //      cout << "position: " << KrPTemp << endl;

////     ContourPointData CPTemp;
////     CPTemp.type = NODE;
////     CPTemp.ID   = J.size();
////     CPTemp.WrOC = KrPTemp.col(0);
//// 
////     J.  push_back(JTemp);
////     KrP.push_back(KrPTemp.col(0));

    return addInterface(JTemp,KrPTemp.col(0));
  }

//  ContourPointData BodyFlexibleLinearExternal::addInterface(const Mat &J_, const Vec &r_) {
//////    ContourPointData CPTemp;
//////    CPTemp.type = NODE;
//////    CPTemp.ID   = J.size();
//////    CPTemp.WrOC = r_;
//////
//////    J.  push_back(J_);
//////    KrP.push_back(r_);
//    ContourPointData CPTemp;
//    CPTemp.type = NODE;
//    CPTemp.ID   = discretization[0]->addInterface(J_,r_);
//    CPTemp.WrOC = r_;
//
//    return CPTemp;
//  }



  void BodyFlexibleLinearExternal::plotParameters() {
    parafile << "BodyFlexibleLinearExternal\n---------------------------\n"  << endl;
    parafile << "# MassMatrix\n" << M << endl;
    parafile << "\n# StiffnessMatrix\n" << -discretization[0]->getJacobianForImplicitIntegrationRegardingPosition() << endl;

    parafile << "\n# JT\n"      << JT   << endl;
    parafile << "\n# JR\n"      << JR   << endl;

    if(port.size()>0) parafile << "\nports:" <<endl;
    for(unsigned int i=0; i<port.size(); i++) { 
	  ContourPointData cp; cp.ID=port[i]->getID();
      parafile << "# J: (port:  name= "<< port[i]->getName()<<",  ID= "<<port[i]->getID()<<") \n"<<discretization[0]->computeJacobianOfMinimalRepresentationRegardingPhysics(qElement[0],cp)<<endl;
    }

	if(contour.size()>0) parafile << "\ncontours:" <<endl;
	for(unsigned int i=0; i<contour.size(); i++) {
	  if(contourType[i].type!=EXTINTERPOL) {
		parafile << "# J: (contour:  name= "<< contour[i]->getName()<<",  ID= "<<contour[i]->getID()<<")"<< endl;
		parafile << discretization[0]->computeJacobianOfMinimalRepresentationRegardingPhysics(qElement[0],contourType[i])<<endl;
	  } else {
		parafile << "# extern -> contour:  name= "<< contour[i]->getName()<<",  ID= "<<contour[i]->getID() << endl;
	  }
	}
  }

  void BodyFlexibleLinearExternal::updateJh_internal(double t) {
//    Mat Jh = mbs->getJh()(Iu,Index(0,mbs->getzSize()-1));
//    Jh(Index(0,uSize-1),Index(    0,qSize      -1)) << -K;
//    Jh(Index(0,uSize-1),Index(qSize,qSize+uSize-1)) << -D;
  }
}
