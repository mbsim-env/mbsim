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
#include "port.h"
#include "contour.h"
#include "multi_body_system.h"

namespace MBSim {

  BodyFlexibleLinearExternal::BodyFlexibleLinearExternal(const string &name) 
    :BodyFlexible(name), K(0), Mread(0) ,alpha(0), beta(0), nContours(0), WrON00(Vec(3)) {
      JT << DiagMat(3,INIT,1.0);
    }

  void BodyFlexibleLinearExternal::updateKinematics(double t) {
    updatePorts(t); 
    updateContours(t); 
  }

  void BodyFlexibleLinearExternal::updatePorts(double t) {
    for(int i=0; i<port.size(); i++) {
      if( (J[port[i]->getID()]).rows() ) {
	Vec rPort = trans(J[port[i]->getID()]) * q ;
	Vec uPort = trans(J[port[i]->getID()]) * u ;

	port[i]->setWrOP(WrON00  + JT * (KrP[port[i]->getID()] + rPort));
	port[i]->setWvP (          JT *                          uPort);
      }
    }
  }

  void BodyFlexibleLinearExternal::updateContours(double t) {
    Vec WrHS(3);
    RHitSphere = 0;

    for(int i=0; i<contour.size(); i++) {
      if( (J[contour[i]->getID()]).rows() ) { // echte, keine Interpolationscontour  // TODO: nutze constContourPosition aus BodyFlexible
	Vec rContour = trans(J[contour[i]->getID()]) * q ;
	Vec uContour = trans(J[contour[i]->getID()]) * u ;

	Vec WrOP = WrON00  + JT * (KrP[contour[i]->getID()] + rContour);

	contour[i]->setWrOP(      WrOP   );
	contour[i]->setWvP (JT * uContour);

	WrHS += WrOP;
	double R = nrm2(WrOHitSphere-WrOP);
	if( R > RHitSphere)
	  RHitSphere = R;	
      }
    }
    WrOHitSphere =  WrHS/nContours;
    //     cout << name << ": midpoint HitSphere " << trans(WrOHitSphere) << "Radius " << RHitSphere << endl;
  }

  Vec BodyFlexibleLinearExternal::computeWrOC(const ContourPointData& CP) {
    //     return (contour[ CP.ID ]->getWrOP());
    if(CP.type == NODE) {
      return (WrON00  + JT * (KrP[CP.ID] + trans(J[CP.ID]) * q) ) ;
    } else if(CP.type == EXTINTERPOL) {
      Vec Temp(3);
      for (int i=0;i<CP.iPoints.size();i++) {
	int ID = CP.iPoints[i]->getID();
	Temp += CP.iWeights(i) * ( KrP[ID] + trans(J[ID]) * q );
      }
      return (WrON00  + JT * (Temp) ) ;
    }
  }
  Vec BodyFlexibleLinearExternal::computeWvC (const ContourPointData& CP) {
    //     return (contour[ CP.ID ]->getWvP() );
    //     return (          JT *               trans(J[CP.ID]) * u  ) ;
    // //     return (contour[ CP.ID ]->getWrOP());
    if(CP.type == NODE) {
      return JT * ( trans(J[CP.ID]) * u) ;
    } else if(CP.type == EXTINTERPOL) {
      Vec Temp(3);
      for (int i=0;i<CP.iPoints.size();i++) {
	int ID = CP.iPoints[i]->getID();
	Temp += CP.iWeights(i) * ( trans(J[ID]) * u );
      }
      return JT * (Temp) ;
    }
  }
  Vec BodyFlexibleLinearExternal::computeWomega(const ContourPointData& CP) {
    //     return (contour[ CP.ID ]->getWomegaC() );
    return Vec(3);
  }

  void BodyFlexibleLinearExternal::updateh(double t) {
    h = - K * q - D * u;

    sumUpForceElements(t);
  }

  void BodyFlexibleLinearExternal::init() {
    BodyFlexible::init();
    // Groessen von Massen- und Steifigkeitsmatrix vergleichen
    if(Mread.size() != K.size()) {
      cout << "Massmatrix and stiffnessmatrix have unequal sizes!!!" << endl;
      throw 1;
    }
    M = Mread;
    LLM = facLL(M);
    D = static_cast<SqrMat>( alpha*M + beta * K ); 

    for(int i=0; i<port.size(); i++) {
      Mat JTemp = J[port[i]->getID()];
      if(JTemp.rows() != M.size()) {
	cout << "Jacobimatrix of port " << i ;
	cout << " does not fit in size to massmatrix and stiffnessmatrix!!!" << endl;
	throw 1;
      }
    }
    for(int i=0; i<contour.size(); i++) {
      Mat JTemp = J[contour[i]->getID()];
      if(JTemp.rows() != M.size() && JTemp.rows() != 0) {
	cout << "Jacobimatrix of contour " << i ;
	cout << " does not fit in size to massmatrix and stiffnessmatrix!!!" << endl;
	cout << JTemp << endl;
	throw 1;
      }
    }
  }

  void BodyFlexibleLinearExternal::setMassMatrix(const SymMat &mat) {
    Mread = mat;
    qSize = Mread.size();
    uSize = qSize;
  }

  void BodyFlexibleLinearExternal::readMassMatrix(const string &massfilename) {
    fstream datafile(massfilename.c_str(),ios::in);
    if (!datafile.is_open()) {
      cout << "File " << massfilename << " containing massmatrix not found." << endl;
      throw 1;
    }

    Mat MTemp;
    datafile >> MTemp;

    Mread = (SymMat)(MTemp);

    qSize = Mread.size();
    uSize = qSize;

    datafile.close();
  }

  void BodyFlexibleLinearExternal::setStiffnessMatrix(const SqrMat &mat) {
    K = mat;
  }

  void BodyFlexibleLinearExternal::readStiffnessMatrix(const string &stiffnessfilename) {
    fstream datafile(stiffnessfilename.c_str(),ios::in);
    if (!datafile.is_open()) {
      cout << "File " << stiffnessfilename << " containing stiffnessmatrix not found." << endl;
      throw 1;
    }

    SqrMat KTemp;
    datafile >> KTemp;

    K = KTemp;

    datafile.close();
  }

  double BodyFlexibleLinearExternal::computePotentialEnergy() {
    return 0.5*trans(q)*K*q;
  }

  //----------------------------------------------------------------------------------------

  void BodyFlexibleLinearExternal::addPort(const string &name, const Mat &J_, const Vec &r_) {
    Port *port_ = new Port(name);
    port_->setID(J.size()); // Stelle, an der die Jacobi steht

    BodyFlexible::addPort(port_, addInterface(J_,r_) );
  }
  void BodyFlexibleLinearExternal::addPort(const string &name, const string &jacobifile) {
    Port *port_ = new Port(name);
    port_->setID(J.size()); // Stelle, an der die Jacobi steht

    BodyFlexible::addPort(port_, addInterface(jacobifile) );
  }
  void BodyFlexibleLinearExternal::addContour(Contour *contour_, const Mat &J_, const Vec &r_) {
    contour_->setID(J.size()); // Stelle, an der die Jacobi steht

    BodyFlexible::addContour(contour_, addInterface(J_,r_) );

    nContours++;
  }
  void BodyFlexibleLinearExternal::addContour(Contour *contour_, const string &jacobifile) {
    contour_->setID(J.size()); // Stelle, an der die Jacobi steht

    BodyFlexible::addContour(contour_, addInterface(jacobifile) );

    nContours++;
  }


  void BodyFlexibleLinearExternal::addContourInterpolation(ContourInterpolation *contour_) {
    contour_->setID(J.size()); // Stelle, an der die Jacobi steht

    J.push_back(Mat(0,0)); // traegt keine eigene Jacobi
    KrP.push_back(Vec(0)); // traegt keinen eigenen Vektor
    ContourPointData cpData;
    cpData.type=EXTINTERPOL;
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

    ContourPointData CPTemp;
    CPTemp.type = NODE;
    CPTemp.ID   = J.size();
    CPTemp.WrOC = KrPTemp.col(0);

    J.  push_back(JTemp);
    KrP.push_back(KrPTemp.col(0));

    return CPTemp;
  }

  ContourPointData BodyFlexibleLinearExternal::addInterface(const Mat &J_, const Vec &r_) {
    ContourPointData CPTemp;
    CPTemp.type = NODE;
    CPTemp.ID   = J.size();
    CPTemp.WrOC = r_;

    J.  push_back(J_);
    KrP.push_back(r_);

    return CPTemp;
  }


  // //---------------------------------------------------------------------------
  Mat BodyFlexibleLinearExternal::computeJacobianMatrix(const ContourPointData& CP) {
    //     return J[ CP.ID ];
    // }
    if(CP.type == NODE) {
      return J[ CP.ID ] ;
    } else if(CP.type == EXTINTERPOL) {
      Mat JTemp(J[0].rows(),J[0].cols());
      for (int i=0;i<CP.iPoints.size();i++) {
	JTemp += CP.iWeights(i) * J[CP.iPoints[i]->getID()];
      }
      return JTemp ;
    }
  }


  void BodyFlexibleLinearExternal::plotParameters() {
    parafile << "BodyFlexibleLinearExternal\n---------------------------\n"  << endl;
    parafile << "# MassMatrix\n" << M << endl;
    parafile << "\n# StiffnessMatrix\n" << K << endl;

    parafile << "\n# JT\n"      << JT   << endl;
    parafile << "\n# JR\n"      << JR   << endl;

    if(port.size()>0) parafile << "\nports:" <<endl;
    for(int i=0; i<port.size(); i++) { 
      parafile << "# J: (port:  name= "<< port[i]->getName()<<",  ID= "<<port[i]->getID()<<") \n"<< J[port[i]->getID()]<<endl;
    }

    if(contour.size()>0) parafile << "\ncontours:" <<endl;
    for(int i=0; i<contour.size(); i++) {
      if(dynamic_cast<Point*>(contour[i])) {
	parafile << "# J: (contour:  name= "<< contour[i]->getName()<<",  ID= "<<contour[i]->getID()<<")"<< endl;
	parafile << J[contour[i]->getID()]<<endl;
      } else {
	parafile << "# extern -> contour:  name= "<< contour[i]->getName()<<",  ID= "<<contour[i]->getID() << endl;
      }
    }

  }

}
