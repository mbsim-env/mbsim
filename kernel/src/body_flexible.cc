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
#include "body_flexible.h"

#include "port.h"
#include <link.h>
#include "contour.h"

#include "elastic.h"

using namespace AMVis;

namespace MBSim {

  BodyFlexible::BodyFlexible(const string &name) : Body(name), WLtmp(6), WFtmp(WLtmp(0,2)), WMtmp(WLtmp(3,5)), boolAMVis(false), bodyAMVis(NULL), boolAMVisBinary(true) { }

  void BodyFlexible::init() {
    Body::init();
    IndexForce  = Index(         0,JT.cols()          -1 );
    IndexMoment = Index( JT.cols(),JT.cols()+JR.cols()-1 );
  }


  Port* BodyFlexible::getPort(const string &name) {
    int i;
    for(i=0; i<port.size(); i++) {
      if(port[i]->getName() == name)
	return port[i];
    }
    assert(i<port.size());
  }

  void BodyFlexible::addPort(const string &name, const ContourPointData &S_) {
    Port *port = new Port(name);
    addPort(port,S_);
  }

  void BodyFlexible::addPort(Port* port, const ContourPointData &S_) {
    Body::addPort(port);
    S_Port.push_back(S_);
  }

  void BodyFlexible::addContour(Contour *contour, const ContourPointData &S_,bool constPosition) {
    Body::addContour(contour);
    constContourPosition.push_back(constPosition);
    S_Contour.push_back(S_);
  }

  void BodyFlexible::initPlotFiles() {
    Element::initPlotFiles(); //hohe Ableitung; ich will nur die Zeit!!!

    if(plotLevel) {
      if(plotLevel>=1)
	for(int i=0; i<qSize; ++i)
	  plotfile <<"# "<< plotNr++ << ": q(" << i << ")" << endl;
      if(plotLevel>=2)
	for(int i=0; i<uSize; ++i) 
	  plotfile <<"# "<< plotNr++ << ": u(" << i << ")" << endl;
      if(plotLevel>=3) {
	plotfile <<"# " << plotNr++ << ": T" << endl;
	plotfile <<"# " << plotNr++ << ": V" << endl;
	plotfile <<"# " << plotNr++ << ": E" << endl;
      }
    }

    if(boolAMVis)
      bodyAMVis->writeBodyFile();
  }

  void BodyFlexible::plot(double t, double dt) {
    Element::plot(t); //hohe Ableitung; ich will nur die Zeit!!!

    // plotlevel-dependent
    if(plotLevel) {
      if(plotLevel>=1)
	for(int i=0; i<qSize; ++i)
	  plotfile<<" "<<q(i);
      if(plotLevel>=2) 
	for(int i=0; i<uSize; ++i)
	  plotfile<<" "<<u(i);
      if(plotLevel>=3) {
	double Ttemp = computeKineticEnergy();
	double Vtemp = computePotentialEnergy();
	plotfile<<" "<< Ttemp;
	plotfile<<" "<< Vtemp;
	plotfile<<" "<< Ttemp + Vtemp;
      }
    }

    // visualisationFile-dependent
    if(boolAMVis) {
      float qDummy[qSize];
      for(int i=0;i<qSize;i++) qDummy[i] = q(i);
      bodyAMVis->setTime(t);
      bodyAMVis->setCoordinates(qDummy);
      bodyAMVis->appendDataset(0);
    }
  }

  void BodyFlexible::plotParameters() {
    parafile << "BodyFlexible"  << endl;
  }

  void BodyFlexible::updatezd(double t) {
    qd = u;
    ud = slvLLFac(LLM, h+r);
  }

  void BodyFlexible::updatedu(double t, double dt) {
    ud = slvLLFac(LLM, h*dt+r);
  }

  void BodyFlexible::updatedq(double t, double dt) {
    qd = u*dt;
  }


  //-------------------------------------------

  void BodyFlexible::sumUpForceElements(double t) {
    Vec WLtmpLocal( JT.cols() + JR.cols() ); // Kraefte und Momente in Dimension des Modells bringen

    for(int i=0; i<linkSingleValuedPortData.size(); i++) {
      const int &portID = linkSingleValuedPortData[i].ID;
      const int &objectID = linkSingleValuedPortData[i].objectID;

      WLtmp = linkSingleValuedPortData[i].link->getLoad(objectID); // Vec

      if(JT.cols()) WLtmpLocal(IndexForce ) = trans(JT)*WFtmp ;
      if(JR.cols()) WLtmpLocal(IndexMoment) = trans(JR)*WMtmp;

      h += computeJacobianMatrix(S_Port[portID]) * WLtmpLocal;
    }

    for(int i=0; i<linkSingleValuedContourData.size(); i++) {
      if(linkSingleValuedContourData[i].link->isActive()) {
	const int &ID       = linkSingleValuedContourData[i].ID;       // ID der Contour in der Koerper-Daten-Verwaltung de
	const int &objectID = linkSingleValuedContourData[i].objectID; // ID der Contour innerhalb der LinkContour-Paarung
	WLtmp = linkSingleValuedContourData[i].link->getLoad(objectID);

	ContourPointData Stmp = linkSingleValuedContourData[i].link->getContourPointData(objectID);

	Vec WrSC = Stmp.WrOC - computeWrOC(Stmp);

	if(JT.cols()) WLtmpLocal(IndexForce ) = trans(JT)*WFtmp ;
	if(JR.cols()) WLtmpLocal(IndexMoment) = trans(JR)*(WMtmp + crossProduct(WrSC,WFtmp));

	if(!constContourPosition[ID])    // alle relevanten Daten kommen aus dem Link
	  h += computeJacobianMatrix(Stmp) * WLtmpLocal;
	else                             // hinzugefuegte Contour: nutze Aufhaenge-Daten (cpData) der Contour auf dem Body
	  h += computeJacobianMatrix(S_Contour[ID]) * WLtmpLocal;

	//		    h += computeJacobianMatrix(Stmp) * WLtmpLocal;
      }
    }
  }

  //-------------------------------------------

  void BodyFlexible::updateW(double t) {
    static Index IF(0,2);
    static Index IM(3,5);

    vector<LinkPortData>::iterator it1=linkSetValuedPortData.begin(); 
    vector<LinkContourData>::iterator it2=linkSetValuedContourData.begin(); 
    vector<Mat>::iterator itW=W.begin(); 
    vector<Vec>::iterator itw=w.begin(); 

    for(int i=0; i<linkSetValuedPortData.size(); i++) {
      const int &portID   = it1->ID;
      const int &objectID = it1->objectID;

      Mat ld = it1->link->getLoadDirections(objectID);

      Index LoadDirCols(0,ld.cols()-1);
      Mat fd = ld(IF,LoadDirCols);
      Mat md = ld(IM,LoadDirCols);

      Mat ldTmp( JT.cols() + JR.cols(), ld.cols() ); // Kraefte und Momente in Dimension des Modells bringen
      if(JT.cols()) ldTmp(IndexForce,LoadDirCols)  = trans(JT) * fd;
      if(JR.cols()) ldTmp(IndexMoment,LoadDirCols) = trans(JR) * md;

      (*itW).init(0.0);
      (*itW) = computeJacobianMatrix(S_Port[portID]) * ldTmp;

      it1++; itW++; itw++;
    }    

    for(int i=0; i<linkSetValuedContourData.size(); i++) {
      if(it2->link->isActive()) {
	const int &ID        = it2->ID;       // ID der Contour in der Koerper-Daten-Verwaltung des Body
	const int &objectID  = it2->objectID; // ID der Contour innerhalb der LinkContour-Paarung
	Mat ld = it2->link->getLoadDirections(objectID);

	Index LoadDirCols(0,ld.cols()-1);
	Mat fd = ld(IF,LoadDirCols);
	Mat md = ld(IM,LoadDirCols);

	ContourPointData Stmp = it2->link->getContourPointData(objectID);

	// Kraefte und Momente in Dimension des Modells bringen
	Mat ldTmp( JT.cols() + JR.cols(), ld.cols() );
	if(JT.cols()) ldTmp(IndexForce,LoadDirCols)  = trans(JT) * fd;
	if(JR.cols()) {
	  Vec WrSC = Stmp.WrOC - computeWrOC(Stmp);
	  ldTmp(IndexMoment,LoadDirCols) = trans(JR) * (md + tilde(WrSC)*fd);
	}

	if(!constContourPosition[ID])   // alle relevanten Daten kommen aus dem Link
	  (*itW) = computeJacobianMatrix(Stmp) * ldTmp;
	else                            // hinzugefuegte Contour: nutze Aufhaenge-Daten (cpData) der Contour auf dem Body
	  (*itW) = computeJacobianMatrix(S_Contour[ID]) * ldTmp;

	//           (*itW) = computeJacobianMatrix(Stmp) * ldTmp;
      }

      it2++; itW++; itw++;
    }
  }



  //####################################################
  //####################################################

  BodyFlexible1s::BodyFlexible1s(const string &name) 
    :BodyFlexible(name), userContourNodes(0) { 

//      contourR = new Contour1sFlexible("R");
//      contourL = new Contour1sFlexible("L");
//      ContourPointData cpTmp;
//      BodyFlexible::addContour(contourR,cpTmp,false);
//      BodyFlexible::addContour(contourL,cpTmp,false);
    }

  void BodyFlexible1s::addPort(const string &name, const double &s) {
    ContourPointData temp;
    temp.type  = CONTINUUM;
    temp.alpha = Vec(1,INIT,s);
    addPort(name,temp); 
  }
  void BodyFlexible1s::addContour(Contour *contour, const double &s) {
    ContourPointData temp;
    temp.type  = CONTINUUM;
    temp.alpha = Vec(1,INIT,s);
    addContour(contour,temp); 
  }

}

  /////////  //####################################################
  /////////  
  /////////  BodyFlexible2s::BodyFlexible2s(const string &name) 
  /////////      :BodyFlexible(name) {
  /////////  
  /////////      contourR = new Contour2sFlexible("R");
  /////////      contourL = new Contour2sFlexible("L");
  /////////      ContourPointData cpTmp;
  /////////      BodyFlexible::addContour(contourR,cpTmp,false);
  /////////      BodyFlexible::addContour(contourL,cpTmp,false);
  /////////  }
