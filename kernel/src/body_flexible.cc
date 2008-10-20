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

#include "eps.h"

#ifdef HAVE_AMVIS
#include "elastic.h"
using namespace AMVis;
#endif

namespace MBSim {

  BodyFlexible::BodyFlexible(const string &name) : Body(name), WLtmp(6), WFtmp(WLtmp(0,2)), WMtmp(WLtmp(3,5)),d_massproportional(0)
# ifdef HAVE_AMVIS
												   ,
												   bodyAMVis(NULL), boolAMVis(false), boolAMVisBinary(true)
# endif
												   { }

  BodyFlexible::~BodyFlexible() {
	for(unsigned int i=0; i<discretization.size(); i++)
	  discretization[i]->~DiscretizationInterface();
#  ifdef HAVE_AMVIS
	delete bodyAMVis; bodyAMVis=NULL;
#  endif
  }

  void BodyFlexible::init() {
	Body::init();
	IndexForce  = Index(         0,JT.cols()          -1 );
	IndexMoment = Index( JT.cols(),JT.cols()+JR.cols()-1 );
	T = SqrMat(qSize,fmatvec::EYE);
  }

  void BodyFlexible::updateh(double t) {
	M.init(0.0);
	h.init(0.0);

	//    if(implicit) { // implicit integration
	//      Jhq = SqrMat(qSize,INIT,0.);
	//      Jhqt = SqrMat(qSize,INIT,0.);
	//    }

	for(unsigned int i=0;i<discretization.size();i++) {
	  discretization[i]->computeEquationsOfMotion(qElement[i],uElement[i]); // compute attributes of FE

	  GlobalMatrixContribution(i); // assemble
	}

	sumUpForceElements(t); // external and bearing forces have to be added to hG (body_flexible.h)
  }

  Port* BodyFlexible::getPort(const string &name) {
	unsigned int i;
	for(i=0; i<port.size(); i++) {
	  if(port[i]->getName() == name)
		return port[i];
	}
	assert(i<port.size());
	return NULL;
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

#ifdef HAVE_AMVIS
	if(boolAMVis)
	  bodyAMVis->writeBodyFile();
#endif
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

#ifdef HAVE_AMVIS
	// visualisationFile-dependent
	if(boolAMVis) {
	  float *qDummy = (float*) malloc(qSize*sizeof(float));
	  for(int i=0;i<qSize;i++) qDummy[i] = q(i);
	  bodyAMVis->setTime(t);
	  bodyAMVis->setCoordinates(qDummy);
	  bodyAMVis->appendDataset(0);
	  free(qDummy);
	}
#endif
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

	for(unsigned int i=0; i<linkSingleValuedPortData.size(); i++) {
	  const int &portID = linkSingleValuedPortData[i].ID;
	  const int &objectID = linkSingleValuedPortData[i].objectID;

	  WLtmp = linkSingleValuedPortData[i].link->getLoad(objectID); // Vec

	  if(JT.cols()) WLtmpLocal(IndexForce ) = trans(JT)*WFtmp ;
	  if(JR.cols()) WLtmpLocal(IndexMoment) = trans(JR)*WMtmp;

	  h += computeJacobianMatrix(S_Port[portID]) * WLtmpLocal;
	}

	for(unsigned int i=0; i<linkSingleValuedContourData.size(); i++) {
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
	if(d_massproportional)
	  h -= d_massproportional*(M*u);
  }

  //-------------------------------------------

  void BodyFlexible::updateWj(double t) {
	static Index IF(0,2);
	static Index IM(3,5);

	vector<LinkPortData>::iterator it1=linkSetValuedPortData.begin(); 
	vector<LinkContourData>::iterator it2=linkSetValuedContourData.begin(); 
	vector<Mat>::iterator itW=W.begin(); 
	vector<Vec>::iterator itw=w.begin(); 

	for(unsigned int i=0; i<linkSetValuedPortData.size(); i++) {
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

	for(unsigned int i=0; i<linkSetValuedContourData.size(); i++) {
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

  double BodyFlexible::computePotentialEnergy()
  {
    double V = 0.;
    for(unsigned int i=0;i<discretization.size();i++) {
     V += discretization[i]->computeElasticEnergy(qElement[i]) + discretization[i]->computeGravitationalEnergy(qElement[i]);
    }
    return V;
  }

  Mat BodyFlexible::computeJp(const ContourPointData &data) {
	static double eps = epsroot();
	Vec qAct     = q.copy();

	ContourPointData dataMod;
	dataMod.type   = data.type;
	dataMod.alpha  = data.alpha.copy();
	dataMod.alphap = data.alphap.copy();

	Mat JAct = computeJacobianMatrix(data);
	Mat Jp(JAct.rows(),JAct.cols(),INIT,0.0);

	for (int i=0; i<qSize ; i++) {
	  q(i) += eps;
	  Jp += (computeJacobianMatrix(data) - JAct)/eps * u(i);
	  q(i) = qAct(i);
	}

	for(int i=0;i<data.alphap.size();i++) {
	  dataMod.alpha(i) += eps;
	  Jp += (computeJacobianMatrix(dataMod) - JAct)/eps * data.alphap(i);
	  dataMod.alpha(i) = data.alpha(i);
	}

	return Jp;
  }

  Mat BodyFlexible::computeDrDs(const ContourPointData &data) {
	static double eps = epsroot();

	ContourPointData dataMod;
	dataMod.type  = data.type;
	dataMod.alpha = data.alpha.copy();

	Vec WrAct   = computeWrOC(data).copy();
	Mat DrDs(3,data.alphap.size(),NONINIT); // alphap used only if contour-parameter is DOF

	for(int i=0;i<DrDs.cols();i++) {
	  dataMod.alpha(i) += eps;
	  DrDs.col(i) = (computeWrOC(dataMod) - WrAct)/eps;
	  dataMod.alpha(i) = data.alpha(i);
	}

	return DrDs;
  }
  Mat BodyFlexible::computeDrDsp(const ContourPointData &data) {
	static double eps = epsroot();

	ContourPointData dataMod;
	dataMod.type  = data.type;
	dataMod.alpha = data.alpha.copy();
	dataMod.alphap = data.alphap.copy();

	Vec WvAct   = computeWvC(data).copy();
	Mat DrDsp(3,data.alphap.size(),NONINIT); // alphap used only if contour-parameter is DOF

	for(int i=0;i<DrDsp.cols();i++) {
	  dataMod.alpha(i) += eps;
	  DrDsp.col(i) = (computeWvC(dataMod) - WvAct)/eps;
	  dataMod.alpha(i) = data.alpha(i);
	}
	if(data.alphap.size()) {
	  Mat DrDsAct = computeDrDs(data);
	  for(int i=0;i<data.alphap.size();i++) {
		dataMod.alpha(i) += eps;
		DrDsp += (computeDrDs(dataMod) - DrDsAct)/eps * data.alphap(i);
		dataMod.alpha(i) = data.alpha(i);  
	  }
	}

	return DrDsp;
  }

  /*  Mat BodyFlexible::computeDrDsp(const ContourPointData &data) {
	  static double eps = epsroot();
	  Vec qAct     = q.copy();

	  ContourPointData dataMod;
	  dataMod.type   = data.type;
	  dataMod.alpha  = data.alpha.copy();
	  dataMod.alphap = data.alphap.copy();

	  Mat DrDsAct = computeDrDs(data);
	  Mat DrDsp(DrDsAct.rows(),DrDsAct.cols(),INIT,0.0);
  // cout << "Mat BodyFlexible::computeDrDsp(const ContourPointData &data)" << endl;
  // cout << "  DrDs  = " << DrDsAct << endl;
  // cout << "  DrDsp = " << DrDsp << endl;

  if(data.alphap.rows()) {
  for (int i=0; i<qSize ; i++) {
  q(i) += eps;
  updateKinematics(0.0);
  // cout << "    DrDs_mod_q("<<i<<") = " << computeDrDs(data) << endl;
  DrDsp += (computeDrDs(data) - DrDsAct)/eps * u(i);
  q(i) = qAct(i);
  }

  // cout << "DrDsp_ns = " << DrDsp << endl;

  for(int i=0;i<data.alphap.size();i++) {
  dataMod.alpha(i) += eps;
  // cout << "    DrDs_mod_s("<<i<<") = " << computeDrDs(data) << endl;
  // cout << "    DrDsAct  = " << DrDsAct << endl;
  // cout << "      => inc = " <<  (computeDrDs(dataMod) - DrDsAct)/eps * data.alphap(i) << endl;
  DrDsp += (computeDrDs(dataMod) - DrDsAct)/eps * data.alphap(i);
  dataMod.alpha(i) = data.alpha(i);
  }
  }

  return DrDsp;
  }
  */

  Mat BodyFlexible::computeK(const ContourPointData &cp) {
	static const double eps = epsroot();
	Mat K(3,cp.alphap.size(),NONINIT);
	SqrMat AWP = computeAWK(cp);
	ContourPointData cpMod;
	cpMod.type   = cp.type;
	cpMod.alpha  = cp.alpha .copy();
	cpMod.alphap = cp.alphap.copy();
	for(int i=0;i<K.cols();i++) {
	  cpMod.alpha(i) += eps;
	  Mat KiTilde = (computeAWK(cpMod) - AWP) / eps * trans(AWP);
	  K(0,i) = KiTilde(2,1);
	  K(1,i) = KiTilde(0,2);
	  K(2,i) = KiTilde(1,0);
	  cpMod.alpha(i)  = cp.alpha(i);
	}

	return K;
  }
  Mat BodyFlexible::computeKp(const ContourPointData &cp) {
	static const double eps = epsroot();
	//    Vec qAct     = q.copy();

	Mat Kp(3,cp.alphap.size(),INIT,0.0);
	Mat KAct = computeK(cp);

	SqrMat AWP  = computeAWK (cp);
	SqrMat AWPp = computeAWKp(cp);
	ContourPointData cpMod;
	cpMod.type   = cp.type;
	cpMod.alpha  = cp.alpha .copy();
	cpMod.alphap = cp.alphap.copy();
	for(int i=0;i<Kp.cols();i++) {
	  cpMod.alpha(i) += eps;
	  Mat KiTilde = (computeAWK(cpMod) - AWP) / eps * trans(AWPp) + (computeAWKp(cpMod) - AWPp) / eps * trans(AWP);
	  Kp(0,i) = KiTilde(2,1);
	  Kp(1,i) = KiTilde(0,2);
	  Kp(2,i) = KiTilde(1,0);
	  cpMod.alpha(i)  = cp.alpha(i);
	}

	return Kp;
  }

  SqrMat BodyFlexible::computeAWKp(const ContourPointData &data) {
	static double eps = epsroot();

	ContourPointData dataMod;
	dataMod.type   = data.type;
	dataMod.alpha  = data.alpha.copy();
	dataMod.alphap = data.alphap.copy();

	SqrMat AWKAct = computeAWK(data);

	// zeitliche Ableitung in Folge Referenzbewegung
	SqrMat AWKp = tilde(computeWomega(data))*AWKAct;

	// Aenderungen aufgrund von RelativBewegungen, numerisch
	for(int i=0;i<data.alphap.size();i++) {
	  dataMod.alpha(i) += eps;
	  AWKp += (computeAWK(dataMod) - AWKAct)/eps * data.alphap(i);
	  dataMod.alpha(i) = data.alpha(i);
	}

	return AWKp.copy();
  }


  //####################################################
  //####################################################

  BodyFlexible1s::BodyFlexible1s(const string &name):BodyFlexible(name), userContourNodes(0) {}

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

  SqrMat BodyFlexible1s::computeAWK  (const ContourPointData &data) {
	SqrMat AWK(3,NONINIT);
	AWK.col(0) = (computeWt(data)).col(0);   // ! Achtung: so nicht allgemeingueltig! TODO; danach evtl allgemeine Implementierung in BodyFlexible???
	AWK.col(1) =  computeWn(data);
	AWK.col(2) =  crossProduct(AWK.col(0),AWK.col(1));
	AWK.col(2) /= nrm2(AWK.col(2));
	return AWK.copy();
  }

  //####################################################
  //####################################################
  BodyFlexible2s::BodyFlexible2s(const string &name) :BodyFlexible(name) {}

  void BodyFlexible2s::addPort(const string &name, const Vec &alpha) {
	ContourPointData temp;
	temp.type  = CONTINUUM;
	temp.alpha = alpha;
	addPort(name,temp); 
  }
  void BodyFlexible2s::addContour(Contour *contour, const Vec &alpha) {
	ContourPointData temp;
	temp.type  = CONTINUUM;
	temp.alpha = alpha;
	addContour(contour,temp); 
  }

  SqrMat BodyFlexible2s::computeAWK  (const ContourPointData &data) {
	SqrMat AWK(3,NONINIT);
	Mat WT = computeWt(data);
	AWK.col(0) = WT.col(0);
	AWK.col(1) = WT.col(1);
	AWK.col(2) =  crossProduct(AWK.col(0),AWK.col(1));
	//    AWK.col(2) /= nrm2(AWK.col(2));
	return AWK.copy();
  }

}
