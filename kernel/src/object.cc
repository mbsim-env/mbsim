/* Copyright (C) 2004-2006  Martin Förg
 
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
 *   mfoerg@users.berlios.de
 *
 */
#include <config.h>
#include "object.h"
#include <stdexcept>
#include "port.h"
#include "contour.h"
#include "link.h"
#include "multi_body_system.h"

namespace MBSim {

  Object::Object(const string &name_) : Element(name_),  qSize(0), uSize(0), xSize(0), qInd(0), uInd(0), xInd(0), q0(qSize), u0(uSize), x0(xSize), WrOHitSphere(3), RHitSphere(0) {

  }

  void Object::writeq(){
    string fname="PREINTEG/"+fullName+".q0.asc";  
    ofstream osq(fname.c_str(), ios::out);
    osq << q;
    osq.close();
  }
  void Object::readq0(){
    string fname="PREINTEG/"+fullName+".q0.asc";  
    ifstream isq(fname.c_str());
    if(isq) isq >> q0;
    else {cout << "Object " << name << ": No Preintegration Data q0 available. Run Preintegration first." << endl; throw 50;}
    isq.close();
  }
  void Object::writeu(){
    string fname="PREINTEG/"+fullName+".u0.asc";  
    ofstream osu(fname.c_str(), ios::out);
    osu << u;
    osu.close();
  }
  void Object::readu0(){
    string fname="PREINTEG/"+fullName+".u0.asc";  
    ifstream isu(fname.c_str());
    if(isu) isu >> u0;
    else {cout << "Object " << name << ": No Preintegration Data u0 available. Run Preintegration first." << endl; throw 50;}
    isu.close();
  }
  void Object::writex(){
    string fname="PREINTEG/"+fullName+".x0.asc";  
    ofstream osx(fname.c_str(), ios::out);
    osx << x;
    osx.close();
  }
  void Object::readx0(){
    string fname="PREINTEG/"+fullName+".x0.asc";  
    ifstream isx(fname.c_str());
    if(isx) isx >> x0;
    else {cout << "Object " << name << ": No Preintegration Data x0 available. Run Preintegration first." << endl; throw 50;}
    isx.close();
  }

  Object::~Object() {
  }

  void Object::updatezRef() {
    updateqRef();
    updateuRef();
    updatexRef();
  }

  void Object::updatezdRef() {
    updateqdRef();
    updateudRef();
    updatexdRef();
  }

  void Object::updateqRef() {
    q>>(mbs->getq()(qInd,qInd+qSize-1));
  }

  void Object::updateqdRef() {
    qd>>(mbs->getqd()(qInd,qInd+qSize-1));
  }

  void Object::updateuRef() {
    u>>(mbs->getu()(uInd,uInd+uSize-1));
  }

  void Object::updateudRef() {
    ud>>(mbs->getud()(uInd,uInd+uSize-1));
  }

  void Object::updatexRef() {
    x>>(mbs->getx()(xInd,xInd+xSize-1));
  }

  void Object::updatexdRef() {
    xd>>(mbs->getxd()(xInd,xInd+xSize-1));
  }

  void Object::updatehRef() {
    h>>(mbs->geth()(uInd,uInd+uSize-1));
  }

  void Object::updaterRef() {
    r>>(mbs->getr()(uInd,uInd+uSize-1));
  }

  void Object::updatefRef() {
    f>>(mbs->getf()(xInd,xInd+xSize-1));
  }

  void Object::updateMRef() {
    Index I = getuIndex();
    M>>mbs->getM()(I);
  }

  void Object::updateTRef() {
    Index Iu = getuIndex();
    Index Iq = Index(qInd,qInd+qSize-1);
    T>>mbs->getT()(Iq,Iu);
  }

  void Object::updateLLMRef() {
    Index I = getuIndex();
    LLM>>mbs->getLLM()(I);
  }

  void Object::initz() {
    q = q0;
    u = u0;
    x = x0;
  }

  //void Object::plot(double t) {
  //  Element::plot(t);
  //
  //  if(plotLevel>1) {
  //    for(int i=0; i<qSize; ++i)
  //      plotfile<<" "<<q(i);
  //    for(int i=0; i<uSize; ++i)
  //      plotfile<<" "<<v(i);
  //    for(int i=0; i<uSize; ++i)
  //      plotfile<<" "<<u(i);
  //    if(plotLevel>2) {
  //      for(int i=0; i<qSize; ++i)
  //	plotfile<<" "<<qd(i);
  //      for(int i=0; i<uSize; ++i)
  //	plotfile<<" "<<ud(i);
  //      for(int i=0; i<uSize; ++i)
  //	plotfile<<" "<<ud(i);
  //      for(int i=0; i<uSize; ++i)
  //	plotfile<<" "<<h(i);
  //      for(int i=0; i<uSize; ++i)
  //	plotfile<<" "<<r(i);
  //    }
  //  }
  //}

  void Object::plot(double t, double dt) {
    Element::plot(t);

    if(plotLevel>1) {
      for(int i=0; i<qSize; ++i)
	plotfile<<" "<<q(i);
      for(int i=0; i<uSize; ++i)
	plotfile<<" "<<u(i);
      for(int i=0; i<xSize; ++i)
	plotfile<<" "<<x(i);
      if(plotLevel>2) {
	for(int i=0; i<qSize; ++i)
	  plotfile<<" "<<qd(i)/dt;
	for(int i=0; i<uSize; ++i)
	  plotfile<<" "<<ud(i)/dt;
	for(int i=0; i<xSize; ++i)
	  plotfile<<" "<<xd(i)/dt;
	for(int i=0; i<uSize; ++i)
	  plotfile<<" "<<h(i);
	for(int i=0; i<uSize; ++i)
	  plotfile<<" "<<r(i)/dt;
	if(plotLevel>3) {							//HR 04.01.07
	  for(int j=0; j<port.size(); j++) {
	    for(int i=0; i<3; i++)
	      plotfile<< " " << port[j]->getWrOP()(i);
	  }    
	}
      }
    }
  }

  void Object::initPlotFiles() {
    Element::initPlotFiles();

    if(plotLevel>1) {
      for(int i=0; i<qSize; ++i)
	plotfile <<"# "<< plotNr++ << ": q(" << i << ")" << endl;

      for(int i=0; i<uSize; ++i)
	plotfile <<"# "<< plotNr++ <<": u("<<i<<")" << endl;

      for(int i=0; i<xSize; ++i)
	plotfile <<"# "<< plotNr++ << ": x(" << i << ")" << endl;

      if(plotLevel>2) {
	for(int i=0; i<qSize; ++i)
	  plotfile <<"# "<< plotNr++ << ": qd(" << i << ")" << endl;

	for(int i=0; i<uSize; ++i)
	  plotfile <<"# "<< plotNr++ <<": ud("<<i<<")" << endl;

	for(int i=0; i<xSize; ++i)
	  plotfile <<"# "<< plotNr++ <<": xd("<<i<<")" << endl;

	for(int i=0; i<uSize; ++i)
	  plotfile <<"# "<< plotNr++ <<": h("<<i<<")" << endl;

	for(int i=0; i<uSize; ++i)
	  plotfile <<"# "<< plotNr++ <<": r("<<i<<")" << endl;

	if (plotLevel>3) {									//HR 04.01.2007
	  for(int j=0; j<port.size(); j++) {
	    for(int i=0; i<3; i++)
	      plotfile<< "# " << plotNr++ <<": WrOP ("<<port[j]->getName()<<")" << endl;
	  }    
	}   
      }
    }
  }

  void Object::addLink(LinkPort *link, Port *port, int objectID) {
    LinkPortData lpd = {link,port->getID(),objectID};
    if(link->isSetValued()) {
      linkSetValuedPortData.push_back(lpd);
    } else {
      linkSingleValuedPortData.push_back(lpd);
    }
  }

  void Object::addLink(LinkContour *link, Contour *contour, int objectID) {
    LinkContourData lpd = {link,contour->getID(),objectID};
    if(link->isSetValued()) {
      linkSetValuedContourData.push_back(lpd);
    } else {
      linkSingleValuedContourData.push_back(lpd);
    }
  }

  void Object::addContour(Contour* contour_) {
    contour_->setID(contour.size());
    contour_->setFullName(getFullName()+"."+contour_->getFullName());
    contour.push_back(contour_);
    contour_->setObject(this);
  }

  void Object::addPort(Port* port_) {
    port_->setID(port.size());
    port_->setFullName(getFullName()+"."+port_->getFullName());
    port.push_back(port_);
    port_->setObject(this);
  }

  Port* Object::getPort(const string &name) {
    int i;
    for(i=0; i<port.size(); i++) {
      if(port[i]->getName() == name)
	return port[i];
    }             
    if(!(i<port.size())) cout << "Error: The object " << this->name <<" comprises no port " << name << "!" << endl; 
    assert(i<port.size());
  }

  Contour* Object::getContour(const string &name) {
    int i;
    for(i=0; i<contour.size(); i++) {
      if(contour[i]->getName() == name)
	return contour[i];
    }
    if(!(i<contour.size())) cout << "Error: The object " << this->name <<" comprises no contour " << name << "!" << endl; 
    assert(i<contour.size());
  }

  void Object::init() {
    vector<Mat>::iterator itW=W.begin(); 
    vector<Vec>::iterator itw=w.begin(); 
    for(int i=0; i<linkSetValuedPortData.size(); i++) {
      linkSetValued.push_back(linkSetValuedPortData[i].link);
      W.push_back(Mat(getWSize(),linkSetValuedPortData[i].link->getlaSize()));
      w.push_back(Vec(linkSetValuedPortData[i].link->getlaSize())); 
      itW++; itw++;
    }
    for(int i=0; i<linkSetValuedContourData.size(); i++) {
      linkSetValued.push_back(linkSetValuedContourData[i].link);
      W.push_back(Mat(getWSize(),linkSetValuedContourData[i].link->getlaSize()));
      w.push_back(Vec(linkSetValuedContourData[i].link->getlaSize())); 
      itW++; itw++;
    }
    for(vector<Contour*>::iterator i=contour.begin(); i!=contour.end(); i++) 
      (*i)->init();
  }

  void Object::updater(double t) {
    r.init(0);
    for(vector<Link*>::iterator i=linkSetValued.begin(); i!=linkSetValued.end(); i++) {
      if((*i)->isActive()) {
	Index I = (*i)->getlaIndex();
	r += mbs->getW()(getuIndex(),I)*(*i)->getla();
      }
    }
  }

  void Object::updateG(double t) {
    updateW(t);
    Vec iMh = slvLLFac(LLM,h);
    vector<Mat>::iterator itW=W.begin(), jtW; 
    vector<Vec>::iterator itw=w.begin(); 
    vector<Link*>::iterator jt1, it1=linkSetValued.begin(); 
    for(int i=0; i<linkSetValued.size(); i++) {
      if((*it1)->isActive()) {
	Index I = (*it1)->getlaIndex();
	Mat Wi = (*itW);
	mbs->getW()(getuIndex(),I) = Wi; 
	mbs->getw()(I) += (*itw); 
	mbs->getb()(I) += trans(Wi)*iMh; 
	Mat iMWi = slvLLFac(LLM,Wi);
	mbs->getG()(I) += SymMat(trans(Wi)*iMWi); 
	jt1 = linkSetValued.begin(); 
	jtW = W.begin(); 
	for(int j=0; j<i; j++) {
	  if((*jt1)->isActive()) {
	    Index J =  (*jt1)->getlaIndex();
	    mbs->getG()(J,I) += trans((*jtW))*iMWi; 
	  }
	  jt1++; jtW++;
	}
      }
      it1++; itW++; itw++;
    }
  }

}

