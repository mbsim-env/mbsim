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
#include <string.h>
#include "link.h"
#include "coordinate_system.h"
#include "contour.h"
#include "object.h"
#include "multi_body_system.h"
#include "subsystem.h"

#ifdef HAVE_AMVIS
#include "arrow.h"
using namespace AMVis;
#endif

namespace MBSim {

  Link::Link(const string &name, bool setValued_) : Element(name), parent(0), xSize(0), xInd(0), svSize(0), svInd(0), setValued(setValued_), gSize(0), laSize(0), rFactorSize(0), active(true), scaleTolQ(1e-9), scaleTolp(1e-5), gdTol(1e-8), laTol(1e-2), rMax(1.0), HSLink(0), checkHSLink(false) {
  }

  Link::~Link() { 
#ifdef HAVE_AMVIS   
    for (unsigned int i=0; i<arrowAMVis.size(); i++) {
      delete arrowAMVis[i];
      delete arrowAMVisUserFunctionColor[i];
    }
#endif

  }

  bool Link::isSetValued() const {
    return setValued;
  }

  void Link::init() {
    rFactor.resize(rFactorSize);
    rFactorUnsure.resize(rFactorSize);
    la.resize(laSize);
    la0.resize(laSize);
    g.resize(gSize);
    gd.resize(laSize);
    gdn.resize(laSize);
    s.resize(laSize);
    res.resize(laSize);

    for(unsigned i=0; i<port.size(); i++) {
      W.push_back(Mat(port[i]->getJacobianOfTranslation().cols(),laSize));
      h.push_back(Vec(port[i]->getJacobianOfTranslation().cols()));
      r.push_back(Vec(port[i]->getJacobianOfTranslation().cols()));
    }
    for(unsigned i=0; i<contour.size(); i++) {
      W.push_back(Mat(contour[i]->getWJP().cols(),laSize));
      h.push_back(Vec(contour[i]->getWJP().cols()));
      r.push_back(Vec(contour[i]->getWJP().cols()));
    }
  }

  string Link::getFullName() const {
    return parent->getFullName() + "." + name;
  }

  void Link::updatexRef() {
    x >> parent->getx()(xInd,xInd+xSize-1);
  }

  void Link::updatexdRef() {
    xd >> parent->getxd()(xInd,xInd+xSize-1);
  }

  void Link::updatesvRef() {
    sv >> parent->getsv()(svInd,svInd+svSize-1);
  }

  void Link::updatejsvRef() {
    jsv >> parent->getjsv()(svInd,svInd+svSize-1);
  }

  void Link::plot(double t, double dt) {

    Element::plot(t,dt);

    if(plotfile>0) {
      if(plotLevel > 1) {

	for(int i=0; i<xSize; ++i)
	  plotfile<<" "<<x(i);

	for(int i=0; i<xSize; ++i)
	  plotfile<<" "<<xd(i)/dt;
      }
      for(int i=0; i<gSize; ++i)
	plotfile<<" "<<g(i);

    }
    if(active || plotLevel > 2) {

      if(plotLevel>0) {
	if(plotLevel>1) {
	  for(int i=0; i<laSize; ++i)
	    plotfile<<" "<<gd(i);
	}
	if(setValued)
	  for(int i=0; i<laSize; ++i)
	    plotfile<<" "<<la(i)/dt;
	else
	  for(int i=0; i<laSize; ++i)
	    plotfile<<" "<<la(i);
      }
    } else {

      if(plotLevel>0) {
	if(plotLevel>1) {
	  for(int i=0; i<laSize; ++i)
	    plotfile<<" "<<0;
	}
	for(int i=0; i<laSize; ++i)
	  plotfile<<" "<<0;

      }
    }
    if(plotLevel>2) {
      plotfile<<" "<<computePotentialEnergy(); 
    }

#ifdef HAVE_AMVIS
    Vec WrOToPoint;
    Vec LoadArrow;
    for (unsigned int i=0; i<arrowAMVis.size(); i++) {
      WrOToPoint = port[arrowAMVisID[i]]->getPosition();
      if(setValued){ 
	if (active) 
	  LoadArrow = loadDir[arrowAMVisID[i]]*la/dt;
	else {
	  LoadArrow = Vec(6,INIT,0.0);
	  WrOToPoint= Vec(3,INIT,0.0);
	}
      }
      else
	LoadArrow = L[arrowAMVisID[i]];
      // Scaling: 1KN or 1KNm scaled to arrowlenght one
      LoadArrow= LoadArrow/1000*arrowAMVisScale[i];

      arrowAMVis[i]->setTime(t);
      arrowAMVis[i]->setToPoint(WrOToPoint(0),WrOToPoint(1),WrOToPoint(2));
      double color;
      if (arrowAMVisMoment[i]) {
	arrowAMVis[i]->setDirection(LoadArrow(3),LoadArrow(4),LoadArrow(5));
	color=0.5;
      }
      else {
	arrowAMVis[i]->setDirection(LoadArrow(0),LoadArrow(1),LoadArrow(2));
	color =1.0;
      }
      if (arrowAMVisUserFunctionColor[i]) {
	color = (*arrowAMVisUserFunctionColor[i])(t)(0);
	if (color>1) color=1;
	if (color<0) color=0;
      }  
      arrowAMVis[i]->setColor(color);
      arrowAMVis[i]->appendDataset(0);
    }

    for (unsigned int i=0; i<arrowAMVis.size(); i++) {
      WrOToPoint = cpData[arrowAMVisID[i]].WrOC;
      if(setValued){ 
	if (active) 
	  LoadArrow = loadDir[arrowAMVisID[i]]*la/dt;
	else {
	  LoadArrow = Vec(6,INIT,0.0);
	  WrOToPoint= Vec(3,INIT,0.0);
	}
      }
      else
	LoadArrow = L[arrowAMVisID[i]];
      // Scaling: 1KN or 1KNm scaled to arrowlenght one
      LoadArrow(0,2)= LoadArrow(0,2)/1000*arrowAMVisScale[i];

      arrowAMVis[i]->setTime(t);
      arrowAMVis[i]->setToPoint(WrOToPoint(0),WrOToPoint(1),WrOToPoint(2));
      double color;
      if (arrowAMVisMoment[i]) {
	arrowAMVis[i]->setDirection(LoadArrow(3),LoadArrow(4),LoadArrow(5));
	color=0.5;
      }
      else {
	arrowAMVis[i]->setDirection(LoadArrow(0),LoadArrow(1),LoadArrow(2));
	color =1.0;
      }
      if (arrowAMVisUserFunctionColor[i]) {
	color = (*arrowAMVisUserFunctionColor[i])(t)(0);
	if (color>1) color=1;
	if (color<0) color=0;
      }  
      arrowAMVis[i]->setColor(color);
      arrowAMVis[i]->appendDataset(0);
    }
#endif

  }

  void Link::initPlotFiles() {

    Element::initPlotFiles();

#ifdef HAVE_AMVIS
    for (unsigned int i=0; i<arrowAMVis.size(); i++)
      arrowAMVis[i]->writeBodyFile();
#endif

    if(plotLevel>0) {
      if(plotLevel>1) {
	for(int i=0; i<xSize; ++i)
	  plotfile <<"# "<< plotNr++ << ": x(" << i << ")" << endl;
	for(int i=0; i<xSize; ++i)
	  plotfile <<"# "<< plotNr++ <<": xd("<<i<<")" << endl;

      }
      for(int i=0; i<gSize; ++i)
	plotfile <<"# "<< plotNr++ << ": g(" << i << ")" << endl;

      if(plotLevel>1) {
	for(int i=0; i<laSize; ++i)
	  plotfile <<"# "<< plotNr++ << ": gd(" << i << ")" << endl;
      }

      for(int i=0; i<laSize; ++i)
	plotfile <<"# "<< plotNr++ << ": la(" << i << ")" << endl;

      if(plotLevel>2) {
	plotfile <<"# "<< plotNr++ << ": V" << endl;
      }
    }
  }

  void Link::updatelaRef() {
    la >> parent->getla()(laInd,laInd+laSize-1);
  }

  void Link::updategRef() {
    g >> parent->getg()(gInd,gInd+gSize-1);
  }

  void Link::updateRef() {
    updateWRef();
    updatelaRef();
    updategdRef();
    updatebRef();
    updatesRef();
    // TODO Nur bei Newton
    updateresRef();
    updaterFactorRef();
  }

  void Link::updatebRef() {
    b >> parent->getb()(laInd,laInd+laSize-1);
  }

  void Link::updategdRef() {
    gd >> parent->getgd()(laInd,laInd+laSize-1);
  }

  void Link::updatesRef() {
    s >> parent->gets()(laInd,laInd+laSize-1);
  }

  void Link::updateresRef() {
    res >> parent->getres()(laInd,laInd+laSize-1);
  }

  void Link::updaterFactorRef() {
    rFactor >> parent->getrFactor()(rFactorInd,rFactorInd+rFactorSize-1);
  }

  void Link::savela() {
    la0 = la;
  }

  void Link::initla() {
    // TODO Prufen ob initilisierung auf 0 besser, wenn vorher inaktiv
    la = la0;
  }

  void Link::decreaserFactors() {
    for(int i=0; i<rFactor.size(); i++)
      if(rFactorUnsure(i))
	rFactor(i) *= 0.9;
  }

  void Link::updater(double t) {
    r[0] += W[0]*la;
    r[1] += W[1]*la;
  }

  int Link::getlaIndMBS() const {
    return parent->getlaIndMBS() + laInd;
  }

  void Link::load(ifstream& inputfile) {
    Element::load(inputfile);
    string dummy;

    getline(inputfile,dummy); // # Connected cosy
    int n = getNumberOfElements(inputfile);
    for(int i=0; i<n; i++) {
      getline(inputfile,dummy); // Connected cosy
      connect(getMultiBodySystem()->findCoordinateSystem(dummy),i);
    }
    getline(inputfile,dummy); // newline

    getline(inputfile,dummy); // # Connected contours
    n = getNumberOfElements(inputfile);
    for(int i=0; i<n; i++) {
      getline(inputfile,dummy); // Connected contour
      connect(getMultiBodySystem()->findContour(dummy),i);
    }
    getline(inputfile,dummy); // newline
  }

#ifdef HAVE_AMVIS
  void Link::addAMVisForceArrow(AMVis::Arrow *arrow, double scale, int ID, UserFunction *funcColor) {
    assert(ID >= 0);
    assert(ID < 2);
    arrowAMVis.push_back(arrow);
    arrowAMVisScale.push_back(scale);
    arrowAMVisID.push_back(ID);
    arrowAMVisUserFunctionColor.push_back(funcColor);
    arrowAMVisMoment.push_back(false);
  }

  void Link::addAMVisMomentArrow(AMVis::Arrow *arrow,double scale ,int ID, UserFunction *funcColor) {
    assert(ID >= 0);
    assert(ID < 2);
    arrowAMVis.push_back(arrow);
    arrowAMVisScale.push_back(scale);
    arrowAMVisID.push_back(ID);
    arrowAMVisUserFunctionColor.push_back(funcColor);
    arrowAMVisMoment.push_back(true);
  }
#endif

  void Link::connect(CoordinateSystem *port_, int id) {
    port.push_back(port_);
  }

  void Link::updateWRef() {
    for(unsigned i=0; i<port.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(port[i]->getParent()->gethInd(),port[i]->getParent()->gethInd()+port[i]->getJacobianOfTranslation().cols()-1);
	W[i]>>parent->getW()(I,J);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(contour[i]->getParent()->gethInd(),contour[i]->getParent()->gethInd()+contour[i]->getWJP().cols()-1);
      W[i]>>parent->getW()(I,J);
    }
  } 

  void Link::updatehRef() {
    for(unsigned i=0; i<port.size(); i++) {
      Index I = Index(port[i]->getParent()->gethInd(),port[i]->getParent()->gethInd()+port[i]->getJacobianOfTranslation().cols()-1);
      h[i]>>parent->geth()(I);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      Index I = Index(contour[i]->getParent()->gethInd(),contour[i]->getParent()->gethInd()+contour[i]->getWJP().cols()-1);
      h[i]>>parent->geth()(I);
    }
  } 

  void Link::updaterRef() {
    for(unsigned i=0; i<port.size(); i++) {
      Index I = Index(port[i]->getParent()->gethInd(),port[i]->getParent()->gethInd()+port[i]->getJacobianOfTranslation().cols()-1);
      r[i]>>parent->getr()(I);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      Index I = Index(contour[i]->getParent()->gethInd(),contour[i]->getParent()->gethInd()+contour[i]->getWJP().cols()-1);
      r[i]>>parent->getr()(I);
    }
  } 


  void Link::connect(Contour *contour_, int id) {
    contour.push_back(contour_);
  }

  void Link::plotParameters() {
    Element::plotParameters();
    parafile << "# Connected coordinate sytems:" << endl;
    for(unsigned int i=0; i<port.size(); i++) {
      parafile << port[i]->getFullName() << endl;
    }
    parafile << endl;

    parafile << "# Connected contours:" << endl;
    for(unsigned int i=0; i<contour.size(); i++) {
      parafile << contour[i]->getFullName() << endl;
    }
    parafile << endl;
  }

  MultiBodySystem* Link::getMultiBodySystem() {
    return parent->getMultiBodySystem();
  }

}
