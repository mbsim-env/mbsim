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
#include<config.h>
#include<string.h>
#include "link.h"
#include "port.h"
#include "contour.h"
#include "object.h"
#include "multi_body_system.h"

#ifdef HAVE_AMVIS
#include "arrow.h"
using namespace AMVis;
#endif

namespace MBSim {

  Link::Link(const string &name, bool setValued_) : Element(name), xSize(0), xInd(0), svSize(0), svInd(0), setValued(setValued_), bilateral(false), gSize(0), laSize(0), rFactorSize(0), active(true), activeForAssembling(0), LinkStatusSize(0), scaleTolQ(1e-9), scaleTolp(1e-5), gdTol(1e-8), laTol(1e-2), rMax(1.0), HSLink(0), checkHSLink(false) {}

  Link::~Link() { 
#ifdef HAVE_AMVIS   
    for(unsigned int i=0; i<arrowAMVis.size(); i++) {
      if(AMVisInstance[i]) delete arrowAMVis[i]; arrowAMVis[i]=NULL;
      delete arrowAMVisUserFunctionColor[i]; arrowAMVisUserFunctionColor[i]=NULL;
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
  }

  void Link::updatexRef() {
    x >> mbs->getx()(xInd,xInd+xSize-1);
  }

  void Link::updatexdRef() {
    xd >> mbs->getxd()(xInd,xInd+xSize-1);
  }

  void Link::updatesvRef() {
    sv >> mbs->getsv()(svInd,svInd+svSize-1);
  }

  void Link::updatejsvRef() {
    jsv >> mbs->getjsv()(svInd,svInd+svSize-1);
  }
  void Link::updateLinkStatusRef() {
    LinkStatus >>mbs->getLinkStatus()(LinkStatusIndex,LinkStatusIndex+LinkStatusSize-1);
  }

  void Link::plot(double t, double dt) {
    Element::plot(t,dt);

    if(plotfile>0) {
      if(plotLevel > 1) {
	for(int i=0; i<xSize; ++i) plotfile << " " << x(i);	
	for(int i=0; i<xSize; ++i) plotfile << " " << xd(i)/dt;
      }
      for(int i=0; i<gSize; ++i) plotfile << " " << g(i);
    }
    if(active || plotLevel > 2) {
      if(plotLevel>0) {
	if(plotLevel>1) for(int i=0; i<laSize; ++i) plotfile << " " << gd(i);
	if(setValued) for(int i=0; i<laSize; ++i) plotfile << " " << la(i)/dt;
	else for(int i=0; i<laSize; ++i) plotfile << " " << la(i);
      }
    }
    else {
      if(plotLevel>0) {
	if(plotLevel>1) for(int i=0; i<laSize; ++i) plotfile << " " << 0;
	for(int i=0; i<laSize; ++i) plotfile << " " << 0;
      }
    }
    if(plotLevel>2) plotfile << " " << computePotentialEnergy(); 

#ifdef HAVE_AMVIS
    Vec LoadArrow;
    for (unsigned int i=0; i<arrowAMVis.size(); i++) {
      if(active) {
	if (setValued)
	  LoadArrow = loadDir[arrowAMVisID[i]]*la/dt;
	else
	  LoadArrow = load[arrowAMVisID[i]];
      }
      else{
	LoadArrow = Vec(6,INIT,0.0);
	arrowAMVisToPoint[i]= Vec(3,INIT,0.0);
      }
      // Scaling: 1KN or 1KNm scaled to arrowlenght one
      LoadArrow= LoadArrow/1000*arrowAMVisScale[i];

      arrowAMVis[i]->setTime(t);
      arrowAMVis[i]->setToPoint(arrowAMVisToPoint[i](0),arrowAMVisToPoint[i](1),arrowAMVisToPoint[i](2));
      double color;
      if (arrowAMVisMoment[i]) {
	arrowAMVis[i]->setDirection(LoadArrow(3),LoadArrow(4),LoadArrow(5));
	color =0.5;
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
      arrowAMVis[i]->appendDataset(AMVisInstance[i]);
    }
#endif
  }



  void Link::initPlotFiles() {

    Element::initPlotFiles();

#ifdef HAVE_AMVIS
    for(unsigned int i=0; i<arrowAMVis.size(); i++)
      if (AMVisInstance[i]==0) arrowAMVis[i]->writeBodyFile();
#endif

    if(plotLevel>0) {
      if(plotLevel>1) {
	for(int i=0; i<xSize; ++i) plotfile <<"# "<< plotNr++ << ": x(" << i << ")" << endl;
	for(int i=0; i<xSize; ++i) plotfile <<"# "<< plotNr++ <<": xd("<< i <<")" << endl;
      }
      for(int i=0; i<gSize; ++i) plotfile <<"# "<< plotNr++ << ": g(" << i << ")" << endl;
      if(plotLevel>1) for(int i=0; i<laSize; ++i) plotfile <<"# "<< plotNr++ << ": gd(" << i << ")" << endl;
      for(int i=0; i<laSize; ++i) plotfile <<"# "<< plotNr++ << ": la(" << i << ")" << endl;
      if(plotLevel>2) plotfile <<"# "<< plotNr++ << ": V" << endl;
    }
  }

  void Link::updatelaRef() {
    la >> mbs->getla()(laInd,laInd+laSize-1);
  }

  void Link::updategRef() {
    g >> mbs->getg()(gInd,gInd+gSize-1);
  }

  void Link::updateRef() {
    la >> mbs->getla()(laInd,laInd+laSize-1);
    gd >> mbs->getgd()(laInd,laInd+laSize-1);
    s >> mbs->gets()(laInd,laInd+laSize-1);
    // TODO only with Newton
    res >> mbs->getres()(laInd,laInd+laSize-1);
    rFactor >> mbs->getrFactor()(rFactorInd,rFactorInd+rFactorSize-1);
  }

  void Link::updategdRef() {
    gd >> mbs->getgd()(laInd,laInd+laSize-1);
  }

  void Link::updatesRef() {
    s >> mbs->gets()(laInd,laInd+laSize-1);
  }

  void Link::updateresRef() {
    res >> mbs->getres()(laInd,laInd+laSize-1);
  }

  void Link::updaterFactorRef() {
    rFactor >> mbs->getrFactor()(rFactorInd,rFactorInd+rFactorSize-1);
  }

  void Link::savela() {la0 = la;}

  void Link::initla() {la = la0;}

  void Link::decreaserFactors() {
    for(int i=0; i<rFactor.size(); i++) if(rFactorUnsure(i)) rFactor(i) *= 0.9;
  }

#ifdef HAVE_AMVIS
  void Link::addAMVisForceArrow(AMVis::Arrow *arrow, double scale, int ID, UserFunction *funcColor, int instance) {
    assert(ID >= 0);
    assert(ID < 2);
    arrowAMVis.push_back(arrow);
    arrowAMVisScale.push_back(scale);
    arrowAMVisID.push_back(ID);
    arrowAMVisUserFunctionColor.push_back(funcColor);
    arrowAMVisMoment.push_back(false);
    arrowAMVisToPoint.push_back(Vec(3));
    AMVisInstance.push_back(instance);
  }

  void Link::addAMVisMomentArrow(AMVis::Arrow *arrow,double scale ,int ID, UserFunction *funcColor, int instance) {
    assert(ID >= 0);
    assert(ID < 2);
    arrowAMVis.push_back(arrow);
    arrowAMVisScale.push_back(scale);
    arrowAMVisID.push_back(ID);
    arrowAMVisUserFunctionColor.push_back(funcColor);
    arrowAMVisMoment.push_back(true);
    arrowAMVisToPoint.push_back(Vec(3));
    AMVisInstance.push_back(instance);
  }
#endif

  LinkPort::LinkPort(const string &name, bool setValued) : Link(name,setValued) {}

  void LinkPort::connect(Port *port_, int id) {
    port.push_back(port_);
    port_->getObject()->addLink(this,port_,id);
  }

  void LinkPort::plot(double t,double dt) {

#ifdef HAVE_AMVIS
    for (unsigned int i=0; i<arrowAMVis.size(); i++) 
      arrowAMVisToPoint[i] = port[arrowAMVisID[i]]->getWrOP();
#endif
    Link::plot(t,dt);

  }

  LinkContour::LinkContour(const string &name, bool setValued) : Link(name,setValued) {}

  void LinkContour::init() {
    Link::init();
  }

  void LinkContour::connect(Contour *contour_, int id) {
    contour.push_back(contour_);
    contour_->getObject()->addLink(this,contour_,id);
  }

  void LinkContour::plot(double t, double dt) {

#ifdef HAVE_AMVIS
    for (unsigned int i=0; i<arrowAMVis.size(); i++) 
      arrowAMVisToPoint[i] = cpData[arrowAMVisID[i]].WrOC;
#endif
    Link::plot(t,dt);
  }
}
