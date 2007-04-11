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
#include "port.h"
#include "contour.h"
#include "object.h"
#include "multi_body_system.h"

namespace MBSim {

  Link::Link(const string &name, bool setValued_) : Element(name), xSize(0), xInd(0), svSize(0), svInd(0), setValued(setValued_), gSize(0), laSize(0), rFactorSize(0), active(true), gdTol(1e-8), laTol(1e-2), rMax(1.0), scaleTolQ(1e-9), scaleTolp(1e-5), HSLink(0), checkHSLink(false) {
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
  }

  void Link::initPlotFiles() {

    Element::initPlotFiles();

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
    la >> mbs->getla()(laInd,laInd+laSize-1);
  }

  void Link::updategRef() {
    g >> mbs->getg()(gInd,gInd+gSize-1);
  }

  void Link::updateRef() {
    la >> mbs->getla()(laInd,laInd+laSize-1);
    gd >> mbs->getgd()(laInd,laInd+laSize-1);
    s >> mbs->gets()(laInd,laInd+laSize-1);
    // TODO Nur bei Newton
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


  LinkPort::LinkPort(const string &name, bool setValued) : Link(name,setValued) {
  }

  void LinkPort::connect(Port *port_, int id) {
    port.push_back(port_);
    port_->getObject()->addLink(this,port_,id);
  }

  LinkContour::LinkContour(const string &name, bool setValued) : Link(name,setValued) {
  }

  void LinkContour::init() {
    Link::init();
  }

  void LinkContour::connect(Contour *contour_, int id) {
    contour.push_back(contour_);
    contour_->getObject()->addLink(this,contour_,id);
  }

}
