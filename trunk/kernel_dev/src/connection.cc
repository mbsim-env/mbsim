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
#include "connection.h"
#include "coordinate_system.h"
#include "data_interface_base.h"

#ifdef HAVE_AMVIS
#include "coilspring.h"
using namespace AMVis;
#endif

namespace MBSim {

  Connection::Connection(const string &name, bool setValued) : Link(name,setValued), coilspringAMVis(0), coilspringAMVisUserFunctionColor(0) {
  }

  Connection::~Connection() { 
#ifdef HAVE_AMVIS   
    delete coilspringAMVis;
    delete coilspringAMVisUserFunctionColor;
#endif
  }
  void Connection::calcSize() {
    Link::calcSize();
    gSize = forceDir.cols()+momentDir.cols();
    laSize = gSize;
    rFactorSize = setValued?laSize:0;
    xSize = momentDir.cols();
  }

  void Connection::init() {
    Link::init();
    IT = Index(0,forceDir.cols()-1);
    IR = Index(forceDir.cols(),forceDir.cols()+momentDir.cols()-1);
    if(forceDir.cols()) 
      Wf = forceDir;
    else {
      forceDir.resize(3,0);
      Wf.resize(3,0);
    }
    if(momentDir.cols())
      Wm = momentDir;
    else {
      momentDir.resize(3,0);
      Wm.resize(3,0);
    }
  }

  void Connection::connect(CoordinateSystem *port0, CoordinateSystem* port1) {
    Link::connect(port0,0);
    Link::connect(port1,1);
  }

  void Connection::updateStage1(double t) {
    Wf = port[0]->getOrientation()*forceDir;
    Wm = port[0]->getOrientation()*momentDir;
    WrP0P1 = port[1]->getPosition()-port[0]->getPosition();
    g(IT) = trans(Wf)*WrP0P1;
    g(IR) = x;
  }

  void Connection::updateStage2(double t) {
    WvP0P1 = port[1]->getVelocity()-port[0]->getVelocity();
    WomP0P1 = port[1]->getAngularVelocity()-port[0]->getAngularVelocity();
    gd(IT) = trans(Wf)*(WvP0P1 - crossProduct(port[0]->getAngularVelocity(), WrP0P1));
    gd(IR) = trans(Wm)*WomP0P1;
    updateKinetics(t);
  }

  void Connection::setForceDirection(const Mat &fd) {
    assert(fd.rows() == 3);

    forceDir = fd;

    for(int i=0; i<fd.cols(); i++)
      forceDir.col(i) = forceDir.col(i)/nrm2(fd.col(i));
  }

  void Connection::setMomentDirection(const Mat &md) {
    assert(md.rows() == 3);

    momentDir = md;

    for(int i=0; i<md.cols(); i++)
      momentDir.col(i) = momentDir.col(i)/nrm2(md.col(i));
  }

  void Connection::updatexd(double t) {
    //cout << gd<<endl;
    //cout << IR.start()<<endl;
    //cout << IR.end()<<endl;
    xd = gd(IR);
  }

  void Connection::updatedx(double t, double dt) {
    xd = gd(IR)*dt;
  }


  void Connection::initPlotFiles() {

    Link::initPlotFiles();

#ifdef HAVE_AMVIS
    if (coilspringAMVis) {
      coilspringAMVis->writeBodyFile();
    }
#endif
  }

  void Connection::plot(double t,double dt) {
    Link::plot(t,dt);

#ifdef HAVE_AMVIS
    if (coilspringAMVis) {
      Vec WrOToPoint;
      Vec WrOFromPoint;

      WrOFromPoint = port[0]->getPosition();
      WrOToPoint   = port[1]->getPosition();
      if (coilspringAMVisUserFunctionColor) {
	double color;
	color = ((*coilspringAMVisUserFunctionColor)(t))(0);
	if (color>1) color=1;
	if (color<0) color=0;
	coilspringAMVis->setColor(color);
      } 
      coilspringAMVis->setTime(t); 
      coilspringAMVis->setFromPoint(WrOFromPoint(0), WrOFromPoint(1), WrOFromPoint(2));
      coilspringAMVis->setToPoint(WrOToPoint(0), WrOToPoint(1), WrOToPoint(2));
      coilspringAMVis->appendDataset(0);
    }
  }
#endif
}
