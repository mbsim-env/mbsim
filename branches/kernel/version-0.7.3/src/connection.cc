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
#include "port.h"
#include "multi_body_system.h"
#include "tree_rigid.h"
#include "body.h"
#include "data_interface_base.h"

#ifdef HAVE_AMVIS
#include "coilspring.h"
using namespace AMVis;
#endif

namespace MBSim {

  Connection::Connection(const string &name, bool setValued) : LinkPort(name,setValued), KOSYID(0)
#ifdef HAVE_AMVIS
    , coilspringAMVis(0), coilspringAMVisUserFunctionColor(0)
#endif
  {
    bilateral=true;
  }

  Connection::~Connection() { 
#ifdef HAVE_AMVIS   
    delete coilspringAMVis; coilspringAMVis=NULL;
    delete coilspringAMVisUserFunctionColor; coilspringAMVisUserFunctionColor=NULL;
#endif
  }
  void Connection::calcSize() {
    LinkPort::calcSize();
    gSize = forceDir.cols()+momentDir.cols();
    laSize = gSize;
    rFactorSize = setValued?laSize:0;
    xSize = momentDir.cols();
  }

  void Connection::init() {
    LinkPort::init();
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
    w.clear();
    for(int i=0; i<2 ; i++) 
      w.push_back(Vec(laSize));
  }

  void Connection::setKOSY(int id) {
    KOSYID = id;
    assert(KOSYID >= 0);
    assert(KOSYID <= 2);
  }

  void Connection::connect(Port *port0, Port* port1) {
    LinkPort::connect(port0,0);
    LinkPort::connect(port1,1);
  }

  void Connection::connect(const string &port0Name, const string &port1Name) {
    if(!mbs) {
      cout << "Method only valid, if connection already added to the parent mbs!" << endl;
      throw(123);
    }
    Port * port0=NULL;
    Port * port1=NULL;
    for (int i=0; i<2; i++) {
      Port * port=NULL;
      vector<string> name;

      string portName;
      if(i==0) portName=port0Name;
      else if(i==1) portName=port1Name;
      while(portName.find(".")!= string::npos) {
	name.push_back(portName.substr(0, portName.find_first_of(".")));
	portName=portName.substr(portName.find_first_of(".")+1, portName.length()-portName.find_first_of(".")-1);
      }
      name.push_back(portName);

      portName=name[1]; // Port of a MultiBodySystem
      if(name.size()==2)
	port=mbs->getPort(portName, false);
      if(!port) { // Port of a BodyRigidAbs
	string objectName=name[0];
	for (unsigned int i=1; i<name.size()-1; i++)
	  objectName+="."+name[i];
	portName=name.back();
	if(dynamic_cast<Body*>(mbs->getObject(objectName, false)))
	  port=(mbs->getObject(objectName, false))->getPort(portName, false);
      }
      if(!port) { // Port of a BodyRigidRel
	string treeName=name[0];
	string objectName= name[name.size()-2];
	for (unsigned int i=1; i<name.size()-2; i++)
	  treeName+="."+name[i];
	if(dynamic_cast<TreeRigid*>(mbs->getObject(treeName, false)))
	  port=static_cast<TreeRigid*>(mbs->getObject(treeName, false))->getObject(objectName)->getPort(portName, false);
      }
      if (!port) {
	cout << "No port \"" << portName << "\" was found. Check input." << endl;
	throw(123);
      }
      if(i==0) port0=port;
      else if(i==1) port1=port;
    }
    connect(port0, port1);
  }

  void Connection::updateStage1(double t) {
    if(KOSYID) {
      Wf = port[0]->getAWP()*forceDir;
      Wm = port[1]->getAWP()*momentDir;
    }
    WrP0P1 = port[1]->getWrOP()-port[0]->getWrOP();
    g(IT) = trans(Wf)*WrP0P1;
    g(IR) = x;
  }

  void Connection::updateStage2(double t) {
    WvP0P1 = port[1]->getWvP()-port[0]->getWvP();
    WomP0P1 = port[1]->getWomegaP()-port[0]->getWomegaP();
    if(KOSYID) 
      gd(IT) = trans(Wf)*(WvP0P1 - crossProduct(port[0]->getWomegaP(), WrP0P1));
    else
      gd(IT) = trans(Wf)*(WvP0P1);
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
    xd = gd(IR);
  }

  void Connection::updatedx(double t, double dt) {
    xd = gd(IR)*dt;
  }


  void Connection::initPlotFiles() {

    LinkPort::initPlotFiles();

#ifdef HAVE_AMVIS
    if (coilspringAMVis) {
      coilspringAMVis->writeBodyFile();
    }
#endif
  }


  void Connection::plot(double t,double dt) {
    LinkPort::plot(t,dt);

#ifdef HAVE_AMVIS
    if (coilspringAMVis) {
      Vec WrOToPoint;
      Vec WrOFromPoint;

      WrOFromPoint = port[0]->getWrOP();
      WrOToPoint   = port[1]->getWrOP();
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
#endif
  }
}
