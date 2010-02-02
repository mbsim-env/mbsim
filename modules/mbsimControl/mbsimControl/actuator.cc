/* Copyright (C) 2004-2009 MBSim Development Team
 *
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
 * Contact: mfoerg@users.berlios.de
 */

#include "mbsimControl/actuator.h"
#include "mbsimControl/signal_.h"
#include "mbsimControl/objectfactory.h"
//#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimControl {

  Actuator::Actuator(const string &name) : LinkMechanics(name), signal(0), KOSYID(1) {
  }

  void Actuator::updateh(double t) {
    la = signal->getSignal();
    if(KOSYID) { // calculation of force / moment direction
      Wf = frame[KOSYID-1]->getOrientation()*forceDir;
      Wm = frame[KOSYID-1]->getOrientation()*momentDir;
    }
    WF[0] = Wf*la(IT);
    WF[1] = -WF[0];
    WM[0] = Wm*la(IR);
    WM[1] = -WM[0];

    h[0] += trans(frame[0]->getJacobianOfTranslation())*WF[0] + trans(frame[0]->getJacobianOfRotation())*WM[0];
    h[1] += trans(frame[1]->getJacobianOfTranslation())*WF[1] + trans(frame[1]->getJacobianOfRotation())*WM[1];
    hLink[0] += trans(frame[0]->getJacobianOfTranslation())*WF[0] + trans(frame[0]->getJacobianOfRotation())*WM[0];
    hLink[1] += trans(frame[1]->getJacobianOfTranslation())*WF[1] + trans(frame[1]->getJacobianOfRotation())*WM[1];
  }

  void Actuator::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      if(saved_inputSignal!="")
        setSignal(getSignalByPath(saved_inputSignal));
      if(saved_ref1!="" && saved_ref2!="") {
        Frame *ref1=getFrameByPath(saved_ref1);
        Frame *ref2=getFrameByPath(saved_ref2);
        connect(ref1,ref2);
      }
      LinkMechanics::init(stage);
    }
    else if (stage==MBSim::resize) {
      LinkMechanics::init(stage);
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
    else
      LinkMechanics::init(stage);
  }

  void Actuator::calclaSize() {
    LinkMechanics::calclaSize();
    laSize = forceDir.cols()+momentDir.cols(); // cols = columns
  }

  void Actuator::connect(Frame *frame0, Frame* frame1) {
    LinkMechanics::connect(frame0);
    LinkMechanics::connect(frame1);
  }

  void Actuator::setForceDirection(const Mat &fd) {
    assert(fd.rows() == 3);

    forceDir = fd;

    for(int i=0; i<fd.cols(); i++)
      forceDir.col(i) = forceDir.col(i)/nrm2(fd.col(i));
  }

  void Actuator::setMomentDirection(const Mat &md) {
    assert(md.rows() == 3);

    momentDir = md;

    for(int i=0; i<md.cols(); i++)
      momentDir.col(i) = momentDir.col(i)/nrm2(md.col(i));
  }
  
  Signal * Actuator::getSignalByPath(string path) {
    int pos=path.find("Signal");
    path.erase(pos, 6);
    path.insert(pos, "Link");
    Link * s = getLinkByPath(path);
    if (dynamic_cast<Signal *>(s))
      return static_cast<Signal *>(s);
    else {
      std::cerr << "ERROR! \"" << path << "\" is not of Signal-Type." << std::endl; 
      _exit(1);
    }
  }

  void Actuator::initializeUsingXML(TiXmlElement *element) {
    LinkMechanics::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMCONTROLNS"forceDirection");
    if(e) setForceDirection(getMat(e,3,0));
    e=element->FirstChildElement(MBSIMCONTROLNS"momentDirection");
    if(e) setMomentDirection(getMat(e,3,0));
    e=element->FirstChildElement(MBSIMCONTROLNS"referenceFrame");
    if(e) setKOSY(getInt(e));
    e=element->FirstChildElement(MBSIMCONTROLNS"inputSignal");
    saved_inputSignal=e->Attribute("ref");
    e=element->FirstChildElement(MBSIMCONTROLNS"connect");
    saved_ref1=e->Attribute("ref1");
    saved_ref2=e->Attribute("ref2");
  }

}

