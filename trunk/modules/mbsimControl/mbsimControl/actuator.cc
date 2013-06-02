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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsimControl/actuator.h"
#include "mbsimControl/signal_.h"
#include "mbsimControl/obsolet_hint.h"
#include "mbsim/frame.h"
#include "mbsimControl/defines.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, Actuator, MBSIMCONTROLNS"Actuator")

  Actuator::Actuator(const string &name) : LinkMechanics(name), signal(0), KOSYID(1) {
  }

  void Actuator::updateh(double t, int j) {
    la = signal->getSignal();
    if(KOSYID) { // calculation of force / moment direction
      Wf = frame[KOSYID-1]->getOrientation()*forceDir;
      Wm = frame[KOSYID-1]->getOrientation()*momentDir;
    }
    WF[1] = Wf*la(IT);
    WF[0] = -WF[1];
    WM[1] = Wm*la(IR);
    WM[0] = -WM[1];

    h[j][0] += frame[0]->getJacobianOfTranslation(j).T()*WF[0] + frame[0]->getJacobianOfRotation(j).T()*WM[0];
    h[j][1] += frame[1]->getJacobianOfTranslation(j).T()*WF[1] + frame[1]->getJacobianOfRotation(j).T()*WM[1];
  }

  void Actuator::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      if(saved_inputSignal!="")
        setSignal(getByPath<Signal>(process_signal_string(saved_inputSignal)));
      if(saved_ref1!="" && saved_ref2!="")
        connect(getByPath<Frame>(saved_ref1), getByPath<Frame>(saved_ref2));
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

  void Actuator::calclaSize(int j) {
    LinkMechanics::calclaSize(j);
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

