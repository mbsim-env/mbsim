/*
   MBSimGUI - A fronted for MBSim.
   Copyright (C) 2012 Martin Förg

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
   */

#include <config.h>
#include "frame.h"
#include "special_properties.h"
#include "ombv_properties.h"
#include "objectfactory.h"
#include "mainwindow.h"
#include "embed.h"
#include "flexible_body_ffr.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  Frame::Frame(const string &str, Element *parent, bool grey, const vector<FQN> &plotFeatureTypes) : Element(str,parent,plotFeatureTypes), visu(0,true) {

    visu.setProperty(new FrameMBSOMBVProperty("NOTSET",grey?"":MBSIM%"enableOpenMBV",getID()));
  }

  Frame* Frame::readXMLFile(const string &filename, Element *parent) {
    shared_ptr<DOMDocument> doc=mw->parser->parse(filename);
    DOMElement *e=doc->getDocumentElement();
    //Frame *frame=ObjectFactory::getInstance()->createFrame(e, parent);
    Frame *frame=Embed<Frame>::createAndInit(e,parent);
    if(frame) {
//      frame->initializeUsingXML(e);
      frame->initialize();
    }
    return frame;
  }

  DOMElement* Frame::initializeUsingXML(DOMElement *element) {
    Element::initializeUsingXML(element);
    visu.initializeUsingXML(element);
    return element;
  }

  DOMElement* Frame::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Element::writeXMLFile(parent);
    visu.writeXMLFile(ele0);
    return ele0;
  }

  void Frame::initializeUsingXML2(DOMElement *element) {
    visu.initializeUsingXML(element);
  }

  void Frame::initializeUsingXML3(DOMElement *element) {
    static_cast<PlotFeatureStatusProperty*>(plotFeature.getProperty())->initializeUsingXML2(element);
  }

  DOMElement* Frame::writeXMLFile2(DOMNode *parent) {
    visu.writeXMLFile(parent);
    return 0;
  }

  DOMElement* Frame::writeXMLFile3(DOMNode *parent) {
    static_cast<PlotFeatureStatusProperty*>(plotFeature.getProperty())->writeXMLFile2(parent);
    return 0;
  }

  FixedRelativeFrame::FixedRelativeFrame(const string &str, Element *parent) : Frame(str,parent,false), refFrame(0,false), position(0,false), orientation(0,false) {

    position.setProperty(new ChoiceProperty2(new VecPropertyFactory(3,MBSIM%"relativePosition"),"",4));

    orientation.setProperty(new ChoiceProperty2(new RotMatPropertyFactory(MBSIM%"relativeOrientation"),"",4));

    refFrame.setProperty(new ParentFrameOfReferenceProperty(getParent()->getFrame(0)->getXMLPath(this,true),this,MBSIM%"frameOfReference"));
  }

  void FixedRelativeFrame::initialize() {
    Frame::initialize();
    refFrame.initialize();
  }

  DOMElement* FixedRelativeFrame::initializeUsingXML(DOMElement *element) {
    Frame::initializeUsingXML(element);
    refFrame.initializeUsingXML(element);
    position.initializeUsingXML(element);
    orientation.initializeUsingXML(element);
    return element;
  }

  DOMElement* FixedRelativeFrame::writeXMLFile(DOMNode *parent) {

    DOMElement *ele0 = Frame::writeXMLFile(parent);
    refFrame.writeXMLFile(ele0);
    position.writeXMLFile(ele0);
    orientation.writeXMLFile(ele0);
    return ele0;
  }

  NodeFrame::NodeFrame(const string &str, Element *parent) : Frame(str,parent,false) {

    nodeNumber.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIMFLEX%"nodeNumber",vector<string>(2,"-")),"",4));
  }

  DOMElement* NodeFrame::initializeUsingXML(DOMElement *element) {
    Frame::initializeUsingXML(element);
    nodeNumber.initializeUsingXML(element);
    return element;
  }

  DOMElement* NodeFrame::writeXMLFile(DOMNode *parent) {

    DOMElement *ele0 = Frame::writeXMLFile(parent);
    nodeNumber.writeXMLFile(ele0);
    return ele0;
  }

  FixedNodalFrame::FixedNodalFrame(const string &str, Element *parent) : Frame(str,parent,false), position(0,false), orientation(0,false), Psi(0,false), sigmahel(0,false), sigmahen(0,false), sigma0(0,false), K0F(0,false), K0M(0,false) {

    position.setProperty(new ChoiceProperty2(new VecPropertyFactory(3,MBSIMFLEX%"relativePosition"),"",4));

    orientation.setProperty(new ChoiceProperty2(new RotMatPropertyFactory(MBSIMFLEX%"relativeOrientation"),"",4));

    int size = static_cast<FlexibleBodyFFR*>(parent)->getqElSize();

    Phi.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(3,size,"0"),MBSIMFLEX%"shapeMatrixOfTranslation",vector<string>(3,"")),"",4));

    Psi.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(3,size,"0"),MBSIMFLEX%"shapeMatrixOfRotation",vector<string>(3,"")),"",4));

    sigmahel.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(6,size,"0"),MBSIMFLEX%"stressMatrix",vector<string>(3,"")),"",4));

    sigmahen.setProperty(new OneDimMatArrayProperty(size,6,size,MBSIMFLEX%"nonlinearStressMatrix",true));

    sigma0.setProperty(new ChoiceProperty2(new VecPropertyFactory(6,MBSIMFLEX%"initialStress",vector<string>(3,"")),"",4));

    K0F.setProperty(new OneDimMatArrayProperty(3,size,size,MBSIMFLEX%"geometricStiffnessMatrixDueToForce"));

    K0M.setProperty(new OneDimMatArrayProperty(3,size,size,MBSIMFLEX%"geometricStiffnessMatrixDueToMoment"));
  }

  DOMElement* FixedNodalFrame::initializeUsingXML(DOMElement *element) {
    Frame::initializeUsingXML(element);
    position.initializeUsingXML(element);
    orientation.initializeUsingXML(element);
    Phi.initializeUsingXML(element);
    Psi.initializeUsingXML(element);
    sigmahel.initializeUsingXML(element);
    sigmahen.initializeUsingXML(element);
    sigma0.initializeUsingXML(element);
    K0F.initializeUsingXML(element);
    K0M.initializeUsingXML(element);
    return element;
  }

  DOMElement* FixedNodalFrame::writeXMLFile(DOMNode *parent) {

    DOMElement *ele0 = Frame::writeXMLFile(parent);
    position.writeXMLFile(ele0);
    orientation.writeXMLFile(ele0);
    Phi.writeXMLFile(ele0);
    Psi.writeXMLFile(ele0);
    sigmahel.writeXMLFile(ele0);
    sigmahen.writeXMLFile(ele0);
    sigma0.writeXMLFile(ele0);
    K0F.writeXMLFile(ele0);
    K0M.writeXMLFile(ele0);
    return ele0;
  }

}
