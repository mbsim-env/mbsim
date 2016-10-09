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
#include "basic_properties.h"
#include "frame.h"
#include "contour.h"
#include "rigid_body.h"
#include "signal_.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "kinematics_widgets.h"
#include "extended_widgets.h"
#include "mainwindow.h"
#include <xercesc/dom/DOMText.hpp>
#include <mbxmlutils/eval.h>
#include <QDir>
#include <QTreeWidget>
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern bool absolutePath;
  extern QDir mbsDir;

  LocalFrameOfReferenceProperty::LocalFrameOfReferenceProperty(const std::string &frame_, Element* element_, const MBXMLUtils::FQN &xmlName_): frame(frame_), framePtr(element_->getFrame(frame.substr(6, frame.length()-7))), element(element_), xmlName(xmlName_) {
  }

  void LocalFrameOfReferenceProperty::setFrame(const std::string &str) {
    frame = str;
    framePtr=element->getFrame(frame.substr(6, frame.length()-7));
  }

  std::string LocalFrameOfReferenceProperty::getFrame() const {
    return framePtr?("Frame[" + framePtr->getName() + "]"):frame;
  }

  DOMElement* LocalFrameOfReferenceProperty::initializeUsingXML(DOMElement *parent) {
    DOMElement *e = E(parent)->getFirstElementChildNamed(xmlName);
    if(e) setFrame(E(e)->getAttribute("ref"));
    return e;
  }

  DOMElement* LocalFrameOfReferenceProperty::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(xmlName);
    E(ele)->setAttribute("ref", getFrame());
    parent->insertBefore(ele, NULL);
    return 0;
  }

  void LocalFrameOfReferenceProperty::fromWidget(QWidget *widget) {
    setFrame(static_cast<LocalFrameOfReferenceWidget*>(widget)->getFrame().toStdString());
  }

  void LocalFrameOfReferenceProperty::toWidget(QWidget *widget) {
    static_cast<LocalFrameOfReferenceWidget*>(widget)->setFrame(QString::fromStdString(frame),framePtr);
    static_cast<LocalFrameOfReferenceWidget*>(widget)->updateWidget();
  }

  ParentFrameOfReferenceProperty::ParentFrameOfReferenceProperty(const std::string &frame_, Element* element_, const MBXMLUtils::FQN &xmlName_): frame(frame_), framePtr(element_->getParent()->getFrame(frame.substr(9, frame.length()-10))), element(element_), xmlName(xmlName_) {
  }

  void ParentFrameOfReferenceProperty::initialize() {
    framePtr=element->getParent()->getFrame((frame.substr(0,2)=="..")?frame.substr(9, frame.length()-10):frame.substr(6, frame.length()-7));
  }

  void ParentFrameOfReferenceProperty::setFrame(const std::string &str) {
    frame = str;
    framePtr=element->getParent()->getFrame(frame.substr(9, frame.length()-10));
  }

  std::string ParentFrameOfReferenceProperty::getFrame() const {
    return framePtr?("../Frame[" + framePtr->getName() + "]"):frame;
  }

  DOMElement* ParentFrameOfReferenceProperty::initializeUsingXML(DOMElement *parent) {
    DOMElement *e = E(parent)->getFirstElementChildNamed(xmlName);
    if(e) frame = E(e)->getAttribute("ref");
    return e;
  }

  DOMElement* ParentFrameOfReferenceProperty::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(xmlName);
    E(ele)->setAttribute("ref", getFrame());
    parent->insertBefore(ele, NULL);
    return 0;
  }

  void ParentFrameOfReferenceProperty::fromWidget(QWidget *widget) {
    setFrame(static_cast<ParentFrameOfReferenceWidget*>(widget)->getFrame().toStdString());
  }

  void ParentFrameOfReferenceProperty::toWidget(QWidget *widget) {
    static_cast<ParentFrameOfReferenceWidget*>(widget)->setFrame(QString::fromStdString(frame),framePtr);
    static_cast<ParentFrameOfReferenceWidget*>(widget)->updateWidget();
  }

  FrameOfReferenceProperty::FrameOfReferenceProperty(const std::string &frame_, Element* element_, const MBXMLUtils::FQN &xmlName_) : frame(frame_), framePtr(element_->getByPath<Frame>(frame)), element(element_), xmlName(xmlName_) {
  }

  void FrameOfReferenceProperty::initialize() {
    framePtr=element->getByPath<Frame>(frame);
  }

  void FrameOfReferenceProperty::setFrame(const std::string &str) {
    frame = str;
    framePtr=element->getByPath<Frame>(frame);
  }

  std::string FrameOfReferenceProperty::getFrame() const {
    return framePtr?framePtr->getXMLPath(element,true):frame;
  }

  DOMElement* FrameOfReferenceProperty::initializeUsingXML(DOMElement *parent) {
    DOMElement *e = E(parent)->getFirstElementChildNamed(xmlName);
    if(e) frame=E(e)->getAttribute("ref");
    return e;
  }

  DOMElement* FrameOfReferenceProperty::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(xmlName);
    E(ele)->setAttribute("ref", getFrame());
    parent->insertBefore(ele, NULL);
    return 0;
  }

  void FrameOfReferenceProperty::fromWidget(QWidget *widget) {
    setFrame(static_cast<FrameOfReferenceWidget*>(widget)->getFrame().toStdString());
  }

  void FrameOfReferenceProperty::toWidget(QWidget *widget) {
    static_cast<FrameOfReferenceWidget*>(widget)->setFrame(QString::fromStdString(frame),framePtr);
    static_cast<FrameOfReferenceWidget*>(widget)->updateWidget();
  }

  ContourOfReferenceProperty::ContourOfReferenceProperty(const std::string &contour_, Element* element_, const FQN &xmlName_) : contour(contour_), contourPtr(element_->getByPath<Contour>(contour)), element(element_), xmlName(xmlName_) {
  }

  void ContourOfReferenceProperty::initialize() {
    contourPtr=element->getByPath<Contour>(contour);
  }

  void ContourOfReferenceProperty::setContour(const std::string &str) {
    contour = str;
    contourPtr=element->getByPath<Contour>(contour);
  }

  std::string ContourOfReferenceProperty::getContour() const {
    return contourPtr?contourPtr->getXMLPath(element,true):contour;
  }

  DOMElement* ContourOfReferenceProperty::initializeUsingXML(DOMElement *parent) {
    DOMElement *e = E(parent)->getFirstElementChildNamed(xmlName);
    if(e) contour=E(e)->getAttribute("ref");
    return e;
  }

  DOMElement* ContourOfReferenceProperty::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(xmlName);
    E(ele)->setAttribute("ref", getContour());
    parent->insertBefore(ele, NULL);
    return 0;
  }

  void ContourOfReferenceProperty::fromWidget(QWidget *widget) {
    setContour(static_cast<ContourOfReferenceWidget*>(widget)->getContour().toStdString());
  }

  void ContourOfReferenceProperty::toWidget(QWidget *widget) {
    static_cast<ContourOfReferenceWidget*>(widget)->setContour(QString::fromStdString(contour),contourPtr);
    static_cast<ContourOfReferenceWidget*>(widget)->updateWidget();
  }

  RigidBodyOfReferenceProperty::RigidBodyOfReferenceProperty(const std::string &body_, Element *element_, const FQN &xmlName_) : body(body_), bodyPtr(element_->getByPath<RigidBody>(body)), element(element_), xmlName(xmlName_) {
  }

  void RigidBodyOfReferenceProperty::initialize() {
    bodyPtr=element->getByPath<RigidBody>(body);
  }

  void RigidBodyOfReferenceProperty::setBody(const std::string &str) {
    body = str;
    bodyPtr=element->getByPath<RigidBody>(body);
  }

  std::string RigidBodyOfReferenceProperty::getBody() const {
    return bodyPtr?bodyPtr->getXMLPath(element,true):body;
  }

  DOMElement* RigidBodyOfReferenceProperty::initializeUsingXML(DOMElement *parent) {
    DOMElement *e = (xmlName==FQN())?parent:E(parent)->getFirstElementChildNamed(xmlName);
    if(e) body=E(e)->getAttribute("ref");
    return e;
  }

  DOMElement* RigidBodyOfReferenceProperty::writeXMLFile(DOMNode *parent) {
    if(xmlName==FQN()) 
      E(static_cast<DOMElement*>(parent))->setAttribute("ref", getBody());
    else {
      DOMDocument *doc=parent->getOwnerDocument();
      DOMElement *ele = D(doc)->createElement(xmlName);
      E(ele)->setAttribute("ref", getBody());
      parent->insertBefore(ele, NULL);
    }
    return 0;
  }

  void RigidBodyOfReferenceProperty::fromWidget(QWidget *widget) {
    setBody(static_cast<RigidBodyOfReferenceWidget*>(widget)->getBody().toStdString());
  }

  void RigidBodyOfReferenceProperty::toWidget(QWidget *widget) {
    static_cast<RigidBodyOfReferenceWidget*>(widget)->setBody(QString::fromStdString(body),bodyPtr);
    static_cast<RigidBodyOfReferenceWidget*>(widget)->updateWidget();
  }

  ObjectOfReferenceProperty::ObjectOfReferenceProperty(const std::string &object_, Element *element_, const FQN &xmlName_) : object(object_), objectPtr(element_->getByPath<Object>(object)), element(element_), xmlName(xmlName_) {
  }

  void ObjectOfReferenceProperty::initialize() {
    objectPtr=element->getByPath<Object>(object);
  }

  void ObjectOfReferenceProperty::setObject(const std::string &str) {
    object = str;
    objectPtr=element->getByPath<Object>(object);
  }

  std::string ObjectOfReferenceProperty::getObject() const {
    return objectPtr?objectPtr->getXMLPath(element,true):object;
  }

  DOMElement* ObjectOfReferenceProperty::initializeUsingXML(DOMElement *parent) {
    DOMElement *e = E(parent)->getFirstElementChildNamed(xmlName);
    if(e) object=E(e)->getAttribute("ref");
    return e;
  }

  DOMElement* ObjectOfReferenceProperty::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(xmlName);
    E(ele)->setAttribute("ref", getObject());
    parent->insertBefore(ele, NULL);
    return 0;
  }

  void ObjectOfReferenceProperty::fromWidget(QWidget *widget) {
    setObject(static_cast<ObjectOfReferenceWidget*>(widget)->getObject().toStdString());
  }

  void ObjectOfReferenceProperty::toWidget(QWidget *widget) {
    static_cast<ObjectOfReferenceWidget*>(widget)->setObject(QString::fromStdString(object),objectPtr);
    static_cast<ObjectOfReferenceWidget*>(widget)->updateWidget();
  }

  LinkOfReferenceProperty::LinkOfReferenceProperty(const std::string &link_, Element *element_, const FQN &xmlName_) : link(link_), linkPtr(element_->getByPath<Link>(link)), element(element_), xmlName(xmlName_) {
  }

  void LinkOfReferenceProperty::initialize() {
    linkPtr=element->getByPath<Link>(link);
  }

  void LinkOfReferenceProperty::setLink(const std::string &str) {
    link = str;
    linkPtr=element->getByPath<Link>(link);
  }

  std::string LinkOfReferenceProperty::getLink() const {
    return linkPtr?linkPtr->getXMLPath(element,true):link;
  }

  DOMElement* LinkOfReferenceProperty::initializeUsingXML(DOMElement *parent) {
    DOMElement *e = E(parent)->getFirstElementChildNamed(xmlName);
    if(e) link=E(e)->getAttribute("ref");
    return e;
  }

  DOMElement* LinkOfReferenceProperty::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(xmlName);
    E(ele)->setAttribute("ref", getLink());
    parent->insertBefore(ele, NULL);
    return 0;
  }

  void LinkOfReferenceProperty::fromWidget(QWidget *widget) {
    setLink(static_cast<LinkOfReferenceWidget*>(widget)->getLink().toStdString());
  }

  void LinkOfReferenceProperty::toWidget(QWidget *widget) {
    static_cast<LinkOfReferenceWidget*>(widget)->setLink(QString::fromStdString(link),linkPtr);
    static_cast<LinkOfReferenceWidget*>(widget)->updateWidget();
  }

  SignalOfReferenceProperty::SignalOfReferenceProperty(const std::string &signal_, Element *element_, const FQN &xmlName_) : signal(signal_), signalPtr(element_->getByPath<Signal>(signal)), element(element_), xmlName(xmlName_) {
  }

  void SignalOfReferenceProperty::initialize() {
    signalPtr=element->getByPath<Signal>(signal);
  }

  void SignalOfReferenceProperty::setSignal(const std::string &str) {
    signal = str;
    signalPtr=element->getByPath<Signal>(signal);
  }

  std::string SignalOfReferenceProperty::getSignal() const {
    return signalPtr?signalPtr->getXMLPath(element,true):signal;
  }

  DOMElement* SignalOfReferenceProperty::initializeUsingXML(DOMElement *parent) {
    DOMElement *e = E(parent)->getFirstElementChildNamed(xmlName);
    if(e) signal=E(e)->getAttribute("ref");
    return e;
  }

  DOMElement* SignalOfReferenceProperty::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(xmlName);
    E(ele)->setAttribute("ref", getSignal());
    parent->insertBefore(ele, NULL);
    return 0;
  }

  void SignalOfReferenceProperty::fromWidget(QWidget *widget) {
    setSignal(static_cast<SignalOfReferenceWidget*>(widget)->getSignal().toStdString());
  }

  void SignalOfReferenceProperty::toWidget(QWidget *widget) {
    static_cast<SignalOfReferenceWidget*>(widget)->setSignal(QString::fromStdString(signal),signalPtr);
    static_cast<SignalOfReferenceWidget*>(widget)->updateWidget();
  }

  DOMElement* FileProperty::initializeUsingXML(DOMElement *element) {
    DOMElement *e = xmlName.second.empty()?element:E(element)->getFirstElementChildNamed(xmlName);
    if(e) {
      DOMText *text = E(e)->getFirstTextChild();
      if(text) {
        setFile(X()%text->getData());
        return e;
      }
    }
    return 0;
  }

  DOMElement* FileProperty::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0 = xmlName.second.empty()?static_cast<DOMElement*>(parent):D(doc)->createElement(xmlName);
    string fileName = file;
    if(absolutePath) {
      QFileInfo fileInfo = QString::fromStdString(fileName.substr(1,fileName.length()-2));
      fileName = string("\"")+fileInfo.absoluteFilePath().toStdString()+"\"";
    }
    DOMText *text = doc->createTextNode(X()%fileName);
    ele0->insertBefore(text, NULL);
    parent->insertBefore(ele0, NULL);

    return 0;
  }

  void FileProperty::fromWidget(QWidget *widget) {
    setFile(static_cast<FileWidget*>(widget)->getFile().toStdString());
  }

  void FileProperty::toWidget(QWidget *widget) {
    static_cast<FileWidget*>(widget)->blockSignals(true);
    static_cast<FileWidget*>(widget)->setFile(QString::fromStdString(getFile()));
    static_cast<FileWidget*>(widget)->blockSignals(false);
  }

  DOMElement* IntegerProperty::initializeUsingXML(DOMElement *element) {
    DOMElement *e=E(element)->getFirstElementChildNamed(xmlName);
    if(e) {
      DOMText *text = E(e)->getFirstTextChild();
      if(text) {
        value = boost::lexical_cast<int>(X()%text->getData());
        return e;
      }
    }
    return 0;
  }

  DOMElement* IntegerProperty::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0 = D(doc)->createElement(xmlName);
    DOMText *text= doc->createTextNode(X()%toStr(value));
    ele0->insertBefore(text, NULL);
    parent->insertBefore(ele0, NULL);

    return 0;
  }

  void IntegerProperty::fromWidget(QWidget *widget) {
    value = static_cast<IntegerWidget*>(widget)->getValue();
  }

  void IntegerProperty::toWidget(QWidget *widget) {
    static_cast<IntegerWidget*>(widget)->setValue(value);
  }

  DOMElement* TextProperty::initializeUsingXML(DOMElement *element) {
    DOMElement *e=(xmlName==FQN())?element:E(element)->getFirstElementChildNamed(xmlName);
    if(e) {
      DOMText *text_ = E(e)->getFirstTextChild();
      if(text_) {
        text = X()%text_->getNodeValue();
        if(quote)
          text = text.substr(1,text.length()-2);
        return e;
      }
    }
    return 0;
  }

  DOMElement* TextProperty::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0;
    if(xmlName!=FQN()) {
      ele0 = D(doc)->createElement(xmlName);
      parent->insertBefore(ele0, NULL);
    }
    else
      ele0 = (DOMElement*)parent;
    DOMText *text_ = doc->createTextNode(X()%(quote?("\""+text+"\""):text));
    ele0->insertBefore(text_, NULL);
    return 0;
  }

  void TextProperty::fromWidget(QWidget *widget) {
    text = static_cast<BasicTextWidget*>(widget)->getText().toStdString();
  }

  void TextProperty::toWidget(QWidget *widget) {
    static_cast<BasicTextWidget*>(widget)->setText(QString::fromStdString(text));
  }

  ConnectFramesProperty::ConnectFramesProperty(int n, Element *element, const FQN &xmlName_) : xmlName(xmlName_)  {

    for(int i=0; i<n; i++) {
      FQN xmlName = MBSIM%"ref";
      if(n>1)
        xmlName.second += toStr(i+1);
      frame.push_back(FrameOfReferenceProperty("",element,xmlName));
    }
  }

  void ConnectFramesProperty::initialize() {
    for(unsigned int i=0; i<frame.size(); i++)
      frame[i].initialize();
  }

  DOMElement* ConnectFramesProperty::initializeUsingXML(DOMElement *element) {
    DOMElement *e = E(element)->getFirstElementChildNamed(xmlName);
    if(e) {
      for(unsigned int i=0; i<frame.size(); i++) {
        string xmlName = "ref";
        if(frame.size()>1)
          xmlName += toStr(int(i+1));
        if(!E(e)->hasAttribute(xmlName))
          return 0;
        frame[i].setFrame(E(e)->getAttribute(xmlName.c_str()));
      }
    }
    return e;
  }

  DOMElement* ConnectFramesProperty::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(xmlName);
    for(unsigned int i=0; i<frame.size(); i++) {
      string xmlName = "ref";
      if(frame.size()>1)
        xmlName += toStr(int(i+1));
      E(ele)->setAttribute(xmlName, frame[i].getFrame()); 
    }
    parent->insertBefore(ele, NULL);
    return ele;
  }

  void ConnectFramesProperty::fromWidget(QWidget *widget) {
    for(unsigned int i=0; i<frame.size(); i++)
      frame[i].fromWidget(static_cast<ConnectFramesWidget*>(widget)->widget[i]);
  }

  void ConnectFramesProperty::toWidget(QWidget *widget) {
    for(unsigned int i=0; i<frame.size(); i++)
      frame[i].toWidget(static_cast<ConnectFramesWidget*>(widget)->widget[i]);
    static_cast<ConnectFramesWidget*>(widget)->update();
  }

  ConnectContoursProperty::ConnectContoursProperty(int n, Element *element, const FQN &xmlName_) : xmlName(xmlName_) {

    for(int i=0; i<n; i++) {
      FQN xmlName = MBSIM%"ref";
      if(n>1)
        xmlName.second += toStr(i+1);
      contour.push_back(ContourOfReferenceProperty("",element,xmlName));
    }
  }

  void ConnectContoursProperty::initialize() {
    for(unsigned int i=0; i<contour.size(); i++)
      contour[i].initialize();
  }

  DOMElement* ConnectContoursProperty::initializeUsingXML(DOMElement *element) {
    DOMElement *e = E(element)->getFirstElementChildNamed(xmlName);
    if(e) {
      for(unsigned int i=0; i<contour.size(); i++) {
        string xmlName = "ref";
        if(contour.size()>1)
          xmlName += toStr(int(i+1));
        if(!E(e)->hasAttribute(xmlName))
          return 0;
        contour[i].setContour(E(e)->getAttribute(xmlName.c_str()));
      }
    }
    return e;
  }

  DOMElement* ConnectContoursProperty::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(xmlName);
    for(unsigned int i=0; i<contour.size(); i++) {
      string xmlName = "ref";
      if(contour.size()>1)
        xmlName += toStr(int(i+1));
      E(ele)->setAttribute(xmlName, contour[i].getContour()); 
    }
    parent->insertBefore(ele, NULL);
    return ele;
  }

  void ConnectContoursProperty::fromWidget(QWidget *widget) {
    for(unsigned int i=0; i<contour.size(); i++)
      contour[i].fromWidget(static_cast<ConnectContoursWidget*>(widget)->widget[i]);
  }

  void ConnectContoursProperty::toWidget(QWidget *widget) {
    for(unsigned int i=0; i<contour.size(); i++)
      contour[i].toWidget(static_cast<ConnectContoursWidget*>(widget)->widget[i]);
    static_cast<ConnectContoursWidget*>(widget)->update();
  }

  DynamicSystemSolverTolerancesProperty::DynamicSystemSolverTolerancesProperty() : projection(0,false), g(0,false), gd(0,false), gdd(0,false), la(0,false), La(0,false) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-15"), "-", MBSIM%"projection"));
    projection.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-8"), "-", MBSIM%"g"));
    g.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-10"), "-", MBSIM%"gd"));
    gd.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-12"), "-", MBSIM%"gdd"));
    gdd.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-12"), "-", MBSIM%"la"));
    la.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1e-10"), "-", MBSIM%"La"));
    La.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* DynamicSystemSolverTolerancesProperty::initializeUsingXML(DOMElement *element) {
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"tolerances");
    if(e) {
      projection.initializeUsingXML(e);
      g.initializeUsingXML(e);
      gd.initializeUsingXML(e);
      gdd.initializeUsingXML(e);
      la.initializeUsingXML(e);
      La.initializeUsingXML(e);
    }
    return e;
  }

  DOMElement* DynamicSystemSolverTolerancesProperty::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *e=D(doc)->createElement(MBSIM%"tolerances");
    parent->insertBefore(e, NULL);
    projection.writeXMLFile(e);
    g.writeXMLFile(e);
    gd.writeXMLFile(e);
    gdd.writeXMLFile(e);
    la.writeXMLFile(e);
    La.writeXMLFile(e);
    return e;
  }

  void DynamicSystemSolverTolerancesProperty::fromWidget(QWidget *widget) {
    projection.fromWidget(static_cast<DynamicSystemSolverTolerancesWidget*>(widget)->projection);
    g.fromWidget(static_cast<DynamicSystemSolverTolerancesWidget*>(widget)->g);
    gd.fromWidget(static_cast<DynamicSystemSolverTolerancesWidget*>(widget)->gd);
    gdd.fromWidget(static_cast<DynamicSystemSolverTolerancesWidget*>(widget)->gdd);
    la.fromWidget(static_cast<DynamicSystemSolverTolerancesWidget*>(widget)->la);
    La.fromWidget(static_cast<DynamicSystemSolverTolerancesWidget*>(widget)->La);
  }

  void DynamicSystemSolverTolerancesProperty::toWidget(QWidget *widget) {
    projection.toWidget(static_cast<DynamicSystemSolverTolerancesWidget*>(widget)->projection);
    g.toWidget(static_cast<DynamicSystemSolverTolerancesWidget*>(widget)->g);
    gd.toWidget(static_cast<DynamicSystemSolverTolerancesWidget*>(widget)->gd);
    gdd.toWidget(static_cast<DynamicSystemSolverTolerancesWidget*>(widget)->gdd);
    la.toWidget(static_cast<DynamicSystemSolverTolerancesWidget*>(widget)->la);
    La.toWidget(static_cast<DynamicSystemSolverTolerancesWidget*>(widget)->La);
  }

  DynamicSystemSolverParametersProperty::DynamicSystemSolverParametersProperty() : constraintSolver(0,false), impactSolver(0,false), numberOfMaximalIterations(0,false), tolerances(0,false) {
    constraintSolver.setProperty(new TextProperty("\"FixedPointSingle\"", MBSIM%"constraintSolver"));
    impactSolver.setProperty(new TextProperty("\"FixedPointSingle\"", MBSIM%"impactSolver"));

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("10000"), "", MBSIM%"numberOfMaximalIterations"));
    numberOfMaximalIterations.setProperty(new ExtPhysicalVarProperty(input));

    tolerances.setProperty(new DynamicSystemSolverTolerancesProperty);
  }

  DOMElement* DynamicSystemSolverParametersProperty::initializeUsingXML(DOMElement *element) {
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"solverParameters");
    if(e) {
      constraintSolver.initializeUsingXML(e);
      impactSolver.initializeUsingXML(e);
      numberOfMaximalIterations.initializeUsingXML(e);
      tolerances.initializeUsingXML(e);
    }
    return e;
  }

  DOMElement* DynamicSystemSolverParametersProperty::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *e=D(doc)->createElement(MBSIM%"solverParameters");
    parent->insertBefore(e, NULL);
    constraintSolver.writeXMLFile(e);
    impactSolver.writeXMLFile(e);
    numberOfMaximalIterations.writeXMLFile(e);
    tolerances.writeXMLFile(e);
    return e;
  }

  void DynamicSystemSolverParametersProperty::fromWidget(QWidget *widget) {
    constraintSolver.fromWidget(static_cast<DynamicSystemSolverParametersWidget*>(widget)->constraintSolver);
    impactSolver.fromWidget(static_cast<DynamicSystemSolverParametersWidget*>(widget)->impactSolver);
    numberOfMaximalIterations.fromWidget(static_cast<DynamicSystemSolverParametersWidget*>(widget)->numberOfMaximalIterations);
    tolerances.fromWidget(static_cast<DynamicSystemSolverParametersWidget*>(widget)->tolerances);
  }

  void DynamicSystemSolverParametersProperty::toWidget(QWidget *widget) {
    constraintSolver.toWidget(static_cast<DynamicSystemSolverParametersWidget*>(widget)->constraintSolver);
    impactSolver.toWidget(static_cast<DynamicSystemSolverParametersWidget*>(widget)->impactSolver);
    numberOfMaximalIterations.toWidget(static_cast<DynamicSystemSolverParametersWidget*>(widget)->numberOfMaximalIterations);
    tolerances.toWidget(static_cast<DynamicSystemSolverParametersWidget*>(widget)->tolerances);
  }

  EmbedProperty::EmbedProperty(std::function<const std::string&()> f) : href(0,false), count(0,false), counterName(0,false), parameterList(0,false) {
    href.setProperty(new FileProperty(""));
    static_cast<FileProperty*>(href.getProperty())->setFile(f()+".xml");
    count.setProperty(new PhysicalVariableProperty(new ScalarProperty("1")));
    counterName.setProperty(new TextProperty("n",""));
    parameterList.setProperty(new FileProperty(""));
  }

  DOMElement* EmbedProperty::initializeUsingXML(DOMElement *parent) {
    if(E(parent)->hasAttribute("href")) {
      href.setActive(true);
      string file = E(parent)->getAttribute("href");
      static_cast<FileProperty*>(href.getProperty())->setFile(file);
    }
    if(E(parent)->hasAttribute("count")) {
      count.setActive(true);
      static_cast<PhysicalVariableProperty*>(count.getProperty())->setValue(E(parent)->getAttribute("count").c_str());
    }
    if(E(parent)->hasAttribute("counterName")) {
      counterName.setActive(true);
      static_cast<TextProperty*>(counterName.getProperty())->setText(E(parent)->getAttribute("counterName"));
    }
    if(E(parent)->hasAttribute("parameterHref")) {
      parameterList.setActive(true);
      static_cast<FileProperty*>(parameterList.getProperty())->setFile(E(parent)->getAttribute("parameterHref"));
    }
    //  DOMElement *ele = E(parent)->getFirstElementChildNamed(PV%"parameterHref");
    //  if(ele) {
    //    parameterList.setActive(true);
    //    string file = E(ele)->getAttribute("href");
    //    static_cast<FileProperty*>(parameterList.getProperty())->setFile(file);
    //  }
    return NULL;
  }

  DOMElement* EmbedProperty::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(PV%"Embed");
    if(not(absolutePath) and href.isActive()) {
      string relFileName =  mbsDir.relativeFilePath(QString::fromStdString(getFile())).toStdString();
      E(ele0)->setAttribute("href", relFileName);
    }
    if(count.isActive())
      E(ele0)->setAttribute("count", static_cast<PhysicalVariableProperty*>(count.getProperty())->getValue());
    if(counterName.isActive())
      E(ele0)->setAttribute("counterName", static_cast<TextProperty*>(counterName.getProperty())->getText());
    if(not(absolutePath) and parameterList.isActive()) {
      //DOMElement *ele1=D(doc)->createElement(PV%"parameterHref");
      //    string filePath = absolutePath?mbsDir.absoluteFilePath(QString::fromStdString(static_cast<FileProperty*>(parameterList.getProperty())->getFile())).toStdString():mbsDir.relativeFilePath(QString::fromStdString(static_cast<FileProperty*>(parameterList.getProperty())->getFile())).toStdString();
      //E(ele1)->setAttribute("parameterHref", filePath);
      string relFileName =  mbsDir.relativeFilePath(QString::fromStdString(getParameterFile())).toStdString();
      E(ele0)->setAttribute("parameterHref", relFileName);
      //ele0->insertBefore(ele1, NULL);
    }
    parent->insertBefore(ele0, NULL);
    return ele0;
  }

  void EmbedProperty::fromWidget(QWidget *widget) {
    href.fromWidget(static_cast<EmbedWidget*>(widget)->href);
    count.fromWidget(static_cast<EmbedWidget*>(widget)->count);
    counterName.fromWidget(static_cast<EmbedWidget*>(widget)->counterName);
    parameterList.fromWidget(static_cast<EmbedWidget*>(widget)->parameterList);
  }

  void EmbedProperty::toWidget(QWidget *widget) {
    href.toWidget(static_cast<EmbedWidget*>(widget)->href);
    count.toWidget(static_cast<EmbedWidget*>(widget)->count);
    counterName.toWidget(static_cast<EmbedWidget*>(widget)->counterName);
    parameterList.toWidget(static_cast<EmbedWidget*>(widget)->parameterList);
  }

  SignalReferenceProperty::SignalReferenceProperty(Element* element) : refSignal("",element) {
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "", MBSIMCONTROL%"factor"));
    factor.setProperty(new ExtPhysicalVarProperty(input));
  } 

  DOMElement* SignalReferenceProperty::initializeUsingXML(DOMElement *ele) {
    factor.initializeUsingXML(ele);
    refSignal.setSignal(E(ele)->getAttribute("ref"));
    return ele;
  }

  DOMElement* SignalReferenceProperty::writeXMLFile(DOMNode *parent) {
    DOMElement *ele = (DOMElement*)parent;
    factor.writeXMLFile(ele);
    E(ele)->setAttribute("ref", refSignal.getSignal());
    return ele;
  }

  void SignalReferenceProperty::fromWidget(QWidget *widget) {
    factor.fromWidget(static_cast<SignalReferenceWidget*>(widget)->factor);
    refSignal.fromWidget(static_cast<SignalReferenceWidget*>(widget)->refSignal);
  }

  void SignalReferenceProperty::toWidget(QWidget *widget) {
    factor.toWidget(static_cast<SignalReferenceWidget*>(widget)->factor);
    refSignal.toWidget(static_cast<SignalReferenceWidget*>(widget)->refSignal);
  }

  ColorProperty::ColorProperty(const FQN &xmlName_) : xmlName(xmlName_) {
    vector<PhysicalVariableProperty> input;
    vector<string> vec(3);
    vec[0] = "0.666667"; vec[1] = "1"; vec[2] = "1";
    input.push_back(PhysicalVariableProperty(new VecProperty(vec), "", ""));
    color.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* ColorProperty::initializeUsingXML(DOMElement *parent) {
    DOMElement *e = E(parent)->getFirstElementChildNamed(xmlName);
    color.initializeUsingXML(e);
    return e;
  }

  DOMElement* ColorProperty::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(xmlName);
    color.writeXMLFile(ele);
    parent->insertBefore(ele, NULL);
    return 0;
  }

  void ColorProperty::fromWidget(QWidget *widget) {
    color.fromWidget(static_cast<ColorWidget*>(widget)->color);
  }

  void ColorProperty::toWidget(QWidget *widget) {
    color.toWidget(static_cast<ColorWidget*>(widget)->color);
    static_cast<ColorWidget*>(widget)->updateWidget();
  }

  PlotFeatureStatusProperty::PlotFeatureStatusProperty(const vector<FQN> &plotFeatureTypes) : types(plotFeatureTypes) {
  }

  DOMElement* PlotFeatureStatusProperty::initializeUsingXML(DOMElement *parent) {
    DOMElement *e=parent->getFirstElementChild();
    while(e && (E(e)->getTagName()==MBSIM%"plotFeature" ||
                E(e)->getTagName()==MBSIM%"plotFeatureForChildren" ||
                E(e)->getTagName()==MBSIM%"plotFeatureRecursive")) {
      string feature = E(e)->getAttribute("feature");
      type.push_back(E(e)->getTagName().second);
      value.push_back(feature);
      e=e->getNextElementSibling();
    }
    return e;
  }

  DOMElement* PlotFeatureStatusProperty::initializeUsingXML2(DOMElement *parent) {
    DOMElement *e=E(parent)->getFirstElementChildNamed(types[0]);
    while(e && (E(e)->getTagName()==types[0])) {
      string feature = E(e)->getAttribute("feature");
      type.push_back(E(e)->getTagName().second);
      value.push_back(feature);
      e=e->getNextElementSibling();
    }
    return e;
  }

  DOMElement* PlotFeatureStatusProperty::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    for(size_t i=0; i<type.size(); i++) {
      DOMElement *ele = D(doc)->createElement(MBSIM%type[i]);
      E(ele)->setAttribute("feature",value[i]);
      parent->insertBefore(ele, NULL);
    }
    return 0;
  }

  DOMElement* PlotFeatureStatusProperty::writeXMLFile2(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    for(size_t i=0; i<type.size(); i++) {
      DOMElement *ele = D(doc)->createElement(FQN(types[0].first,type[i]));
      E(ele)->setAttribute("feature",value[i]);
      parent->insertBefore(ele, NULL);
    }
    return 0;
  }

  void PlotFeatureStatusProperty::fromWidget(QWidget *widget) {
    QTreeWidget *tree = static_cast<PlotFeatureStatusWidget*>(widget)->tree;
    type.clear();
    value.clear();
    for(int i=0; i<tree->topLevelItemCount(); i++) {
      type.push_back(tree->topLevelItem(i)->text(0).toStdString());
      value.push_back(tree->topLevelItem(i)->text(1).toStdString());
    }
  }

  void PlotFeatureStatusProperty::toWidget(QWidget *widget) {
    QTreeWidget *tree = static_cast<PlotFeatureStatusWidget*>(widget)->tree;
    tree->clear();
    for(size_t i=0; i<type.size(); i++) {
      QTreeWidgetItem *item = new QTreeWidgetItem;
      item->setText(0, QString::fromStdString(type[i]));
      item->setText(1, QString::fromStdString(value[i]));
      tree->addTopLevelItem(item);
    }
  }

}
