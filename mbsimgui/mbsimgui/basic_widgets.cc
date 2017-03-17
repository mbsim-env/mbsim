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
#include "basic_widgets.h"
#include "frame.h"
#include "contour.h"
#include "group.h"
#include "rigid_body.h"
#include "signal_.h"
#include "constraint.h"
#include "dialogs.h"
#include "mainwindow.h"
#include <QtGui>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern bool absolutePath;
  extern QDir mbsDir;
  extern MainWindow *mw;

  LocalFrameComboBox::LocalFrameComboBox(Element *element_, QWidget *parent) : CustomComboBox(parent), element(element_) {
    connect(this,SIGNAL(highlighted(const QString&)),this,SLOT(highlightObject(const QString&)));
  }

  void LocalFrameComboBox::showPopup() {
    CustomComboBox::showPopup();
    highlightObject(currentText());
  }

  void LocalFrameComboBox::hidePopup() {
    CustomComboBox::hidePopup();
    if(oldID!="") {
      mw->highlightObject(oldID);
      oldID="";
    }
  }

  void LocalFrameComboBox::highlightObject(const QString &frame) {
    if(oldID=="") 
      oldID = mw->getHighlightedObject();
    Frame *selection = element->getFrame(frame.toStdString().substr(6, frame.length()-7));
    if(selection)
      mw->highlightObject(selection->getID());
  }

  ParentFrameComboBox::ParentFrameComboBox(Element *element_, QWidget *parent) : CustomComboBox(parent), element(element_) {
    connect(this,SIGNAL(highlighted(const QString&)),this,SLOT(highlightObject(const QString&)));
  }

  void ParentFrameComboBox::showPopup() {
    CustomComboBox::showPopup();
    highlightObject(currentText());
  }

  void ParentFrameComboBox::hidePopup() {
    CustomComboBox::hidePopup();
    if(oldID!="") {
      mw->highlightObject(oldID);
      oldID="";
    }
  }

  void ParentFrameComboBox::highlightObject(const QString &frame) {
    if(oldID=="") 
      oldID = mw->getHighlightedObject();
    Frame *selection = element->getParent()->getFrame(frame.toStdString().substr(9, frame.length()-10));
    if(selection)
      mw->highlightObject(selection->getID());
  }

  LocalFrameOfReferenceWidget::LocalFrameOfReferenceWidget(Element *element_, Frame* omitFrame_) : element(element_), selectedFrame(0), omitFrame(omitFrame_) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    frame = new LocalFrameComboBox(element);
    frame->setEditable(true);
    layout->addWidget(frame);
    selectedFrame = element->getFrame(0);
    connect(frame,SIGNAL(currentIndexChanged(const QString&)),this,SLOT(setFrame(const QString&)));
    updateWidget();
  }

  void LocalFrameOfReferenceWidget::updateWidget() {
    frame->blockSignals(true);
    frame->clear();
    int oldIndex = 0;
    QString oldText = frame->currentText();
    for(int i=0, k=0; i<element->getNumberOfFrames(); i++) {
      if(omitFrame!=element->getFrame(i)) {
        frame->addItem("Frame["+QString::fromStdString(element->getFrame(i)->getName())+"]");
        if(element->getFrame(i) == selectedFrame)
          oldIndex = k;
        k++;
      }
    }
    if(selectedFrame)
      frame->setCurrentIndex(oldIndex);
    else
      frame->setEditText(oldText);
    frame->blockSignals(false);
  }

  void LocalFrameOfReferenceWidget::setFrame(const QString &str) {
    selectedFrame = element->getFrame(str.mid(6, str.length()-7).toStdString());
    frame->setEditText(str);
  }

  QString LocalFrameOfReferenceWidget::getFrame() const {
    return frame->currentText();
  }

  DOMElement* LocalFrameOfReferenceWidget::initializeUsingXML(DOMElement *element) {
    setFrame(QString::fromStdString(E(element)->getAttribute("ref")));
    return element;
  }

  DOMElement* LocalFrameOfReferenceWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    E(static_cast<DOMElement*>(parent))->setAttribute("ref", getFrame().toStdString());
    return NULL;
  }

  ParentFrameOfReferenceWidget::ParentFrameOfReferenceWidget(Element *element_, Frame* omitFrame_) : element(element_), selectedFrame(0), omitFrame(omitFrame_) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    frame = new ParentFrameComboBox(element);
    frame->setEditable(true);
    layout->addWidget(frame);
    selectedFrame = element->getParent()->getFrame(0);
    connect(frame,SIGNAL(currentIndexChanged(const QString&)),this,SLOT(setFrame(const QString&)));
    updateWidget();
  }

  void ParentFrameOfReferenceWidget::updateWidget() {
    frame->blockSignals(true);
    frame->clear();
    int oldIndex = 0;
    QString oldText = frame->currentText();
    for(int i=0, k=0; i<element->getParent()->getNumberOfFrames(); i++) {
      if(omitFrame!=element->getParent()->getFrame(i)) {
        frame->addItem("../Frame["+QString::fromStdString(element->getParent()->getFrame(i)->getName())+"]");
        if(element->getParent()->getFrame(i) == selectedFrame)
          oldIndex = k;
        k++;
      }
    }
    if(selectedFrame)
      frame->setCurrentIndex(oldIndex);
    else
      frame->setEditText(oldText);
    frame->blockSignals(false);
  }

  void ParentFrameOfReferenceWidget::setFrame(const QString &str) {
    selectedFrame = element->getParent()->getFrame(str.mid(9, str.length()-10).toStdString());
    frame->setEditText(str);
  }

  QString ParentFrameOfReferenceWidget::getFrame() const {
    return frame->currentText();
  }

  DOMElement* ParentFrameOfReferenceWidget::initializeUsingXML(DOMElement *element) {
    setFrame(QString::fromStdString(E(element)->getAttribute("ref")));
    return element;
  }

  DOMElement* ParentFrameOfReferenceWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    E(static_cast<DOMElement*>(parent))->setAttribute("ref", getFrame().toStdString());
    return NULL;
  }

  FrameOfReferenceWidget::FrameOfReferenceWidget(Element *element_, Frame* selectedFrame_) : element(element_), selectedFrame(selectedFrame_) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    frame = new QLineEdit;
    if(selectedFrame)
      frame->setText(QString::fromStdString(selectedFrame->getXMLPath(element,true)));
    frameBrowser = new FrameBrowser(element->getRoot(),selectedFrame,this);
    connect(frameBrowser,SIGNAL(accepted()),this,SLOT(setFrame()));
    layout->addWidget(frame);
    QPushButton *button = new QPushButton(tr("Browse"));
    connect(button,SIGNAL(clicked(bool)),frameBrowser,SLOT(show()));
    layout->addWidget(button);
    updateWidget();
  }

  void FrameOfReferenceWidget::setDefaultFrame(const QString &def_) {
    def = def_;
    frame->setPlaceholderText(def);
  }

  void FrameOfReferenceWidget::updateWidget() {
    frameBrowser->updateWidget(selectedFrame);
    if(selectedFrame) {
      setFrame();
    }
  }

  void FrameOfReferenceWidget::setFrame() { 
    if(frameBrowser->getFrameList()->currentItem())
      selectedFrame = (Frame*)static_cast<ElementItem*>(frameBrowser->getFrameList()->currentItem())->getElement();
    else
      selectedFrame = 0;
    frame->setText(selectedFrame?QString::fromStdString(selectedFrame->getXMLPath(element,true)):"");
  }

  void FrameOfReferenceWidget::setFrame(const QString &str) {
    if(str!=def) {
      selectedFrame = element->getByPath<Frame>(str.toStdString());
      frame->setText(str);
    }
  }

  QString FrameOfReferenceWidget::getFrame() const {
    return frame->text().isEmpty()?def:frame->text();
  }

  DOMElement* FrameOfReferenceWidget::initializeUsingXML(DOMElement *element) {
    setFrame(QString::fromStdString(E(element)->getAttribute("ref")));
    return element;
  }

  DOMElement* FrameOfReferenceWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    E(static_cast<DOMElement*>(parent))->setAttribute("ref", getFrame().toStdString());
    return NULL;
  }

  ContourOfReferenceWidget::ContourOfReferenceWidget(Element *element_, Contour* selectedContour_) : element(element_), selectedContour(selectedContour_) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    contour = new QLineEdit;
    if(selectedContour)
      contour->setText(QString::fromStdString(selectedContour->getXMLPath(element,true)));
    contourBrowser = new ContourBrowser(element->getRoot(),selectedContour,this);
    connect(contourBrowser,SIGNAL(accepted()),this,SLOT(setContour()));
    layout->addWidget(contour);
    QPushButton *button = new QPushButton(tr("Browse"));
    connect(button,SIGNAL(clicked(bool)),contourBrowser,SLOT(show()));
    layout->addWidget(button);
  }

  void ContourOfReferenceWidget::updateWidget() {
    contourBrowser->updateWidget(selectedContour);
    if(selectedContour) {
      setContour();
    }
  }

  void ContourOfReferenceWidget::setContour() { 
    if(contourBrowser->getContourList()->currentItem())
      selectedContour = (Contour*)static_cast<ElementItem*>(contourBrowser->getContourList()->currentItem())->getElement();
    else
      selectedContour = 0;
    contour->setText(selectedContour?QString::fromStdString(selectedContour->getXMLPath(element,true)):"");
  }

  void ContourOfReferenceWidget::setContour(const QString &str, Contour *contourPtr) {
    selectedContour = contourPtr;
    contour->setText(str);
  }

  QString ContourOfReferenceWidget::getContour() const {
    return contour->text();
  }

  RigidBodyOfReferenceWidget::RigidBodyOfReferenceWidget(Element *element_, RigidBody* selectedBody_) : element(element_), selectedBody(selectedBody_) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    body = new QLineEdit;
    if(selectedBody)
      body->setText(QString::fromStdString(selectedBody->getXMLPath(element,true)));
    bodyBrowser = new RigidBodyBrowser(element->getRoot(),0,this);
    connect(bodyBrowser,SIGNAL(accepted()),this,SLOT(setBody()));
    layout->addWidget(body);
    QPushButton *button = new QPushButton(tr("Browse"));
    connect(button,SIGNAL(clicked(bool)),bodyBrowser,SLOT(show()));
    layout->addWidget(button);
  }

  void RigidBodyOfReferenceWidget::updateWidget() {
    bodyBrowser->updateWidget(selectedBody); 
    if(selectedBody) {
      setBody();
    }
  }

  void RigidBodyOfReferenceWidget::setBody() {
    if(bodyBrowser->getRigidBodyList()->currentItem())
      selectedBody = static_cast<RigidBody*>(static_cast<ElementItem*>(bodyBrowser->getRigidBodyList()->currentItem())->getElement());
    else
      selectedBody = 0;
    body->setText(selectedBody?QString::fromStdString(selectedBody->getXMLPath(element,true)):"");
    emit bodyChanged();
    emit Widget::resize_();
  }

  void RigidBodyOfReferenceWidget::setBody(const QString &str, RigidBody *bodyPtr) {
    selectedBody = bodyPtr;
    body->setText(str);
    emit bodyChanged();
    emit Widget::resize_();
  }

  QString RigidBodyOfReferenceWidget::getBody() const {
    return body->text();
  }

  GearInputReferenceWidget::GearInputReferenceWidget(Element *element_, RigidBody* selectedBody_) : element(element_), selectedBody(selectedBody_) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    body = new QLineEdit;
    if(selectedBody)
      body->setText(QString::fromStdString(selectedBody->getXMLPath(element,true)));
    bodyBrowser = new RigidBodyBrowser(element->getRoot(),0,this);
    connect(bodyBrowser,SIGNAL(accepted()),this,SLOT(setBody()));
    layout->addWidget(body);
    QPushButton *button = new QPushButton(tr("Browse"));
    connect(button,SIGNAL(clicked(bool)),bodyBrowser,SLOT(show()));
    layout->addWidget(button);

    QLabel *label = new QLabel("Ratio");
    layout->addWidget(label);
    ratio = new QLineEdit;
    layout->addWidget(ratio);
  }

  void GearInputReferenceWidget::updateWidget() {
    bodyBrowser->updateWidget(selectedBody);
    if(selectedBody) {
      setBody();
    }
  }

  void GearInputReferenceWidget::setBody() {
    if(bodyBrowser->getRigidBodyList()->currentItem())
      selectedBody = static_cast<RigidBody*>(static_cast<ElementItem*>(bodyBrowser->getRigidBodyList()->currentItem())->getElement());
    else
      selectedBody = 0;
    body->setText(selectedBody?QString::fromStdString(selectedBody->getXMLPath(element,true)):"");
    emit bodyChanged();
  }

  void GearInputReferenceWidget::setBody(const QString &str, RigidBody *bodyPtr) {
    selectedBody = bodyPtr;
    body->setText(str);
    emit bodyChanged();
  }

  QString GearInputReferenceWidget::getBody() const {
    return body->text();
  }

  ObjectOfReferenceWidget::ObjectOfReferenceWidget(Element *element_, Object* selectedObject_) : element(element_), selectedObject(selectedObject_) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    object = new QLineEdit;
    if(selectedObject)
      object->setText(QString::fromStdString(selectedObject->getXMLPath(element,true)));
    objectBrowser = new ObjectBrowser(element->getRoot(),0,this);
    connect(objectBrowser,SIGNAL(accepted()),this,SLOT(setObject()));
    layout->addWidget(object);
    QPushButton *button = new QPushButton(tr("Browse"));
    connect(button,SIGNAL(clicked(bool)),objectBrowser,SLOT(show()));
    layout->addWidget(button);
  }

  void ObjectOfReferenceWidget::updateWidget() {
    objectBrowser->updateWidget(selectedObject); 
    if(selectedObject) {
      setObject();
    }
  }

  void ObjectOfReferenceWidget::setObject() {
    if(objectBrowser->getObjectList()->currentItem())
      selectedObject = static_cast<Object*>(static_cast<ElementItem*>(objectBrowser->getObjectList()->currentItem())->getElement());
    else
      selectedObject = 0;
    object->setText(selectedObject?QString::fromStdString(selectedObject->getXMLPath(element,true)):"");
    emit objectChanged();
  }

  void ObjectOfReferenceWidget::setObject(const QString &str, Object *objectPtr) {
    selectedObject = objectPtr;
    object->setText(str);
    emit objectChanged();
  }

  QString ObjectOfReferenceWidget::getObject() const {
    return object->text();
  }

  LinkOfReferenceWidget::LinkOfReferenceWidget(Element *element_, Link* selectedLink_) : element(element_), selectedLink(selectedLink_) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    link = new QLineEdit;
    if(selectedLink)
      link->setText(QString::fromStdString(selectedLink->getXMLPath(element,true)));
    linkBrowser = new LinkBrowser(element->getRoot(),0,this);
    connect(linkBrowser,SIGNAL(accepted()),this,SLOT(setLink()));
    layout->addWidget(link);
    QPushButton *button = new QPushButton(tr("Browse"));
    connect(button,SIGNAL(clicked(bool)),linkBrowser,SLOT(show()));
    layout->addWidget(button);
  }

  void LinkOfReferenceWidget::updateWidget() {
    linkBrowser->updateWidget(selectedLink); 
    if(selectedLink) {
      setLink();
    }
  }

  void LinkOfReferenceWidget::setLink() {
    if(linkBrowser->getLinkList()->currentItem())
      selectedLink = static_cast<Link*>(static_cast<ElementItem*>(linkBrowser->getLinkList()->currentItem())->getElement());
    else
      selectedLink = 0;
    link->setText(selectedLink?QString::fromStdString(selectedLink->getXMLPath(element,true)):"");
    emit linkChanged();
  }

  void LinkOfReferenceWidget::setLink(const QString &str, Link *linkPtr) {
    selectedLink = linkPtr;
    link->setText(str);
    emit linkChanged();
  }

  QString LinkOfReferenceWidget::getLink() const {
    return link->text();
  }

  ConstraintOfReferenceWidget::ConstraintOfReferenceWidget(Element *element_, Constraint* selectedConstraint_) : element(element_), selectedConstraint(selectedConstraint_) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    constraint = new QLineEdit;
    if(selectedConstraint)
      constraint->setText(QString::fromStdString(selectedConstraint->getXMLPath(element,true)));
    constraintBrowser = new ConstraintBrowser(element->getRoot(),0,this);
    connect(constraintBrowser,SIGNAL(accepted()),this,SLOT(setConstraint()));
    layout->addWidget(constraint);
    QPushButton *button = new QPushButton(tr("Browse"));
    connect(button,SIGNAL(clicked(bool)),constraintBrowser,SLOT(show()));
    layout->addWidget(button);
  }

  void ConstraintOfReferenceWidget::updateWidget() {
    constraintBrowser->updateWidget(selectedConstraint);
    if(selectedConstraint) {
      setConstraint();
    }
  }

  void ConstraintOfReferenceWidget::setConstraint() {
    if(constraintBrowser->getConstraintList()->currentItem())
      selectedConstraint = static_cast<Constraint*>(static_cast<ElementItem*>(constraintBrowser->getConstraintList()->currentItem())->getElement());
    else
      selectedConstraint = 0;
    constraint->setText(selectedConstraint?QString::fromStdString(selectedConstraint->getXMLPath(element,true)):"");
    emit constraintChanged();
  }

  void ConstraintOfReferenceWidget::setConstraint(const QString &str, Constraint *constraintPtr) {
    selectedConstraint = constraintPtr;
    constraint->setText(str);
    emit constraintChanged();
  }

  QString ConstraintOfReferenceWidget::getConstraint() const {
    return constraint->text();
  }

  SignalOfReferenceWidget::SignalOfReferenceWidget(Element *element_, Signal* selectedSignal_) : element(element_), selectedSignal(selectedSignal_) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    signal = new QLineEdit;
    if(selectedSignal)
      signal->setText(QString::fromStdString(selectedSignal->getXMLPath(element,true)));
    signalBrowser = new SignalBrowser(element->getRoot(),0,this);
    connect(signalBrowser,SIGNAL(accepted()),this,SLOT(setSignal()));
    layout->addWidget(signal);
    QPushButton *button = new QPushButton(tr("Browse"));
    connect(button,SIGNAL(clicked(bool)),signalBrowser,SLOT(show()));
    layout->addWidget(button);
  }

  void SignalOfReferenceWidget::updateWidget() {
    signalBrowser->updateWidget(selectedSignal); 
    if(selectedSignal) {
      setSignal();
    }
  }

  void SignalOfReferenceWidget::setSignal() {
    if(signalBrowser->getSignalList()->currentItem())
      selectedSignal = static_cast<Signal*>(static_cast<ElementItem*>(signalBrowser->getSignalList()->currentItem())->getElement());
    else
      selectedSignal = 0;
    signal->setText(selectedSignal?QString::fromStdString(selectedSignal->getXMLPath(element,true)):"");
    emit signalChanged();
  }

  void SignalOfReferenceWidget::setSignal(const QString &str, Signal *signalPtr) {
    selectedSignal = signalPtr;
    signal->setText(str);
    emit signalChanged();
  }

  QString SignalOfReferenceWidget::getSignal() const {
    return signal->text();
  }

  FileWidget::FileWidget(const QString &description_, const QString &extensions_, int mode_) : description(description_), extensions(extensions_), mode(mode_) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    relativeFilePath = new QLineEdit;
    layout->addWidget(relativeFilePath);
    QPushButton *button = new QPushButton("Browse");
    layout->addWidget(button);
    connect(button,SIGNAL(clicked(bool)),this,SLOT(selectFile()));
    connect(relativeFilePath,SIGNAL(textChanged(const QString&)),this,SIGNAL(fileChanged(const QString&)));
  }

  void FileWidget::selectFile() {
    QString file = getFile();
    if(mode<3)
      file = file.mid(1,file.length()-2);
    if(mode==0) 
      file = QFileDialog::getOpenFileName(0, description, file, extensions);
    else if(mode==1)
      file = QFileDialog::getSaveFileName(0, description, file, extensions);
    else
      file = QFileDialog::getExistingDirectory ( 0, description, file);
    if(file!="") {
      if(mode==3)
        setFile(mbsDir.relativeFilePath(file));
      else
        setFile(QString("'")+mbsDir.relativeFilePath(file)+"'");
    }
  }

  DOMElement* FileWidget::initializeUsingXML(DOMElement *parent) {
    DOMText *text = E(parent)->getFirstTextChild();
    if(text) {
      setFile(QString::fromStdString(X()%text->getData()));
      return parent;
    }
    return 0;
  }

  DOMElement* FileWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0 = static_cast<DOMElement*>(parent);
    string fileName = getFile().toStdString();
    if(true) {
      QFileInfo fileInfo = QString::fromStdString(fileName.substr(1,fileName.length()-2));
      fileName = string("\"")+fileInfo.absoluteFilePath().toStdString()+"\"";
    }
    DOMText *text = doc->createTextNode(X()%fileName);
    ele0->insertBefore(text, NULL);
    return 0;
  }

  SpinBoxWidget::SpinBoxWidget(int val, int min, int max) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    value = new QSpinBox;
    value->setValue(val);
    value->setMinimum(min);
    value->setMaximum(max);
    layout->addWidget(value);
    connect(value,SIGNAL(valueChanged(int)),this,SIGNAL(valueChanged(int)));
  }

  ComboBoxWidget::ComboBoxWidget(const QStringList &names, int currentIndex) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    value = new CustomComboBox;
    value->addItems(names);
    value->setCurrentIndex(currentIndex);
    layout->addWidget(value);
    connect(value,SIGNAL(currentIndexChanged(int)),this,SIGNAL(valueChanged(int)));
  }

  TextWidget::TextWidget(const QString &text_, bool readOnly) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    text = new QLineEdit;
    text->setText(text_);
    text->setReadOnly(readOnly);
    layout->addWidget(text);
  }

  TextChoiceWidget::TextChoiceWidget(const vector<QString> &list, int num, bool editable) { 
    text = new CustomComboBox;
    text->setEditable(editable);
    for(unsigned int i=0; i<list.size(); i++)
      text->addItem(list[i]);
    text->setCurrentIndex(num);
    QHBoxLayout* layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    layout->addWidget(text);
  }

  ConnectFramesWidget::ConnectFramesWidget(int n, Element *element_) : element(element_) {

    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    for(int i=0; i<n; i++) {
      QString subname = "Frame";
      if(n>1) {
        subname += QString::number(i+1);
        //layout->addWidget(new QLabel(QString("Frame") + QString::number(i+1) +":"));
      }
      widget.push_back(new FrameOfReferenceWidget(element,0));
      QWidget *subwidget = new ExtWidget(subname,widget[i]);
      layout->addWidget(subwidget);
    }
  }

  void ConnectFramesWidget::updateWidget() {
    for(unsigned int i=0; i<widget.size(); i++)
      widget[i]->updateWidget();
  }

  ConnectContoursWidget::ConnectContoursWidget(int n, Element *element_) : element(element_) {

    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    for(int i=0; i<n; i++) {
      QString subname = "Contour";
      if(n>1) {
        subname += QString::number(i+1);
        //layout->addWidget(new QLabel(QString("Contour") + QString::number(i+1) +":"));
      }
      widget.push_back(new ContourOfReferenceWidget(element,0));
      QWidget *subwidget = new ExtWidget(subname,widget[i]);
      layout->addWidget(subwidget);
    }
  }

  void ConnectContoursWidget::updateWidget() {
    for(unsigned int i=0; i<widget.size(); i++)
      widget[i]->updateWidget();
  }

  ConnectRigidBodiesWidget::ConnectRigidBodiesWidget(int n, Element *element_) : element(element_) {

    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    for(int i=0; i<n; i++) {
      QString subname = "Frame";
      if(n>1) {
        subname += QString::number(i+1);
        //layout->addWidget(new QLabel(QString("Frame") + QString::number(i+1) +":"));
      }
      widget.push_back(new RigidBodyOfReferenceWidget(element,0));
      QWidget *subwidget = new ExtWidget(subname,widget[i]);
      layout->addWidget(subwidget);
      connect(widget[widget.size()-1],SIGNAL(resize_()),this,SIGNAL(resize_()));
    }
  }

  void ConnectRigidBodiesWidget::updateWidget() {
    for(unsigned int i=0; i<widget.size(); i++)
      widget[i]->updateWidget();
  }

  DynamicSystemSolverTolerancesWidget::DynamicSystemSolverTolerancesWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    layout->setMargin(0);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1e-15"), noUnitUnits(), 1));
    projection = new ExtWidget("Projection",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(projection);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1e-8"), noUnitUnits(), 1));
    g = new ExtWidget("g",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(g);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1e-10"), noUnitUnits(), 1));
    gd = new ExtWidget("gd",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(gd);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1e-12"), noUnitUnits(), 1));
    gdd = new ExtWidget("gdd",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(gdd);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1e-12"), noUnitUnits(), 1));
    la = new ExtWidget("la",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(la);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1e-10"), noUnitUnits(), 1));
    La = new ExtWidget("La",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(La);
  }

  DynamicSystemSolverParametersWidget::DynamicSystemSolverParametersWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    vector<QString> list;
    list.push_back("\"FixedPointSingle\"");
    list.push_back("\"GaussSeidel\"");
    list.push_back("\"LinearEquations\"");
    list.push_back("\"RootFinding\"");
    constraintSolver = new ExtWidget("Constraint solver",new TextChoiceWidget(list,1,true),true);
    layout->addWidget(constraintSolver);

    impactSolver = new ExtWidget("Imapct solver",new TextChoiceWidget(list,1,true),true);
    layout->addWidget(impactSolver);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("10000"), QStringList(), 0));
    numberOfMaximalIterations = new ExtWidget("Number of maximal iterations",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(numberOfMaximalIterations);

    tolerances = new ExtWidget("Tolerances",new DynamicSystemSolverTolerancesWidget,true);
    layout->addWidget(tolerances);
  }

  EmbedWidget::EmbedWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    layout->setMargin(0);
    href = new ExtWidget("File", new FileWidget("XML model files", "xml files (*.xml)", 1), true);
    layout->addWidget(href);
    count = new ExtWidget("Count",new PhysicalVariableWidget(new ScalarWidget("1")), true);
    layout->addWidget(count);
    counterName = new ExtWidget("Counter name", new TextWidget("n"), true);
    layout->addWidget(counterName);
    parameterList = new ExtWidget("Parameter file", new FileWidget("XML parameter files", "xml files (*.xml)", 1), true);
    layout->addWidget(parameterList);
  }

  SignalReferenceWidget::SignalReferenceWidget(Element *element) {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    layout->setMargin(0);

    refSignal = new SignalOfReferenceWidget(element,0);
    layout->addWidget(refSignal);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget, QStringList(), 1));
    factor = new ExtWidget("Factor",new ExtPhysicalVarWidget(input));
    layout->addWidget(factor);
  }

  ColorWidget::ColorWidget() {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    vector<QString> vec(3);
    vec[0] = "0.666667"; vec[1] = "1"; vec[2] = "1";
    color = new ExtWidget("HSV",new ChoiceWidget2(new VecWidgetFactory(3,vector<QStringList>(3,QStringList())),QBoxLayout::RightToLeft));
    static_cast<VecWidget*>(static_cast<ChoiceWidget2*>(color->getWidget())->getWidget())->setVec(vec);
    layout->addWidget(color);

    button = new QPushButton(tr("Select"));
    connect(button,SIGNAL(clicked(bool)),this,SLOT(setColor()));
    layout->addWidget(button);
    updateWidget();
  }

  void ColorWidget::updateWidget() { 
    QString val = static_cast<ExtPhysicalVarWidget*>(color->getWidget())->getValue();
    vector<QString> vec = strToVec(val);
    button->setPalette(QPalette(QColor::fromHsvF(vec[0].toDouble(),vec[1].toDouble(),vec[2].toDouble())));
  }

  void ColorWidget::setColor() { 
    QString val = static_cast<ExtPhysicalVarWidget*>(color->getWidget())->getValue();
    vector<QString> vec = strToVec(val);
    QColor col;
    if(vec.size()==3)
      col = QColorDialog::getColor(QColor::fromHsvF(vec[0].toDouble(),vec[1].toDouble(),vec[2].toDouble()));
    else
      col = QColorDialog::getColor(Qt::blue);
    if(col.isValid()) {
      QString str = "[" + QString::number(col.hueF()) + ";" + QString::number(col.saturationF()) + ";" + QString::number(col.valueF()) + "]";
      static_cast<ExtPhysicalVarWidget*>(color->getWidget())->setValue(str);
      updateWidget();
    }
  }

  DOMElement* ColorWidget::initializeUsingXML(DOMElement *parent) {
 //   DOMElement *e = E(parent)->getFirstElementChildNamed(xmlName);
    color->initializeUsingXML(parent);
    return parent;
  }

  DOMElement* ColorWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
//    DOMDocument *doc=parent->getOwnerDocument();
//    DOMElement *ele = D(doc)->createElement(xmlName);
    color->writeXMLFile(parent);
//    parent->insertBefore(ele, NULL);
    return 0;
  }
 
  PlotFeatureStatusWidget::PlotFeatureStatusWidget(const vector<MBXMLUtils::FQN> &types) {
    QGridLayout *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);

    QStringList type_;
    for(unsigned int i=0; i<types.size(); i++)
      type_ << QString::fromStdString(types[i].second);
    if(type_.isEmpty()) {
      type_ << "plotFeature";
      type_ << "plotFeatureForChildren";
      type_ << "plotFeatureRecursive";
    }

    tree = new QTreeWidget;
    QStringList labels;
    labels << "Type" << "Value" << "Status";
    tree->setHeaderLabels(labels);
    layout->addWidget(tree,0,0,3,1);

    type = new CustomComboBox;
    type->addItems(type_);
    layout->addWidget(type,0,1);

    value = new CustomComboBox;
    value->setEditable(true);
    layout->addWidget(value,1,1);

    status = new CustomComboBox;
    status->setEditable(true);
    status->insertItem(0,"+");
    status->insertItem(1,"-");
    layout->addWidget(status,2,1);

    QPushButton *add = new QPushButton("Add");
    connect(add,SIGNAL(pressed()),this,SLOT(addFeature()));
    layout->addWidget(add,3,0);

    QPushButton *remove = new QPushButton("Remove");
    connect(remove,SIGNAL(pressed()),this,SLOT(removeFeature()));
    layout->addWidget(remove,3,1);

    connect(type,SIGNAL(currentIndexChanged(int)),this,SLOT(updateFeature()));
    connect(value,SIGNAL(currentIndexChanged(int)),this,SLOT(updateFeature()));
    connect(status,SIGNAL(currentIndexChanged(int)),this,SLOT(updateFeature()));
    connect(tree,SIGNAL(currentItemChanged(QTreeWidgetItem*,QTreeWidgetItem*)),this,SLOT(currentItemChanged(QTreeWidgetItem*,QTreeWidgetItem*)));
  }

  void PlotFeatureStatusWidget::addFeature(const QString &feature) {
    value->addItem(feature);
  }

  void PlotFeatureStatusWidget::addFeature() {
    QTreeWidgetItem *item = new QTreeWidgetItem;
    item->setText(0, type->currentText());
    item->setText(1, value->currentText());
    item->setText(2, status->currentText());
    tree->addTopLevelItem(item);
  }

  void PlotFeatureStatusWidget::removeFeature() {
    tree->takeTopLevelItem(tree->indexOfTopLevelItem(tree->currentItem()));
  }

  void PlotFeatureStatusWidget::updateFeature() {
    QTreeWidgetItem *item = tree->currentItem();
    if(item) {
      item->setText(0, type->currentText());
      item->setText(1, value->currentText());
      item->setText(2, status->currentText());
    }
  }

  void PlotFeatureStatusWidget::currentItemChanged(QTreeWidgetItem *item, QTreeWidgetItem *prev) {
    if(item) {
      type->blockSignals(true);
      type->setCurrentIndex(type->findText(item->text(0)));
      type->blockSignals(false);
      value->setEditText(item->text(1));
      status->setEditText(item->text(2));
    }
  }

}
