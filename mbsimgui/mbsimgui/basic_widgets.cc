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
#include <boost/lexical_cast.hpp>

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
    Frame *selection = element->getFrame(frame.mid(6, frame.length()-7));
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
    Frame *selection = element->getParent()->getFrame(frame.mid(9, frame.length()-10));
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
        frame->addItem("Frame["+element->getFrame(i)->getName()+"]");
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
    selectedFrame = element->getFrame(str.mid(6, str.length()-7));
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
        frame->addItem("../Frame["+element->getParent()->getFrame(i)->getName()+"]");
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
    selectedFrame = element->getParent()->getFrame(str.mid(9, str.length()-10));
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
      frame->setText(selectedFrame->getXMLPath(element,true));
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
    frame->setText(selectedFrame?selectedFrame->getXMLPath(element,true):"");
  }

  void FrameOfReferenceWidget::setFrame(const QString &str) {
    if(str!=def) {
      selectedFrame = element->getByPath<Frame>(str);
      frameBrowser->updateWidget(selectedFrame);
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
      contour->setText(selectedContour->getXMLPath(element,true));
    contourBrowser = new ContourBrowser(element->getRoot(),selectedContour,this);
    connect(contourBrowser,SIGNAL(accepted()),this,SLOT(setContour()));
    layout->addWidget(contour);
    QPushButton *button = new QPushButton(tr("Browse"));
    connect(button,SIGNAL(clicked(bool)),contourBrowser,SLOT(show()));
    layout->addWidget(button);
    updateWidget();
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
    contour->setText(selectedContour?selectedContour->getXMLPath(element,true):"");
  }

  void ContourOfReferenceWidget::setContour(const QString &str) {
    selectedContour = element->getByPath<Contour>(str);
    contourBrowser->updateWidget(selectedContour);
    contour->setText(str);
  }

  QString ContourOfReferenceWidget::getContour() const {
    return contour->text();
  }

  DOMElement* ContourOfReferenceWidget::initializeUsingXML(DOMElement *element) {
    setContour(QString::fromStdString(E(element)->getAttribute("ref")));
    return element;
  }

  DOMElement* ContourOfReferenceWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    E(static_cast<DOMElement*>(parent))->setAttribute("ref", getContour().toStdString());
    return NULL;
  }

  RigidBodyOfReferenceWidget::RigidBodyOfReferenceWidget(Element *element_, RigidBody* selectedBody_) : element(element_), selectedBody(selectedBody_) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    body = new QLineEdit;
    if(selectedBody)
      body->setText(selectedBody->getXMLPath(element,true));
    bodyBrowser = new RigidBodyBrowser(element->getRoot(),0,this);
    connect(bodyBrowser,SIGNAL(accepted()),this,SLOT(setBody()));
    layout->addWidget(body);
    QPushButton *button = new QPushButton(tr("Browse"));
    connect(button,SIGNAL(clicked(bool)),bodyBrowser,SLOT(show()));
    layout->addWidget(button);
    updateWidget();
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
    body->setText(selectedBody?selectedBody->getXMLPath(element,true):"");
    emit bodyChanged();
    emit Widget::resize_();
  }

  void RigidBodyOfReferenceWidget::setBody(const QString &str) {
    selectedBody = element->getByPath<RigidBody>(str);
    bodyBrowser->updateWidget(selectedBody);
    body->setText(str);
    emit bodyChanged();
    emit Widget::resize_();
  }

  QString RigidBodyOfReferenceWidget::getBody() const {
    return body->text();
  }

  DOMElement* RigidBodyOfReferenceWidget::initializeUsingXML(DOMElement *element) {
    setBody(QString::fromStdString(E(element)->getAttribute("ref")));
    return element;
  }

  DOMElement* RigidBodyOfReferenceWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    E(static_cast<DOMElement*>(parent))->setAttribute("ref", getBody().toStdString());
    return NULL;
  }

  GearInputReferenceWidget::GearInputReferenceWidget(Element *element_, RigidBody* selectedBody_) : element(element_), selectedBody(selectedBody_) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    body = new QLineEdit;
    if(selectedBody)
      body->setText(selectedBody->getXMLPath(element,true));
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
    body->setText(selectedBody?selectedBody->getXMLPath(element,true):"");
    emit bodyChanged();
  }

  void GearInputReferenceWidget::setBody(const QString &str) {
    selectedBody = element->getByPath<RigidBody>(str);
    bodyBrowser->updateWidget(selectedBody);
    body->setText(str);
    emit bodyChanged();
//    emit Widget::resize_();
  }

  QString GearInputReferenceWidget::getBody() const {
    return body->text();
  }

  DOMElement* GearInputReferenceWidget::initializeUsingXML(DOMElement *element) {
    setBody(QString::fromStdString(E(element)->getAttribute("ref")));
    setRatio(QString::fromStdString(E(element)->getAttribute("ratio")));
    return element;
  }

  DOMElement* GearInputReferenceWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    E(static_cast<DOMElement*>(parent))->setAttribute("ref", getBody().toStdString());
    E(static_cast<DOMElement*>(parent))->setAttribute("ratio", getRatio().toStdString());
    return NULL;
  }

  ObjectOfReferenceWidget::ObjectOfReferenceWidget(Element *element_, Object* selectedObject_) : element(element_), selectedObject(selectedObject_) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    object = new QLineEdit;
    if(selectedObject)
      object->setText(selectedObject->getXMLPath(element,true));
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
    object->setText(selectedObject?selectedObject->getXMLPath(element,true):"");
    emit objectChanged();
  }

  void ObjectOfReferenceWidget::setObject(const QString &str) {
    selectedObject = element->getByPath<Object>(str);
    objectBrowser->updateWidget(selectedObject);
    object->setText(str);
    emit objectChanged();
  }

  QString ObjectOfReferenceWidget::getObject() const {
    return object->text();
  }

  DOMElement* ObjectOfReferenceWidget::initializeUsingXML(DOMElement *element) {
    setObject(QString::fromStdString(E(element)->getAttribute("ref")));
    return element;
  }

  DOMElement* ObjectOfReferenceWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    E(static_cast<DOMElement*>(parent))->setAttribute("ref", getObject().toStdString());
    return NULL;
  }

  LinkOfReferenceWidget::LinkOfReferenceWidget(Element *element_, Link* selectedLink_) : element(element_), selectedLink(selectedLink_) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    link = new QLineEdit;
    if(selectedLink)
      link->setText(selectedLink->getXMLPath(element,true));
    linkBrowser = new LinkBrowser(element->getRoot(),0,this);
    connect(linkBrowser,SIGNAL(accepted()),this,SLOT(setLink()));
    layout->addWidget(link);
    QPushButton *button = new QPushButton(tr("Browse"));
    connect(button,SIGNAL(clicked(bool)),linkBrowser,SLOT(show()));
    layout->addWidget(button);
    updateWidget();
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
    link->setText(selectedLink?selectedLink->getXMLPath(element,true):"");
    emit linkChanged();
  }

  void LinkOfReferenceWidget::setLink(const QString &str) {
    selectedLink = element->getByPath<Link>(str);
    linkBrowser->updateWidget(selectedLink);
    link->setText(str);
    emit linkChanged();
  }

  QString LinkOfReferenceWidget::getLink() const {
    return link->text();
  }

  DOMElement* LinkOfReferenceWidget::initializeUsingXML(DOMElement *element) {
    setLink(QString::fromStdString(E(element)->getAttribute("ref")));
    return element;
  }

  DOMElement* LinkOfReferenceWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    E(static_cast<DOMElement*>(parent))->setAttribute("ref", getLink().toStdString());
    return NULL;
  }

  ConstraintOfReferenceWidget::ConstraintOfReferenceWidget(Element *element_, Constraint* selectedConstraint_) : element(element_), selectedConstraint(selectedConstraint_) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    constraint = new QLineEdit;
    if(selectedConstraint)
      constraint->setText(selectedConstraint->getXMLPath(element,true));
    constraintBrowser = new ConstraintBrowser(element->getRoot(),0,this);
    connect(constraintBrowser,SIGNAL(accepted()),this,SLOT(setConstraint()));
    layout->addWidget(constraint);
    QPushButton *button = new QPushButton(tr("Browse"));
    connect(button,SIGNAL(clicked(bool)),constraintBrowser,SLOT(show()));
    layout->addWidget(button);
    updateWidget();
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
    constraint->setText(selectedConstraint?selectedConstraint->getXMLPath(element,true):"");
    emit constraintChanged();
  }

  void ConstraintOfReferenceWidget::setConstraint(const QString &str) {
    selectedConstraint = element->getByPath<Constraint>(str);
    constraintBrowser->updateWidget(selectedConstraint);
    constraint->setText(str);
    emit constraintChanged();
  }

  QString ConstraintOfReferenceWidget::getConstraint() const {
    return constraint->text();
  }

  DOMElement* ConstraintOfReferenceWidget::initializeUsingXML(DOMElement *element) {
    setConstraint(QString::fromStdString(E(element)->getAttribute("ref")));
    return element;
  }

  DOMElement* ConstraintOfReferenceWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    E(static_cast<DOMElement*>(parent))->setAttribute("ref", getConstraint().toStdString());
    return NULL;
  }

  SignalOfReferenceWidget::SignalOfReferenceWidget(Element *element_, Signal* selectedSignal_) : element(element_), selectedSignal(selectedSignal_) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    signal = new QLineEdit;
    if(selectedSignal)
      signal->setText(selectedSignal->getXMLPath(element,true));
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
    signal->setText(selectedSignal?selectedSignal->getXMLPath(element,true):"");
    emit signalChanged();
  }

  void SignalOfReferenceWidget::setSignal(const QString &str) {
    selectedSignal = element->getByPath<Signal>(str);
    signalBrowser->updateWidget(selectedSignal);
    signal->setText(str);
    emit signalChanged();
  }

  QString SignalOfReferenceWidget::getSignal() const {
    return signal->text();
  }

  DOMElement* SignalOfReferenceWidget::initializeUsingXML(DOMElement *element) {
    setSignal(QString::fromStdString(E(element)->getAttribute("ref")));
    return element;
  }

  DOMElement* SignalOfReferenceWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    E(static_cast<DOMElement*>(parent))->setAttribute("ref", getSignal().toStdString());
    return NULL;
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
    QString fileName = getFile();
    if(true) {
      QFileInfo fileInfo = fileName.mid(1,fileName.length()-2);
      fileName = QString("\"")+fileInfo.absoluteFilePath()+"\"";
    }
    DOMText *text = doc->createTextNode(X()%fileName.toStdString());
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

  DOMElement* SpinBoxWidget::initializeUsingXML(DOMElement *element) {
    DOMText *text = E(element)->getFirstTextChild();
    if(text) {
      value->blockSignals(true);
      setValue(boost::lexical_cast<int>(X()%text->getData()));
      value->blockSignals(false);
      return element;
    }
    return NULL;
  }

  DOMElement* SpinBoxWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMText *text= doc->createTextNode(X()%toStr(getValue()));
    parent->insertBefore(text, NULL);
    return NULL;
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

  DOMElement* BasicTextWidget::initializeUsingXML(DOMElement *element) {
    DOMText *text_ = E(element)->getFirstTextChild();
    if(text_) {
      QString str = QString::fromStdString(X()%text_->getNodeValue());
      if(quote)
        setText(str.mid(1,str.length()-2));
      else
        setText(str);
      return element;
    }
    return NULL;
  }

  DOMElement* BasicTextWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMText *text_ = doc->createTextNode(X()%(quote?("\""+getText().toStdString()+"\""):getText().toStdString()));
    parent->insertBefore(text_, NULL);
    return NULL;
  }

  TextWidget::TextWidget(const QString &text_, bool readOnly, bool quote) : BasicTextWidget(quote) {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    text = new QLineEdit;
    text->setText(text_);
    text->setReadOnly(readOnly);
    layout->addWidget(text);
  }

  TextChoiceWidget::TextChoiceWidget(const vector<QString> &list, int num, bool editable, bool quote) : BasicTextWidget(quote) {
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

  DOMElement* ConnectFramesWidget::initializeUsingXML(DOMElement *element) {
    for(unsigned int i=0; i<widget.size(); i++) {
      string xmlName = "ref";
      if(widget.size()>1)
        xmlName += toStr(int(i+1));
      if(E(element)->hasAttribute(xmlName))
        widget[i]->setFrame(QString::fromStdString(E(element)->getAttribute(xmlName)));
    }
    return element;
  }

  DOMElement* ConnectFramesWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    for(unsigned int i=0; i<widget.size(); i++) {
      string xmlName = "ref";
      if(widget.size()>1)
        xmlName += toStr(int(i+1));
      if(i>0 or widget[i]->getFrame()!=def)
        E(static_cast<DOMElement*>(parent))->setAttribute(xmlName, widget[i]->getFrame().toStdString());
    }
    return NULL;
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

  DOMElement* ConnectContoursWidget::initializeUsingXML(DOMElement *element) {
    for(unsigned int i=0; i<widget.size(); i++) {
      string xmlName = "ref";
      if(widget.size()>1)
        xmlName += toStr(int(i+1));
      if(E(element)->hasAttribute(xmlName))
        widget[i]->setContour(QString::fromStdString(E(element)->getAttribute(xmlName)));
    }
    return element;
  }

  DOMElement* ConnectContoursWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    for(unsigned int i=0; i<widget.size(); i++) {
      string xmlName = "ref";
      if(widget.size()>1)
        xmlName += toStr(int(i+1));
      E(static_cast<DOMElement*>(parent))->setAttribute(xmlName, widget[i]->getContour().toStdString());
    }
    return NULL;
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

  DOMElement* ConnectRigidBodiesWidget::initializeUsingXML(DOMElement *element) {
    for(unsigned int i=0; i<widget.size(); i++) {
      string xmlName = "ref";
      if(widget.size()>1)
        xmlName += toStr(int(i+1));
      if(!E(element)->hasAttribute(xmlName))
        return NULL;
      if(E(element)->hasAttribute(xmlName))
        widget[i]->setBody(QString::fromStdString(E(element)->getAttribute(xmlName)));
    }
    return element;
  }

  DOMElement* ConnectRigidBodiesWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    for(unsigned int i=0; i<widget.size(); i++) {
      string xmlName = "ref";
      if(widget.size()>1)
        xmlName += toStr(int(i+1));
      E(static_cast<DOMElement*>(parent))->setAttribute(xmlName, widget[i]->getBody().toStdString());
    }
    return NULL;
  }

  DynamicSystemSolverTolerancesWidget::DynamicSystemSolverTolerancesWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    layout->setMargin(0);

    projection = new ExtWidget("Projection",new ChoiceWidget2(new ScalarWidgetFactory("1e-15"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"projection");
    layout->addWidget(projection);

    g = new ExtWidget("g",new ChoiceWidget2(new ScalarWidgetFactory("1e-8"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"g");
    layout->addWidget(g);

    gd = new ExtWidget("gd",new ChoiceWidget2(new ScalarWidgetFactory("1e-10"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"gd");
    layout->addWidget(gd);

    gdd = new ExtWidget("gdd",new ChoiceWidget2(new ScalarWidgetFactory("1e-12"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"gdd");
    layout->addWidget(gdd);

    la = new ExtWidget("la",new ChoiceWidget2(new ScalarWidgetFactory("1e-12"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"la");
    layout->addWidget(la);

    La = new ExtWidget("La",new ChoiceWidget2(new ScalarWidgetFactory("1e-10"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"La");
    layout->addWidget(La);
  }

  DOMElement* DynamicSystemSolverTolerancesWidget::initializeUsingXML(DOMElement *parent) {
    projection->initializeUsingXML(parent);
    g->initializeUsingXML(parent);
    gd->initializeUsingXML(parent);
    gdd->initializeUsingXML(parent);
    la->initializeUsingXML(parent);
    La->initializeUsingXML(parent);
    return NULL;
  }

  DOMElement* DynamicSystemSolverTolerancesWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    projection->writeXMLFile(parent);
    g->writeXMLFile(parent);
    gd->writeXMLFile(parent);
    gdd->writeXMLFile(parent);
    la->writeXMLFile(parent);
    La->writeXMLFile(parent);
    return NULL;
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
    constraintSolver = new ExtWidget("Constraint solver",new TextChoiceWidget(list,1,true),true,false,MBSIM%"constraintSolver");
    layout->addWidget(constraintSolver);

    impactSolver = new ExtWidget("Impact solver",new TextChoiceWidget(list,1,true),true,false,MBSIM%"impactSolver");
    layout->addWidget(impactSolver);

    numberOfMaximalIterations = new ExtWidget("Number of maximal iterations",new ChoiceWidget2(new ScalarWidgetFactory("10000"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"numberOfMaximalIterations");
    layout->addWidget(numberOfMaximalIterations);

    tolerances = new ExtWidget("Tolerances",new DynamicSystemSolverTolerancesWidget,true,false,MBSIM%"tolerances");
    layout->addWidget(tolerances);
  }

  DOMElement* DynamicSystemSolverParametersWidget::initializeUsingXML(DOMElement *parent) {
    constraintSolver->initializeUsingXML(parent);
    impactSolver->initializeUsingXML(parent);
    numberOfMaximalIterations->initializeUsingXML(parent);
    tolerances->initializeUsingXML(parent);
    return NULL;
  }

  DOMElement* DynamicSystemSolverParametersWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    constraintSolver->writeXMLFile(parent);
    impactSolver->writeXMLFile(parent);
    numberOfMaximalIterations->writeXMLFile(parent);
    tolerances->writeXMLFile(parent);
    return NULL;
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

  QString EmbedWidget::getCounterName() const {
    return counterName->isActive()?static_cast<TextWidget*>(counterName->getWidget())->getText():"";
  }

  QString EmbedWidget::getCount() const {
    return count->isActive()?static_cast<PhysicalVariableWidget*>(count->getWidget())->getValue():"";
  }

  DOMElement* EmbedWidget::initializeUsingXML(DOMElement *parent) {
    if(E(parent)->hasAttribute("href")) {
      href->setActive(true);
      static_cast<FileWidget*>(href->getWidget())->setFile(QString::fromStdString(E(parent)->getAttribute("href")));
    }
    if(E(parent)->hasAttribute("count")) {
      count->setActive(true);
      static_cast<PhysicalVariableWidget*>(count->getWidget())->setValue(QString::fromStdString(E(parent)->getAttribute("count")));
    }
    if(E(parent)->hasAttribute("counterName")) {
      counterName->setActive(true);
      static_cast<TextWidget*>(counterName->getWidget())->setText(QString::fromStdString(E(parent)->getAttribute("counterName")));
    }
    if(E(parent)->hasAttribute("parameterHref")) {
      parameterList->setActive(true);
      static_cast<FileWidget*>(parameterList->getWidget())->setFile(QString::fromStdString(E(parent)->getAttribute("parameterHref")));
    }
    return parent;
  }

  DOMElement* EmbedWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    if(href->isActive())
      E(static_cast<DOMElement*>(parent))->setAttribute("href", static_cast<FileWidget*>(href->getWidget())->getFile().toStdString());
    if(count->isActive())
      E(static_cast<DOMElement*>(parent))->setAttribute("count", static_cast<PhysicalVariableWidget*>(count->getWidget())->getValue().toStdString());
    if(counterName->isActive())
      E(static_cast<DOMElement*>(parent))->setAttribute("counterName", static_cast<TextWidget*>(counterName->getWidget())->getText().toStdString());
    if(parameterList->isActive())
      E(static_cast<DOMElement*>(parent))->setAttribute("parameterHref", static_cast<FileWidget*>(parameterList->getWidget())->getFile().toStdString());
    return NULL;
  }

//  SignalReferenceWidget::SignalReferenceWidget(Element *element) {
//    QVBoxLayout *layout = new QVBoxLayout;
//    setLayout(layout);
//    layout->setMargin(0);
//
//    refSignal = new SignalOfReferenceWidget(element,0);
//    layout->addWidget(refSignal);
//
//    vector<PhysicalVariableWidget*> input;
//    input.push_back(new PhysicalVariableWidget(new ScalarWidget, QStringList(), 1));
//    factor = new ExtWidget("Factor",new ExtPhysicalVarWidget(input));
//    layout->addWidget(factor);
//  }

  ColorWidget::ColorWidget() {
    QHBoxLayout *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    vector<QString> vec(3);
    vec[0] = "0.666667"; vec[1] = "1"; vec[2] = "1";
    color = new ExtWidget("HSV",new ChoiceWidget2(new VecWidgetFactory(vec),QBoxLayout::RightToLeft,5),false,false,"");
    layout->addWidget(color);

    button = new QPushButton(tr("Select"));
    connect(button,SIGNAL(clicked(bool)),this,SLOT(setColor()));
    layout->addWidget(button);
    updateWidget();
  }

  void ColorWidget::updateWidget() { 
    QString val = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget2*>(color->getWidget())->getWidget())->getValue();
    vector<QString> vec = strToVec(val);
    button->setPalette(QPalette(QColor::fromHsvF(vec[0].toDouble(),vec[1].toDouble(),vec[2].toDouble())));
  }

  void ColorWidget::setColor() { 
    QString val = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget2*>(color->getWidget())->getWidget())->getValue();
    vector<QString> vec = strToVec(val);
    QColor col;
    if(vec.size()==3)
      col = QColorDialog::getColor(QColor::fromHsvF(vec[0].toDouble(),vec[1].toDouble(),vec[2].toDouble()));
    else
      col = QColorDialog::getColor(Qt::blue);
    if(col.isValid()) {
      QString str = "[" + QString::number(col.hueF()) + ";" + QString::number(col.saturationF()) + ";" + QString::number(col.valueF()) + "]";
      static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget2*>(color->getWidget())->getWidget())->setValue(str);
      updateWidget();
    }
  }

  DOMElement* ColorWidget::initializeUsingXML(DOMElement *parent) {
    color->initializeUsingXML(parent);
    return parent;
  }

  DOMElement* ColorWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    color->writeXMLFile(parent);
    return 0;
  }
 
  PlotFeatureStatusWidget::PlotFeatureStatusWidget(const QString &types, const NamespaceURI &uri_) : uri(uri_) {
    QGridLayout *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);

    QStringList type_;
    if(types.isEmpty()) {
      type_ << "plotFeature";
      type_ << "plotFeatureForChildren";
      type_ << "plotFeatureRecursive";
    }
    else
      type_ << types;

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

  DOMElement* PlotFeatureStatusWidget::initializeUsingXML(DOMElement *parent) {
    DOMElement *e=parent->getFirstElementChild();
    while(e && (E(e)->getTagName()==uri%"plotFeature" ||
                E(e)->getTagName()==uri%"plotFeatureForChildren" ||
                E(e)->getTagName()==uri%"plotFeatureRecursive")) {
      string feature = E(e)->getAttribute("feature");
      QTreeWidgetItem *item = new QTreeWidgetItem;
      item->setText(0, QString::fromStdString(E(e)->getTagName().second));
      item->setText(1, QString::fromStdString(feature.substr(1)));
      item->setText(2, QString::fromStdString(feature.substr(0,1)));
      tree->addTopLevelItem(item);
      e=e->getNextElementSibling();
    }
    return e;
  }

  DOMElement* PlotFeatureStatusWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    for(size_t i=0; i<tree->topLevelItemCount(); i++) {
      DOMElement *ele = D(doc)->createElement(uri%tree->topLevelItem(i)->text(0).toStdString());
      E(ele)->setAttribute("feature",(tree->topLevelItem(i)->text(2)+tree->topLevelItem(i)->text(1)).toStdString());
      parent->insertBefore(ele, ref);
    }
    return 0;
  }

  DOMElement* PlotFeatureStatusWidget::initializeUsingXML2(DOMElement *parent) {
    DOMElement *e=E(parent)->getFirstElementChildNamed(uri%type->itemText(0).toStdString());
    while(e && E(e)->getTagName()==uri%type->itemText(0).toStdString()) {
      string feature = E(e)->getAttribute("feature");
      QTreeWidgetItem *item = new QTreeWidgetItem;
      item->setText(0, QString::fromStdString(E(e)->getTagName().second));
      item->setText(1, QString::fromStdString(feature.substr(1)));
      item->setText(2, QString::fromStdString(feature.substr(0,1)));
      tree->addTopLevelItem(item);
      e=e->getNextElementSibling();
    }
    return e;
  }

  DOMElement* PlotFeatureStatusWidget::writeXMLFile2(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    for(size_t i=0; i<tree->topLevelItemCount(); i++) {
      DOMElement *ele = D(doc)->createElement(uri%tree->topLevelItem(i)->text(0).toStdString());
      E(ele)->setAttribute("feature",(tree->topLevelItem(i)->text(2)+tree->topLevelItem(i)->text(1)).toStdString());
      parent->insertBefore(ele, NULL);
    }
    return 0;
  }

}
