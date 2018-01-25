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
#include "utils.h"
#include "variable_widgets.h"
#include "mainwindow.h"
#include <QLabel>
#include <QFileDialog>
#include <QColorDialog>
#include <boost/lexical_cast.hpp>
#include <utility>
#include <xercesc/dom/DOMDocument.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

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

  LocalFrameOfReferenceWidget::LocalFrameOfReferenceWidget(Element *element_, Frame* omitFrame_) : element(element_), selectedFrame(nullptr), omitFrame(omitFrame_) {
    auto *layout = new QVBoxLayout;
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
    return nullptr;
  }

  ParentFrameOfReferenceWidget::ParentFrameOfReferenceWidget(Element *element_, Frame* omitFrame_) : element(element_), selectedFrame(nullptr), omitFrame(omitFrame_) {
    auto *layout = new QVBoxLayout;
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
    return nullptr;
  }

  FrameOfReferenceWidget::FrameOfReferenceWidget(Element *element_, Frame* selectedFrame_) : element(element_), selectedFrame(selectedFrame_) {
    auto *layout = new QHBoxLayout;
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
      selectedFrame = nullptr;
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
    return nullptr;
  }

  ContourOfReferenceWidget::ContourOfReferenceWidget(Element *element_, Contour* selectedContour_) : element(element_), selectedContour(selectedContour_) {
    auto *layout = new QHBoxLayout;
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
      selectedContour = nullptr;
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
    return nullptr;
  }

  RigidBodyOfReferenceWidget::RigidBodyOfReferenceWidget(Element *element_, RigidBody* selectedBody_) : element(element_), selectedBody(selectedBody_) {
    auto *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    body = new QLineEdit;
    if(selectedBody)
      body->setText(selectedBody->getXMLPath(element,true));
    bodyBrowser = new RigidBodyBrowser(element->getRoot(),nullptr,this);
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
      selectedBody = nullptr;
    body->setText(selectedBody?selectedBody->getXMLPath(element,true):"");
    emit bodyChanged();
    emit Widget::updateWidget();
  }

  void RigidBodyOfReferenceWidget::setBody(const QString &str) {
    selectedBody = element->getByPath<RigidBody>(str);
    bodyBrowser->updateWidget(selectedBody);
    body->setText(str);
    emit bodyChanged();
    emit Widget::updateWidget();
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
    return nullptr;
  }

  GearInputReferenceWidget::GearInputReferenceWidget(Element *element_, RigidBody* selectedBody_) : element(element_), selectedBody(selectedBody_) {
    auto *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    body = new QLineEdit;
    if(selectedBody)
      body->setText(selectedBody->getXMLPath(element,true));
    bodyBrowser = new RigidBodyBrowser(element->getRoot(),nullptr,this);
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
      selectedBody = nullptr;
    body->setText(selectedBody?selectedBody->getXMLPath(element,true):"");
    emit bodyChanged();
  }

  void GearInputReferenceWidget::setBody(const QString &str) {
    selectedBody = element->getByPath<RigidBody>(str);
    bodyBrowser->updateWidget(selectedBody);
    body->setText(str);
    emit bodyChanged();
//    emit Widget::updateWidget();
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
    return nullptr;
  }

  ObjectOfReferenceWidget::ObjectOfReferenceWidget(Element *element_, Object* selectedObject_) : element(element_), selectedObject(selectedObject_) {
    auto *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    object = new QLineEdit;
    if(selectedObject)
      object->setText(selectedObject->getXMLPath(element,true));
    objectBrowser = new ObjectBrowser(element->getRoot(),nullptr,this);
    connect(objectBrowser,SIGNAL(accepted()),this,SLOT(setObject()));
    layout->addWidget(object);
    QPushButton *button = new QPushButton(tr("Browse"));
    connect(button,SIGNAL(clicked(bool)),objectBrowser,SLOT(show()));
    layout->addWidget(button);
    updateWidget();
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
      selectedObject = nullptr;
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
    return nullptr;
  }

  LinkOfReferenceWidget::LinkOfReferenceWidget(Element *element_, Link* selectedLink_) : element(element_), selectedLink(selectedLink_) {
    auto *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    link = new QLineEdit;
    if(selectedLink)
      link->setText(selectedLink->getXMLPath(element,true));
    linkBrowser = new LinkBrowser(element->getRoot(),nullptr,this);
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
      selectedLink = nullptr;
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
    return nullptr;
  }

  ConstraintOfReferenceWidget::ConstraintOfReferenceWidget(Element *element_, Constraint* selectedConstraint_) : element(element_), selectedConstraint(selectedConstraint_) {
    auto *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    constraint = new QLineEdit;
    if(selectedConstraint)
      constraint->setText(selectedConstraint->getXMLPath(element,true));
    constraintBrowser = new ConstraintBrowser(element->getRoot(),nullptr,this);
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
      selectedConstraint = nullptr;
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
    return nullptr;
  }

  SignalOfReferenceWidget::SignalOfReferenceWidget(Element *element_, Signal* selectedSignal_) : element(element_), selectedSignal(selectedSignal_) {
    auto *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    signal = new QLineEdit;
    if(selectedSignal)
      signal->setText(selectedSignal->getXMLPath(element,true));
    signalBrowser = new SignalBrowser(element->getRoot(),nullptr,this);
    connect(signalBrowser,SIGNAL(accepted()),this,SLOT(setSignal()));
    layout->addWidget(signal);
    QPushButton *button = new QPushButton(tr("Browse"));
    connect(button,SIGNAL(clicked(bool)),signalBrowser,SLOT(show()));
    layout->addWidget(button);
    updateWidget();
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
      selectedSignal = nullptr;
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
    return nullptr;
  }

  FileWidget::FileWidget(const QString &file, const QString &description_, const QString &extensions_, int mode_, bool quote_) : description(description_), extensions(extensions_), mode(mode_), quote(quote_) {
    auto *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    filePath = new QLineEdit;
    layout->addWidget(filePath);
    QPushButton *button = new QPushButton("Browse");
    layout->addWidget(button);
    connect(button,SIGNAL(clicked(bool)),this,SLOT(selectFile()));
    path = new QCheckBox;
    layout->addWidget(new QLabel("Absolute"));
    layout->addWidget(path);
    setFile(file);
    connect(path,SIGNAL(stateChanged(int)),this,SLOT(changePath(int)));
  }

  void FileWidget::setFile(const QString &str) {
    filePath->setText(str);
    QString file = quote?str.mid(1,str.length()-2):str;
    path->setChecked(file[0]=='/'?1:0);
  }

  void FileWidget::selectFile() {
    QString file = getFile();
    if(quote) file = file.mid(1,file.length()-2);
    if(mode==0) 
      file = QFileDialog::getOpenFileName(nullptr, description, file, extensions);
    else if(mode==1)
      file = QFileDialog::getSaveFileName(nullptr, description, file, extensions);
    else
      file = QFileDialog::getExistingDirectory ( nullptr, description, file);
    if(not file.isEmpty()) {
      if(path->isChecked())
        filePath->setText(quote?("\""+mbsDir.absoluteFilePath(file)+"\""):mbsDir.absoluteFilePath(file));
      else
        filePath->setText(quote?("\""+mbsDir.relativeFilePath(file)+"\""):mbsDir.relativeFilePath(file));
    }
  }

  DOMElement* FileWidget::initializeUsingXML(DOMElement *parent) {
    DOMText *text = E(parent)->getFirstTextChild();
    if(text) {
      setFile(QString::fromStdString(X()%text->getData()));
      return parent;
    }
    return nullptr;
  }

  DOMElement* FileWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    auto *ele0 = static_cast<DOMElement*>(parent);
    //DOMText *text = doc->createTextNode(X()%(quote?("\""+getFile().toStdString()+"\""):getFile().toStdString()));
    DOMText *text = doc->createTextNode(X()%getFile().toStdString());
    ele0->insertBefore(text, nullptr);
    return nullptr;
  }

  void FileWidget::changePath(int i) {
    QString file = quote?getFile().mid(1,getFile().length()-2):getFile();
    if(i)
      filePath->setText(quote?("\""+mbsDir.absoluteFilePath(file)+"\""):mbsDir.absoluteFilePath(file));
    else
      filePath->setText(quote?("\""+mbsDir.relativeFilePath(file)+"\""):mbsDir.relativeFilePath(file));
  }

  SpinBoxWidget::SpinBoxWidget(int val, int min, int max) {
    auto *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    value = new CustomSpinBox;
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
    return nullptr;
  }

  DOMElement* SpinBoxWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMText *text= doc->createTextNode(X()%toStr(getValue()));
    parent->insertBefore(text, nullptr);
    return nullptr;
  }

  ComboBoxWidget::ComboBoxWidget(const QStringList &names, int currentIndex) {
    auto *layout = new QHBoxLayout;
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
      setText(QString::fromStdString(X()%text_->getData()));
      return element;
    }
    return nullptr;
  }

  DOMElement* BasicTextWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMText *text_ = doc->createTextNode(X()%getText().toStdString());
    parent->insertBefore(text_, nullptr);
    return nullptr;
  }

  TextWidget::TextWidget(const QString &text_, bool readOnly) {
    auto *layout = new QHBoxLayout;
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
    for(const auto & i : list)
      text->addItem(i);
    text->setCurrentIndex(num);
    auto* layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    layout->addWidget(text);
  }

  ConnectFramesWidget::ConnectFramesWidget(int n, Element *element_) : element(element_) {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    for(int i=0; i<n; i++) {
      QString subname = "Frame";
      if(n>1) {
        subname += QString::number(i+1);
        //layout->addWidget(new QLabel(QString("Frame") + QString::number(i+1) +":"));
      }
      widget.push_back(new FrameOfReferenceWidget(element,nullptr));
      QWidget *subwidget = new ExtWidget(subname,widget[i]);
      layout->addWidget(subwidget);
    }
  }

  void ConnectFramesWidget::updateWidget() {
    for(auto & i : widget)
      i->updateWidget();
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
    return nullptr;
  }

  ConnectContoursWidget::ConnectContoursWidget(int n, Element *element_) : element(element_) {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    for(int i=0; i<n; i++) {
      QString subname = "Contour";
      if(n>1) {
        subname += QString::number(i+1);
        //layout->addWidget(new QLabel(QString("Contour") + QString::number(i+1) +":"));
      }
      widget.push_back(new ContourOfReferenceWidget(element,nullptr));
      QWidget *subwidget = new ExtWidget(subname,widget[i]);
      layout->addWidget(subwidget);
    }
  }

  void ConnectContoursWidget::updateWidget() {
    for(auto & i : widget)
      i->updateWidget();
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
    return nullptr;
  }

  ConnectRigidBodiesWidget::ConnectRigidBodiesWidget(int n, Element *element_) : element(element_) {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    for(int i=0; i<n; i++) {
      QString subname = "Frame";
      if(n>1) {
        subname += QString::number(i+1);
        //layout->addWidget(new QLabel(QString("Frame") + QString::number(i+1) +":"));
      }
      widget.push_back(new RigidBodyOfReferenceWidget(element,nullptr));
      QWidget *subwidget = new ExtWidget(subname,widget[i]);
      layout->addWidget(subwidget);
    }
  }

  void ConnectRigidBodiesWidget::updateWidget() {
    for(auto & i : widget)
      i->updateWidget();
  }

  DOMElement* ConnectRigidBodiesWidget::initializeUsingXML(DOMElement *element) {
    for(unsigned int i=0; i<widget.size(); i++) {
      string xmlName = "ref";
      if(widget.size()>1)
        xmlName += toStr(int(i+1));
      if(!E(element)->hasAttribute(xmlName))
        return nullptr;
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
    return nullptr;
  }

  ColorWidget::ColorWidget() {
    auto *layout = new QHBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    vector<QString> vec(3);
    vec[0] = "0.666667"; vec[1] = "1"; vec[2] = "1";
    color = new ExtWidget("HSV",new ChoiceWidget2(new VecWidgetFactory(vec),QBoxLayout::RightToLeft,5),false,false,"");
    layout->addWidget(color);

    button = new QPushButton(tr("Select"));
    connect(button,SIGNAL(clicked(bool)),this,SLOT(setColor()));
    layout->addWidget(button);
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
    }
  }

  DOMElement* ColorWidget::initializeUsingXML(DOMElement *parent) {
    color->initializeUsingXML(parent);
    return parent;
  }

  DOMElement* ColorWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    color->writeXMLFile(parent);
    return nullptr;
  }
 
  PlotFeatureWidget::PlotFeatureWidget(const QString &types, NamespaceURI uri_) : uri(std::move(uri_)) {
    auto *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);

    feature.push_back(MBSIM%"plotRecursive");
    feature.push_back(MBSIM%"separateFilePerGroup");
    feature.push_back(MBSIM%"openMBV");
    feature.push_back(MBSIM%"debug");
    feature.push_back(MBSIM%"position");
    feature.push_back(MBSIM%"angle");
    feature.push_back(MBSIM%"velocity");
    feature.push_back(MBSIM%"angularVelocity");
    feature.push_back(MBSIM%"acceleration");
    feature.push_back(MBSIM%"angularAcceleration");
    feature.push_back(MBSIM%"generalizedPosition");
    feature.push_back(MBSIM%"generalizedVelocity");
    feature.push_back(MBSIM%"derivativeOfGeneralizedPosition");
    feature.push_back(MBSIM%"generalizedAcceleration");
    feature.push_back(MBSIM%"generalizedRelativePosition");
    feature.push_back(MBSIM%"generalizedRelativeVelocity");
    feature.push_back(MBSIM%"generalizedForce");
    feature.push_back(MBSIM%"energy");
    feature.push_back(MBSIMCONTROL%"signal");

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
    labels << "Type" << "Value" << "Status" << "Namespace";
    tree->setHeaderLabels(labels);
    layout->addWidget(tree,0,0,1,3);
    tree->setColumnWidth(0,200);
    tree->setColumnWidth(1,150);
    tree->setColumnWidth(2,50);
    tree->setColumnWidth(3,250);

    layout->addWidget(new QLabel("Type:"),3,0);
    type = new CustomComboBox;
    type->addItems(type_);
    layout->addWidget(type,3,1);
    type->setCurrentIndex(2);

    layout->addWidget(new QLabel("Value:"),4,0);
    value = new CustomComboBox;
    value->setEditable(true);
    layout->addWidget(value,4,1);
    for(auto & i : feature)
      value->addItem(QString::fromStdString(i.second));
    value->setCurrentIndex(4);
    connect(value,SIGNAL(currentIndexChanged(int)),this,SLOT(updateNamespace(int)));

    layout->addWidget(new QLabel("Namespace:"),6,0);
    nspace = new CustomComboBox;
    nspace->setEditable(true);
    layout->addWidget(nspace,6,1);

    layout->addWidget(new QLabel("Status:"),5,0);
    status = new ChoiceWidget2(new BoolWidgetFactory("true"),QBoxLayout::RightToLeft,5);
    layout->addWidget(status,5,1);

    nspace->blockSignals(true);
    nspace->addItem(QString::fromStdString(MBSIM.getNamespaceURI()));
    nspace->addItem(QString::fromStdString(MBSIMCONTROL.getNamespaceURI()));
    nspace->blockSignals(false);

    QPushButton *add = new QPushButton("Add");
    connect(add,SIGNAL(pressed()),this,SLOT(addFeature()));
    layout->addWidget(add,3,2);

    QPushButton *remove = new QPushButton("Remove");
    connect(remove,SIGNAL(pressed()),this,SLOT(removeFeature()));
    layout->addWidget(remove,4,2);

    QPushButton *update = new QPushButton("Update");
    connect(update,SIGNAL(pressed()),this,SLOT(updateFeature()));
    layout->addWidget(update,5,2);

    layout->setColumnStretch(1,10);

    connect(tree,SIGNAL(currentItemChanged(QTreeWidgetItem*,QTreeWidgetItem*)),this,SLOT(currentItemChanged(QTreeWidgetItem*,QTreeWidgetItem*)));
  }

  void PlotFeatureWidget::updateNamespace(int i) {
    nspace->setEditText(QString::fromStdString(feature[i].first));
  }

  void PlotFeatureWidget::addFeature(const FQN &feature_) {
    value->blockSignals(true);
    value->addItem(QString::fromStdString(feature_.second));
    value->blockSignals(false);
    feature.push_back(feature_);
  }

  void PlotFeatureWidget::addFeature() {
    auto *item = new QTreeWidgetItem;
    item->setText(0, type->currentText());
    item->setText(1, value->currentText());
    item->setText(2, static_cast<BoolWidget*>(status->getWidget())->getValue());
    item->setText(3, nspace->currentText());
    tree->addTopLevelItem(item);
  }

  void PlotFeatureWidget::removeFeature() {
    tree->takeTopLevelItem(tree->indexOfTopLevelItem(tree->currentItem()));
  }

  void PlotFeatureWidget::updateFeature() {
    QTreeWidgetItem *item = tree->currentItem();
    if(item) {
      item->setText(0, type->currentText());
      item->setText(1, value->currentText());
      item->setText(2, static_cast<BoolWidget*>(status->getWidget())->getValue());
      item->setText(3, nspace->currentText());
    }
  }

  void PlotFeatureWidget::currentItemChanged(QTreeWidgetItem *item, QTreeWidgetItem *prev) {
    if(item) {
      type->blockSignals(true);
      type->setCurrentIndex(type->findText(item->text(0)));
      type->blockSignals(false);
      value->setEditText(item->text(1));
      QString str = item->text(2);
      if(str=="0" or str=="1" or str=="false" or str=="true")
        status->setIndex(0);
      else
        status->setIndex(1);
      static_cast<BoolWidget*>(status->getWidget())->setValue(item->text(2));
      nspace->setEditText(item->text(3));
    }
  }

  DOMElement* PlotFeatureWidget::initializeUsingXML(DOMElement *parent) {
    DOMElement *e=parent->getFirstElementChild();
    while(e && (E(e)->getTagName()==uri%"plotFeature" ||
                E(e)->getTagName()==uri%"plotFeatureForChildren" ||
                E(e)->getTagName()==uri%"plotFeatureRecursive")) {
      auto *item = new QTreeWidgetItem;
      item->setText(0, QString::fromStdString(E(e)->getTagName().second));
      item->setText(1, QString::fromStdString(E(e)->getAttributeQName("value").second));
      item->setText(2, QString::fromStdString(X()%E(e)->getFirstTextChild()->getData()));
      item->setText(3, QString::fromStdString(E(e)->getAttributeQName("value").first));
      tree->addTopLevelItem(item);
      e=e->getNextElementSibling();
    }
    return e;
  }

  DOMElement* PlotFeatureWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    for(size_t i=0; i<tree->topLevelItemCount(); i++) {
      DOMElement *ele = D(doc)->createElement(uri%tree->topLevelItem(i)->text(0).toStdString());
      E(ele)->setAttribute("value",NamespaceURI(tree->topLevelItem(i)->text(3).toStdString())%tree->topLevelItem(i)->text(1).toStdString());
      ele->insertBefore(doc->createTextNode(X()%tree->topLevelItem(i)->text(2).toStdString()), nullptr);
      parent->insertBefore(ele, ref);
    }
    return nullptr;
  }

  DOMElement* PlotFeatureWidget::initializeUsingXML2(DOMElement *parent) {
    DOMElement *e=E(parent)->getFirstElementChildNamed(uri%type->itemText(0).toStdString());
    while(e && E(e)->getTagName()==uri%type->itemText(0).toStdString()) {
      auto *item = new QTreeWidgetItem;
      item->setText(0, QString::fromStdString(E(e)->getTagName().second));
      item->setText(1, QString::fromStdString(E(e)->getAttributeQName("value").second));
      item->setText(2, QString::fromStdString(X()%E(e)->getFirstTextChild()->getData()));
      item->setText(3, QString::fromStdString(E(e)->getAttributeQName("value").first));
      tree->addTopLevelItem(item);
      e=e->getNextElementSibling();
    }
    return e;
  }

  DOMElement* PlotFeatureWidget::writeXMLFile2(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    for(size_t i=0; i<tree->topLevelItemCount(); i++) {
      DOMElement *ele = D(doc)->createElement(uri%tree->topLevelItem(i)->text(0).toStdString());
      E(ele)->setAttribute("value",NamespaceURI(tree->topLevelItem(i)->text(3).toStdString())%tree->topLevelItem(i)->text(1).toStdString());
      ele->insertBefore(doc->createTextNode(X()%tree->topLevelItem(i)->text(2).toStdString()), nullptr);
      parent->insertBefore(ele, ref);
    }
    return nullptr;
  }

}
