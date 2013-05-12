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
#include "rigidbody.h"
#include "frame.h"
#include "contour.h"
#include "group.h"
#include "signal_.h"
#include "dialogs.h"
#include <QtGui>

using namespace std;

extern bool absolutePath;
extern QDir mbsDir;

LocalFrameOfReferenceWidget::LocalFrameOfReferenceWidget(Element *element_, Frame* omitFrame_) : element(element_), selectedFrame(0), omitFrame(omitFrame_) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  frame = new QComboBox;
  frame->setEditable(true);
  layout->addWidget(frame);
  selectedFrame = element->getFrame(0);
  connect(frame,SIGNAL(currentIndexChanged(const QString&)),this,SLOT(setFrame(const QString&)));
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

void LocalFrameOfReferenceWidget::setFrame(const QString &str, Frame *framePtr) {
  selectedFrame = framePtr;
  frame->setEditText(str);
}

void LocalFrameOfReferenceWidget::setFrame(const QString &str) {
  selectedFrame = element->getFrame(str.mid(6, str.length()-7).toStdString());
  frame->setEditText(str);
}

QString LocalFrameOfReferenceWidget::getFrame() const {
  return frame->currentText();
}

ParentFrameOfReferenceWidget::ParentFrameOfReferenceWidget(Element *element_, Frame* omitFrame_) : element(element_), selectedFrame(0), omitFrame(omitFrame_) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  frame = new QComboBox;
  frame->setEditable(true);
  layout->addWidget(frame);
  selectedFrame = element->getParent()->getFrame(0);
  connect(frame,SIGNAL(currentIndexChanged(const QString&)),this,SLOT(setFrame(const QString&)));
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

void ParentFrameOfReferenceWidget::setFrame(const QString &str, Frame *framePtr) {
  selectedFrame = framePtr;
  frame->setEditText(str);
}

void ParentFrameOfReferenceWidget::setFrame(const QString &str) {
  selectedFrame = element->getParent()->getFrame(str.mid(9, str.length()-10).toStdString());
  frame->setEditText(str);
}

QString ParentFrameOfReferenceWidget::getFrame() const {
  return frame->currentText();
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
    selectedFrame = element->getParent()->getFrame(0);
  frame->setText(QString::fromStdString(selectedFrame->getXMLPath(element,true)));
}

void FrameOfReferenceWidget::setFrame(const QString &str, Frame *framePtr) {
  selectedFrame = framePtr; 
  frame->setText(str);
}

QString FrameOfReferenceWidget::getFrame() const {
  return frame->text();
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
    selectedContour = ((Group*)element->getParent())->getContour(0);
  contour->setText(QString::fromStdString(selectedContour->getXMLPath(element,true)));
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
}

void RigidBodyOfReferenceWidget::setBody(const QString &str, RigidBody *bodyPtr) {
  selectedBody = bodyPtr;
  body->setText(str);
  emit bodyChanged();
}

QString RigidBodyOfReferenceWidget::getBody() const {
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
  relativeFilePath->setReadOnly(true);
  layout->addWidget(relativeFilePath);
  QPushButton *button = new QPushButton("Browse");
  layout->addWidget(button);
  connect(button,SIGNAL(clicked(bool)),this,SLOT(selectFile()));
  connect(relativeFilePath,SIGNAL(textChanged(const QString&)),this,SIGNAL(fileChanged(const QString&)));
}

void FileWidget::setFile(const QString &str) {
  file = str;
  relativeFilePath->setText(mbsDir.relativeFilePath(file));
}

void FileWidget::selectFile() {
  QString file;
  if(mode==0) 
    file = QFileDialog::getOpenFileName(0, description, getFile(), extensions);
  else
    file = QFileDialog::getSaveFileName(0, description, getFile(), extensions);
  if(file!="")
    setFile(file);
}

TextWidget::TextWidget(bool readOnly) {
  QHBoxLayout *layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  text = new QLineEdit;
  text->setReadOnly(readOnly);
  layout->addWidget(text);
}

TextChoiceWidget::TextChoiceWidget(const vector<QString> &list_, int num) : list(list_) { 
  text = new QComboBox;
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

DependenciesWidget::DependenciesWidget(Element *element_) : element(element_) {
  QHBoxLayout *layout = new QHBoxLayout;
  setLayout(layout);
  layout->setMargin(0);
  bodyList = new QListWidget;
  bodyList->setContextMenuPolicy (Qt::CustomContextMenu);
  bodyList->setMinimumWidth(bodyList->sizeHint().width()/3);
  bodyList->setMaximumWidth(bodyList->sizeHint().width()/3);
  layout->addWidget(bodyList);
  stackedWidget = new QStackedWidget;
  //connect(bodyList,SIGNAL(currentRowChanged(int)),this,SLOT(changeCurrent(int)));
  connect(bodyList,SIGNAL(currentRowChanged(int)),stackedWidget,SLOT(setCurrentIndex(int)));
  connect(bodyList,SIGNAL(customContextMenuRequested(const QPoint &)),this,SLOT(openContextMenu(const QPoint &)));
  connect(this,SIGNAL(bodyChanged()),this,SLOT(updateGeneralizedCoordinatesOfBodies()));
  layout->addWidget(stackedWidget,0,Qt::AlignTop);
}

void DependenciesWidget::openContextMenu(const QPoint &pos) {
 if(bodyList->itemAt(pos)) {
   QMenu menu(this);
   QAction *add = new QAction(tr("Remove"), this);
   connect(add, SIGNAL(triggered()), this, SLOT(removeDependency()));
   menu.addAction(add);
   menu.exec(QCursor::pos());
 }
 else {
   QMenu menu(this);
   QAction *add = new QAction(tr("Add"), this);
   connect(add, SIGNAL(triggered()), this, SLOT(addDependency()));
   menu.addAction(add);
   menu.exec(QCursor::pos());
 }
}

void DependenciesWidget::setNumberOfBodies(int n) {
  if(refBody.size() != n) {
    for(unsigned int i=0; i<refBody.size(); i++)
      stackedWidget->removeWidget(refBody[i]);
    selectedBody.clear();
    refBody.clear();
    bodyList->clear();
    for(unsigned int i=0; i<n; i++) {
      selectedBody.push_back(0);
      refBody.push_back(new RigidBodyOfReferenceWidget(element,0));
      connect(refBody[i],SIGNAL(bodyChanged()),this,SLOT(updateList()));
      bodyList->addItem("Undefined");
      stackedWidget->addWidget(refBody[i]);
    }
  }
}

void DependenciesWidget::updateWidget() {
  for(unsigned int i=0; i<refBody.size(); i++)
    refBody[i]->updateWidget();
}

void DependenciesWidget::updateGeneralizedCoordinatesOfBodies() {
  for(unsigned int i=0; i<refBody.size(); i++) {
    if(selectedBody[i])
      selectedBody[i]->setConstrained(false);
    selectedBody[i] = refBody[i]->getSelectedBody();
    if(selectedBody[i])
      selectedBody[i]->setConstrained(true);
  }
}

void DependenciesWidget::updateList() {
  emit bodyChanged();
  for(int i=0; i<bodyList->count(); i++)
    if(refBody[i]->getSelectedBody())
      bodyList->item(i)->setText(QString::fromStdString(refBody[i]->getSelectedBody()->getName()));
}

void DependenciesWidget::addDependency() {
  int i = refBody.size();
  selectedBody.push_back(0);
  refBody.push_back(new RigidBodyOfReferenceWidget(element,0));
  connect(refBody[i],SIGNAL(bodyChanged()),this,SLOT(updateList()));
  bodyList->addItem("Undefined");
  stackedWidget->addWidget(refBody[i]);
  updateWidget();
}

void DependenciesWidget::removeDependency() {
  int i = bodyList->currentRow();
  if(selectedBody[i])
    selectedBody[i]->setConstrained(false);
  selectedBody.pop_back();

  stackedWidget->removeWidget(refBody[i]);
  delete refBody[i];
  refBody.erase(refBody.begin()+i);
  delete bodyList->takeItem(i);

  updateList();
}

SolverTolerancesWidget::SolverTolerancesWidget() {
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

SolverParametersWidget::SolverParametersWidget() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  tolerances = new ExtWidget("Tolerances",new SolverTolerancesWidget,true);
  layout->addWidget(tolerances);
}

PlotFeature::PlotFeature(const QString &name_) : name(name_) {
  QHBoxLayout *layout = new QHBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  status = new QComboBox;
  status->addItem("enabled");
  status->addItem("disabled");
  layout->addWidget(status);
}

GearDependencyWidget::GearDependencyWidget(Element *element) {
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  layout->setMargin(0);

  refBody = new RigidBodyOfReferenceWidget(element,0);
  layout->addWidget(refBody);

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), QStringList(), 1));
  ratio = new ExtWidget("Transmission ratio",new ExtPhysicalVarWidget(input));
  layout->addWidget(ratio);
}

GearDependenciesWidget::GearDependenciesWidget(Element *element_) : element(element_) {
  QHBoxLayout *layout = new QHBoxLayout;
  setLayout(layout);
  layout->setMargin(0);
  bodyList = new QListWidget;
  bodyList->setContextMenuPolicy (Qt::CustomContextMenu);
  bodyList->setMinimumWidth(bodyList->sizeHint().width()/3);
  bodyList->setMaximumWidth(bodyList->sizeHint().width()/3);
  layout->addWidget(bodyList);
  stackedWidget = new QStackedWidget;
  //connect(bodyList,SIGNAL(currentRowChanged(int)),this,SLOT(changeCurrent(int)));
  connect(bodyList,SIGNAL(currentRowChanged(int)),stackedWidget,SLOT(setCurrentIndex(int)));
  connect(bodyList,SIGNAL(customContextMenuRequested(const QPoint &)),this,SLOT(openContextMenu(const QPoint &)));
  layout->addWidget(stackedWidget,0,Qt::AlignTop);
}

void GearDependenciesWidget::openContextMenu(const QPoint &pos) {
 if(bodyList->itemAt(pos)) {
   QMenu menu(this);
   QAction *add = new QAction(tr("Remove"), this);
   connect(add, SIGNAL(triggered()), this, SLOT(removeDependency()));
   menu.addAction(add);
   menu.exec(QCursor::pos());
 }
 else {
   QMenu menu(this);
   QAction *add = new QAction(tr("Add"), this);
   connect(add, SIGNAL(triggered()), this, SLOT(addDependency()));
   menu.addAction(add);
   menu.exec(QCursor::pos());
 }
}

void GearDependenciesWidget::setNumberOfBodies(int n) {
  if(refBody.size() != n) {
    for(unsigned int i=0; i<refBody.size(); i++)
      stackedWidget->removeWidget(refBody[i]);
    selectedBody.clear();
    refBody.clear();
    bodyList->clear();
    for(unsigned int i=0; i<n; i++) {
      selectedBody.push_back(0);
      refBody.push_back(new GearDependencyWidget(element));
      connect(refBody[i]->getRigidBodyOfReferenceWidget(),SIGNAL(bodyChanged()),this,SLOT(updateList()));
      bodyList->addItem("Undefined");
      stackedWidget->addWidget(refBody[i]);
    }
  }
}

void GearDependenciesWidget::updateWidget() {
  for(unsigned int i=0; i<refBody.size(); i++)
    refBody[i]->updateWidget();
}

void GearDependenciesWidget::updateList() {
  emit bodyChanged();
  for(int i=0; i<bodyList->count(); i++)
    if(refBody[i]->getSelectedBody())
      bodyList->item(i)->setText(QString::fromStdString(refBody[i]->getSelectedBody()->getName()));
}

void GearDependenciesWidget::addDependency() {
  int i = refBody.size();
  selectedBody.push_back(0);
  refBody.push_back(new GearDependencyWidget(element));
  connect(refBody[i]->getRigidBodyOfReferenceWidget(),SIGNAL(bodyChanged()),this,SLOT(updateList()));
  bodyList->addItem("Undefined");
  stackedWidget->addWidget(refBody[i]);
  updateWidget();
}

void GearDependenciesWidget::removeDependency() {
  int i = bodyList->currentRow();
  if(selectedBody[i])
    selectedBody[i]->setConstrained(false);
  selectedBody.pop_back();

  stackedWidget->removeWidget(refBody[i]);
  delete refBody[i];
  refBody.erase(refBody.begin()+i);
  delete bodyList->takeItem(i);

  updateList();
}

EmbedWidget::EmbedWidget() {
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  layout->setMargin(0);
  href = new ExtWidget("File", new FileWidget("XML model files", "xml files (*.xml)", 1), true);
  layout->addWidget(href);
  count = new ExtWidget("Count", new TextWidget, true);
  layout->addWidget(count);
  counterName = new ExtWidget("Counter name", new TextWidget, true);
  layout->addWidget(counterName);
  parameterList = new ExtWidget("Parameter file", new FileWidget("XML parameter files", "xml files (*.mbsimparam.xml)"), true);
  layout->addWidget(parameterList);
}

SignalReferenceWidget::SignalReferenceWidget(Element *element) {
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  layout->setMargin(0);

  refSignal = new SignalOfReferenceWidget(element,0);
  layout->addWidget(refSignal);

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), QStringList(), 1));
  factor = new ExtWidget("Factor",new ExtPhysicalVarWidget(input));
  layout->addWidget(factor);
}

SignalReferencesWidget::SignalReferencesWidget(Element *element_) : element(element_) {
  QHBoxLayout *layout = new QHBoxLayout;
  setLayout(layout);
  layout->setMargin(0);
  signalList = new QListWidget;
  signalList->setContextMenuPolicy (Qt::CustomContextMenu);
  signalList->setMinimumWidth(signalList->sizeHint().width()/3);
  signalList->setMaximumWidth(signalList->sizeHint().width()/3);
  layout->addWidget(signalList);
  stackedWidget = new QStackedWidget;
  //connect(signalList,SIGNAL(currentRowChanged(int)),this,SLOT(changeCurrent(int)));
  connect(signalList,SIGNAL(currentRowChanged(int)),stackedWidget,SLOT(setCurrentIndex(int)));
  connect(signalList,SIGNAL(customContextMenuRequested(const QPoint &)),this,SLOT(openContextMenu(const QPoint &)));
  layout->addWidget(stackedWidget,0,Qt::AlignTop);
}

void SignalReferencesWidget::openContextMenu(const QPoint &pos) {
 if(signalList->itemAt(pos)) {
   QMenu menu(this);
   QAction *add = new QAction(tr("Remove"), this);
   connect(add, SIGNAL(triggered()), this, SLOT(removeReference()));
   menu.addAction(add);
   menu.exec(QCursor::pos());
 }
 else {
   QMenu menu(this);
   QAction *add = new QAction(tr("Add"), this);
   connect(add, SIGNAL(triggered()), this, SLOT(addReference()));
   menu.addAction(add);
   menu.exec(QCursor::pos());
 }
}

void SignalReferencesWidget::setNumberOfSignals(int n) {
  if(refSignal.size() != n) {
    for(unsigned int i=0; i<refSignal.size(); i++)
      stackedWidget->removeWidget(refSignal[i]);
    selectedSignal.clear();
    refSignal.clear();
    signalList->clear();
    for(unsigned int i=0; i<n; i++) {
      selectedSignal.push_back(0);
      refSignal.push_back(new SignalReferenceWidget(element));
      connect(refSignal[i]->getSignalOfReferenceWidget(),SIGNAL(signalChanged()),this,SLOT(updateList()));
      signalList->addItem("Undefined");
      stackedWidget->addWidget(refSignal[i]);
    }
  }
}

void SignalReferencesWidget::updateWidget() {
  for(unsigned int i=0; i<refSignal.size(); i++)
    refSignal[i]->updateWidget();
}

void SignalReferencesWidget::updateList() {
  for(int i=0; i<signalList->count(); i++)
    if(refSignal[i]->getSelectedSignal())
      signalList->item(i)->setText(QString::fromStdString(refSignal[i]->getSelectedSignal()->getName()));
}

void SignalReferencesWidget::addReference() {
  int i = refSignal.size();
  selectedSignal.push_back(0);
  refSignal.push_back(new SignalReferenceWidget(element));
  connect(refSignal[i]->getSignalOfReferenceWidget(),SIGNAL(signalChanged()),this,SLOT(updateList()));
  signalList->addItem("Undefined");
  stackedWidget->addWidget(refSignal[i]);
  updateWidget();
}

void SignalReferencesWidget::removeReference() {
  int i = signalList->currentRow();
  selectedSignal.pop_back();

  stackedWidget->removeWidget(refSignal[i]);
  delete refSignal[i];
  refSignal.erase(refSignal.begin()+i);
  delete signalList->takeItem(i);

  updateList();
}


