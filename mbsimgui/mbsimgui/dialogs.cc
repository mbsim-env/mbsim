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
#include "dialogs.h"
#include "frame.h"
#include "contour.h"
#include "group.h"
#include "rigidbody.h"
#include "extra_dynamic.h"
#include "signal_.h"
#include <QVBoxLayout>
#include <QDialogButtonBox>
#include <QTreeWidget>
#include <QScrollArea>

EvalDialog::EvalDialog() {
  var = new MatWidget(0,0);
  QScrollArea *tab = new QScrollArea;
  tab->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  tab->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  tab->setWidgetResizable(true);

  var->setReadOnly(true);
  tab->setWidget(var);

  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  layout->addWidget(tab);
  QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
  buttonBox->addButton(QDialogButtonBox::Close);
  connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

  layout->addWidget(buttonBox);
  setWindowTitle("Octave expression evaluation");
}

RigidBodyBrowser::RigidBodyBrowser(Element* element_, RigidBody* rigidBody, QWidget *parentObject_) : QDialog(parentObject_), selection(rigidBody), savedItem(0), element(element_) {
  QGridLayout* mainLayout=new QGridLayout;
  setLayout(mainLayout);
  rigidBodyList = new QTreeWidget;
  rigidBodyList->setColumnCount(1);
  mainLayout->addWidget(rigidBodyList,0,0);
  QObject::connect(rigidBodyList, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(checkForRigidBody(QTreeWidgetItem*,int)));

  okButton = new QPushButton("Ok");
  if(!selection)
    okButton->setDisabled(true);
  mainLayout->addWidget(okButton,1,0);
  connect(okButton, SIGNAL(clicked(bool)), this, SLOT(accept()));

  QPushButton *button = new QPushButton("Cancel");
  mainLayout->addWidget(button,1,1);
  connect(button, SIGNAL(clicked(bool)), this, SLOT(reject()));

  setWindowTitle("RigidBody browser");
}

void RigidBodyBrowser::updateWidget(RigidBody *sel) {
  selection = sel;
  rigidBodyList->clear();
  savedItem = 0;
  mbs2RigidBodyTree(element,rigidBodyList->invisibleRootItem());
  rigidBodyList->setCurrentItem(savedItem);
}

void RigidBodyBrowser::mbs2RigidBodyTree(Element* ele, QTreeWidgetItem* parentItem) {
  if(dynamic_cast<Group*>(ele) || dynamic_cast<RigidBody*>(ele)) {

    ElementItem *item = new ElementItem(ele);
    item->setText(0,QString::fromStdString(ele->getName()));

    if(ele == selection)
      savedItem = item;

    parentItem->addChild(item);

    for(int i=0; i<ele->getNumberOfGroups(); i++)
      mbs2RigidBodyTree(ele->getGroup(i),item);
    for(int i=0; i<ele->getNumberOfObjects(); i++)
      mbs2RigidBodyTree(ele->getObject(i),item);
  }
}

void RigidBodyBrowser::checkForRigidBody(QTreeWidgetItem* item_,int) {
  ElementItem* item = static_cast<ElementItem*>(item_);
  if(dynamic_cast<RigidBody*>(item->getElement()))
    okButton->setDisabled(false);
  else
    okButton->setDisabled(true);
}

ObjectBrowser::ObjectBrowser(Element* element_, Object* object, QWidget *parentObject_) : QDialog(parentObject_), selection(object), savedItem(0), element(element_) {
  QGridLayout* mainLayout=new QGridLayout;
  setLayout(mainLayout);
  objectList = new QTreeWidget;
  objectList->setColumnCount(1);
  mainLayout->addWidget(objectList,0,0);
  QObject::connect(objectList, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(checkForObject(QTreeWidgetItem*,int)));

  okButton = new QPushButton("Ok");
  if(!selection)
    okButton->setDisabled(true);
  mainLayout->addWidget(okButton,1,0);
  connect(okButton, SIGNAL(clicked(bool)), this, SLOT(accept()));

  QPushButton *button = new QPushButton("Cancel");
  mainLayout->addWidget(button,1,1);
  connect(button, SIGNAL(clicked(bool)), this, SLOT(reject()));

  setWindowTitle("Object browser");
}

void ObjectBrowser::updateWidget(Object *sel) {
  selection = sel;
  objectList->clear();
  savedItem = 0;
  mbs2ObjectTree(element,objectList->invisibleRootItem());
  objectList->setCurrentItem(savedItem);
}

void ObjectBrowser::mbs2ObjectTree(Element* ele, QTreeWidgetItem* parentItem) {
  if(dynamic_cast<Group*>(ele) || dynamic_cast<Object*>(ele)) {

    ElementItem *item = new ElementItem(ele);
    item->setText(0,QString::fromStdString(ele->getName()));

    if(ele == selection)
      savedItem = item;

    parentItem->addChild(item);

    for(int i=0; i<ele->getNumberOfGroups(); i++)
      mbs2ObjectTree(ele->getGroup(i),item);
    for(int i=0; i<ele->getNumberOfObjects(); i++)
      mbs2ObjectTree(ele->getObject(i),item);
  }
}

void ObjectBrowser::checkForObject(QTreeWidgetItem* item_,int) {
  ElementItem* item = static_cast<ElementItem*>(item_);
  if(dynamic_cast<Object*>(item->getElement()))
    okButton->setDisabled(false);
  else
    okButton->setDisabled(true);
}

FrameBrowser::FrameBrowser(Element* element_, Frame* frame, QWidget *parentObject_) : QDialog(parentObject_), selection(frame), savedItem(0), element(element_) {
  QGridLayout* mainLayout=new QGridLayout;
  setLayout(mainLayout);
  frameList = new QTreeWidget;
  frameList->setColumnCount(1);
  mainLayout->addWidget(frameList,0,0);
  QObject::connect(frameList, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(checkForFrame(QTreeWidgetItem*,int)));

  okButton = new QPushButton("Ok");
  if(!selection)
    okButton->setDisabled(true);
  mainLayout->addWidget(okButton,1,0);
  connect(okButton, SIGNAL(clicked(bool)), this, SLOT(accept()));

  QPushButton *button = new QPushButton("Cancel");
  mainLayout->addWidget(button,1,1);
  connect(button, SIGNAL(clicked(bool)), this, SLOT(reject()));

  setWindowTitle("Frame browser");
}

void FrameBrowser::updateWidget(Frame *sel) {
  selection = sel;
  frameList->clear();
  savedItem = 0;
  mbs2FrameTree(element,frameList->invisibleRootItem());
  frameList->setCurrentItem(savedItem);
}

void FrameBrowser::mbs2FrameTree(Element* ele, QTreeWidgetItem* parentItem) {
  if(dynamic_cast<Group*>(ele) || dynamic_cast<Object*>(ele) || dynamic_cast<Frame*>(ele)) {

    ElementItem *item = new ElementItem(ele);
    item->setText(0,QString::fromStdString(ele->getName()));

    if(ele == selection)
      savedItem = item;

    parentItem->addChild(item);

    for(int i=0; i<ele->getNumberOfFrames(); i++)
      mbs2FrameTree(ele->getFrame(i),item);
    for(int i=0; i<ele->getNumberOfGroups(); i++)
      mbs2FrameTree(ele->getGroup(i),item);
    for(int i=0; i<ele->getNumberOfObjects(); i++)
      mbs2FrameTree(ele->getObject(i),item);
  }
}

void FrameBrowser::checkForFrame(QTreeWidgetItem* item_,int) {
  ElementItem* item = static_cast<ElementItem*>(item_);
  if(dynamic_cast<Frame*>(item->getElement()))
    okButton->setDisabled(false);
  else
    okButton->setDisabled(true);
}

ContourBrowser::ContourBrowser(Element* element_, Contour* contour, QWidget *parentObject_) : QDialog(parentObject_), selection(contour), savedItem(0), element(element_) {
  QGridLayout* mainLayout=new QGridLayout;
  setLayout(mainLayout);
  contourList = new QTreeWidget;
  contourList->setColumnCount(1);
  mainLayout->addWidget(contourList,0,0);
  QObject::connect(contourList, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(checkForContour(QTreeWidgetItem*,int)));

  okButton = new QPushButton("Ok");
  if(!selection)
    okButton->setDisabled(true);
  mainLayout->addWidget(okButton,1,0);
  connect(okButton, SIGNAL(clicked(bool)), this, SLOT(accept()));

  QPushButton *button = new QPushButton("Cancel");
  mainLayout->addWidget(button,1,1);
  connect(button, SIGNAL(clicked(bool)), this, SLOT(reject()));

  setWindowTitle("Contour browser");
}

void ContourBrowser::updateWidget(Contour *sel) {
  selection = sel;
  contourList->clear();
  savedItem = 0;
  mbs2ContourTree(element,contourList->invisibleRootItem());
  contourList->setCurrentItem(savedItem);
}

void ContourBrowser::mbs2ContourTree(Element* ele, QTreeWidgetItem* parentItem) {
  if(dynamic_cast<Group*>(ele) || dynamic_cast<Object*>(ele)  || dynamic_cast<Contour*>(ele)) {

    ElementItem *item = new ElementItem(ele);
    item->setText(0,QString::fromStdString(ele->getName()));

    if(ele == selection)
      savedItem = item;

    parentItem->addChild(item);

    for(int i=0; i<ele->getNumberOfContours(); i++)
      mbs2ContourTree(ele->getContour(i),item);
    for(int i=0; i<ele->getNumberOfGroups(); i++)
      mbs2ContourTree(ele->getGroup(i),item);
    for(int i=0; i<ele->getNumberOfObjects(); i++)
      mbs2ContourTree(ele->getObject(i),item);
  }
}

void ContourBrowser::checkForContour(QTreeWidgetItem* item_,int) {
  ElementItem* item = static_cast<ElementItem*>(item_);
  if(dynamic_cast<Contour*>(item->getElement()))
    okButton->setDisabled(false);
  else
    okButton->setDisabled(true);
}

SignalBrowser::SignalBrowser(Element* element_, Signal* signal, QWidget *parentSignal_) : QDialog(parentSignal_), selection(signal), savedItem(0), element(element_) {
  QGridLayout* mainLayout=new QGridLayout;
  setLayout(mainLayout);
  signalList = new QTreeWidget;
  signalList->setColumnCount(1);
  mainLayout->addWidget(signalList,0,0);
  QObject::connect(signalList, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(checkForSignal(QTreeWidgetItem*,int)));

  okButton = new QPushButton("Ok");
  if(!selection)
    okButton->setDisabled(true);
  mainLayout->addWidget(okButton,1,0);
  connect(okButton, SIGNAL(clicked(bool)), this, SLOT(accept()));

  QPushButton *button = new QPushButton("Cancel");
  mainLayout->addWidget(button,1,1);
  connect(button, SIGNAL(clicked(bool)), this, SLOT(reject()));

  setWindowTitle("Signal browser");
}

void SignalBrowser::updateWidget(Signal *sel) {
  selection = sel;
  signalList->clear();
  savedItem = 0;
  mbs2SignalTree(element,signalList->invisibleRootItem());
  signalList->setCurrentItem(savedItem);
}

void SignalBrowser::mbs2SignalTree(Element* ele, QTreeWidgetItem* parentItem) {
  if(dynamic_cast<Group*>(ele) || dynamic_cast<Signal*>(ele)) {

    ElementItem *item = new ElementItem(ele);
    item->setText(0,QString::fromStdString(ele->getName()));

    if(ele == selection)
      savedItem = item;

    parentItem->addChild(item);

    for(int i=0; i<ele->getNumberOfGroups(); i++)
      mbs2SignalTree(ele->getGroup(i),item);
    for(int i=0; i<ele->getNumberOfLinks(); i++)
      mbs2SignalTree(ele->getLink(i),item);
  }
}

void SignalBrowser::checkForSignal(QTreeWidgetItem* item_,int) {
  ElementItem* item = static_cast<ElementItem*>(item_);
  if(dynamic_cast<Signal*>(item->getElement()))
    okButton->setDisabled(false);
  else
    okButton->setDisabled(true);
}

ExtraDynamicBrowser::ExtraDynamicBrowser(Element* element_, ExtraDynamic* ed, QWidget *parentExtraDynamic_) : QDialog(parentExtraDynamic_), selection(ed), savedItem(0), element(element_) {
  QGridLayout* mainLayout=new QGridLayout;
  setLayout(mainLayout);
  edList = new QTreeWidget;
  edList->setColumnCount(1);
  mainLayout->addWidget(edList,0,0);
  QObject::connect(edList, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(checkForExtraDynamic(QTreeWidgetItem*,int)));

  okButton = new QPushButton("Ok");
  if(!selection)
    okButton->setDisabled(true);
  mainLayout->addWidget(okButton,1,0);
  connect(okButton, SIGNAL(clicked(bool)), this, SLOT(accept()));

  QPushButton *button = new QPushButton("Cancel");
  mainLayout->addWidget(button,1,1);
  connect(button, SIGNAL(clicked(bool)), this, SLOT(reject()));

  setWindowTitle("ExtraDynamic browser");
}

void ExtraDynamicBrowser::updateWidget(ExtraDynamic *sel) {
  selection = sel;
  edList->clear();
  savedItem = 0;
  mbs2ExtraDynamicTree(element,edList->invisibleRootItem());
  edList->setCurrentItem(savedItem);
}

void ExtraDynamicBrowser::mbs2ExtraDynamicTree(Element* ele, QTreeWidgetItem* parentItem) {
  if(dynamic_cast<Group*>(ele) || dynamic_cast<ExtraDynamic*>(ele)) {

    ElementItem *item = new ElementItem(ele);
    item->setText(0,QString::fromStdString(ele->getName()));

    if(ele == selection)
      savedItem = item;

    parentItem->addChild(item);

    for(int i=0; i<ele->getNumberOfGroups(); i++)
      mbs2ExtraDynamicTree(ele->getGroup(i),item);
    for(int i=0; i<ele->getNumberOfExtraDynamics(); i++)
      mbs2ExtraDynamicTree(ele->getExtraDynamic(i),item);
  }
}

void ExtraDynamicBrowser::checkForExtraDynamic(QTreeWidgetItem* item_,int) {
  ElementItem* item = static_cast<ElementItem*>(item_);
  if(dynamic_cast<ExtraDynamic*>(item->getElement()))
    okButton->setDisabled(false);
  else
    okButton->setDisabled(true);
}
