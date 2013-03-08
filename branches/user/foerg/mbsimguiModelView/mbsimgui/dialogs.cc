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
#include "group.h"
#include "rigidbody.h"
#include "frame.h"
#include "contour.h"
#include <QVBoxLayout>
#include <QDialogButtonBox>
#include <QTreeWidget>

EvalDialog::EvalDialog(StringWidget *var_) : var(var_) {
  QScrollArea *tab = new QScrollArea;
  tab->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  tab->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  tab->setWidgetResizable(true);

  var->setReadOnly(true);
  tab->setWidget(var);

  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  layout->addWidget(tab);
//  QWidget *extension = new QWidget;
//  button = new QPushButton(tr("Assign to Schema 1"));
//  button = new QPushButton(QString("Assign to ") + var->getType().c_str());
//  connect(button,SIGNAL(clicked(bool)),this,SIGNAL(clicked(bool)));
//  QVBoxLayout *extensionLayout = new QVBoxLayout;
//  extensionLayout->setMargin(0);
//  extensionLayout->addWidget(button);
//  extension->setLayout(extensionLayout);

//  QPushButton *buttonBox = new QPushButton("Ok");
//  okButton->setDefault(true);
  QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
  buttonBox->addButton(QDialogButtonBox::Close);
//  QPushButton *moreButton = new QPushButton(tr("&More"));
//  moreButton->setCheckable(true);
//  moreButton->setAutoDefault(false);
//  //buttonBox->addButton(moreButton, QDialogButtonBox::ActionRole);
//  connect(moreButton, SIGNAL(toggled(bool)), extension, SLOT(setVisible(bool)));
  connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

  layout->addWidget(buttonBox);
 // layout->addWidget(extension);
 // extension->hide();
  setWindowTitle("Octave expression evaluation");
}

RigidBodyBrowser::RigidBodyBrowser(QTreeWidget* tree_, RigidBody* rigidBody, QWidget *parentObject_) : QDialog(parentObject_), selection(rigidBody), savedItem(0), tree(tree_) {
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
  mbs2RigidBodyTree((Element*)tree->topLevelItem(0),rigidBodyList->invisibleRootItem());
  rigidBodyList->setCurrentItem(savedItem);
}

void RigidBodyBrowser::mbs2RigidBodyTree(Element* ele, QTreeWidgetItem* parentItem) {
  if(dynamic_cast<Group*>(ele) || dynamic_cast<RigidBody*>(ele) || dynamic_cast<RigidBody*>(ele)) {

    ElementItem *item = new ElementItem(ele);
    item->setText(0,ele->getName());

    if(ele == selection)
      savedItem = item;

    parentItem->addChild(item);

    if(ele->getContainerFrame())
      for(int i=0; i<ele->getContainerFrame()->childCount(); i++) {
        mbs2RigidBodyTree((Element*)ele->getContainerFrame()->child(i),item);
      }
    if(ele->getContainerGroup())
      for(int i=0; i<ele->getContainerGroup()->childCount(); i++) {
        mbs2RigidBodyTree((Element*)ele->getContainerGroup()->child(i),item);
      }
    if(ele->getContainerObject())
      for(int i=0; i<ele->getContainerObject()->childCount(); i++) {
        mbs2RigidBodyTree((Element*)ele->getContainerObject()->child(i),item);
      }
  }
}

void RigidBodyBrowser::checkForRigidBody(QTreeWidgetItem* item_,int) {
  ElementItem* item = static_cast<ElementItem*>(item_);
  if(dynamic_cast<RigidBody*>(item->getElement()))
    okButton->setDisabled(false);
  else
    okButton->setDisabled(true);
}

FrameBrowser::FrameBrowser(QTreeWidget* tree_, Frame* frame, QWidget *parentObject_) : QDialog(parentObject_), selection(frame), savedItem(0), tree(tree_) {
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
  mbs2FrameTree((Element*)tree->topLevelItem(0),frameList->invisibleRootItem());
  frameList->setCurrentItem(savedItem);
}

void FrameBrowser::mbs2FrameTree(Element* ele, QTreeWidgetItem* parentItem) {
  if(dynamic_cast<Group*>(ele) || dynamic_cast<RigidBody*>(ele) || dynamic_cast<Frame*>(ele)) {

    ElementItem *item = new ElementItem(ele);
    item->setText(0,ele->getName());

    if(ele == selection)
      savedItem = item;

    parentItem->addChild(item);

    if(ele->getContainerFrame())
      for(int i=0; i<ele->getContainerFrame()->childCount(); i++) {
        mbs2FrameTree((Element*)ele->getContainerFrame()->child(i),item);
      }
    if(ele->getContainerGroup())
      for(int i=0; i<ele->getContainerGroup()->childCount(); i++) {
        mbs2FrameTree((Element*)ele->getContainerGroup()->child(i),item);
      }
    if(ele->getContainerObject())
      for(int i=0; i<ele->getContainerObject()->childCount(); i++) {
        mbs2FrameTree((Element*)ele->getContainerObject()->child(i),item);
      }
  }
}

void FrameBrowser::checkForFrame(QTreeWidgetItem* item_,int) {
  ElementItem* item = static_cast<ElementItem*>(item_);
  if(dynamic_cast<Frame*>(item->getElement()))
    okButton->setDisabled(false);
  else
    okButton->setDisabled(true);
}

ContourBrowser::ContourBrowser(QTreeWidget* tree_, Contour* contour, QWidget *parentObject_) : QDialog(parentObject_), selection(contour), savedItem(0), tree(tree_) {
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
  mbs2ContourTree((Element*)tree->topLevelItem(0),contourList->invisibleRootItem());
  contourList->setCurrentItem(savedItem);
}

void ContourBrowser::mbs2ContourTree(Element* ele, QTreeWidgetItem* parentItem) {
  if(dynamic_cast<Group*>(ele) || dynamic_cast<RigidBody*>(ele) || dynamic_cast<Contour*>(ele)) {

    ElementItem *item = new ElementItem(ele);
    item->setText(0,ele->getName());

    if(ele == selection)
      savedItem = item;

    parentItem->addChild(item);

    if(ele->getContainerContour())
      for(int i=0; i<ele->getContainerContour()->childCount(); i++) {
        mbs2ContourTree((Element*)ele->getContainerContour()->child(i),item);
      }
    if(ele->getContainerGroup())
      for(int i=0; i<ele->getContainerGroup()->childCount(); i++) {
        mbs2ContourTree((Element*)ele->getContainerGroup()->child(i),item);
      }
    if(ele->getContainerObject())
      for(int i=0; i<ele->getContainerObject()->childCount(); i++) {
        mbs2ContourTree((Element*)ele->getContainerObject()->child(i),item);
      }
  }
}

void ContourBrowser::checkForContour(QTreeWidgetItem* item_,int) {
  ElementItem* item = static_cast<ElementItem*>(item_);
  if(dynamic_cast<Contour*>(item->getElement()))
    okButton->setDisabled(false);
  else
    okButton->setDisabled(true);
}
