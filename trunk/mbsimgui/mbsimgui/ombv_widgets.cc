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
#include "ombv_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "rigidbody.h"
#include "frame.h"
#include <QtGui>

using namespace std;

OMBVFrameWidget::OMBVFrameWidget(const QString &name) : OMBVObjectWidget(name) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), lengthUnits(), 4));
  size = new ExtWidget("Size",new ExtPhysicalVarWidget(input));
  size->setToolTip("Set the size of the frame");
  layout->addWidget(size);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), noUnitUnits(), 1));
  offset = new ExtWidget("Offset",new ExtPhysicalVarWidget(input));
  offset->setToolTip("Set the offset of the frame");
  layout->addWidget(offset);
}

OMBVDynamicColoredObjectWidget::OMBVDynamicColoredObjectWidget(const QString &name) : OMBVObjectWidget(name) {
  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"), noUnitUnits(), 1));
  minimalColorValue = new ExtWidget("Minimal color value",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(minimalColorValue);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), noUnitUnits(), 1));
  maximalColorValue = new ExtWidget("Maximal color value",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(maximalColorValue);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"), noUnitUnits(), 1));
  staticColor = new ExtWidget("Static color",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(staticColor);
}

OMBVArrowWidget::OMBVArrowWidget(const QString &name, bool fromPoint) : OMBVDynamicColoredObjectWidget(name) {

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("0.1"), lengthUnits(), 4));
  diameter = new ExtWidget("Diameter",new ExtPhysicalVarWidget(input));
  layout->addWidget(diameter);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("0.2"), lengthUnits(), 4));
  headDiameter = new ExtWidget("Head diameter",new ExtPhysicalVarWidget(input));
  layout->addWidget(headDiameter);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("0.2"), lengthUnits(), 4));
  headLength = new ExtWidget("Head length",new ExtPhysicalVarWidget(input));
  layout->addWidget(headLength);

  vector<QString> list;
  list.push_back("line");
  list.push_back("fromHead");
  list.push_back("toHead");
  list.push_back("bothHeads");
  list.push_back("formDoubleHead");
  list.push_back("toDoubleHead");
  list.push_back("bothDoubleHeads");
  type = new ExtWidget("Type",new TextChoiceWidget(list,2));
  layout->addWidget(type);

  list.clear();
  list.push_back("toPoint");
  list.push_back("fromPoint");
  list.push_back("midPoint");
  referencePoint = new ExtWidget("Reference point",new TextChoiceWidget(list,fromPoint?1:0),true);
  if(fromPoint)
    referencePoint->setChecked(true);
  layout->addWidget(referencePoint);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), noUnitUnits(), 1));
  scaleLength = new ExtWidget("Scale length",new ExtPhysicalVarWidget(input));
  layout->addWidget(scaleLength);
}

OMBVCoilSpringWidget::OMBVCoilSpringWidget(const QString &name) : OMBVObjectWidget(name) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<QString> list;
  list.push_back("tube");
  list.push_back("scaledTube");
  list.push_back("polyline");
  type = new ExtWidget("Type",new TextChoiceWidget(list,0));
  layout->addWidget(type);

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("3"), noUnitUnits(), 1));
  numberOfCoils= new ExtWidget("Number of coils",new ExtPhysicalVarWidget(input));
  layout->addWidget(numberOfCoils);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("0.1"), lengthUnits(), 4));
  springRadius= new ExtWidget("Spring radius",new ExtPhysicalVarWidget(input));
  layout->addWidget(springRadius);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("-1"), lengthUnits(), 4));
  crossSectionRadius = new ExtWidget("Cross section radius",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(crossSectionRadius);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("-1"), lengthUnits(), 4));
  nominalLength= new ExtWidget("Nominal length",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(nominalLength);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), noUnitUnits(), 1));
  scaleFactor = new ExtWidget("Scale factor",new ExtPhysicalVarWidget(input));
  layout->addWidget(scaleFactor);
}

OMBVBodyWidget::OMBVBodyWidget(const QString &name) : OMBVObjectWidget(name) {
  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"), noUnitUnits(), 1));
  color = new ExtWidget("Static color",new ExtPhysicalVarWidget(input));
  layout->addWidget(color);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new VecWidget(3,true), lengthUnits(), 4));
  trans = new ExtWidget("Initial translation",new ExtPhysicalVarWidget(input));
  layout->addWidget(trans);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new VecWidget(3,true), angleUnits(), 0));
  rot = new ExtWidget("Initial rotation",new ExtPhysicalVarWidget(input));
  layout->addWidget(rot);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), noUnitUnits(), 1));
  scale = new ExtWidget("Scale factor",new ExtPhysicalVarWidget(input));
  layout->addWidget(scale);
}

CubeWidget::CubeWidget(const QString &name) : OMBVBodyWidget(name) {

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), lengthUnits(), 4));
  length = new ExtWidget("Length",new ExtPhysicalVarWidget(input));
  layout->addWidget(length);
}

CuboidWidget::CuboidWidget(const QString &name) : OMBVBodyWidget(name) {

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new VecWidget(getScalars<QString>(3,"1"),true), lengthUnits(), 4));
  length = new ExtWidget("Length",new ExtPhysicalVarWidget(input));
  layout->addWidget(length);
}

SphereWidget::SphereWidget(const QString &name) : OMBVBodyWidget(name) {

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), lengthUnits(), 4));
  radius = new ExtWidget("Radius",new ExtPhysicalVarWidget(input));
  layout->addWidget(radius);
}

FrustumWidget::FrustumWidget(const QString &name) : OMBVBodyWidget(name) {

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), lengthUnits(), 4));
  top = new ExtWidget("Top radius",new ExtPhysicalVarWidget(input));
  layout->addWidget(top);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), lengthUnits(), 4));
  base = new ExtWidget("Base radius",new ExtPhysicalVarWidget(input));
  layout->addWidget(base);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), lengthUnits(), 4));
  height = new ExtWidget("Height",new ExtPhysicalVarWidget(input));
  layout->addWidget(height);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"), lengthUnits(), 4));
  innerTop = new ExtWidget("Inner top radius",new ExtPhysicalVarWidget(input));
  layout->addWidget(innerTop);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"), lengthUnits(), 4));
  innerBase = new ExtWidget("Inner base radius",new ExtPhysicalVarWidget(input));
  layout->addWidget(innerBase);
}

IvBodyWidget::IvBodyWidget(const QString &name) : OMBVBodyWidget(name) {

  ivFileName = new ExtWidget("Iv file name",new FileWidget("XML model files", "iv files (*.iv *.wrl)"));
  layout->addWidget(ivFileName);

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("-1"), angleUnits(), 0));
  creaseEdges = new ExtWidget("Crease edges",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(creaseEdges);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new BoolWidget("0"), QStringList(), 4));
  boundaryEdges = new ExtWidget("Boundary edges",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(boundaryEdges);
}

CompoundRigidBodyWidget::CompoundRigidBodyWidget(const QString &name) : OMBVBodyWidget(name) {
  QGroupBox *box = new QGroupBox("Bodies");
  QHBoxLayout *sublayout = new QHBoxLayout;
  box->setLayout(sublayout);
  layout->addWidget(box);
  bodyList = new QListWidget;
  bodyList->setContextMenuPolicy (Qt::CustomContextMenu);
  bodyList->setMinimumWidth(bodyList->sizeHint().width()/3);
  bodyList->setMaximumWidth(bodyList->sizeHint().width()/3);
  sublayout->addWidget(bodyList);
  stackedWidget = new QStackedWidget;
  connect(bodyList,SIGNAL(currentRowChanged(int)),this,SLOT(changeCurrent(int)));
  //  connect(bodyList,SIGNAL(currentRowChanged(int)),stackedWidget,SLOT(setCurrentIndex(int)));
  connect(bodyList,SIGNAL(customContextMenuRequested(const QPoint &)),this,SLOT(openContextMenu(const QPoint &)));
  sublayout->addWidget(stackedWidget);
}

void CompoundRigidBodyWidget::changeCurrent(int idx) {
  if (stackedWidget->currentWidget() !=0)
    stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->setCurrentIndex(idx);
  stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  adjustSize();
}

void CompoundRigidBodyWidget::openContextMenu(const QPoint &pos) {
  if(bodyList->itemAt(pos)) {
    QMenu menu(this);
    QAction *add = new QAction(tr("Remove"), this);
    connect(add, SIGNAL(triggered()), this, SLOT(removeBody()));
    menu.addAction(add);
    menu.exec(QCursor::pos());
  }
  else {
    QMenu menu(this);
    QAction *add = new QAction(tr("Add"), this);
    connect(add, SIGNAL(triggered()), this, SLOT(addBody()));
    menu.addAction(add);
    menu.exec(QCursor::pos());
  }
}

void CompoundRigidBodyWidget::addBody() {
  int i = body.size();
  body.push_back(new OMBVBodyChoiceWidget((QString("Body")+QString::number(i+1)),false));
  bodyList->addItem((QString("Body")+QString::number(i+1)));
  stackedWidget->addWidget(body[i]);
  //stackedWidget->widget(i)->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
}

void CompoundRigidBodyWidget::removeBody() {
  int i = bodyList->currentRow();

  stackedWidget->removeWidget(body[i]);
  delete body[i];
  body.erase(body.begin()+i);
  delete bodyList->takeItem(i);
  for(int i=0; i<bodyList->count(); i++) {
    bodyList->item(i)->setText((QString("Body")+QString::number(i+1)));
    body[i]->setName(bodyList->item(i)->text());
  }
}

OMBVBodyChoiceWidget::OMBVBodyChoiceWidget(const QString &name_, bool flag) : ombv(0), name(name_) {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  comboBox->addItem(tr("Cube"));
  comboBox->addItem(tr("Cuboid"));
  comboBox->addItem(tr("Frustum"));
  comboBox->addItem(tr("Sphere"));
  comboBox->addItem(tr("IvBody"));
  if(flag)
    comboBox->addItem(tr("CompoundRigidBody"));
  comboBox->addItem(tr("InvisibleBody"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(ombvSelection(int)));
  ombvSelection(0);
}

void OMBVBodyChoiceWidget::ombvSelection(int index) {
  layout->removeWidget(ombv);
  delete ombv;
  //ombv->deleteLater();
  if(index==0)
    ombv = new CubeWidget(name);  
  if(index==1)
    ombv = new CuboidWidget(name);  
  else if(index==2)
    ombv = new FrustumWidget(name);  
  else if(index==3)
    ombv = new SphereWidget(name);  
  else if(index==4)
    ombv = new IvBodyWidget(name);  
  else if(index==5)
    ombv = new CompoundRigidBodyWidget(name);  
  else if(index==6)
    ombv = new InvisibleBodyWidget(name);  
  layout->addWidget(ombv);
  ombv->updateWidget();
}

OMBVBodySelectionWidget::OMBVBodySelectionWidget(RigidBody *body) : ombv(0), ref(0) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  ombv = new OMBVBodyChoiceWidget("NOTSET", true);
  ref=new LocalFrameOfReferenceWidget(body);
  ExtWidget *widget = new ExtWidget("Frame of reference",ref);
  layout->addWidget(ombv);
  layout->addWidget(widget);
}

OMBVEmptyWidget::OMBVEmptyWidget() : OMBVObjectWidget("Empty") {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
}

OMBVPlaneWidget::OMBVPlaneWidget() : OMBVObjectWidget("Plane") {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("0.1"), lengthUnits(), 4));
  size = new ExtWidget("Size",new ExtPhysicalVarWidget(input));
  layout->addWidget(size);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("10"), QStringList(), 0));
  numberOfLines = new ExtWidget("Number of lines",new ExtPhysicalVarWidget(input));
  layout->addWidget(numberOfLines);
}
