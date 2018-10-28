/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2016 Martin Förg

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
#include "import_widgets.h"
#include "extended_widgets.h"
#include "basic_widgets.h"
#include <QGridLayout>
#include <QPushButton>
#include <QLabel>
#include <QDir>

using namespace std;

namespace MBSimGUI {

  ImportWidget::ImportWidget() {
    auto *mainlayout = new QVBoxLayout;
    mainlayout->setMargin(0);
    setLayout(mainlayout);

    QGroupBox *groupBox = new QGroupBox("Import from");
    mainlayout->addWidget(groupBox);

    auto *prglayout = new QVBoxLayout;
    prglayout->setMargin(0);
    groupBox->setLayout(prglayout);

    CustomComboBox *choice = new CustomComboBox;
    choice->addItem("CalculiX");
//    choice->addItem("Ansys");
//    choice->addItem("Abaqus");
    prglayout->addWidget(choice);

    groupBox = new QGroupBox("CalculiX result file");
    mainlayout->addWidget(groupBox);

    prglayout = new QVBoxLayout;
    prglayout->setMargin(0);
    groupBox->setLayout(prglayout);

    QDir dir;
    QString name;
    QStringList list = dir.entryList(QStringList("*.frd"));
    if(list.size())
      name = list[0];
    resultFile = new FileWidget(name,"Choose CalculiX result file","**.frd",0,false,false);
    prglayout->addWidget(resultFile);

    groupBox = new QGroupBox("Import");
    mainlayout->addWidget(groupBox);

    auto *layout = new QGridLayout;
    layout->setMargin(0);
    groupBox->setLayout(layout);

    labelMass = new QCheckBox("Mass");
    labelMass->setChecked(true);
    layout->addWidget(labelMass,0,0);

    labelrdm = new QCheckBox("Position integral");
    labelrdm->setChecked(true);
    layout->addWidget(labelrdm,1,0);

    labelrrdm = new QCheckBox("Position position integral");
    labelrrdm->setChecked(true);
    layout->addWidget(labelrrdm,2,0);

    labelPdm = new QCheckBox("Shape function integral");
    labelPdm->setChecked(true);
    layout->addWidget(labelPdm,3,0);
    checkPdm = new QCheckBox("File");
    layout->addWidget(checkPdm,3,1);
    filePdm = new FileWidget("Pdm.asc","Choose shape function integral file","*.asc",1,false,false);
    filePdm->setEnabled(false);
    layout->addWidget(filePdm,3,2);
    connect(labelPdm,SIGNAL(toggled(bool)),this,SLOT(updatePdm()));
    connect(checkPdm,SIGNAL(toggled(bool)),this,SLOT(updatePdm()));

    labelrPdm = new QCheckBox("Position shape function integral");
    labelrPdm->setChecked(true);
    layout->addWidget(labelrPdm,4,0);
    checkrPdm = new QCheckBox("File");
    layout->addWidget(checkrPdm,4,1);
    filerPdm = new FileWidget("rPdm.asc","Choose position shape function integral file","*.asc",1,false,false);
    filerPdm->setEnabled(false);
    layout->addWidget(filerPdm,4,2);
    connect(labelrPdm,SIGNAL(toggled(bool)),this,SLOT(updaterPdm()));
    connect(checkrPdm,SIGNAL(toggled(bool)),this,SLOT(updaterPdm()));

    labelPPdm = new QCheckBox("Shape function shape function integral");
    labelPPdm->setChecked(true);
    layout->addWidget(labelPPdm,5,0);
    checkPPdm = new QCheckBox("File");
    layout->addWidget(checkPPdm,5,1);
    filePPdm = new FileWidget("PPdm.asc","Choose shape function shape function integral file","*.asc",1,false,false);
    filePPdm->setEnabled(false);
    layout->addWidget(filePPdm,5,2);
    connect(labelPPdm,SIGNAL(toggled(bool)),this,SLOT(updatePPdm()));
    connect(checkPPdm,SIGNAL(toggled(bool)),this,SLOT(updatePPdm()));

    labelKe = new QCheckBox("Stiffness matrix");
    labelKe->setChecked(true);
    layout->addWidget(labelKe,6,0);
    checkKe = new QCheckBox("File");
    layout->addWidget(checkKe,6,1);
    fileKe = new FileWidget("Ke.asc","Choose stiffness matrix file","*.asc",1,false,false);
    fileKe->setEnabled(false);
    layout->addWidget(fileKe,6,2);
    connect(labelKe,SIGNAL(toggled(bool)),this,SLOT(updateKe()));
    connect(checkKe,SIGNAL(toggled(bool)),this,SLOT(updateKe()));

    labelu0 = new QCheckBox("Nodal relative position");
    labelu0->setChecked(true);
    layout->addWidget(labelu0,7,0);
    checku0 = new QCheckBox("File");
    checku0->setChecked(true);
    layout->addWidget(checku0,7,1);
    fileu0 = new FileWidget("u0.asc","Choose nodal relative position file","*.asc",1,false,false);
    layout->addWidget(fileu0,7,2);
    connect(labelu0,SIGNAL(toggled(bool)),this,SLOT(updateu0()));
    connect(checku0,SIGNAL(toggled(bool)),this,SLOT(updateu0()));

    labelPhi = new QCheckBox("Nodal shape matrix of translation");
    labelPhi->setChecked(true);
    layout->addWidget(labelPhi,8,0);
    checkPhi = new QCheckBox("File");
    checkPhi->setChecked(true);
    layout->addWidget(checkPhi,8,1);
    filePhi = new FileWidget("Phi.asc","Choose nodal shape matrix of translation file","*.asc",1,false,false);
    layout->addWidget(filePhi,8,2);
    connect(labelPhi,SIGNAL(toggled(bool)),this,SLOT(updatePhi()));
    connect(checkPhi,SIGNAL(toggled(bool)),this,SLOT(updatePhi()));

    labelSr = new QCheckBox("Nodal stress matrix");
    labelSr->setChecked(true);
    layout->addWidget(labelSr,9,0);
    checkSr = new QCheckBox("File");
    checkSr->setChecked(true);
    layout->addWidget(checkSr,9,1);
    fileSr = new FileWidget("Sr.asc","Choose nodal stress matrix file","*.asc",1,false,false);
    layout->addWidget(fileSr,9,2);
    connect(labelSr,SIGNAL(toggled(bool)),this,SLOT(updateSr()));
    connect(checkSr,SIGNAL(toggled(bool)),this,SLOT(updateSr()));

    labelVisu = new QCheckBox("Visualisation");
    labelVisu->setChecked(true);
    layout->addWidget(labelVisu,13,0);
    choiceVisu = new CustomComboBox;
//    choiceVisu->addItem("Lines");
    choiceVisu->addItem("Points");
    choiceVisu->addItem("Lines");
    choiceVisu->addItem("Faces");
    choiceVisu->setCurrentIndex(0);
    layout->addWidget(choiceVisu,13,1);
    connect(labelVisu,SIGNAL(toggled(bool)),choiceVisu,SLOT(setEnabled(bool)));

    labelNodes = new QCheckBox("Nodes");
    labelNodes->setChecked(true);
    layout->addWidget(labelNodes,10,0);
    checkNodes = new QCheckBox("File");
    layout->addWidget(checkNodes,10,1);
    fileNodes = new FileWidget("nodes.asc","Choose nodes file","*.asc",1,false,false);
    fileNodes->setEnabled(false);
    layout->addWidget(fileNodes,10,2);
    connect(labelNodes,SIGNAL(toggled(bool)),this,SLOT(updateNodes()));
    connect(checkNodes,SIGNAL(toggled(bool)),this,SLOT(updateNodes()));

    labelIndices = new QCheckBox("Indices");
    labelIndices->setChecked(true);
    layout->addWidget(labelIndices,11,0);
    checkIndices = new QCheckBox("File");
    layout->addWidget(checkIndices,11,1);
    fileIndices = new FileWidget("indices.asc","Choose indices file","*.asc",1,false,false);
    fileIndices->setEnabled(false);
    layout->addWidget(fileIndices,11,2);
    connect(labelIndices,SIGNAL(toggled(bool)),this,SLOT(updateIndices()));
    connect(checkIndices,SIGNAL(toggled(bool)),this,SLOT(updateIndices()));

//    QLabel *labelnm = new QLabel("Number of modes");
    labelnm = new QCheckBox("Number of modes");
    layout->addWidget(labelnm,12,0);
    nm = new CustomSpinBox;
    nm->setValue(6);
    layout->addWidget(nm,12,1);
    nm->setEnabled(false);
    connect(labelnm,SIGNAL(toggled(bool)),nm,SLOT(setEnabled(bool)));
  }

  void ImportWidget::updatePdm() {
    checkPdm->setEnabled(labelPdm->isChecked());
    filePdm->setEnabled(labelPdm->isChecked() and checkPdm->isChecked());
  }

  void ImportWidget::updaterPdm() {
    checkrPdm->setEnabled(labelrPdm->isChecked());
    filerPdm->setEnabled(labelrPdm->isChecked() and checkrPdm->isChecked());
  }

  void ImportWidget::updatePPdm() {
    checkPPdm->setEnabled(labelPPdm->isChecked());
    filePPdm->setEnabled(labelPPdm->isChecked() and checkPPdm->isChecked());
  }

  void ImportWidget::updateKe() {
    checkKe->setEnabled(labelKe->isChecked());
    fileKe->setEnabled(labelKe->isChecked() and checkKe->isChecked());
  }

  void ImportWidget::updateu0() {
    checku0->setEnabled(labelu0->isChecked());
    fileu0->setEnabled(labelu0->isChecked() and checku0->isChecked());
  }

  void ImportWidget::updatePhi() {
    checkPhi->setEnabled(labelPhi->isChecked());
    filePhi->setEnabled(labelPhi->isChecked() and checkPhi->isChecked());
  }

  void ImportWidget::updateSr() {
    checkSr->setEnabled(labelSr->isChecked());
    fileSr->setEnabled(labelSr->isChecked() and checkSr->isChecked());
  }

  void ImportWidget::updateNodes() {
    checkNodes->setEnabled(labelNodes->isChecked());
    fileNodes->setEnabled(labelNodes->isChecked() and checkNodes->isChecked());
  }

  void ImportWidget::updateIndices() {
    checkIndices->setEnabled(labelIndices->isChecked());
    fileIndices->setEnabled(labelIndices->isChecked() and checkIndices->isChecked());
  }

}
