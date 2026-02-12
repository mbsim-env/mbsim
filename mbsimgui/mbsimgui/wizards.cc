/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2022 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#include <config.h>
#include "wizards.h"
#include "extended_widgets.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "special_widgets.h"
#include "mainwindow.h"
#include "fe_type.h"
#include "project.h"
#include <QRadioButton>
#include <QMessageBox>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMLSSerializer.hpp>

using namespace std;
using namespace fmatvec;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  FirstPage::FirstPage(QWidget *parent) : WizardPage(parent) {
    setTitle("Flexible body tool");

    auto *layout = new QVBoxLayout;
    setLayout(layout);

    auto label = new QLabel("Create input data for <i>flexible ffr body</i> by");
    label->setWordWrap(true);

    rb[0] = new QRadioButton("&external mass matrix and stiffness matrix");
    rb[1] = new QRadioButton("&calculix result file");
    rb[2] = new QRadioButton("&beam elements");
    rb[3] = new QRadioButton("&finite elements");

    rb[0]->setChecked(true);

    //registerField("first.c1*", rb[0]);
    //registerField("first.c2", rb[1]);

    layout->addWidget(label);
    for(int i=0; i<4; i++)
      layout->addWidget(rb[i]);
  }

  int FirstPage::nextId() const {
    if(rb[0]->isChecked())
      return FlexibleBodyTool::PageExtFE;
    else if(rb[1]->isChecked())
      return FlexibleBodyTool::PageCalculix;
    else if(rb[2]->isChecked())
      return FlexibleBodyTool::PageFlexibleBeam;
    else if(rb[3]->isChecked())
      return FlexibleBodyTool::PageFiniteElements;
    else 
      QMessageBox::information(this->wizard(),"Information","This option is not yet available."); 
    return FlexibleBodyTool::PageFirst;
  }

  void FirstPage::setVisible(bool visible) {
    WizardPage::setVisible(visible);
  }

  bool FirstPage::isComplete() const {
    return WizardPage::isComplete();
  }

  LastPage::LastPage(QWidget *parent) : WizardPage(parent) {
    setTitle("Save");

    auto *layout = new QVBoxLayout;
    setLayout(layout);

    auto label = new QLabel("Select a file to save the input data");
    layout->addWidget(label);

    inputFile = new ExtWidget("Input data file name", new FileWidget("\"input_data.h5\"", "Save input data file", "H5 files (*.h5)", 0, true, true, QFileDialog::DontConfirmOverwrite),false,false,MBSIMFLEX%"inputDataFileName");
    layout->addWidget(inputFile);
  }

  QString LastPage::getFile() const {
    return inputFile->getWidget<FileWidget>()->getFile(); 
  }

  int LastPage::nextId() const {
    return -1;
  }

  void LastPage::setVisible(bool visible) {
    WizardPage::setVisible(visible);
  }

  DOMElement* LastPage::initializeUsingXML(DOMElement *element) {
    inputFile->initializeUsingXML(element);
    return element;
  }

  DOMElement* LastPage::writeXMLFile(DOMNode *element, DOMNode *ref) {
    inputFile->writeXMLFile(element);
    return nullptr;
  }

  ExternalFiniteElementsPage::ExternalFiniteElementsPage(QWidget *parent) : WizardPage(parent) {
    setTitle("External finite elements data");

    auto *mainlayout = new QVBoxLayout;
    setLayout(mainlayout);

    auto label = new QLabel("Define the external finite elements data.");
    mainlayout->addWidget(label);

    auto *tab = new QScrollArea;
    tab->setWidgetResizable(true);
    QWidget *box = new QWidget;
    auto *layout = new QVBoxLayout;
//    layout->setSpacing(15);
    box->setLayout(layout);
    tab->setWidget(box);

    mainlayout->addWidget(tab);

    nodes = new ExtWidget("Nodes",new FileWidget("","Nodes file","ASCII files (*.asc);;All files (*.*)",0,true),false,false,MBSIMFLEX%"nodes");
    layout->addWidget(nodes);

    mass = new ExtWidget("Mass matrix",new FileWidget("","Open mass matrix file", "ASCII files (*.asc);;All files (*.*)",0,true),false,false,MBSIMFLEX%"massMatrix");
    layout->addWidget(mass);

    stiff = new ExtWidget("Stiffness matrix",new FileWidget("", "Open stiffness matrix file", "ASCII files (*.asc);;All files (*.*)",0,true),false,false,MBSIMFLEX%"stiffnessMatrix");
    layout->addWidget(stiff);

    layout->addStretch(1);
  }

  int ExternalFiniteElementsPage::nextId() const {
    return FlexibleBodyTool::PageRedMeth;
  }

  bool ExternalFiniteElementsPage::isComplete() const {
    return WizardPage::isComplete();
  }

  DOMElement* ExternalFiniteElementsPage::initializeUsingXML(DOMElement *element) {
    nodes->initializeUsingXML(element);
    mass->initializeUsingXML(element);
    DOMElement *ele = stiff->initializeUsingXML(element);
    return ele;
  }

  DOMElement* ExternalFiniteElementsPage::writeXMLFile(DOMNode *element, DOMNode *ref) {
    nodes->writeXMLFile(element);
    mass->writeXMLFile(element);
    stiff->writeXMLFile(element);
    return nullptr;
  }

  CalculixPage::CalculixPage(QWidget *parent) : WizardPage(parent) {
    setTitle("CalculiX data");

    auto *mainlayout = new QVBoxLayout;
    setLayout(mainlayout);

    auto label = new QLabel("Define the CalculiX result file.");
    mainlayout->addWidget(label);

    auto *tab = new QScrollArea;
    tab->setWidgetResizable(true);
    QWidget *box = new QWidget;
    auto *layout = new QVBoxLayout;
    box->setLayout(layout);
    tab->setWidget(box);

    mainlayout->addWidget(tab);

    file = new ExtWidget("Result file name",new FileWidget("", "Open CalculiX result file", "CalculiX result files (*.frd)", 0, true),false,false,MBSIMFLEX%"resultFileName");
    layout->addWidget(file);

    layout->addStretch(1);
  }

  int CalculixPage::nextId() const {
    return FlexibleBodyTool::PageDamp;
  }

  DOMElement* CalculixPage::initializeUsingXML(DOMElement *element) {
    DOMElement *ele = file->initializeUsingXML(element);
    return ele;
  }

  DOMElement* CalculixPage::writeXMLFile(DOMNode *element, DOMNode *ref) {
    file->writeXMLFile(element);
    return nullptr;
  }

  FlexibleBeamPage::FlexibleBeamPage(QWidget *parent) : WizardPage(parent) {
    setTitle("Flexible beam data");

    auto *mainlayout = new QVBoxLayout;
    setLayout(mainlayout);

    auto label = new QLabel("Define the flexible beam.");
    mainlayout->addWidget(label);

    auto *tab = new QScrollArea;
    tab->setWidgetResizable(true);
    QWidget *box = new QWidget;
    auto *layout = new QVBoxLayout;
    box->setLayout(layout);
    tab->setWidget(box);

    mainlayout->addWidget(tab);

    n = new ExtWidget("Number of nodes",new ChoiceWidget(new ScalarWidgetFactory("3"),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"numberOfNodes");
    layout->addWidget(n);

    l = new ExtWidget("Length",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"length");
    layout->addWidget(l);

    A = new ExtWidget("Cross-section area",new ChoiceWidget(new ScalarWidgetFactory("1e-4",vector<QStringList>(2,areaUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"crossSectionArea");
    layout->addWidget(A);

    vector<QString> I_(3); I_[0] = "1e-10"; I_[1] = "1e-10"; I_[2] = "0";
    I = new ExtWidget("Moment of inertia",new ChoiceWidget(new VecWidgetFactory(I_),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"momentOfInertia");
    layout->addWidget(I);

    E = new ExtWidget("Young's modulus",new ChoiceWidget(new ScalarWidgetFactory("2e11",vector<QStringList>(2,bulkModulusUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"youngsModulus");
    layout->addWidget(E);

    rho = new ExtWidget("Density",new ChoiceWidget(new ScalarWidgetFactory("7870",vector<QStringList>(2,densityUnits()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"density");
    layout->addWidget(rho);

    ten = new ExtWidget("Tension",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"tension");
    layout->addWidget(ten);

    beny = new ExtWidget("Bending about y axis",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"bendingAboutYAxis");
    layout->addWidget(beny);

    benz = new ExtWidget("Bending about z axis",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,true,MBSIMFLEX%"bendingAboutZAxis");
    layout->addWidget(benz);

    tor = new ExtWidget("Torsion",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"torsion");
    layout->addWidget(tor);

    layout->addStretch(1);
  }

  int FlexibleBeamPage::nextId() const {
    return FlexibleBodyTool::PageDL;
  }

  DOMElement* FlexibleBeamPage::initializeUsingXML(DOMElement *element) {
    n->initializeUsingXML(element);
    l->initializeUsingXML(element);
    A->initializeUsingXML(element);
    DOMElement *ele = I->initializeUsingXML(element);
    E->initializeUsingXML(element);
    rho->initializeUsingXML(element);
    ten->initializeUsingXML(element);
    beny->initializeUsingXML(element);
    benz->initializeUsingXML(element);
    tor->initializeUsingXML(element);
    return ele;
  }

  DOMElement* FlexibleBeamPage::writeXMLFile(DOMNode *element, DOMNode *ref) {
    n->writeXMLFile(element);
    l->writeXMLFile(element);
    A->writeXMLFile(element);
    I->writeXMLFile(element);
    E->writeXMLFile(element);
    rho->writeXMLFile(element);
    ten->writeXMLFile(element);
    beny->writeXMLFile(element);
    benz->writeXMLFile(element);
    tor->writeXMLFile(element);
    return nullptr;
  }

  FiniteElementsPage::FiniteElementsPage(QWidget *parent) : WizardPage(parent) {
    setTitle("Flexible beam data");

    auto *mainlayout = new QVBoxLayout;
    setLayout(mainlayout);

    auto label = new QLabel("Define the flexible beam.");
    mainlayout->addWidget(label);

    auto *tab = new QScrollArea;
    tab->setWidgetResizable(true);
    QWidget *box = new QWidget;
    auto *layout = new QVBoxLayout;
    box->setLayout(layout);
    tab->setWidget(box);

    mainlayout->addWidget(tab);

    E = new ExtWidget("Young's modulus",new ChoiceWidget(new ScalarWidgetFactory("2e11",vector<QStringList>(2,bulkModulusUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"youngsModulus");
    layout->addWidget(E);

    nu = new ExtWidget("Poisson's ratio",new ChoiceWidget(new ScalarWidgetFactory("0.3",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"poissonsRatio");
    layout->addWidget(nu);

    rho = new ExtWidget("Density",new ChoiceWidget(new ScalarWidgetFactory("7870",vector<QStringList>(2,densityUnits()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"density");
    layout->addWidget(rho);

    nodes = new ExtWidget("Nodes",new FileWidget("","Nodes file","ASCII files (*.asc);;All files (*.*)",0,true),false,false,MBSIMFLEX%"nodes");
    layout->addWidget(nodes);

    elements = new ExtWidget("Element data",new ListWidget(new FiniteElementsDataWidgetFactory(this),"Element data",1,3,false,1),false,false,"");
    layout->addWidget(elements);

    exs = new ExtWidget("Calculate stresses by extrapolation",new ChoiceWidget(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"calculateStressesByExtrapolation");
    layout->addWidget(exs);

    layout->addStretch(1);
  }

  int FiniteElementsPage::nextId() const {
    return FlexibleBodyTool::PageDL;
  }

  DOMElement* FiniteElementsPage::initializeUsingXML(DOMElement *element) {
    E->initializeUsingXML(element);
    DOMElement *ele = nu->initializeUsingXML(element);
    rho->initializeUsingXML(element);
    nodes->initializeUsingXML(element);
    elements->initializeUsingXML(element);
    exs->initializeUsingXML(element);
    return ele;
  }

  DOMElement* FiniteElementsPage::writeXMLFile(DOMNode *element, DOMNode *ref) {
    E->writeXMLFile(element);
    nu->writeXMLFile(element);
    rho->writeXMLFile(element);
    nodes->writeXMLFile(element);
    elements->writeXMLFile(element);
    exs->writeXMLFile(element);
    return nullptr;
  }

  ReductionMethodsPage::ReductionMethodsPage(QWidget *parent) : WizardPage(parent) {
    setTitle("Reduction method");

    auto *layout = new QVBoxLayout;
    setLayout(layout);

    auto label = new QLabel("Define the reduction method.");
    layout->addWidget(label);

    rb[0] = new QRadioButton("&Component mode synthesis");
    rb[1] = new QRadioButton("Mode shape matrix");

    rb[0]->setChecked(true);

    layout->addWidget(label);
    layout->addWidget(rb[0]);
    layout->addWidget(rb[1]);
  }

  int ReductionMethodsPage::nextId() const {
   if(rb[0]->isChecked())
     return FlexibleBodyTool::PageBC;
   else
     return FlexibleBodyTool::PageModeShapes;
  }

  BoundaryConditionsPage::BoundaryConditionsPage(QWidget *parent) : WizardPage(parent) {
    setTitle("Boundary conditions");

    auto *mainlayout = new QVBoxLayout;
    setLayout(mainlayout);

    auto label = new QLabel("Define the boundary conditions.");
    mainlayout->addWidget(label);

    auto *tab = new QScrollArea;
    tab->setWidgetResizable(true);
    QWidget *box = new QWidget;
    auto *layout = new QVBoxLayout;
    layout->setSpacing(15);
    box->setLayout(layout);
    tab->setWidget(box);

    mainlayout->addWidget(tab);

    bc = new ExtWidget("Boundary condition",new ListWidget(new BoundaryConditionWidgetFactory(this),"Boundary condition",0,3,false),false,false,"");
    layout->addWidget(bc);

    layout->addStretch(1);
  }

  int BoundaryConditionsPage::nextId() const {
    return FlexibleBodyTool::PageCMS;
  }

  DOMElement* BoundaryConditionsPage::initializeUsingXML(DOMElement *element) {
    bc->initializeUsingXML(element);
    return element;
  }

  DOMElement* BoundaryConditionsPage::writeXMLFile(DOMNode *element, DOMNode *ref) {
    bc->writeXMLFile(element);
    return nullptr;
  }

  ModeShapesPage::ModeShapesPage(QWidget *parent) : WizardPage(parent) {
    setTitle("Mode shapes");

    auto *mainlayout = new QVBoxLayout;
    setLayout(mainlayout);

    auto label = new QLabel("Define the mode shapes.");
    mainlayout->addWidget(label);

    auto *tab = new QScrollArea;
    tab->setWidgetResizable(true);
    QWidget *box = new QWidget;
    auto *layout = new QVBoxLayout;
    layout->setSpacing(15);
    box->setLayout(layout);
    tab->setWidget(box);

    mainlayout->addWidget(tab);

    V = new ExtWidget("Mode shape matrix",new FileWidget("","Mode shape matrix file","ASCII files (*.asc);;All files (*.*)",0,true),false,false,MBSIMFLEX%"modeShapeMatrix");
    layout->addWidget(V);

    S = new ExtWidget("Stress matrix",new FileWidget("","Stress matrix file","ASCII files (*.asc);;All files (*.*)",0,true),true,false,MBSIMFLEX%"stressMatrix");
    layout->addWidget(S);

    layout->addStretch(1);
  }

  int ModeShapesPage::nextId() const {
    return FlexibleBodyTool::PageRRBM;
  }

  DOMElement* ModeShapesPage::initializeUsingXML(DOMElement *element) {
    DOMElement *ele = V->initializeUsingXML(element);
    S->initializeUsingXML(element);
    return ele;
  }

  DOMElement* ModeShapesPage::writeXMLFile(DOMNode *element, DOMNode *ref) {
    V->writeXMLFile(element);
    S->writeXMLFile(element);
    return nullptr;
  }

  ComponentModeSynthesisPage::ComponentModeSynthesisPage(QWidget *parent) : WizardPage(parent) {
    setTitle("Component mode synthesis");

    auto *mainlayout = new QVBoxLayout;
    setLayout(mainlayout);

    auto label = new QLabel("Define the interface nodes and the normal modes.");
    mainlayout->addWidget(label);

    auto *tab = new QScrollArea;
    tab->setWidgetResizable(true);
    QWidget *box = new QWidget;
    auto *layout = new QVBoxLayout;
//    layout->setSpacing(15);
    box->setLayout(layout);
    tab->setWidget(box);

    mainlayout->addWidget(tab);

    vector<QString> list;
    list.emplace_back("\"distributing\"");
    list.emplace_back("\"kinematic\"");
    typeOfConstraint = new ExtWidget("Type of constraint",new TextChoiceWidget(list,0,true),true,false,MBSIMFLEX%"typeOfConstraint");
    layout->addWidget(typeOfConstraint);

    idata = new ExtWidget("Interface data",new ListWidget(new CMSDataWidgetFactory(this),"Interface data",0,3,false,0),false,false,"");
    layout->addWidget(idata);

    nmodes = new ExtWidget("Normal mode numbers",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"normalModeNumbers");
    layout->addWidget(nmodes);

    list.clear();
    list.emplace_back("\"freeBoundaryNormalModes\"");
    list.emplace_back("\"fixedBoundaryNormalModes\"");
    list.emplace_back("\"constrainedBoundaryNormalModes\"");
    normalModes = new ExtWidget("Normal modes",new TextChoiceWidget(list,0,true),true,false,MBSIMFLEX%"normalModes");
    layout->addWidget(normalModes);

    layout->addStretch(1);
  }

  int ComponentModeSynthesisPage::nextId() const {
    return FlexibleBodyTool::PageRRBM;
  }

  DOMElement* ComponentModeSynthesisPage::initializeUsingXML(DOMElement *element) {
    typeOfConstraint->initializeUsingXML(element);
    idata->initializeUsingXML(element);
    DOMElement *ele = nmodes->initializeUsingXML(element);
    normalModes->initializeUsingXML(element);
    return idata->getWidget<ListWidget>()->getSize()?element:ele;
  }

  DOMElement* ComponentModeSynthesisPage::writeXMLFile(DOMNode *element, DOMNode *ref) {
    typeOfConstraint->writeXMLFile(element);
    idata->writeXMLFile(element);
    nmodes->writeXMLFile(element);
    normalModes->writeXMLFile(element);
    return nullptr;
  }

  RemoveRigidBodyModesPage::RemoveRigidBodyModesPage(QWidget *parent) : WizardPage(parent) {
    setTitle("Remove rigid body modes");

    auto *mainlayout = new QVBoxLayout;
    setLayout(mainlayout);

    auto label = new QLabel("Remove rigid body modes.");
    mainlayout->addWidget(label);

    auto *tab = new QScrollArea;
    tab->setWidgetResizable(true);
    QWidget *box = new QWidget;
    auto *layout = new QVBoxLayout;
//    layout->setSpacing(15);
    box->setLayout(layout);
    tab->setWidget(box);

    mainlayout->addWidget(tab);

    rrbm = new ExtWidget("Remove rigid body modes",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"removeRigidBodyModes");
    layout->addWidget(rrbm);

    nrb = new ExtWidget("Number of rigid body modes",new SpinBoxWidget(6,1,6),true,false,MBSIMFLEX%"numberOfRigidBodyModes");
    layout->addWidget(nrb);

    ft = new ExtWidget("Frequency threshold",new ChoiceWidget(new ScalarWidgetFactory("100"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"frequencyThreshold");
    layout->addWidget(ft);

    layout->addStretch(1);
  }

  int RemoveRigidBodyModesPage::nextId() const {
    if(wizard()->hasVisitedPage(FlexibleBodyTool::PageExtFE))
      return FlexibleBodyTool::PageOMBV;
    else
      return FlexibleBodyTool::PageDamp;
  }

  DOMElement* RemoveRigidBodyModesPage::initializeUsingXML(DOMElement *element) {
    rrbm->initializeUsingXML(element);
    nrb->initializeUsingXML(element);
    ft->initializeUsingXML(element);
    return element;
  }

  DOMElement* RemoveRigidBodyModesPage::writeXMLFile(DOMNode *element, DOMNode *ref) {
    rrbm->writeXMLFile(element);
    nrb->writeXMLFile(element);
    ft->writeXMLFile(element);
    return nullptr;
  }

  OpenMBVPage::OpenMBVPage(QWidget *parent) : WizardPage(parent) {
    setTitle("OpenMBV");

    auto *mainlayout = new QVBoxLayout;
    setLayout(mainlayout);

    auto label = new QLabel("Define the OpenMBV data.");
    mainlayout->addWidget(label);

    auto *tab = new QScrollArea;
    tab->setWidgetResizable(true);
    QWidget *box = new QWidget;
    auto *layout = new QVBoxLayout;
//    layout->setSpacing(15);
    box->setLayout(layout);
    tab->setWidget(box);

    mainlayout->addWidget(tab);

    ombvIndices = new ExtWidget("OpenMBV indices",new FileWidget("","Indices file","ASCII files (*.asc);;All files (*.*)",0,true),true,false,MBSIMFLEX%"openmbvIndices");
    layout->addWidget(ombvIndices);

    layout->addStretch(1);
  }

  int OpenMBVPage::nextId() const {
    return FlexibleBodyTool::PageDamp;
  }

  DOMElement* OpenMBVPage::initializeUsingXML(DOMElement *element) {
    ombvIndices->initializeUsingXML(element);
    return element;
  }

  DOMElement* OpenMBVPage::writeXMLFile(DOMNode *element, DOMNode *ref) {
    ombvIndices->writeXMLFile(element);
    return nullptr;
  }

  DampingPage::DampingPage(QWidget *parent) : WizardPage(parent) {
    setTitle("Damping");

    auto *mainlayout = new QVBoxLayout;
    setLayout(mainlayout);

    auto label = new QLabel("Define the damping model.");
    mainlayout->addWidget(label);

    auto *tab = new QScrollArea;
    tab->setWidgetResizable(true);
    QWidget *box = new QWidget;
    auto *layout = new QVBoxLayout;
    box->setLayout(layout);
    tab->setWidget(box);

    mainlayout->addWidget(tab);

    mDamp = new ExtWidget("Modal damping",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"modalDamping");
    layout->addWidget(mDamp);

    pDamp = new ExtWidget("Proportional damping",new ChoiceWidget(new VecWidgetFactory(2),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"proportionalDamping");
    layout->addWidget(pDamp);

    layout->addStretch(1);
  }

  int DampingPage::nextId() const {
    return FlexibleBodyTool::PageLast;
  }

  DOMElement* DampingPage::initializeUsingXML(DOMElement *element) {
    mDamp->initializeUsingXML(element);
    pDamp->initializeUsingXML(element);
    return element;
  }

  DOMElement* DampingPage::writeXMLFile(DOMNode *element, DOMNode *ref) {
    mDamp->writeXMLFile(element);
    pDamp->writeXMLFile(element);
    return nullptr;
  }

  DistributedLoadsPage::DistributedLoadsPage(QWidget *parent) : WizardPage(parent) {
    setTitle("Distributed loads");

    auto *mainlayout = new QVBoxLayout;
    setLayout(mainlayout);

    auto label = new QLabel("Define the distributed loads.");
    mainlayout->addWidget(label);

    auto *tab = new QScrollArea;
    tab->setWidgetResizable(true);
    QWidget *box = new QWidget;
    auto *layout = new QVBoxLayout;
    box->setLayout(layout);
    tab->setWidget(box);

    mainlayout->addWidget(tab);

    dloads = new ExtWidget("Distributed load",new ListWidget(new DistributedLoadsWidgetFactory(this),"Distributed load",0,3,false,0),false,false,"");
    layout->addWidget(dloads);

    layout->addStretch(1);
  }

  int DistributedLoadsPage::nextId() const {
    return FlexibleBodyTool::PageBC;
  }

  DOMElement* DistributedLoadsPage::initializeUsingXML(DOMElement *element) {
    dloads->initializeUsingXML(element);
    return element;
  }

  DOMElement* DistributedLoadsPage::writeXMLFile(DOMNode *element, DOMNode *ref) {
    dloads->writeXMLFile(element);
    return nullptr;
  }

  void Wizard::showEvent(QShowEvent *event) {
    mw->setCurrentlyEditedItem(mw->getProject()->getDynamicSystemSolver());
    mw->updateParameters(mw->getProject());
    QSettings settings;
    restoreGeometry(settings.value("wizard/geometry").toByteArray());
    QWizard::showEvent(event);
  }

  void Wizard::hideEvent(QHideEvent *event) {
    QSettings settings;
    settings.setValue("wizard/geometry", saveGeometry());
    QWizard::hideEvent(event);
  }

  FlexibleBodyTool::FlexibleBodyTool(QWidget *parent) : Wizard(parent), npl(mw->eval)  {
    mw->updateParameters(mw->getProject());
    setPage(PageFirst, new FirstPage(this));
    setPage(PageExtFE, new ExternalFiniteElementsPage(this));
    setPage(PageCalculix, new CalculixPage(this));
    setPage(PageFlexibleBeam, new FlexibleBeamPage(this));
    setPage(PageFiniteElements, new FiniteElementsPage(this));
    setPage(PageRedMeth, new ReductionMethodsPage(this));
    setPage(PageBC, new BoundaryConditionsPage(this));
    setPage(PageCMS, new ComponentModeSynthesisPage(this));
    setPage(PageModeShapes, new ModeShapesPage(this));
    setPage(PageRRBM, new RemoveRigidBodyModesPage(this));
    setPage(PageOMBV, new OpenMBVPage(this));
    setPage(PageDamp, new DampingPage(this));
    setPage(PageDL, new DistributedLoadsPage(this));
    setPage(PageLast, new LastPage(this));
    setButtonText(QWizard::CustomButton1, "&Load");
    setOption(QWizard::HaveCustomButton1, true);
    setButtonText(QWizard::CustomButton2, "&Save");
    setOption(QWizard::HaveCustomButton2, true);
    button(QWizard::CustomButton2)->setDisabled(true);
    connect(button(QWizard::FinishButton),&QAbstractButton::clicked,this,&FlexibleBodyTool::create);
    connect(this, &QWizard::customButtonClicked,this,[=](int which){
	if(which==QWizard::CustomButton1)
          load();
	else
	  save();
	});
    connect(this, &QWizard::currentIdChanged,this,[=](){ button(QWizard::CustomButton2)->setEnabled(hasVisitedPage(PageLast)); });
  }

  void FlexibleBodyTool::create() {
    m = 0;
    rdm.init(0);
    rrdm.init(0);
    rPdm.clear();
    PPdm.clear();
    r.clear();
    Phi.clear();
    Psi.clear();
    PPdm.clear();
    sigmahel.clear();
    if(not Km.size()) {
      delete Ks.Ip();
      delete Ks.Jp();
      delete PPdm2s[0].Ip();
      delete PPdm2s[0].Jp();
    }
    Phis.clear();
    Psis.clear();
    sigs.clear();
    Mm.clear();
    Km.clear();
    De0 <<= SymMatV();
    mDamp <<= VecV();
    pDamp.init(0);
    rif.clear();
    Phiif.clear();
    Psiif.clear();
    sigmahelif.clear();
    nodeTable.clear();
    nodeCount.clear();
    nodeNumbers.clear();
    singleNodeNumbers.clear();
    indices.clear();
    ele.clear();
    for(auto & i : type)
      delete i;
    type.clear();
    links.clear();
    try {
      if(hasVisitedPage(PageExtFE)) {
        extfe();
        if(hasVisitedPage(PageCMS))
          cms();
        else if(hasVisitedPage(PageModeShapes))
          msm();
        ombv();
        lma();
      }
      else if(hasVisitedPage(PageCalculix)) {
        calculix();
        lma();
      }
      else if(hasVisitedPage(PageFlexibleBeam)) {
        beam();
        cms();
        fma();
      }
      else if(hasVisitedPage(PageFiniteElements)) {
        fe();
        cms();
        fma();
      }
      damp();
      exp();
    }
    catch (std::exception& e)
    {
      QMessageBox::critical(this, "Flexible body tool", e.what());
    }
  }

  void FlexibleBodyTool::save() {
    auto inputFile = getInputDataFile();
    inputFile = inputFile.mid(1,inputFile.size()-2);
    auto projectPath = mw->getProjectPath();
    if(not projectPath.isEmpty())
      projectPath += "/";
    QString file=QFileDialog::getSaveFileName(this, "Save finite elements input data file", projectPath+QFileInfo(inputFile).baseName()+".xml", "XML files (*.xml)");
    if(not(file.isEmpty())) {
      file = file.endsWith(".xml")?file:file+".xml";
      auto doc = mw->mbxmlparserNoVal->createDocument();
      auto element=MBXMLUtils::D(doc)->createElement(MBSIMFLEX%"InputData");
      doc->insertBefore(element, nullptr);
      auto pageList = pageIds();
      for(int i=0; i<pageList.size(); i++) {
	if(hasVisitedPage(pageList.at(i)))
	  page<WizardPage>(pageList.at(i))->writeXMLFile(element);
      }
      mw->serializer->writeToURI(doc.get(), MBXMLUtils::X()%file.toStdString());
    }
  }

  void FlexibleBodyTool::load() {
    restart();
    QString file=QFileDialog::getOpenFileName(this, "Open finite elements input data file", QFileInfo(mw->getProjectFilePath()).absolutePath(), "XML files (*.xml);;All files (*.*)");
    if(file.startsWith("//"))
      file.replace('/','\\'); // xerces-c is not able to parse files from network shares that begin with "//"
    if(not file.isEmpty()) {
      auto doc = mw->mbxmlparserNoVal->parse(file.toStdString());
      if(!doc) {
        mw->statusBar()->showMessage("Unable to load or parse XML file: "+file);
        return;
      }
      auto element = doc->getDocumentElement();
      auto pageList = pageIds();
      vector<bool> pageActive(pageList.size());
      for(int i=0; i<pageList.size(); i++)
	pageActive[i] = page<WizardPage>(pageList.at(i))->initializeUsingXML(element);
      for(size_t i=0; i<4; i++) {
	if(pageActive[i+1])
	  page<FirstPage>(PageFirst)->rb[i]->setChecked(true);
      }
      for(size_t i=0; i<2; i++) {
	if(pageActive[i+7])
	  page<ReductionMethodsPage>(PageRedMeth)->rb[i]->setChecked(true);
      }
    }
  }

  QString FlexibleBodyTool::getInputDataFile() const {
    return page<LastPage>(PageLast)->getFile(); 
  }

}
