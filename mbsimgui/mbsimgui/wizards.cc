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
#include <QRadioButton>
#include <QMessageBox>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMLSSerializer.hpp>

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  extern MainWindow *mw;

  FirstPage::FirstPage(QWidget *parent) : QWizardPage(parent) {
    setTitle("Flexible body tool");

    auto *layout = new QVBoxLayout;
    setLayout(layout);

    auto label = new QLabel("Create input data for <i>flexible ffr body</i> by");
    label->setWordWrap(true);

    rb1 = new QRadioButton("&external mass matrix and stiffness matrix");
    rb2 = new QRadioButton("&calculix result file");
    rb3 = new QRadioButton("&beam elements");
    rb4 = new QRadioButton("&finite elements");

    rb1->setChecked(true);

    //registerField("first.c1*", rb1);
    //registerField("first.c2", rb2);

    layout->addWidget(label);
    layout->addWidget(rb1);
    layout->addWidget(rb2);
    layout->addWidget(rb3);
    layout->addWidget(rb4);
  }

  int FirstPage::nextId() const {
    if(rb1->isChecked())
      return FlexibleBodyTool::PageExtFE;
    else if(rb2->isChecked())
      return FlexibleBodyTool::PageCalculix;
    else if(rb3->isChecked())
      return FlexibleBodyTool::PageFlexibleBeam;
    else if(rb4->isChecked())
      return FlexibleBodyTool::PageFiniteElements;
    else 
      QMessageBox::information(this->wizard(),"Information","This option is not yet available."); 
    return FlexibleBodyTool::PageFirst;
  }

  void FirstPage::setVisible(bool visible) {
    QWizardPage::setVisible(visible);

    if(visible) {
      wizard()->setButtonText(QWizard::CustomButton1, "&Load");
      wizard()->setOption(QWizard::HaveCustomButton1, true);
      connect(wizard(), &QWizard::customButtonClicked,static_cast<FlexibleBodyTool*>(wizard()),&FlexibleBodyTool::load);
    } else {
      wizard()->setOption(QWizard::HaveCustomButton1, false);
      disconnect(wizard(), &QWizard::customButtonClicked,static_cast<FlexibleBodyTool*>(wizard()),&FlexibleBodyTool::load);
    }
  }

  bool FirstPage::isComplete() const {
    return QWizardPage::isComplete();
  }

  LastPage::LastPage(QWidget *parent) : QWizardPage(parent) {
    setTitle("Save");

    auto *layout = new QVBoxLayout;
    setLayout(layout);

    auto label = new QLabel("Select a file to save the input data");
    layout->addWidget(label);

    inputFile = new ExtWidget("Input data file name", new FileWidget("\"input_data.h5\"", "Save input data file", "H5 files (*.h5)", 0, true, true, QFileDialog::DontConfirmOverwrite),false,false,MBSIMFLEX%"inputDataFileName");
    layout->addWidget(inputFile);
  }

  QString LastPage::getFile() const {
    return static_cast<FileWidget*>(inputFile->getWidget())->getFile(); 
  }

  int LastPage::nextId() const {
    return -1;
  }

  void LastPage::setVisible(bool visible) {
    QWizardPage::setVisible(visible);

    if(visible) {
      wizard()->setButtonText(QWizard::CustomButton1, "&Save");
      wizard()->setOption(QWizard::HaveCustomButton1, true);
      connect(wizard(), &QWizard::customButtonClicked,static_cast<FlexibleBodyTool*>(wizard()),&FlexibleBodyTool::save);
    } else {
      wizard()->setOption(QWizard::HaveCustomButton1, false);
      disconnect(wizard(), &QWizard::customButtonClicked,static_cast<FlexibleBodyTool*>(wizard()),&FlexibleBodyTool::save);
    }
  }

  ExternalFiniteElementsPage::ExternalFiniteElementsPage(QWidget *parent) : QWizardPage(parent) {
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
    return QWizardPage::isComplete();
  }

  CalculixPage::CalculixPage(QWidget *parent) : QWizardPage(parent) {
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

  FlexibleBeamPage::FlexibleBeamPage(QWidget *parent) : QWizardPage(parent) {
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
    return FlexibleBodyTool::PageBC;
  }

  FiniteElementsPage::FiniteElementsPage(QWidget *parent) : QWizardPage(parent) {
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
    return FlexibleBodyTool::PageBC;
  }

  ReductionMethodsPage::ReductionMethodsPage(QWidget *parent) : QWizardPage(parent) {
    setTitle("Reduction method");

    auto *layout = new QVBoxLayout;
    setLayout(layout);

    auto label = new QLabel("Define the reduction method.");
    layout->addWidget(label);

    rb1 = new QRadioButton("&Component mode synthesis");
    rb2 = new QRadioButton("Mode shape matrix");

    rb1->setChecked(true);

    layout->addWidget(label);
    layout->addWidget(rb1);
    layout->addWidget(rb2);
  }

  int ReductionMethodsPage::nextId() const {
   if(rb1->isChecked())
     return FlexibleBodyTool::PageBC;
   else
     return FlexibleBodyTool::PageModeShapes;
  }

  BoundaryConditionsPage::BoundaryConditionsPage(QWidget *parent) : QWizardPage(parent) {
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

  ModeShapesPage::ModeShapesPage(QWidget *parent) : QWizardPage(parent) {
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
    return FlexibleBodyTool::PageOMBV;
  }

  ComponentModeSynthesisPage::ComponentModeSynthesisPage(QWidget *parent) : QWizardPage(parent) {
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

    inodes = new ExtWidget("Interface node numbers",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"interfaceNodeNumbers");
    layout->addWidget(inodes);

    nmodes = new ExtWidget("Normal mode numbers",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"normalModeNumbers");
    layout->addWidget(nmodes);

    fbnm = new ExtWidget("Fixed-boundary normal modes",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"fixedBoundaryNormalModes");
    layout->addWidget(fbnm);

    layout->addStretch(1);
  }

  int ComponentModeSynthesisPage::nextId() const {
    if(wizard()->hasVisitedPage(FlexibleBodyTool::PageExtFE))
      return FlexibleBodyTool::PageOMBV;
    else
      return FlexibleBodyTool::PageDamp;
  }

  OpenMBVPage::OpenMBVPage(QWidget *parent) : QWizardPage(parent) {
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

  DampingPage::DampingPage(QWidget *parent) : QWizardPage(parent) {
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

  FlexibleBodyTool::FlexibleBodyTool(QWidget *parent) : QWizard(parent) {
    setPage(PageFirst, new FirstPage(this));
    setPage(PageExtFE, new ExternalFiniteElementsPage(this));
    setPage(PageCalculix, new CalculixPage(this));
    setPage(PageFlexibleBeam, new FlexibleBeamPage(this));
    setPage(PageFiniteElements, new FiniteElementsPage(this));
    setPage(PageRedMeth, new ReductionMethodsPage(this));
    setPage(PageBC, new BoundaryConditionsPage(this));
    setPage(PageCMS, new ComponentModeSynthesisPage(this));
    setPage(PageModeShapes, new ModeShapesPage(this));
    setPage(PageOMBV, new OpenMBVPage(this));
    setPage(PageDamp, new DampingPage(this));
    setPage(PageLast, new LastPage(this));
    connect(button(QWizard::FinishButton),&QAbstractButton::clicked,this,&FlexibleBodyTool::create);
  }

  void FlexibleBodyTool::create() {
    nodeTable.clear();
    nodeNumbers.clear();
    rPdm.clear();
    PPdm.clear();
    KrKP.clear();
    Phi.clear();
    Psi.clear();
    sigmahel.clear();
    MKm.clear();
    PPm.clear();
    Phim.clear();
    Psim.clear();
    sigm.clear();
    indices.clear();

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

  void FlexibleBodyTool::save() {
    QString inputFile = getInputDataFile();
    inputFile = inputFile.mid(1,inputFile.size()-2);
    QString file=QFileDialog::getSaveFileName(this, "Save finite elements input data file", QFileInfo(mw->getProjectFilePath()).absolutePath()+"/"+QFileInfo(inputFile).baseName()+".xml", "XML files (*.xml)");
    if(not(file.isEmpty())) {
      file = file.endsWith(".xml")?file:file+".xml";
      auto doc = mw->impl->createDocument();
      auto element=MBXMLUtils::D(doc)->createElement(MBSIMFLEX%"InputData");
      doc->insertBefore(element, nullptr);
      if(hasVisitedPage(PageExtFE)) {
	static_cast<ExternalFiniteElementsPage*>(page(PageExtFE))->nodes->writeXMLFile(element);
	static_cast<ExternalFiniteElementsPage*>(page(PageExtFE))->mass->writeXMLFile(element);
	static_cast<ExternalFiniteElementsPage*>(page(PageExtFE))->stiff->writeXMLFile(element);
      }
      if(hasVisitedPage(PageCalculix))
	static_cast<CalculixPage*>(page(PageCalculix))->file->writeXMLFile(element);
      if(hasVisitedPage(PageFlexibleBeam)) {
	static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->n->writeXMLFile(element);
	static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->l->writeXMLFile(element);
	static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->A->writeXMLFile(element);
	static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->I->writeXMLFile(element);
	static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->E->writeXMLFile(element);
	static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->rho->writeXMLFile(element);
	static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->ten->writeXMLFile(element);
	static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->beny->writeXMLFile(element);
	static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->benz->writeXMLFile(element);
	static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->tor->writeXMLFile(element);
      }
      if(hasVisitedPage(PageFiniteElements)) {
	static_cast<FiniteElementsPage*>(page(PageFiniteElements))->E->writeXMLFile(element);
	static_cast<FiniteElementsPage*>(page(PageFiniteElements))->nu->writeXMLFile(element);
	static_cast<FiniteElementsPage*>(page(PageFiniteElements))->rho->writeXMLFile(element);
	static_cast<FiniteElementsPage*>(page(PageFiniteElements))->nodes->writeXMLFile(element);
	static_cast<FiniteElementsPage*>(page(PageFiniteElements))->elements->writeXMLFile(element);
	static_cast<FiniteElementsPage*>(page(PageFiniteElements))->exs->writeXMLFile(element);
      }
      if(hasVisitedPage(PageBC))
	static_cast<BoundaryConditionsPage*>(page(PageBC))->bc->writeXMLFile(element);
      if(hasVisitedPage(PageCMS)) {
	static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->inodes->writeXMLFile(element);
	static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->nmodes->writeXMLFile(element);
	static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->fbnm->writeXMLFile(element);
      }
      if(hasVisitedPage(PageModeShapes)) {
	static_cast<ModeShapesPage*>(page(PageModeShapes))->V->writeXMLFile(element);
	static_cast<ModeShapesPage*>(page(PageModeShapes))->S->writeXMLFile(element);
      }
      if(hasVisitedPage(PageOMBV))
	static_cast<OpenMBVPage*>(page(PageOMBV))->ombvIndices->writeXMLFile(element);
      if(hasVisitedPage(PageDamp)) {
	static_cast<DampingPage*>(page(PageDamp))->mDamp->writeXMLFile(element);
	static_cast<DampingPage*>(page(PageDamp))->pDamp->writeXMLFile(element);
      }
      static_cast<LastPage*>(page(PageLast))->inputFile->writeXMLFile(element);
      mw->serializer->writeToURI(doc, MBXMLUtils::X()%file.toStdString());
    }
  }

  void FlexibleBodyTool::load() {
    QString file=QFileDialog::getOpenFileName(this, "Open finite elements input data file", QFileInfo(mw->getProjectFilePath()).absolutePath(), "XML files (*.xml);;All files (*.*)");
    if(file.startsWith("//"))
      file.replace('/','\\'); // xerces-c is not able to parse files from network shares that begin with "//"
    if(not file.isEmpty()) {
      auto doc = mw->parser->parseURI(MBXMLUtils::X()%file.toStdString());
      auto element = doc->getDocumentElement();
      static_cast<ExternalFiniteElementsPage*>(page(PageExtFE))->nodes->initializeUsingXML(element);
      static_cast<ExternalFiniteElementsPage*>(page(PageExtFE))->mass->initializeUsingXML(element);
      static_cast<ExternalFiniteElementsPage*>(page(PageExtFE))->stiff->initializeUsingXML(element);
      static_cast<BoundaryConditionsPage*>(page(PageBC))->bc->initializeUsingXML(element);
      static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->inodes->initializeUsingXML(element);
      static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->nmodes->initializeUsingXML(element);
      static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->fbnm->initializeUsingXML(element);
      static_cast<ModeShapesPage*>(page(PageModeShapes))->V->initializeUsingXML(element);
      static_cast<ModeShapesPage*>(page(PageModeShapes))->S->initializeUsingXML(element);
      static_cast<OpenMBVPage*>(page(PageOMBV))->ombvIndices->initializeUsingXML(element);
      static_cast<CalculixPage*>(page(PageCalculix))->file->initializeUsingXML(element);
      static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->n->initializeUsingXML(element);
      static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->l->initializeUsingXML(element);
      static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->A->initializeUsingXML(element);
      static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->I->initializeUsingXML(element);
      static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->E->initializeUsingXML(element);
      static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->rho->initializeUsingXML(element);
      static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->ten->initializeUsingXML(element);
      static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->beny->initializeUsingXML(element);
      static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->benz->initializeUsingXML(element);
      static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->tor->initializeUsingXML(element);
      static_cast<FiniteElementsPage*>(page(PageFiniteElements))->E->initializeUsingXML(element);
      static_cast<FiniteElementsPage*>(page(PageFiniteElements))->nu->initializeUsingXML(element);
      static_cast<FiniteElementsPage*>(page(PageFiniteElements))->rho->initializeUsingXML(element);
      static_cast<FiniteElementsPage*>(page(PageFiniteElements))->nodes->initializeUsingXML(element);
      static_cast<FiniteElementsPage*>(page(PageFiniteElements))->elements->initializeUsingXML(element);
      static_cast<FiniteElementsPage*>(page(PageFiniteElements))->exs->initializeUsingXML(element);
      static_cast<DampingPage*>(page(PageDamp))->mDamp->initializeUsingXML(element);
      static_cast<DampingPage*>(page(PageDamp))->pDamp->initializeUsingXML(element);
      static_cast<LastPage*>(page(PageLast))->inputFile->initializeUsingXML(element);
      if(MBXMLUtils::E(element->getFirstElementChild())->getTagName()==MBSIMFLEX%"nodes")
	static_cast<FirstPage*>(page(PageFirst))->rb1->setChecked(true);
      else if(MBXMLUtils::E(element->getFirstElementChild())->getTagName()==MBSIMFLEX%"resultFileName")
	static_cast<FirstPage*>(page(PageFirst))->rb2->setChecked(true);
      else if(MBXMLUtils::E(element->getFirstElementChild())->getTagName()==MBSIMFLEX%"numberOfNodes")
	static_cast<FirstPage*>(page(PageFirst))->rb3->setChecked(true);
      if(MBXMLUtils::E(element->getFirstElementChild())->getTagName()==MBSIMFLEX%"youngsModulus")
	static_cast<FirstPage*>(page(PageFirst))->rb4->setChecked(true);
      if(MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"modeShapeMatrix"))
	static_cast<ReductionMethodsPage*>(page(PageRedMeth))->rb2->setChecked(true);
      else
	static_cast<ReductionMethodsPage*>(page(PageRedMeth))->rb1->setChecked(true);
    }
  }

  QString FlexibleBodyTool::getInputDataFile() const {
    return static_cast<LastPage*>(page(PageLast))->getFile(); 
  }

}
