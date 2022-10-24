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
#include "hdf5serie/file.h"
#include "hdf5serie/simpledataset.h"
#include <QRadioButton>
#include <QMessageBox>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMLSSerializer.hpp>
#include <fmatvec/sparse_linear_algebra_double.h>

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  extern MainWindow *mw;

  MatV reduceMat(const map<int,map<int,double[4]>> &Km, const Indices &iN, const Indices &iH) {
    vector<int> mapR(Km.size()), mapC(Km.size());
    size_t hN=0, kN=0, hH=0, kH=0;
    for(size_t i=0; i<mapR.size(); i++) {
      if(hN<iN.size() and i==iN[hN]) {
	hN++;
	mapR[i] = kN++;
      }
      else
	mapR[i] = -1;
      if(hH<iH.size() and i==iH[hH]) {
	hH++;
	mapC[i] = kH++;
      }
      else
	mapC[i] = -1;
    }
    MatV Kmr(iN.size(),iH.size());
    for(const auto & i : Km) {
      for(const auto & j : i.second) {
	if(mapR[i.first]>=0 and mapC[j.first]>=0)
	  Kmr(mapR[i.first],mapC[j.first]) = j.second[3];
	else if(mapR[j.first]>=0 and mapC[i.first]>=0)
	  Kmr(mapR[j.first],mapC[i.first]) = j.second[3];
      }
    }
    return Kmr;
  }

  map<int,map<int,double[4]>> reduceMat(const map<int,map<int,double[4]>> &MKm, const Indices &iF) {
    map<int,map<int,double[4]>> MKmr;
    vector<int> map(MKm.size());
    size_t h=0, k=0;
    for(size_t i=0; i<map.size(); i++) {
      if(h<iF.size() and i==iF[h]) {
	h++;
	map[i] = k++;
      }
      else
	map[i] = -1;
    }
    for(const auto & i : MKm) {
      if(map[i.first]>=0) {
	for(const auto & j : i.second) {
	  if(map[j.first]>=0) {
	    auto d = MKmr[map[i.first]][map[j.first]];
	    for(int k=0; k<4; k++)
	      d[k] = j.second[k];
	  }
	}
      }
    }
    return MKmr;
  }

  vector<SymSparseMat> createPPKs(const map<int,map<int,double[4]>> &PPKm) {
    int nze=0;
    for(const auto & i : PPKm)
      nze+=i.second.size();
    int n = PPKm.size();
    vector<SymSparseMat> PPKs(4,SymSparseMat(n,nze,NONINIT));
    int k=0, l=0;
    for(int h=0; h<4; h++)
      PPKs[h].Ip()[0] = 0;
    for(const auto & i : PPKm) {
      for(const auto & j : i.second) {
	for(int h=0; h<4; h++) {
	  PPKs[h].Jp()[l] = j.first;
	  PPKs[h]()[l] = j.second[h];
	}
	l++;
      }
      k++;
      for(int h=0; h<4; h++)
	PPKs[h].Ip()[k] = l;
    }
    return PPKs;
  }

  pair<SymSparseMat,SymSparseMat> createMKs(const map<int,map<int,double[4]>> &MKm) {
    int nze=0;
    for(const auto & i : MKm)
      nze+=i.second.size();
    int n = MKm.size();
    SymSparseMat Ks(n,nze,NONINIT);;
    SymSparseMat Ms(n,nze,NONINIT);;
    int k=0, l=0;
    Ms.Ip()[0] = 0;
    Ks.Ip()[0] = 0;
    for(const auto & i : MKm) {
      for(const auto & j : i.second) {
	Ms.Jp()[l] = j.first;
	Ms()[l] = j.second[0]+j.second[1]+j.second[2];
	Ks.Jp()[l] = j.first;
	Ks()[l] = j.second[3];
	l++;
      }
      k++;
      Ms.Ip()[k] = l;
      Ks.Ip()[k] = l;
    }
    return make_pair(Ms,Ks);
  }

  vector<SparseMat> createPPs(const map<int,map<int,double[3]>> &PPm) {
    int nze=0;
    for(const auto & i : PPm)
      nze+=i.second.size();
    int n = PPm.size();
    vector<SparseMat> PPs(3,SparseMat(n,n,nze,NONINIT));
    int k=0, l=0;
    for(int h=0; h<3; h++)
      PPs[h].Ip()[0] = 0;
    for(const auto & i : PPm) {
      for(const auto & j : i.second) {
	for(int h=0; h<3; h++) {
	  PPs[h].Jp()[l] = j.first;
	  PPs[h]()[l] = j.second[h];
	}
	l++;
      }
      k++;
      for(int h=0; h<3; h++)
	PPs[h].Ip()[k] = l;
    }
    return PPs;
  }

  MatV readMat(const std::string &file) {
    ifstream is(file);
    string buf, buf2;
    getline(is,buf);
    int m=0;
    while(!is.eof()) {
      getline(is,buf2);
      m++;
    }
    is.close();

    istringstream iss(buf);
    double val;
    char s;
    int n=0;
    while(!iss.eof()) {
      iss >> val;
      int c = iss.peek();
      if(c==44 or c==59 or c==13)
	iss >> s;
      n++;
    }

    MatV A(m,n);
    is.open(file);
    for(int i=0; i<m; i++) {
      for(int j=0; j<n; j++) {
	is >> A(i,j);
	int c = is.peek();
	if(c==44 or c==59)
	  is >> s;
      }
    }
    is.close();

    return A;
  }

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

    V = new ExtWidget("Mode shape matrix",new FileWidget("","Nodes file","ASCII files (*.asc);;All files (*.*)",0,true),false,false,MBSIMFLEX%"modeShapeMatrix");
    layout->addWidget(V);

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
    if(wizard()->hasVisitedPage(FlexibleBodyTool::PageFlexibleBeam))
      return FlexibleBodyTool::PageDamp;
    else
      return FlexibleBodyTool::PageOMBV;
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
    nodeMap.clear();
    rPdm.clear();
    PPdm.clear();
    KrKP.clear();
    Phi.clear();
    Psi.clear();
    sigmahel.clear();
    Phis.clear();
    Psis.clear();
    sigmahels.clear();
    indices.clear();

    if(hasVisitedPage(PageExtFE)) {
      stiff();
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
	if(hasVisitedPage(PageCMS)) {
	  static_cast<BoundaryConditionsPage*>(page(PageBC))->bc->writeXMLFile(element);
	  static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->inodes->writeXMLFile(element);
	  static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->nmodes->writeXMLFile(element);
	  static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->fbnm->writeXMLFile(element);
	}
	else if(hasVisitedPage(PageModeShapes))
	  static_cast<ModeShapesPage*>(page(PageModeShapes))->V->writeXMLFile(element);
	static_cast<OpenMBVPage*>(page(PageOMBV))->ombvIndices->writeXMLFile(element);
      }
      else if(hasVisitedPage(PageCalculix))
	static_cast<CalculixPage*>(page(PageCalculix))->file->writeXMLFile(element);
      else if(hasVisitedPage(PageFlexibleBeam)) {
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
	static_cast<BoundaryConditionsPage*>(page(PageBC))->bc->writeXMLFile(element);
	static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->inodes->writeXMLFile(element);
	static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->nmodes->writeXMLFile(element);
	static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->fbnm->writeXMLFile(element);
      }
      static_cast<DampingPage*>(page(PageDamp))->mDamp->writeXMLFile(element);
      static_cast<DampingPage*>(page(PageDamp))->pDamp->writeXMLFile(element);
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
      static_cast<DampingPage*>(page(PageDamp))->mDamp->initializeUsingXML(element);
      static_cast<DampingPage*>(page(PageDamp))->pDamp->initializeUsingXML(element);
      static_cast<LastPage*>(page(PageLast))->inputFile->initializeUsingXML(element);
      if(MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodes"))
	static_cast<FirstPage*>(page(PageFirst))->rb1->setChecked(true);
      else if(MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"resultFileName"))
	static_cast<FirstPage*>(page(PageFirst))->rb2->setChecked(true);
      else if(MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"numberOfNodes"))
	static_cast<FirstPage*>(page(PageFirst))->rb3->setChecked(true);
      if(MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"modeShapeMatrix"))
	static_cast<ReductionMethodsPage*>(page(PageRedMeth))->rb2->setChecked(true);
    }
  }

  QString FlexibleBodyTool::getInputDataFile() const {
    return static_cast<LastPage*>(page(PageLast))->getFile(); 
  }

  void FlexibleBodyTool::stiff() {

    string str = static_cast<FileWidget*>(static_cast<ExternalFiniteElementsPage*>(page(PageExtFE))->nodes->getWidget())->getFile(true).toStdString();
    if(!str.empty())
      r <<= readMat(str);
    str = static_cast<FileWidget*>(static_cast<ExternalFiniteElementsPage*>(page(PageExtFE))->mass->getWidget())->getFile(true).toStdString();
    if(!str.empty())
      M <<= readMat(str);
    str = static_cast<FileWidget*>(static_cast<ExternalFiniteElementsPage*>(page(PageExtFE))->stiff->getWidget())->getFile(true).toStdString();
    if(!str.empty())
      K <<= readMat(str);

    if(K.cols()==3) {
      if(M.cols()==K.cols()) {
	for(int i=0; i<K.rows(); i++) {
	  auto d = MKm[K(i,0)-1][K(i,1)-1];
	  d[0] = M(i,2);
	  d[3] = K(i,2);
	}
      }
      else {
	for(int i=0; i<K.rows(); i++) {
	  auto d = MKm[K(i,0)-1][K(i,1)-1];
	  d[3] = K(i,2);
	}
      }
    } else {
      for(int i=0; i<K.rows(); i++) {
	auto d = MKm[3*(K(i,2)-1)+K(i,3)-1][3*(K(i,0)-1)+K(i,1)-1];
	d[3] = K(i,4);
      }
    }

    if(nodeMap.empty()) {
      for(int i=0; i<r.rows(); i++)
	nodeMap[i+1] = i;
    }
    nN = nodeMap.size();

    net = 3;
    ner = 0;
    nen = net + ner;
  }

  void FlexibleBodyTool::cms() {
    std::map<int,VecVI> bc;
    std::vector<VecVI> dof;
    std::vector<VecVI> bnodes;
    bool fixedBoundaryNormalModes = false;
    int nr = 0;
    auto *list = static_cast<ListWidget*>(static_cast<BoundaryConditionsPage*>(page(PageBC))->bc->getWidget());
    for(int i=0; i<list->getSize(); i++) {
      bnodes.push_back(VecVI(static_cast<BoundaryConditionWidget*>(list->getWidget(i))->getNodes().toStdString().c_str()));
      //dof.push_back(VecVI(static_cast<BoundaryConditionWidget*>(list->getWidget(i))->getDof().toStdString().c_str()));
      auto dof_ = VecVI(static_cast<BoundaryConditionWidget*>(list->getWidget(i))->getDof().toStdString().c_str());
      for(int i=0; i<dof_.size(); i++)
        dof_.e(i)--;
      dof.push_back(dof_);
    }

    VecVI inodes;
    if(static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->inodes->isActive()) {
      auto mat = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->inodes->getWidget())->getWidget())->getEvalMat();
      inodes.resize(mat.size(),NONINIT);
      for(size_t i=0; i<mat.size(); i++)
	inodes(i) = mat[i][0].toDouble();
    }

    VecVI nmodes;
    if(static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->nmodes->isActive()) {
      auto mat = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->nmodes->getWidget())->getWidget())->getEvalMat();
      nmodes.resize(mat.size(),NONINIT);
      for(size_t i=0; i<mat.size(); i++)
	nmodes(i) = mat[i][0].toDouble();
    }

    if(bnodes.size() != dof.size())
      runtime_error("(FlexibleBodyTool::init): number of boundary nodes (" + to_string(bnodes.size()) + ") must equal number of degrees of freedom (" + to_string(dof.size()) + ")");
    for(size_t i=0; i<bnodes.size(); i++) {
      for(int j=0; j<bnodes[i].size(); j++) {
	bc[bnodes[i](j)].resize(nen);
	for(int k=0; k<dof[i].size(); k++) {
	  if(dof[i](k)<0 or dof[i](k)>nen-1)
	    runtime_error("(FlexibleBodyTool::init): degrees of freedom of boundary node number (" + to_string(i) + ") must be within range [0,3]");
	  bc[bnodes[i](j)](dof[i](k)) = 1;
	}
      }
    }

    for(const auto & i : bc)
      for(int j=0; j<i.second.size(); j++)
	nr += i.second(j);

    int ng = nN*nen;
    int n = ng-nr;

    vector<int> c;
    for(const auto & i : bc) {
      for(int j=0; j<i.second.size(); j++)
	if(i.second(j)) c.push_back(nodeMap[i.first]*nen+j);
    }
    sort(c.begin(), c.end());

    size_t h=0;
    Indices iF, iX;
    for(int i=0; i<ng; i++) {
      if(h<c.size() and i==c[h]) {
	h++;
	iX.add(i);
      }
      else
	iF.add(i);
    }

    c.clear();
    for(int i=0; i<inodes.size(); i++) {
      VecVI bci = bc[inodes(i)];
      for(int j=0; j<nen; j++) {
	if((not bci.size()) or (not bci(j)))
	  c.push_back(nodeMap[inodes(i)]*nen+j);
      }
    }
    sort(c.begin(), c.end());
    h=0;
    Indices iH, iN;
    for(int i=0; i<iF.size(); i++) {
      if(h<c.size() and iF[i]==c[h]) {
	iH.add(i);
	h++;
      }
      else
	iN.add(i);
    }

    auto MKmr = reduceMat(MKm,iF);
    auto MKmrn = reduceMat(MKmr,iN);
    auto MKsr = createMKs(MKmr);
    auto MKsrn = createMKs(MKmrn);

    MatV Vsd(n,iH.size()+nmodes.size(),NONINIT);
    if(iH.size()) {
      Indices IJ;
      for(int i=0; i<iH.size(); i++)
	IJ.add(i);
      MatV Vs(iF.size(),iH.size(),NONINIT);
      Vs.set(iN,IJ,-slvLU(MKsrn.second,reduceMat(MKmr,iN,iH)));
      Vs.set(iH,IJ,MatV(iH.size(),iH.size(),Eye()));
      Vsd.set(RangeV(0,n-1),RangeV(0,Vs.cols()-1),Vs);
    }

    if(nmodes.size()) {
      Mat V;
      Vec w;
      if(fixedBoundaryNormalModes) {
	eigvec(MKsrn.second,MKsrn.first,6+nmodes.size(),1,V,w);
	vector<int> imod;
	for(int i=0; i<w.size(); i++) {
	  if(w(i)>pow(2*M_PI*0.1,2))
	    imod.push_back(i);
	}
	if(min(nmodes)<1 or max(nmodes)>(int)imod.size())
	  runtime_error(string("(FlexibleBodyTool::init): node numbers do not match, must be within the range [1,") + to_string(imod.size()) + "]");
	for(int i=0; i<nmodes.size(); i++) {
	  Vsd.set(iN,iH.size()+i,V.col(imod[nmodes(i)-1]));
	  Vsd.set(iH,iH.size()+i,Vec(iH.size()));
	}
      }
      else {
	eigvec(MKsr.second,MKsr.first,6+nmodes.size(),1,V,w);
	vector<int> imod;
	for(int i=0; i<w.size(); i++) {
	  if(w(i)>pow(2*M_PI*0.1,2))
	    imod.push_back(i);
	}
	if(min(nmodes)<1 or max(nmodes)>(int)imod.size())
	  runtime_error(string("(FlexibleBodyTool::init): node numbers do not match, must be within the range [1,") + to_string(imod.size()) + "]");
	for(int i=0; i<nmodes.size(); i++)
	  Vsd.set(iH.size()+i,V.col(imod[nmodes(i)-1]));
      }
    }

    if(iH.size()) {
      SqrMat V;
      Vec w;
      eigvec(JTMJ(MKsr.second,Vsd),JTMJ(MKsr.first,Vsd),V,w);
      vector<int> imod;
      for(int i=0; i<w.size(); i++) {
	if(w(i)>pow(2*M_PI*0.1,2))
	  imod.push_back(i);
      }
      MatV Vr(w.size(),imod.size(),NONINIT);
      for(size_t i=0; i<imod.size(); i++)
	Vr.set(i,V.col(imod[i]));
      Vsd <<= Vsd*Vr;
    }
    nM = Vsd.cols();
    Phi_.resize(ng,nM,NONINIT);
    Indices IJ;
    for(int i=0; i<nM; i++)
      IJ.add(i);
    Phi_.set(iF,IJ,Vsd);
    Phi_.set(iX,IJ,Mat(iX.size(),IJ.size()));
  }

  void FlexibleBodyTool::msm() {
    string str = static_cast<FileWidget*>(static_cast<ModeShapesPage*>(page(PageModeShapes))->V->getWidget())->getFile(true).toStdString();
    if(!str.empty())
      Phi_ <<= readMat(str);
    nM = Phi_.cols();
  }

  void FlexibleBodyTool::ombv() {
    if(static_cast<OpenMBVPage*>(page(PageOMBV))->ombvIndices->isActive()) {
      string str = static_cast<FileWidget*>(static_cast<OpenMBVPage*>(page(PageOMBV))->ombvIndices->getWidget())->getFile(true).toStdString();
      MatV ombvIndices;
      if(!str.empty())
	ombvIndices <<= readMat(str);
      indices.resize(ombvIndices.rows());
      for(int i=0; i<ombvIndices.rows(); i++)
	indices[i] = ombvIndices(i,0)-1;
    }
  }

  void FlexibleBodyTool::calculix() {
    net = 3;
    ner = 0;
    nen = net + ner;
    string resultFileName = static_cast<FileWidget*>(static_cast<CalculixPage*>(page(PageCalculix))->file->getWidget())->getFile(true).toStdString();
    string jobname = resultFileName.substr(0,resultFileName.length()-4);

    ifstream isDOF(jobname+".dof");
    // dof
    std::vector<std::pair<size_t,size_t>> dof;
    double d;
    while(true) {
      isDOF >> d;
      if(isDOF.eof()) break;
      dof.push_back(make_pair(size_t(d)-1,int(d*10)-size_t(d)*10-1));
    }
    isDOF.close();

    ifstream isRes(jobname+".frd");
    // nodes
    string str;
    while(isRes) {
      getline(isRes,str);
      if(str.length()>6 and str.substr(4,2)=="2C")
	break;
    }
    stringstream sN(str);
    sN >> str >> nN;
    r.resize(nN,3);
    for(size_t i=0; i<nN; i++) {
      isRes >> d >> d;
      nodeMap[d] = i;
      for(size_t k=0; k<3; k++)
	isRes >> r.e(i,k);
    }
    // elements
    while(isRes) {
      getline(isRes,str);
      if(str.length()>6 and str.substr(4,2)=="3C")
	break;
    }
    stringstream sE(str);
    sE >> str >> nE;
    Matrix<General,Var,Var,int> eles(nE,20,NONINIT);
    size_t type, nNpE;
    for(size_t i=0; i<nE; i++) {
      isRes >> str >> str >> type;
      if(type==4)
	nNpE = 20;
      else
	runtime_error("Unknown element type.");
      getline(isRes,str);
      for(size_t j=0; j<nNpE;) {
	isRes >> d;
	if(d>0) {
	  eles.e(i,j) = d;
	  j++;
	}
      }
      getline(isRes,str);
    }
    std::vector<VecV> disp;
    std::vector<VecV> stress;
    size_t i, nN_;
    while(isRes) {
      getline(isRes,str);
      if(str.length()>6 and str.substr(2,4)=="100C") {
	//        cout << str[57] << endl;
	stringstream s(str);
	s >> str >> str >> str >> nN_;
	if(nN != nN_) runtime_error("Number of nodes does not match.");
	isRes >> i >> str;
	if(str=="DISP") {
	  VecV dispi(3*nN,NONINIT);
	  double d;
	  string str;
	  for(size_t i=0; i<5; i++)
	    getline(isRes,str);
	  for(size_t i=0; i<nN; i++) {
	    isRes >> d >> d;
	    for(size_t k=0; k<3; k++)
	      isRes >> dispi.e(3*i+k);
	  }
	  disp.push_back(dispi);
	}
	else if(str=="STRESS") {
	  VecV stressi(6*nN,NONINIT);
	  double d;
	  string str;
	  for(size_t i=0; i<7; i++)
	    getline(isRes,str);
	  for(size_t i=0; i<nN; i++) {
	    isRes >> d >> d;
	    for(size_t k=0; k<6; k++)
	      isRes >> stressi.e(6*i+k);
	  }
	  stress.push_back(stressi);
	}
      }
    }
    nM = disp.size();
    isRes.close();

    M <<= readMat(jobname+".mas");
    K <<= readMat(jobname+".sti");
    for(int i=0; i<K.rows(); i++) {
      auto d = MKm[K(i,0)-1][K(i,1)-1];
      d[0] = M(i,2);
      d[3] = K(i,2);
    }

    Phi_.resize(3*nN,nM,NONINIT);
    Sr.resize(6*nN,nM,NONINIT);
    for(int j=0; j<Phi_.cols(); j++) {
      for(int i=0; i<Phi_.rows(); i++)
	Phi_.e(i,j) = disp[j].e(i);
      for(int i=0; i<Sr.rows(); i++)
	Sr.e(i,j) = stress[j].e(i);
    }

    indices.resize(5*6*eles.rows());
    int j = 0;
    for(int i=0; i<eles.rows(); i++) {
      indices[j++] = nodeMap[eles(i,3)];
      indices[j++] = nodeMap[eles(i,2)];
      indices[j++] = nodeMap[eles(i,1)];
      indices[j++] = nodeMap[eles(i,0)];
      indices[j++] = -1;
      indices[j++] = nodeMap[eles(i,4)];
      indices[j++] = nodeMap[eles(i,5)];
      indices[j++] = nodeMap[eles(i,6)];
      indices[j++] = nodeMap[eles(i,7)];
      indices[j++] = -1;
      indices[j++] = nodeMap[eles(i,1)];
      indices[j++] = nodeMap[eles(i,2)];
      indices[j++] = nodeMap[eles(i,6)];
      indices[j++] = nodeMap[eles(i,5)];
      indices[j++] = -1;
      indices[j++] = nodeMap[eles(i,2)];
      indices[j++] = nodeMap[eles(i,3)];
      indices[j++] = nodeMap[eles(i,7)];
      indices[j++] = nodeMap[eles(i,6)];
      indices[j++] = -1;
      indices[j++] = nodeMap[eles(i,4)];
      indices[j++] = nodeMap[eles(i,7)];
      indices[j++] = nodeMap[eles(i,3)];
      indices[j++] = nodeMap[eles(i,0)];
      indices[j++] = -1;
      indices[j++] = nodeMap[eles(i,0)];
      indices[j++] = nodeMap[eles(i,1)];
      indices[j++] = nodeMap[eles(i,5)];
      indices[j++] = nodeMap[eles(i,4)];
      indices[j++] = -1;
    }
  }

  void FlexibleBodyTool::beam() {
    nN = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->n->getWidget())->getWidget())->getEvalMat()[0][0].toInt();
    auto l = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->l->getWidget())->getWidget())->getEvalMat()[0][0].toDouble();
    auto A = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->A->getWidget())->getWidget())->getEvalMat()[0][0].toDouble();
    auto I_ = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->I->getWidget())->getWidget())->getEvalMat();
    auto Iy = I_[0][0].toDouble();
    auto Iz = I_[1][0].toDouble();
    auto Iyz = I_[2][0].toDouble();
    auto E = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->E->getWidget())->getWidget())->getEvalMat()[0][0].toDouble();
    auto rho = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->rho->getWidget())->getWidget())->getEvalMat()[0][0].toDouble();
    auto ten = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->ten->getWidget())->getWidget())->getEvalMat()[0][0].toInt();
    auto benz = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->benz->getWidget())->getWidget())->getEvalMat()[0][0].toInt();
    auto beny = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->beny->getWidget())->getWidget())->getEvalMat()[0][0].toInt();
    auto tor = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<FlexibleBeamPage*>(page(PageFlexibleBeam))->tor->getWidget())->getWidget())->getEvalMat()[0][0].toInt();
    for(int i=0; i<nN; i++)
      nodeMap[i+1] = i;
    nE = nN-1;
    int nee = 0;
    const int x = 0;
    const int y = 1;
    const int z = 2;
    int ul=-1, vl=-1, wl=-1, all=-1, bel=-1, gal=-1, ur=-1, vr=-1, wr=-1, alr=-1, ber=-1, gar=-1;
    if(ten)
      ul = nee++;
    if(benz)
      vl = nee++;
    if(beny)
      wl = nee++;
    net = nee;
    if(tor)
      all = nee++;
    if(beny)
      bel = nee++;
    if(benz)
      gal = nee++;
    nen = nee;
    ner = nen - net;
    if(ten)
      ur = nee++;
    if(benz)
      vr = nee++;
    if(beny)
      wr = nee++;
    if(tor)
      alr = nee++;
    if(beny)
      ber = nee++;
    if(benz)
      gar = nee++;
    int ng = nN*nen;

    vector<Mat3xV> rPdme(3,Mat3xV(nee));
    rPdm.resize(3,Mat3xV(ng));
    vector<vector<SqrMatV>> PPdme(3,vector<SqrMatV>(3,SqrMatV(nee)));
    PPdm.resize(3,vector<SqrMatV>(3,SqrMatV(ng)));
    Mat3xV Pdme(nee);
    Pdm.resize(ng);
    SymMatV Kee(nee);
    Ke0.resize(ng);
    KrKP.resize(nN,Vec3());

    double D = l/nE;

    m = rho*A*l;
    double me = m/nE;

    rdm(x) = m*l/2;

    rrdm(x,x) = m/3*pow(l,2);
    rrdm(y,y) = rho*l*Iz;
    rrdm(y,z) = -rho*l*Iyz;
    rrdm(z,z) = rho*l*Iy;

    if(ten) {
      Pdme(x,ul) = me/2;
      Pdme(x,ur) = me/2;
    }
    if(benz) {
      Pdme(y,vl) = me/2;
      Pdme(y,gal) = D*me/12;
      Pdme(y,vr) = me/2;
      Pdme(y,gar) = -D*me/12;
    }
    if(beny) {
      Pdme(z,wl) = me/2;
      Pdme(z,bel) = -D*me/12;
      Pdme(z,wr) = me/2;
      Pdme(z,ber) = D*me/12;
    }

    if(ten) {
      PPdme[x][x](ul,ul) = me/3;
      PPdme[x][x](ul,ur) = me/6;
      PPdme[x][x](ur,ul) = me/6;
      PPdme[x][x](ur,ur) = me/3;
    }

    if(benz) { // v and ga
      PPdme[x][x](vl,vl) = 6./5/D*rho*Iz;
      PPdme[x][x](vl,gal) = 1./10*rho*Iz;
      PPdme[x][x](vl,vr) = -6./5/D*rho*Iz;
      PPdme[x][x](vl,gar) = 1./10*rho*Iz;
      PPdme[x][x](gal,gal) = 2*D/15*rho*Iz;
      PPdme[x][x](gal,vr) = -1./10*rho*Iz;
      PPdme[x][x](gal,gar) = -D/30*rho*Iz;
      PPdme[x][x](vr,vr) = 6./5/D*rho*Iz;
      PPdme[x][x](vr,gar) = -1./10*rho*Iz;
      PPdme[x][x](gar,gar) = 2*D/15*rho*Iz;
      if(beny) { // w and be
	PPdme[x][x](vl,wl) = -6./5/D*rho*Iyz;
	PPdme[x][x](vl,bel) = 1./10*rho*Iyz;
	PPdme[x][x](vl,wr) = 6./5/D*rho*Iyz;
	PPdme[x][x](vl,ber) = 1./10*rho*Iyz;
	PPdme[x][x](gal,wr) = 1./10*rho*Iyz;
	PPdme[x][x](gal,ber) = -D/30*rho*Iyz;
	PPdme[x][x](vr,wr) = -6./5/D*rho*Iyz;
	PPdme[x][x](vr,ber) = -1./10*rho*Iyz;
      }
    }
    if(beny) { // w and be
      PPdme[x][x](wl,wl) = 6./5/D*rho*Iy;
      PPdme[x][x](wl,bel) = -1./10*rho*Iy;
      PPdme[x][x](wl,wr) = -6./5/D*rho*Iy;
      PPdme[x][x](wl,ber) = -1./10*rho*Iy;
      PPdme[x][x](bel,bel) = 2*D/15*rho*Iy;
      PPdme[x][x](bel,wr) = 1./10*rho*Iy;
      PPdme[x][x](bel,ber) = -D/30*rho*Iy;
      PPdme[x][x](wr,wr) = 6./5/D*rho*Iy;
      PPdme[x][x](wr,ber) = 1./10*rho*Iy;
      PPdme[x][x](ber,ber) = 2*D/15*rho*Iy;
      if(benz) { // v and ga
	PPdme[x][x](wl,gal) = -1./10*rho*Iyz;
	PPdme[x][x](wl,vr) = 6./5/D*rho*Iyz;
	PPdme[x][x](wl,gar) = -1./10*rho*Iyz;
	PPdme[x][x](bel,gal) = 2*D/15*rho*Iyz;
	PPdme[x][x](bel,vr) = -1./10*rho*Iyz;
	PPdme[x][x](bel,gar) = -D/30*rho*Iyz;
	PPdme[x][x](wr,gar) = 1./10*rho*Iyz;
	PPdme[x][x](ber,gar) = 2*D/15*rho*Iyz;
      }
    }
    if(benz) { // v and ga
      PPdme[y][y](vl,vl) = 13./35*me;
      PPdme[y][y](vl,gal) = 11./210*me*D;
      PPdme[y][y](vl,vr) = 9./70*me;
      PPdme[y][y](vl,gar) = -13./420*me*D;
      PPdme[y][y](gal,gal) = me*pow(D,2)/105;
      PPdme[y][y](gal,vr) = 13./420*me*D;
      PPdme[y][y](gal,gar) = -me*pow(D,2)/140;
      PPdme[y][y](vr,vr) = 13./35*me;
      PPdme[y][y](vr,gar) = -11./210*me*D;
      PPdme[y][y](gar,gar) = me*pow(D,2)/105;
    }
    if(beny) { // w and be
      PPdme[z][z](wl,wl) = 13./35*me;
      PPdme[z][z](wl,bel) = -11./210*me*D;
      PPdme[z][z](wl,wr) = 9./70*me;
      PPdme[z][z](wl,ber) = 13./420*me*D;
      PPdme[z][z](bel,bel) = me*pow(D,2)/105;
      PPdme[z][z](bel,wr) = -13./420*me*D;
      PPdme[z][z](bel,ber) = -me*pow(D,2)/140;
      PPdme[z][z](wr,wr) = 13./35*me;
      PPdme[z][z](wr,ber) = 11./210*me*D;
      PPdme[z][z](ber,ber) = me*pow(D,2)/105;
    }
    if(tor) {
      PPdme[y][y](all,all) = rho*Iy*D/3;
      PPdme[y][y](all,alr) = rho*Iy*D/6;
      PPdme[y][y](alr,alr) = rho*Iy*D/3;
      PPdme[z][z](all,all) = rho*Iz*D/3;
      PPdme[z][z](all,alr) = rho*Iz*D/6;
      PPdme[z][z](alr,alr) = rho*Iz*D/3;
      PPdme[y][z](all,all) = D/3*rho*Iyz;
      PPdme[y][z](all,alr) = D/6*rho*Iyz;
      PPdme[y][z](alr,all) = D/6*rho*Iyz;
      PPdme[y][z](alr,alr) = D/3*rho*Iyz;
    }
    if(ten and benz) { // u, v and ga
      PPdme[x][y](ul,vl) = 7./20*me;
      PPdme[x][y](ul,gal) = 1./20*me*D;
      PPdme[x][y](ul,vr) = 3./20*me;
      PPdme[x][y](ul,gar) = -1./30*me*D;
      PPdme[x][y](ur,vl) = 3./20*me;
      PPdme[x][y](ur,gal) = 1./30*me*D;
      PPdme[x][y](ur,vr) = 7./20*me;
      PPdme[x][y](ur,gar) = -1./20*me*D;
    }
    if(ten and beny) { // u, w and be
      PPdme[x][z](ul,wl) = 7./20*me;
      PPdme[x][z](ul,bel) = -1./20*me*D;
      PPdme[x][z](ul,wr) = 3./20*me;
      PPdme[x][z](ul,ber) = 1./30*me*D;
      PPdme[x][z](ur,wl) = 3./20*me;
      PPdme[x][z](ur,bel) = -1./30*me*D;
      PPdme[x][z](ur,wr) = 7./20*me;
      PPdme[x][z](ur,ber) = 1./20*me*D;
    }
    if(tor and benz) { // al, v and ga
      PPdme[x][y](vl,all) = 1./2*rho*Iyz;
      PPdme[x][y](vl,alr) = 1./2*rho*Iyz;
      PPdme[x][y](gal,all) = -D/12*rho*Iyz;
      PPdme[x][y](gal,alr) = D/12*rho*Iyz;
      PPdme[x][y](vr,all) = -1./2*rho*Iyz;
      PPdme[x][y](vr,alr) = -1./2*rho*Iyz;
      PPdme[x][y](gar,all) = D/12*rho*Iyz;
      PPdme[x][y](gar,alr) = -D/12*rho*Iyz;
      PPdme[x][z](vl,all) = 1./2*rho*Iz;
      PPdme[x][z](vl,alr) = 1./2*rho*Iz;
      PPdme[x][z](gal,all) = -D/12*rho*Iz;
      PPdme[x][z](gal,alr) = D/12*rho*Iz;
      PPdme[x][z](vr,all) = -1./2*rho*Iz;
      PPdme[x][z](vr,alr) = -1./2*rho*Iz;
      PPdme[x][z](gar,all) = D/12*rho*Iz;
      PPdme[x][z](gar,alr) = -D/12*rho*Iz;
    }
    if(tor and beny) { // al, w and be
      PPdme[x][y](wl,all) = -1./2*rho*Iy;
      PPdme[x][y](wl,alr) = -1./2*rho*Iy;
      PPdme[x][y](bel,all) = -D/12*rho*Iy;
      PPdme[x][y](bel,alr) = D/12*rho*Iy;
      PPdme[x][y](wr,all) = 1./2*rho*Iy;
      PPdme[x][y](wr,alr) = 1./2*rho*Iy;
      PPdme[x][y](ber,all) = D/12*rho*Iy;
      PPdme[x][y](ber,alr) = -D/12*rho*Iy;
      PPdme[x][z](wl,all) = -1./2*rho*Iyz;
      PPdme[x][z](wl,alr) = -1./2*rho*Iyz;
      PPdme[x][z](bel,all) = -D/12*rho*Iyz;
      PPdme[x][z](bel,alr) = D/12*rho*Iyz;
      PPdme[x][z](wr,all) = 1./2*rho*Iyz;
      PPdme[x][z](wr,alr) = 1./2*rho*Iyz;
      PPdme[x][z](ber,all) = D/12*rho*Iyz;
      PPdme[x][z](ber,alr) = -D/12*rho*Iyz;
    }
    if(beny and benz) {
      PPdme[y][z](vl,wl) = 13./35*me;
      PPdme[y][z](vl,bel) = -11./210*me*D;
      PPdme[y][z](vl,wr) = 9./70*me;
      PPdme[y][z](vl,ber) = 13./420*me*D;
      PPdme[y][z](gal,wl) = 11./210*me*D;
      PPdme[y][z](gal,bel) = -me*pow(D,2)/105;
      PPdme[y][z](gal,wr) = 13./420*me*D;
      PPdme[y][z](gal,ber) = me*pow(D,2)/140;
      PPdme[y][z](vr,wl) = 9./70*me;
      PPdme[y][z](vr,bel) = -13./420*me*D;
      PPdme[y][z](vr,wr) = 13./35*me;
      PPdme[y][z](vr,ber) = 11./210*me*D;
      PPdme[y][z](gar,wl) = -13./420*me*D;
      PPdme[y][z](gar,bel) = me*pow(D,2)/140;
      PPdme[y][z](gar,wr) = -11./210*me*D;
      PPdme[y][z](gar,ber) = -me*pow(D,2)/105;
    }
    for(int k=0; k<nee; k++) {
      for(int j=0; j<k; j++) {
	PPdme[x][x](k,j) = PPdme[x][x](j,k);
	PPdme[y][y](k,j) = PPdme[y][y](j,k);
	PPdme[z][z](k,j) = PPdme[z][z](j,k);
      }
    }
    PPdme[y][x] = PPdme[x][y].T();
    PPdme[z][x] = PPdme[x][z].T();
    PPdme[z][y] = PPdme[y][z].T();

    if(benz) { // v and ga
      rPdme[y](x,vl) = rho*Iz;
      rPdme[y](x,vr) = -rho*Iz;
      rPdme[z](x,vl) = -rho*Iyz;
      rPdme[z](x,vr) = rho*Iyz;
    }
    if(beny) { // w and be
      rPdme[y](x,wl) = -rho*Iyz;
      rPdme[y](x,wr) = rho*Iyz;
      rPdme[z](x,wl) = rho*Iy;
      rPdme[z](x,wr) = -rho*Iy;
    }
    if(tor) {
      rPdme[y](y,all) = D/2*rho*Iyz;
      rPdme[y](y,alr) = D/2*rho*Iyz;
      rPdme[y](z,all) = D/2*rho*Iz;
      rPdme[y](z,alr) = D/2*rho*Iz;
      rPdme[z](y,all) = -D/2*rho*Iy;
      rPdme[z](y,alr) = -D/2*rho*Iy;
      rPdme[z](z,all) = -D/2*rho*Iyz;
      rPdme[z](z,alr) = -D/2*rho*Iyz;
    }

    if(ten) {
      Kee(ul,ul) = E*A/D;
      Kee(ul,ur) = -E*A/D;
      Kee(ur,ur) = E*A/D;
    }
    if(benz) { // v and ga
      Kee(vl,vl) = 12./pow(D,3)*E*Iz;
      Kee(vl,gal) = 6./pow(D,2)*E*Iz;
      Kee(vl,vr) = -12./pow(D,3)*E*Iz;
      Kee(vl,gar) = 6./pow(D,2)*E*Iz;
      Kee(gal,gal) = 4./D*E*Iz;
      Kee(gal,vr) = -6./pow(D,2)*E*Iz;
      Kee(gal,gar) = 2./D*E*Iz;
      Kee(vr,vr) = 12./pow(D,3)*E*Iz;
      Kee(vr,gar) = -6./pow(D,2)*E*Iz;
      Kee(gar,gar) = 4./D*E*Iz;
      if(beny) { // w and be
	Kee(vl,wl) = -12./pow(D,3)*E*Iyz;
	Kee(vl,bel) = 6./pow(D,2)*E*Iyz;
	Kee(vl,wr) = 12./pow(D,3)*E*Iyz;
	Kee(vl,ber) = 6./pow(D,2)*E*Iyz;
	Kee(gal,wr) = 6./pow(D,2)*E*Iyz;
	Kee(gal,ber) = 2./D*E*Iyz;
	Kee(vr,wr) = -12./pow(D,3)*E*Iyz;
	Kee(vr,ber) = -6./pow(D,2)*E*Iyz;
      }
    }
    if(beny) { // w and be
      Kee(wl,wl) = 12./pow(D,3)*E*Iy;
      Kee(wl,bel) = -6./pow(D,2)*E*Iy;
      Kee(wl,wr) = -12./pow(D,3)*E*Iy;
      Kee(wl,ber) = -6./pow(D,2)*E*Iy;
      Kee(bel,bel) = 4./D*E*Iy;
      Kee(bel,wr) = 6./pow(D,2)*E*Iy;
      Kee(bel,ber) = 2./D*E*Iy;
      Kee(wr,wr) = 12./pow(D,3)*E*Iy;
      Kee(wr,ber) = 6./pow(D,2)*E*Iy;
      Kee(ber,ber) = 4./D*E*Iy;
      if(benz) { // v and ga
	Kee(wl,gal) = -6./pow(D,2)*E*Iyz;
	Kee(wl,vr) = 12./pow(D,3)*E*Iyz;
	Kee(wl,gar) = -6./pow(D,2)*E*Iyz;
	Kee(bel,gal) = 4./D*E*Iyz;
	Kee(bel,vr) = -6./pow(D,2)*E*Iyz;
	Kee(bel,gar) = 2./D*E*Iyz;
	Kee(wr,gar) = 6./pow(D,2)*E*Iyz;
	Kee(ber,gar) = 4./D*E*Iyz;
      }
    }
    if(tor) {
      Kee(all,all) = E/2/D*(Iy+Iz);
      Kee(all,alr) = -E/2/D*(Iy+Iz);
      Kee(alr,alr) = E/2/D*(Iy+Iz);
    }

//    map<int,map<int,double[4]>> MKm;
//    map<int,map<int,double[3]>> PPm;
    RangeV I(0,2);
    for(int i=0; i<nE; i++) {
      RangeV J(i*nen,i*nen+nee-1);
      if(ten) {
	rPdme[x](x,ul) = me*D*(i/2.+1./6);
	rPdme[x](x,ur) = me*D*(i/2.+1./3);
      }
      if(benz) { // v and ga
	rPdme[x](y,vl) = me*D*(i/2.+3./20);
	rPdme[x](y,gal) = me*D*(i*D/12.+D/30);
	rPdme[x](y,vr) = me*D*(i/2.+7./20);
	rPdme[x](y,gar) = -me*D*(i*D/12.+D/20);
      }
      if(beny) { // w and be
	rPdme[x](z,wl) = me*D*(i/2.+3./20);
	rPdme[x](z,bel) = -me*D*(i*D/12.+D/30);
	rPdme[x](z,wr) = me*D*(i/2.+7./20);
	rPdme[x](z,ber) = me*D*(i*D/12.+D/20);
      }
      Pdm.add(I,J,Pdme);
      for(int j=0; j<3; j++) {
	rPdm[j].add(I,J,rPdme[j]);
	for(int k=0; k<3; k++)
	  PPdm[j][k].add(J,J,PPdme[j][k]);
      }
      for(int j=0; j<nee; j++) {
	for(int k=0; k<j; k++) {
	  auto d2 = PPm[i*nen+j][i*nen+k];
	  d2[0] += PPdme[0][1].e(j,k);
	  d2[1] += PPdme[0][2].e(j,k);
	  d2[2] += PPdme[1][2].e(j,k);
	}
	for(int k=j; k<nee; k++) {
	  auto d = MKm[i*nen+j][i*nen+k];
	  auto d2 = PPm[i*nen+j][i*nen+k];
	  d[3] += Kee.ej(j,k);
	  d[0] += PPdme[0][0].e(j,k);
	  d[1] += PPdme[1][1].e(j,k);
	  d[2] += PPdme[2][2].e(j,k);
	  d2[0] += PPdme[0][1].e(j,k);
	  d2[1] += PPdme[0][2].e(j,k);
	  d2[2] += PPdme[1][2].e(j,k);
	}
      }
    }

    r.resize(nN,3,NONINIT);
    for(int i=0; i<nN; i++) {
      r(i,0) = i*D;
      r(i,1) = 0;
      r(i,2) = 0;
    }
    int Ip[4];
    if(ten)
      Ip[0] = 0;
    else if(benz)
      Ip[0] = 1;
    else if(beny)
      Ip[0] = 2;
    Ip[1] = ten?Ip[0]+1:Ip[0];
    Ip[2] = benz?Ip[1]+1:Ip[1];
    Ip[3] = beny?Ip[2]+1:Ip[2];
    Phis.resize(nN,SparseMat(3,ng,3,NONINIT));
    for(size_t i=0; i<nN; i++) {
      for(int j=0; j<3; j++) {
	Phis[i]()[j] = 1;
	Phis[i].Ip()[j] = Ip[j];
      }
      Phis[i].Ip()[3] = Ip[3];
      Phis[i].Jp()[0] = nen*i+ul;
      Phis[i].Jp()[1] = nen*i+vl;
      Phis[i].Jp()[2] = nen*i+wl;
    }
    if(tor)
      Ip[0] = 0;
    else if(beny)
      Ip[0] = 1;
    else if(benz)
      Ip[0] = 2;
    Ip[1] = tor?Ip[0]+1:Ip[0];
    Ip[2] = beny?Ip[1]+1:Ip[1];
    Ip[3] = benz?Ip[2]+1:Ip[2];
    Psis.resize(nN,SparseMat(3,ng,3,NONINIT));
    for(size_t i=0; i<nN; i++) {
      for(int j=0; j<3; j++) {
	Psis[i]()[j] = 1;
	Psis[i].Ip()[j] = Ip[j];
      }
      Psis[i].Ip()[3] = Ip[3];
      Psis[i].Jp()[0] = nen*i+all;
      Psis[i].Jp()[1] = nen*i+bel;
      Psis[i].Jp()[2] = nen*i+gal;
    }
    Ip[0] = ten?0:2;
    sigmahels.resize(nN,SparseMat(6,ng,2,NONINIT));
    for(size_t i=0; i<nN; i++) {
      if(i>0 and i<nN-1) {
	sigmahels[i]()[0] = -E/D/2;
	sigmahels[i]()[1] = E/D/2;
	sigmahels[i].Jp()[0] = (i-1)*nen+ul;
	sigmahels[i].Jp()[1] = (i+1)*nen+ul;
      }
      else if(i<nN-1) { // i=0
	sigmahels[i]()[0] = -E/D;
	sigmahels[i]()[1] = E/D;
	sigmahels[i].Jp()[0] = i*nen+ul;
	sigmahels[i].Jp()[1] = (i+1)*nen+ul;
      }
      else { // i=nN-1
	sigmahels[i]()[0] = -E/D;
	sigmahels[i]()[1] = E/D;
	sigmahels[i].Jp()[0] = (i-1)*nen+ul;
	sigmahels[i].Jp()[1] = i*nen+ul;
      }
      sigmahels[i].Ip()[0] = Ip[0];
      for(int j=1; j<7; j++)
	sigmahels[i].Ip()[j] = 2;
    }

    indices.resize(3*(nN-1));
    int j = 0;
    for(int i=0; i<nN-1; i++) {
      indices[j++] = i;
      indices[j++] = i+1;
      indices[j++] = -1;
    }
  }

  void FlexibleBodyTool::fma() {
   KrKP.resize(nN,Vec3(NONINIT));
    for(int i=0; i<nN; i++)
      KrKP[i] = r.row(i).T();

    Phi.resize(nN,Mat3xV(nM,NONINIT));
    for(size_t i=0; i<nN; i++)
      Phi[i] = Phis[i]*Phi_;

    if(Psis.size()) {
      Psi.resize(nN,Mat3xV(nM,NONINIT));
      for(size_t i=0; i<nN; i++)
	Psi[i] = Psis[i]*Phi_;
    }

    if(sigmahels.size()) {
      sigmahel.resize(nN,Matrix<General,Fixed<6>,Var,double>(nM,NONINIT));
      for(size_t i=0; i<nN; i++)
	sigmahel[i] = sigmahels[i]*Phi_;
    }
    auto PPKs = createPPKs(MKm);
    auto PPs = createPPs(PPm);
    Pdm <<= Pdm*Phi_;
    for(int i=0; i<3; i++) {
      rPdm[i] <<= rPdm[i]*Phi_;
      for(int j=0; j<3; j++)
	PPdm[j][j] <<= Phi_.T()*(PPKs[j]*Phi_);
      PPdm[0][1] <<= Phi_.T()*(PPs[0]*Phi_);
      PPdm[0][2] <<= Phi_.T()*(PPs[1]*Phi_);
      PPdm[1][2] <<= Phi_.T()*(PPs[2]*Phi_);
    }
    //Ke0 <<= JTMJ(Ke0,Phi_);
    //Ke0 <<= JTMJ(createSymSparseMat(K,format),Phi_);
    Ke0 <<= JTMJ(PPKs[3],Phi_);
  }

  void FlexibleBodyTool::lma() {
    Phi.resize(nN,Mat3xV(nM,NONINIT));
    for(size_t i=0; i<nN; i++)
      Phi[i] = Phi_(RangeV(nen*i,nen*i+net-1),RangeV(0,nM-1));

    if(Sr.rows()) {
      sigmahel.resize(nN,Matrix<General,Fixed<6>,Var,double>(nM,NONINIT));
      for(size_t i=0; i<nN; i++)
	sigmahel[i] = Sr(RangeV(6*i,6*i+6-1),RangeV(0,nM-1));
    }

    //Ke0 <<= JTMJ(Ke0r,Vsd);
    //Ke0 <<= JTMJ(Ke0,Phi_);
    auto MKs = createMKs(MKm);
    Ke0 <<= JTMJ(MKs.second,Phi_);

    std::map<int,Vec3> nodalPos;
    if(r.cols()==4) {
      for(int i=0; i<r.rows(); i++)
	nodalPos[r(i,0)] = r.row(i)(RangeV(1,3)).T();
    }
    else if(r.cols()==3) {
      for(int i=0; i<r.rows(); i++)
	nodalPos[i+1] = r.row(i).T();
    }
    else
      runtime_error("(FlexibleBodyTool::init): number of columns in nodes does not match, must be 3 or 4");

    KrKP.resize(nN,Vec3(NONINIT));
    for(const auto & i : nodeMap)
      KrKP[i.second] = nodalPos[i.first];

    bool lumpedMass = true;
    // compute mass and lumped mass matrix
    vector<double> mi(nN);
    m = 0;
    if(M.cols()==3) {
//      SymSparseMat Ms = createSymSparseMat(M,format);
      double ds = 0;
      for(int i=0; i<nN; i++) {
	int r = MKs.first.Ip()[i*nen];
	ds += MKs.first()[r];
	m += MKs.first()[r];
	for(int c=r+1; c<MKs.first.Ip()[i*nen+1]; c++)
	  m += 2*MKs.first()[c];
      }
      for(int i=0; i<nN; i++) {
	int r = MKs.first.Ip()[i*nen];
	mi[i] = MKs.first()[r]/ds*m;
      }
    } else if(M.cols()==1) {
      for(int i=0; i<M.rows(); i++) {
	mi[i] = M(i,0);
	m += mi[i];
      }
    }
    else
      runtime_error("lumped mass approach not available");

    Pdm.resize(nM);
    rPdm.resize(3,Mat3xV(nM));
    PPdm.resize(3,vector<SqrMatV>(3,SqrMatV(nM)));
    if(lumpedMass) {
      // compute integrals
      for(int i=0; i<nN; i++) {
	rdm += mi[i]*KrKP[i];
	rrdm += mi[i]*JTJ(KrKP[i].T());
	Pdm += mi[i]*Phi[i];
      }
      for(int k=0; k<3; k++) {
	for(int i=0; i<nN; i++)
	  rPdm[k] += mi[i]*KrKP[i](k)*Phi[i];
	for(int l=k; l<3; l++) {
	  for(int i=0; i<nN; i++)
	    PPdm[k][l] += mi[i]*Phi[i].row(k).T()*Phi[i].row(l);
	}
      }
    }
    else {
      // compute reduced mass matrix
      if(not MKs.first.size())
	runtime_error("full mass approach not available");
      PPdm[0][0] = JTMJ(MKs.first,Phi_);
    }
  }

  void FlexibleBodyTool::damp() {
    if(static_cast<DampingPage*>(page(PageDamp))->mDamp->isActive()) {
      auto mat = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<DampingPage*>(page(PageDamp))->mDamp->getWidget())->getWidget())->getEvalMat();
      mDamp.resize(mat.size(),NONINIT);
      for(size_t i=0; i<mat.size(); i++)
	mDamp(i) = mat[i][0].toDouble();
    }

    if(static_cast<DampingPage*>(page(PageDamp))->pDamp->isActive()) {
      auto mat = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<DampingPage*>(page(PageDamp))->pDamp->getWidget())->getWidget())->getEvalMat();
      for(size_t i=0; i<mat.size(); i++)
	pDamp(i) = mat[i][0].toDouble();
    }
    if(mDamp.size()) {
      SquareMatrix<Ref,double> V;
      Vector<Ref,double> w;
      eigvec(Ke0,SymMatV(PPdm[0][0]+PPdm[1][1]+PPdm[2][2]),V,w);
      Pdm <<= Pdm*V;
      for(int i=0; i<3; i++) {
	rPdm[i] <<= rPdm[i]*V;
	for(int j=i; j<3; j++)
	  PPdm[i][j] <<= V.T()*PPdm[i][j]*V;
      }
      Ke0 <<= JTMJ(Ke0,V);
      for(size_t i=0; i<Phi.size(); i++)
	Phi[i] <<= Phi[i]*V;
      for(size_t i=0; i<sigmahel.size(); i++)
	sigmahel[i] <<= sigmahel[i]*V;
      De0.resize(V.cols(),INIT,0);
      for(int i=0; i<De0.size(); i++)
	De0(i,i) = 2*sqrt((PPdm[0][0](i,i)+PPdm[1][1](i,i)+PPdm[2][2](i,i))*Ke0(i,i))*(i>=mDamp.size()?mDamp(mDamp.size()-1):mDamp(i));
    }
    else if(pDamp.e(0)>0 or pDamp.e(1)>0)
      De0 <<= pDamp.e(0)*SymMatV(PPdm[0][0]+PPdm[1][1]+PPdm[2][2]) + pDamp.e(1)*Ke0;
  }

  void FlexibleBodyTool::exp() {
    if(Pdm.cols()) {
      QString fileName = static_cast<FileWidget*>(static_cast<LastPage*>(page(PageLast))->inputFile->getWidget())->getFile(true);
      if(not(fileName.isEmpty())) {
	fileName = fileName.endsWith(".h5")?fileName:fileName+".h5";
	H5::File file(fileName.toStdString(), H5::File::write);
	auto sdata=file.createChildObject<H5::SimpleDataset<double>>("mass")();
	sdata->write(m);
	auto vdata=file.createChildObject<H5::SimpleDataset<vector<double>>>("position integral")(rdm.size());
	vdata->write((vector<double>)rdm);
	auto mdata=file.createChildObject<H5::SimpleDataset<vector<vector<double>>>>("position position integral")(rrdm.rows(),rrdm.cols());
	mdata->write((vector<vector<double>>)rrdm);
	mdata=file.createChildObject<H5::SimpleDataset<vector<vector<double>>>>("shape function integral")(Pdm.rows(),Pdm.cols());
	mdata->write((vector<vector<double>>)Pdm);
	mdata=file.createChildObject<H5::SimpleDataset<vector<vector<double>>>>("stiffness matrix")(Ke0.rows(),Ke0.cols());
	mdata->write((vector<vector<double>>)Ke0);
	if(De0.size()) {
	  mdata=file.createChildObject<H5::SimpleDataset<vector<vector<double>>>>("damping matrix")(De0.rows(),De0.cols());
	  mdata->write((vector<vector<double>>)De0);
	}
	vector<vector<double>> rPdm_(3*3,vector<double>(Pdm.cols()));
	for(int i=0; i<3; i++) {
	  for(int j=0; j<Pdm.cols(); j++) {
	    rPdm_[0*3+i][j] = rPdm[0](i,j);
	    rPdm_[1*3+i][j] = rPdm[1](i,j);
	    rPdm_[2*3+i][j] = rPdm[2](i,j);
	  }
	}
	mdata=file.createChildObject<H5::SimpleDataset<vector<vector<double>>>>("position shape function integral")(rPdm_.size(),rPdm_[0].size());
	mdata->write((vector<vector<double>>)rPdm_);
	vector<vector<double>> PPdm_(6*Pdm.cols(),vector<double>(Pdm.cols()));
	for(int i=0; i<Pdm.cols(); i++) {
	  for(int j=0; j<Pdm.cols(); j++) {
	    PPdm_[0*Pdm.cols()+i][j] = PPdm[0][0](i,j);
	    PPdm_[1*Pdm.cols()+i][j] = PPdm[0][1](i,j);
	    PPdm_[2*Pdm.cols()+i][j] = PPdm[0][2](i,j);
	    PPdm_[3*Pdm.cols()+i][j] = PPdm[1][1](i,j);
	    PPdm_[4*Pdm.cols()+i][j] = PPdm[1][2](i,j);
	    PPdm_[5*Pdm.cols()+i][j] = PPdm[2][2](i,j);
	  }
	}
	mdata=file.createChildObject<H5::SimpleDataset<vector<vector<double>>>>("shape function shape function integral")(PPdm_.size(),PPdm_[0].size());
	mdata->write((vector<vector<double>>)PPdm_);

	vector<double> r(3*nN);
	vector<vector<double>> Phi_(3*nN,vector<double>(Pdm.cols()));
	for(int i=0; i<nN; i++) {
	  for(int j=0; j<3; j++) {
	    r[i*3+j] = KrKP[i](j);
	    for(int k=0; k<Pdm.cols(); k++)
	      Phi_[i*3+j][k] = Phi[i](j,k);
	  }
	}

	vdata=file.createChildObject<H5::SimpleDataset<vector<double>>>("nodal relative position")(r.size());
	vdata->write(r);
	mdata=file.createChildObject<H5::SimpleDataset<vector<vector<double>>>>("nodal shape matrix of translation")(Phi_.size(),Phi_[0].size());
	mdata->write(Phi_);

	if(Psi.size()) {
	  vector<vector<double>> Psi_(3*nN,vector<double>(Pdm.cols()));
	  for(int i=0; i<nN; i++) {
	    for(int j=0; j<3; j++) {
	      for(int k=0; k<Pdm.cols(); k++)
		Psi_[i*3+j][k] = Psi[i](j,k);
	    }
	  }
	  mdata=file.createChildObject<H5::SimpleDataset<vector<vector<double>>>>("nodal shape matrix of rotation")(Psi_.size(),Psi_[0].size());
	  mdata->write(Psi_);
	}

	if(sigmahel.size()) {
	  vector<vector<double>> sigmahel_(6*nN,vector<double>(Pdm.cols()));
	  for(int i=0; i<nN; i++) {
	    for(int j=0; j<6; j++) {
	      for(int k=0; k<Pdm.cols(); k++) {
		sigmahel_[i*6+j][k] = sigmahel[i](j,k);
	      }
	    }
	  }
	  mdata=file.createChildObject<H5::SimpleDataset<vector<vector<double>>>>("nodal stress matrix")(sigmahel_.size(),sigmahel_[0].size());
	  mdata->write(sigmahel_);
	}

	if(indices.size()) {
	  auto vidata=file.createChildObject<H5::SimpleDataset<vector<int>>>("openmbv indices")(indices.size());
	  vidata->write(indices);
	}
      }
    }
  }

}
