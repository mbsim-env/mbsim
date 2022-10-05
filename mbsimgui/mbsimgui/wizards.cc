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
//#include <QDialogButtonBox>
#include <QRadioButton>
#include <QMessageBox>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMLSSerializer.hpp>

using namespace std;
using namespace fmatvec;

namespace MBSimGUI {

  extern MainWindow *mw;

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
    nen = net + ner;
    if(hasVisitedPage(PageExtFE)) {
      stiff();
      if(hasVisitedPage(PageCMS))
	cms();
      else if(hasVisitedPage(PageModeShapes))
	msm();
      ombv();
    }
    else
      calculix();
    damp();
    exp();
  }


  void FlexibleBodyTool::save() {
    QString file=QFileDialog::getSaveFileName(this, "Save finite elements input data file", QFileInfo(mw->getProjectFilePath()).absolutePath()+"/input_data.xml", "XML files (*.xml)");
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
      static_cast<DampingPage*>(page(PageDamp))->mDamp->initializeUsingXML(element);
      static_cast<DampingPage*>(page(PageDamp))->pDamp->initializeUsingXML(element);
      static_cast<LastPage*>(page(PageLast))->inputFile->initializeUsingXML(element);
      if(MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"resultFileName"))
	static_cast<FirstPage*>(page(PageFirst))->rb2->setChecked(true);
      if(MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"modeShapeMatrix"))
	static_cast<ReductionMethodsPage*>(page(PageRedMeth))->rb2->setChecked(true);
    }
  }

  QString FlexibleBodyTool::getInputDataFile() const {
    return static_cast<LastPage*>(page(PageLast))->getFile(); 
  }

  void FlexibleBodyTool::stiff() {
    nodeMap.clear();

    string str = static_cast<FileWidget*>(static_cast<ExternalFiniteElementsPage*>(page(PageExtFE))->nodes->getWidget())->getFile(true).toStdString();
    if(!str.empty())
      r <<= readMat(str);
    str = static_cast<FileWidget*>(static_cast<ExternalFiniteElementsPage*>(page(PageExtFE))->mass->getWidget())->getFile(true).toStdString();
    if(!str.empty())
      M <<= readMat(str);
    str = static_cast<FileWidget*>(static_cast<ExternalFiniteElementsPage*>(page(PageExtFE))->stiff->getWidget())->getFile(true).toStdString();
    if(!str.empty())
      K <<= readMat(str);

    if(nodeMap.empty()) {
      for(int i=0; i<r.rows(); i++)
	nodeMap[i+1] = i;
    }
    nN = nodeMap.size();

    if(M.cols()==3) {
      M0.resize(nN*nen);
      for(int i=0; i<M.rows(); i++)
	M0.e(M(i,0)-1,M(i,1)-1) = M(i,2);
    }

    Ke0.resize(nN*nen);
    if(K.cols()==3) {
      for(int i=0; i<K.rows(); i++)
	Ke0.e(K(i,0)-1,K(i,1)-1) = K(i,2);
    } else if(K.cols()==5) {
      for(int i=0; i<K.rows(); i++)
	Ke0(3*(K(i,0)-1)+K(i,1)-1,3*(K(i,2)-1)+K(i,3)-1) = K(i,4);
    }
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
      dof.push_back(VecVI(static_cast<BoundaryConditionWidget*>(list->getWidget(i))->getDof().toStdString().c_str()));
    }

    auto mat = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->inodes->getWidget())->getWidget())->getEvalMat();
    VecVI inodes(mat.size(),NONINIT);
    for(size_t i=0; i<mat.size(); i++)
      inodes(i) = mat[i][0].toDouble();

    mat = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<ComponentModeSynthesisPage*>(page(PageCMS))->nmodes->getWidget())->getWidget())->getEvalMat();
    VecVI nmodes(mat.size(),NONINIT);
    for(size_t i=0; i<mat.size(); i++)
      nmodes(i) = mat[i][0].toDouble();

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
    Indices IF;
    Indices IX;
    for(int i=0; i<ng; i++) {
      if(h<c.size() and i==c[h]) {
	h++;
	IX.add(i);
      }
      else
	IF.add(i);
    }

    // M0 <<= M0(IF);
    //Ke0 <<= Ke0(IF);
    SymMatV Ke0r = Ke0(IF);
    SymMatV M0r = M0(IF);

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
    Indices IH;
    Indices IN;
    for(int i=0; i<IF.size(); i++) {
      if(h<c.size() and IF[i]==c[h]) {
	IH.add(i);
	h++;
      }
      else
	IN.add(i);
    }
    MatV Vsd(n,IH.size()+nmodes.size(),NONINIT);
    if(IH.size()) {
      Indices IJ;
      for(int i=0; i<IH.size(); i++)
	IJ.add(i);
      MatV Vs(IF.size(),IH.size(),NONINIT);
      Vs.set(IN,IJ,-slvLL(Ke0r(IN),Ke0r(IN,IH)));
      Vs.set(IH,IJ,MatV(IH.size(),IH.size(),Eye()));
      Vsd.set(RangeV(0,n-1),RangeV(0,Vs.cols()-1),Vs);
    }

    if(nmodes.size()) {
      SqrMat V;
      Vec w;
      if(fixedBoundaryNormalModes) {
	eigvec(Ke0r(IN),M0r(IN),V,w);
	vector<int> imod;
	for(int i=0; i<w.size(); i++) {
	  if(w(i)>pow(2*M_PI*0.1,2))
	    imod.push_back(i);
	}
	if(min(nmodes)<1 or max(nmodes)>(int)imod.size())
	  runtime_error(string("(FlexibleBodyTool::init): node numbers do not match, must be within the range [1,") + to_string(imod.size()) + "]");
	for(int i=0; i<nmodes.size(); i++) {
	  Vsd.set(IN,IH.size()+i,V.col(imod[nmodes(i)-1]));
	  Vsd.set(IH,IH.size()+i,Vec(IH.size()));
	}
      }
      else {
	eigvec(Ke0r,M0r,V,w);

	vector<int> imod;
	for(int i=0; i<w.size(); i++) {
	  if(w(i)>pow(2*M_PI*0.1,2))
	    imod.push_back(i);
	}
	if(min(nmodes)<1 or max(nmodes)>(int)imod.size())
	  runtime_error(string("(FlexibleBodyTool::init): node numbers do not match, must be within the range [1,") + to_string(imod.size()) + "]");
	for(int i=0; i<nmodes.size(); i++)
	  Vsd.set(IH.size()+i,V.col(imod[nmodes(i)-1]));
      }
    }

    if(IH.size()) {
      SqrMat V;
      Vec w;
      eigvec(JTMJ(Ke0r,Vsd),JTMJ(M0r,Vsd),V,w);
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
    Phi_.set(IF,IJ,Vsd);
    Phi_.set(IX,IJ,Mat(IX.size(),IJ.size()));
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

    ifstream isMass(jobname+".mas");
    // mass matrix
    M0.resize(dof.size());
    int j;
    while(true) {
      isMass >> i >> j >> d;
      if(isMass.eof()) break;
      M0.e(i-1,j-1) = d;
    }
    isMass.close();

    ifstream isStiff(jobname+".sti");
    // stiffness matrix
    Ke0.resize(dof.size());
    while(true) {
      isStiff >> i >> j >> d;
      if(isStiff.eof()) break;
      Ke0.e(i-1,j-1) = d;
    }
    isStiff.close();

    Phi_.resize(3*nN,nM,NONINIT);
    Sr.resize(6*nN,nM,NONINIT);
    for(int j=0; j<Phi_.cols(); j++) {
      for(int i=0; i<Phi_.rows(); i++)
	Phi_.e(i,j) = disp[j].e(i);
      for(int i=0; i<Sr.rows(); i++)
	Sr.e(i,j) = stress[j].e(i);
    }

    indices.resize(5*6*eles.rows());
    j = 0;
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

  void FlexibleBodyTool::damp() {
    if(static_cast<DampingPage*>(page(PageDamp))->mDamp->isActive()) {
      auto mat = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<DampingPage*>(page(PageDamp))->mDamp->getWidget())->getWidget())->getEvalMat();
      for(size_t i=0; i<mat.size(); i++)
	mDamp(i) = mat[i][0].toDouble();
    }

    if(static_cast<DampingPage*>(page(PageDamp))->pDamp->isActive()) {
      auto mat = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(static_cast<DampingPage*>(page(PageDamp))->pDamp->getWidget())->getWidget())->getEvalMat();
      for(size_t i=0; i<mat.size(); i++)
	pDamp(i) = mat[i][0].toDouble();
    }
  }

  void FlexibleBodyTool::exp() {
    std::vector<Mat3xV> Phi = vector<Mat3xV>(nN,Mat3xV(nM,NONINIT));
    for(size_t i=0; i<nN; i++)
      Phi[i] = Phi_(RangeV(nen*i,nen*i+net-1),RangeV(0,nM-1));

    std::vector<Matrix<General, Fixed<6>, Var, double>> sigmahel;
    if(Sr.rows()) {
      sigmahel.resize(nN,Matrix<General,Fixed<6>,Var,double>(nM));
      for(size_t i=0; i<nN; i++)
	sigmahel[i] = Sr(RangeV(6*i,6*i+6-1),RangeV(0,nM-1));
    }

    //Ke0 <<= JTMJ(Ke0r,Vsd);
    Ke0 <<= JTMJ(Ke0,Phi_);

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

    std::vector<Vec3> KrKP(nN);
    for(const auto & i : nodeMap)
      KrKP[i.second] = nodalPos[i.first];

    bool lumpedMass = true;
    // compute mass and lumped mass matrix
    vector<double> mi(nN);
    m = 0;
    if(M0.size()) {
      double ds = 0;
      for(int i=0; i<nN; i++) {
	ds += M0.e(i*nen,i*nen);
	m += M0.e(i*nen,i*nen);
	for(int j=i+1; j<nN; j++)
	  m += 2*M0.e(i*nen,j*nen);
      }
      for(int i=0; i<nN; i++)
	mi[i] = M0.e(i*nen,i*nen)/ds*m;
    } else if(M.cols()==1) {
      for(int i=0; i<M.rows(); i++) {
	mi[i] = M(i,0);
	m += mi[i];
      }
    }
    else
      runtime_error("lumped mass approach not available");

    Vec3 rdm;
    SymMat3 rrdm;

    Mat3xV Pdm(nM);
    std::vector<Mat3xV> rPdm(3,Mat3xV(nM));
    std::vector<std::vector<SqrMatV>> PPdm(3,vector<SqrMatV>(3,SqrMatV(nM)));
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
	for(int l=0; l<3; l++) {
	  for(int i=0; i<nN; i++)
	    PPdm[k][l] += mi[i]*Phi[i].row(k).T()*Phi[i].row(l);
	}
      }
    }
    else {
      // compute reduced mass matrix
      if(not M0.size())
	runtime_error("full mass approach not available");
      PPdm[0][0] = JTMJ(M0,Phi_);
    }

    if(mDamp.size()) {
      SquareMatrix<Ref,double> V;
      Vector<Ref,double> w;
      eigvec(Ke0,SymMatV(PPdm[0][0]+PPdm[1][1]+PPdm[2][2]),V,w);
      Pdm <<= Pdm*V;
      for(int i=0; i<3; i++) {
	rPdm[i] <<= rPdm[i]*V;
	for(int j=0; j<3; j++)
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
