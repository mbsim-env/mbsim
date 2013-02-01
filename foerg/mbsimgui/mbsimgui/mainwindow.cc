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
#include "mainwindow.h"
#include "element.h"
#include "integrator.h"
#include "objectfactory.h"
#include <QtGui>
#include "frame.h"
#include "rigidbody.h"
#include "parameter.h"
#include "octaveutils.h"
#include <mbxmlutils/utils.h>
#include <openmbv/mainwindow.h>
#include <mbxmlutilstinyxml/getinstallpath.h>
#include <boost/bind.hpp>
#ifdef _WIN32 // Windows
#  include <windows.h>
#  include <process.h>
#else
#  include <spawn.h>
#  include <sys/wait.h>
#endif

using namespace std;
namespace bfs=boost::filesystem;

int runProgramSyncronous(const vector<string> &arg) {
  char **argv=new char*[arg.size()+1];
  for(size_t i=0; i<arg.size(); i++)
    argv[i]=const_cast<char*>(arg[i].c_str());
  argv[arg.size()]=NULL;

#if !defined _WIN32
  pid_t child;
  int ret;
  extern char** environ;
  ret=posix_spawnp(&child, argv[0], NULL, NULL, argv, environ);
  delete[]argv;
  return ret;
//  if(ret!=0)
//    return -1;
//
//  int status;
//  waitpid(child, &status, 0);
//
//  if(WIFEXITED(status))
//    return WEXITSTATUS(status);
//  else
//    return -1;
#else
  int ret;
  ret=_spawnvp(_P_NOWAIT, argv[0], argv);
  delete[]argv;
  return ret;
#endif
}

MBXMLUtils::OctaveEvaluator *MainWindow::octEval=NULL;

MainWindow::MainWindow() : inlineOpenMBVMW(0) {
#ifdef INLINE_OPENMBV
  initInlineOpenMBV();
#endif

  MBSimObjectFactory::initialize();
  octEval=new MBXMLUtils::OctaveEvaluator;

//  QMenu *projectMenu=new QMenu("Project", menuBar());
//  projectMenu->addAction("New", this, SLOT(newProject()));
//  projectMenu->addAction("Load", this, SLOT(loadProject()));
//  projectMenu->addAction("Save as", this, SLOT(saveProjectAs()));
//  menuBar()->addMenu(projectMenu);

  QMenu *MBSMenu=new QMenu("MBS", menuBar());
  MBSMenu->addAction("New", this, SLOT(newMBS()));
  MBSMenu->addAction("Load", this, SLOT(loadMBS()));
  actionSaveMBSAs = MBSMenu->addAction("Save as", this, SLOT(saveMBSAs()));
  actionSaveMBSAs->setDisabled(true);
  actionSaveMBS = MBSMenu->addAction("Save", this, SLOT(saveMBS()));
  actionSaveMBS->setDisabled(true);
  menuBar()->addMenu(MBSMenu);

  QMenu *integratorMenu=new QMenu("Integrator", menuBar());
  QMenu *submenu = integratorMenu->addMenu("New");

  QAction *action=new QAction(Utils::QIconCached("newobject.svg"),"DOPRI5", this);
  connect(action,SIGNAL(triggered()),this,SLOT(newDOPRI5Integrator()));
  submenu->addAction(action);
  
  action=new QAction(Utils::QIconCached("newobject.svg"),"RADAU5", this);
  connect(action,SIGNAL(triggered()),this,SLOT(newRADAU5Integrator()));
  submenu->addAction(action);

  action=new QAction(Utils::QIconCached("newobject.svg"),"LSODE", this);
  connect(action,SIGNAL(triggered()),this,SLOT(newLSODEIntegrator()));
  submenu->addAction(action);

  action=new QAction(Utils::QIconCached("newobject.svg"),"LSODAR", this);
  connect(action,SIGNAL(triggered()),this,SLOT(newLSODARIntegrator()));
  submenu->addAction(action);

  action=new QAction(Utils::QIconCached("newobject.svg"),"Time stepping", this);
  connect(action,SIGNAL(triggered()),this,SLOT(newTimeSteppingIntegrator()));
  submenu->addAction(action);

  action=new QAction(Utils::QIconCached("newobject.svg"),"Euler explicit", this);
  connect(action,SIGNAL(triggered()),this,SLOT(newEulerExplicitIntegrator()));
  submenu->addAction(action);

  action=new QAction(Utils::QIconCached("newobject.svg"),"RKSuite", this);
  connect(action,SIGNAL(triggered()),this,SLOT(newRKSuiteIntegrator()));
  submenu->addAction(action);

  integratorMenu->addAction("Load", this, SLOT(loadIntegrator()));
  integratorMenu->addAction("Save as", this, SLOT(saveIntegratorAs()));
  actionSaveIntegrator = integratorMenu->addAction("Save", this, SLOT(saveIntegrator()));
  actionSaveIntegrator->setDisabled(true);
  menuBar()->addMenu(integratorMenu);

  QMenu *parameterMenu=new QMenu("Parameter", menuBar());
  parameterMenu->addAction("New", this, SLOT(newParameter()));
  parameterMenu->addAction("Load", this, SLOT(loadParameter()));
  parameterMenu->addAction("Save as", this, SLOT(saveParameterAs()));
  actionSaveParameter = parameterMenu->addAction("Save", this, SLOT(saveParameter()));
  actionSaveParameter->setDisabled(true);
  submenu = parameterMenu->addMenu("New parameter");
  action=new QAction(Utils::QIconCached("newobject.svg"),"Scalar", this);
  connect(action,SIGNAL(triggered()),this,SLOT(newDoubleParameter()));
  submenu->addAction(action);
  menuBar()->addMenu(parameterMenu);

  menuBar()->addSeparator();
  QMenu *helpMenu=new QMenu("Help", menuBar());
  helpMenu->addAction("GUI Help...", this, SLOT(help()));
  helpMenu->addAction("About MBSim GUI", this, SLOT(about()));
  menuBar()->addMenu(helpMenu);

  QToolBar *toolBar = addToolBar("Tasks");
  action = toolBar->addAction("Preview");
  connect(action,SIGNAL(triggered()),this,SLOT(preview()));
  toolBar->addAction(action);
  actionSimulate = toolBar->addAction("Simulate");
  connect(actionSimulate,SIGNAL(triggered()),this,SLOT(simulate()));
  toolBar->addAction(actionSimulate);
  actionOpenMBV = toolBar->addAction("OpenMBV");
  actionOpenMBV->setDisabled(true);
  connect(actionOpenMBV,SIGNAL(triggered()),this,SLOT(openmbv()));
  toolBar->addAction(actionOpenMBV);
  actionH5plotserie = toolBar->addAction("H5plotserie");
  actionH5plotserie->setDisabled(true);
  connect(actionH5plotserie,SIGNAL(triggered()),this,SLOT(h5plotserie()));
  toolBar->addAction(actionH5plotserie);
  action= toolBar->addAction("Resize");
  connect(action,SIGNAL(triggered()),this,SLOT(resizeVariables()));
  toolBar->addAction(action);

  setWindowTitle("MBSim GUI");

  elementList = new QTreeWidget;
  elementList->setColumnCount(2);
  QStringList list;
  list << "Name" << "Type";
  elementList->setHeaderLabels(list);
  connect(elementList,SIGNAL(pressed(QModelIndex)), this, SLOT(elementListClicked()));

  integratorList = new QTreeWidget;
  integratorList->setHeaderLabel("Type");
  connect(integratorList,SIGNAL(pressed(QModelIndex)), this, SLOT(integratorListClicked()));

  parameterList = new QTreeWidget;
  parameterList->setColumnCount(2);
  list.clear();
  list << "Name" << "Value";
  parameterList->setHeaderLabels(list);
  connect(parameterList,SIGNAL(pressed(QModelIndex)), this, SLOT(parameterListClicked()));
  //connect(parameterList,SIGNAL(customContextMenuRequested(const QPoint &)),this,SLOT(parameterListClicked(const QPoint &)));
  //parameterList->header()->setContextMenuPolicy (Qt::CustomContextMenu);
  //connect(parameterList->header(),SIGNAL(customContextMenuRequested(const QPoint &)),this,SLOT(parameterListClicked(const QPoint &)));

  QDockWidget *dockWidget = new QDockWidget("MBS");
  addDockWidget(Qt::RightDockWidgetArea,dockWidget);
  QWidget *box = new QWidget;
  dockWidget->setWidget(box);
  QGridLayout* gl = new QGridLayout;
  box->setLayout(gl);
  fileMBS = new QLineEdit("");
  fileMBS->setReadOnly(true);
  gl->addWidget(new QLabel("File:"),0,0);
  gl->addWidget(fileMBS,0,1);
  gl->addWidget(elementList,1,0,1,2);

  QDockWidget *dockWidget2 = new QDockWidget("Integrator");
  addDockWidget(Qt::RightDockWidgetArea,dockWidget2);
  box = new QWidget;
  dockWidget2->setWidget(box);
  gl = new QGridLayout;
  box->setLayout(gl);
  fileIntegrator = new QLineEdit("");
  fileIntegrator->setReadOnly(true);
  gl->addWidget(new QLabel("File:"),0,0);
  gl->addWidget(fileIntegrator,0,1);
  gl->addWidget(integratorList,1,0,1,2);

  QDockWidget *dockWidget3 = new QDockWidget("Parameter");
  addDockWidget(Qt::RightDockWidgetArea,dockWidget3);
  box = new QWidget;
  dockWidget3->setWidget(box);
  gl = new QGridLayout;
  box->setLayout(gl);
  fileParameter = new QLineEdit("");
  fileParameter->setReadOnly(true);
  gl->addWidget(new QLabel("File:"),0,0);
  gl->addWidget(fileParameter,0,1);
  gl->addWidget(parameterList,1,0,1,2);


  tabifyDockWidget(dockWidget,dockWidget2);
  tabifyDockWidget(dockWidget2,dockWidget3);

  QWidget *centralWidget = new QWidget;  
  setCentralWidget(centralWidget);
  QHBoxLayout *mainlayout = new QHBoxLayout;
  centralWidget->setLayout(mainlayout);
  pagesWidget = new QStackedWidget;
  mainlayout->addWidget(pagesWidget);
#ifdef INLINE_OPENMBV
  QDockWidget *dockWidget4 = new QDockWidget("OpenMBV");
  addDockWidget(Qt::RightDockWidgetArea,dockWidget4);
  dockWidget4->setWidget(inlineOpenMBVMW);
#endif
  
  newDOPRI5Integrator();
  newMBS();
  QTreeWidgetItem* parentItem = new QTreeWidgetItem;
}

void MainWindow::initInlineOpenMBV() {
  if(bfs::is_directory("/dev/shm"))
    uniqueTempDir=bfs::unique_path("/dev/shm/mbsimgui_tmp_%%%%-%%%%-%%%%-%%%%");
  else
    uniqueTempDir=bfs::unique_path(bfs::temp_directory_path()/"mbsimgui_tmp_%%%%-%%%%-%%%%-%%%%");
  bfs::create_directories(uniqueTempDir);
  bfs::copy_file(MBXMLUtils::getInstallPath()+"/share/mbsimgui/empty.ombv.xml",
                 uniqueTempDir/"openmbv1.ombv.xml");
  bfs::copy_file(MBXMLUtils::getInstallPath()+"/share/mbsimgui/empty.ombv.h5",
                 uniqueTempDir/"openmbv1.ombv.h5");
  std::list<string> arg;
  arg.push_back("--wst");
  arg.push_back(MBXMLUtils::getInstallPath()+"/share/mbsimgui/inlineopenmbv.ombv.wst");
  arg.push_back("--autoreload");
  arg.push_back((uniqueTempDir/"openmbv1.ombv.xml").generic_string());
  inlineOpenMBVMW=new OpenMBVGUI::MainWindow(arg);

  connect(inlineOpenMBVMW, SIGNAL(objectSelected(std::string, Object*)), this, SLOT(selectElement(std::string)));
  connect(inlineOpenMBVMW, SIGNAL(fileReloaded()), this, SLOT(elementListClicked()));
}

MainWindow::~MainWindow() {
  delete inlineOpenMBVMW;

  bfs::remove_all(uniqueTempDir);
}

void MainWindow::closeEvent(QCloseEvent *event) {
  //if(actionSaveMBS->isEnabled() || actionSaveMBS->isEnabled() || actionSaveParameter->isEnabled()) {
  QMessageBox::StandardButton ret;
  ret = QMessageBox::warning(this, tr("Application"),
      tr("A document may has been modified.\n"
        "Do you want to save your changes?"),
      QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
  if (ret == QMessageBox::Save) {
    if(actionSaveMBS->isEnabled())
      saveMBS();
    else
      saveMBSAs();
    if(actionSaveIntegrator->isEnabled())
      saveIntegrator();
    else
      saveIntegratorAs();
    if(actionSaveParameter->isEnabled())
      saveParameter();
    else
      saveParameterAs();
    event->accept();
  } 
  else if (ret == QMessageBox::Discard) 
    event->accept();
  else if(ret == QMessageBox::Cancel) 
    event->ignore();
  //  if (maybeSave()) {
  ////    writeSettings();
  //    event->accept();
  //  } else {
  //    event->ignore();
  //  }
}

void MainWindow::elementListClicked() {
  if(QApplication::mouseButtons()==Qt::RightButton) {
    Element *element=dynamic_cast<Element*>(elementList->currentItem());
    if(element) {
      QMenu* menu=element->getContextMenu();
      menu->exec(QCursor::pos());
    }
  } 
  else if(QApplication::mouseButtons()==Qt::LeftButton) {
    Element *element=dynamic_cast<Element*>(elementList->currentItem());
    if(element) {
      pagesWidget->insertWidget(0,element->getPropertyWidget());
      pagesWidget->setCurrentWidget(element->getPropertyWidget());
    }
  }

  Element *element=dynamic_cast<Element*>(elementList->currentItem());
#ifdef INLINE_OPENMBV
  if(element)
    emit inlineOpenMBVMW->highlightObject(element->getID());
  else
    emit inlineOpenMBVMW->highlightObject("");
#endif
}

void MainWindow::integratorListClicked() {
  if(QApplication::mouseButtons()==Qt::RightButton) {
//    Element *element=(Element*)elementList->currentItem();
//    QMenu* menu=element->getContextMenu();
//    menu->exec(QCursor::pos());
  } 
  else if(QApplication::mouseButtons()==Qt::LeftButton) {
    Integrator *integrator=(Integrator*)integratorList->currentItem();
    pagesWidget->insertWidget(0,integrator->getPropertyWidget());
    pagesWidget->setCurrentWidget(integrator->getPropertyWidget());
  }
}

void MainWindow::parameterListClicked() {
  if(QApplication::mouseButtons()==Qt::RightButton) {
    Parameter *parameter=dynamic_cast<Parameter*>(parameterList->currentItem());
    if(parameter) {
      QMenu* menu=parameter->getContextMenu();
      menu->exec(QCursor::pos());
    }
  } 
  else if(QApplication::mouseButtons()==Qt::LeftButton) {
    Parameter *parameter=(Parameter*)parameterList->currentItem();
    pagesWidget->insertWidget(0,parameter->getPropertyWidget());
    pagesWidget->setCurrentWidget(parameter->getPropertyWidget());
  }
}

//void MainWindow::parameterListClicked(const QPoint &pos) {
//  QMenu menu(this);
//  menu.addMenu(newParameterMenu);
//  menu.exec(QCursor::pos());
//}

void MainWindow::resizeVariables() {
//  Solver *solver = (Solver*)elementList->topLevelItem(0);
  Element *element=dynamic_cast<Element*>(elementList->currentItem());
  if(element)
    element->resizeVariables();
//  Integrator *integrator = (Integrator*)integratorList->topLevelItem(0);
  Integrator *integrator=(Integrator*)integratorList->currentItem();
  if(integrator)
    integrator->resizeVariables();
}

void MainWindow::newMBS() {
  actionOpenMBV->setDisabled(true);
  actionH5plotserie->setDisabled(true);
  elementList->clear();
  QTreeWidgetItem* parentItem = elementList->invisibleRootItem();
  Solver *sys=new Solver("MBS", parentItem,1);
  parentItem->addChild(sys);
  ((Integrator*)integratorList->topLevelItem(0))->setSolver(sys);

  actionSaveMBS->setDisabled(true);
  actionSaveMBSAs->setDisabled(false);
  fileMBS->setText("");

#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
}

void MainWindow::loadMBS(const QString &file) {
  QDir dir;
  cout << dir.relativeFilePath(file).toStdString() << endl;
  fileMBS->setText(file);
  actionOpenMBV->setDisabled(true);
  actionH5plotserie->setDisabled(true);
  actionSaveMBS->setDisabled(true);
  if(file!="") {
    elementList->clear();
    QTreeWidgetItem* parentItem = elementList->invisibleRootItem();
    Solver *sys = Solver::readXMLFile(file,parentItem);
    sys->update();
    ((Integrator*)integratorList->topLevelItem(0))->setSolver(sys);
    actionSaveMBS->setDisabled(false);
    actionSaveMBSAs->setDisabled(false);
    //if(QFile::exists("Parameter.mbsimparam.xml")) {
      //loadParameter("Parameter.mbsimparam.xml");
    //}
  }

#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
}

void MainWindow::loadMBS() {
  QString file=QFileDialog::getOpenFileName(0, "XML model files", ".", "XML files (*.mbsim.xml)");
  if(file!="") 
    loadMBS(file);
}

void MainWindow::saveMBSAs() {
  if(elementList->topLevelItemCount()) {
    QString file=QFileDialog::getSaveFileName(0, "XML model files", QString("./")+((Solver*)elementList->topLevelItem(0))->getName()+".mbsim.xml", "XML files (*.mbsim.xml)");
    if(file!="") {
      fileMBS->setText(file.right(10)==".mbsim.xml"?file:file+".mbsim.xml");
      actionSaveMBS->setDisabled(false);
      saveMBS();
    }
  }
}

void MainWindow::saveMBS() {
  QString file = fileMBS->text();
  ((Solver*)elementList->topLevelItem(0))->writeXMLFile(file);
}

//void MainWindow::newIntegrator(const QString &str) {
//  bool ok;
//  integratorList->clear();
//  QString text;
//  if(str=="") {
//    QStringList strList;
//    strList << "DOPRI5Integrator" << "LSODEIntegrator";
//    text = QInputDialog::getItem(0, tr("New integrator"), tr("Integrators:"),strList, 0, false, &ok);
//  } else {
//    text = str;
//    ok = true;
//  }
//  QTreeWidgetItem* parentItem = integratorList->invisibleRootItem();
//  if (ok && !text.isEmpty()) {
//    Integrator *integrator = 0;
//    if(text == "DOPRI5Integrator") {
//      integrator = new DOPRI5Integrator("DOPRI5",parentItem, 1);
//    }
//    else if(text == "LSODEIntegrator") {
//      integrator = new LSODEIntegrator("LSODE",parentItem, 1);
//    }
//    parentItem->addChild(integrator);
//  }
//}

void MainWindow::newDOPRI5Integrator() {
  actionSaveIntegrator->setDisabled(true);
  integratorList->clear();
  QTreeWidgetItem* parentItem = integratorList->invisibleRootItem();
  Integrator *integrator = new DOPRI5Integrator("DOPRI5",parentItem, 1);
  integrator->setSolver(((Solver*)elementList->topLevelItem(0)));
  parentItem->addChild(integrator);
  fileIntegrator->setText("");

#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
}

void MainWindow::newRADAU5Integrator() {
  actionSaveIntegrator->setDisabled(true);
  integratorList->clear();
  QTreeWidgetItem* parentItem = integratorList->invisibleRootItem();
  Integrator *integrator = new RADAU5Integrator("RADAU5",parentItem, 1);
  integrator->setSolver(((Solver*)elementList->topLevelItem(0)));
  parentItem->addChild(integrator);
  fileIntegrator->setText("");

#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
}

void MainWindow::newLSODEIntegrator() {
  integratorList->clear();
  QTreeWidgetItem* parentItem = integratorList->invisibleRootItem();
  Integrator *integrator = new LSODEIntegrator("LSODE",parentItem, 1);
  integrator->setSolver(((Solver*)elementList->topLevelItem(0)));
  parentItem->addChild(integrator);
  fileIntegrator->setText("");

#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
}

void MainWindow::newLSODARIntegrator() {
  integratorList->clear();
  QTreeWidgetItem* parentItem = integratorList->invisibleRootItem();
  Integrator *integrator = new LSODARIntegrator("LSODAR",parentItem, 1);
  integrator->setSolver(((Solver*)elementList->topLevelItem(0)));
  parentItem->addChild(integrator);
  fileIntegrator->setText("");

#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
}

void MainWindow::newTimeSteppingIntegrator() {
  integratorList->clear();
  QTreeWidgetItem* parentItem = integratorList->invisibleRootItem();
  Integrator *integrator = new TimeSteppingIntegrator("Time stepping",parentItem, 1);
  integrator->setSolver(((Solver*)elementList->topLevelItem(0)));
  parentItem->addChild(integrator);
  fileIntegrator->setText("");

#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
}

void MainWindow::newEulerExplicitIntegrator() {
  integratorList->clear();
  QTreeWidgetItem* parentItem = integratorList->invisibleRootItem();
  Integrator *integrator = new EulerExplicitIntegrator("Euler explicit",parentItem, 1);
  integrator->setSolver(((Solver*)elementList->topLevelItem(0)));
  parentItem->addChild(integrator);
  fileIntegrator->setText("");

#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
}

void MainWindow::newRKSuiteIntegrator() {
  integratorList->clear();
  QTreeWidgetItem* parentItem = integratorList->invisibleRootItem();
  Integrator *integrator = new RKSuiteIntegrator("RKSuite",parentItem, 1);
  integrator->setSolver(((Solver*)elementList->topLevelItem(0)));
  parentItem->addChild(integrator);
  fileIntegrator->setText("");

#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
}

void MainWindow::loadIntegrator(const QString &file) {
  fileIntegrator->setText(file);
  actionSaveIntegrator->setDisabled(true);
  if(file!="") {
    integratorList->clear();
    QTreeWidgetItem* parentItem = 0;
    if(parentItem==NULL) parentItem=integratorList->invisibleRootItem();
    Integrator *integrator = Integrator::readXMLFile(file,parentItem);
    integrator->setSolver(((Solver*)elementList->topLevelItem(0)));
    actionSaveIntegrator->setDisabled(false);
  }
}

void MainWindow::loadIntegrator() {
  QString file=QFileDialog::getOpenFileName(0, "MBSim integrator files", ".", "XML files (*.mbsimint.xml)");
  if(file!="")
    loadIntegrator(file);
}

void MainWindow::saveIntegratorAs() {
  if(integratorList->topLevelItemCount()) {
    QString file=QFileDialog::getSaveFileName(0, "MBSim integrator files", QString("./")+((Solver*)elementList->topLevelItem(0))->getName()+".mbsimint.xml", "XML files (*.mbsimint.xml)");
    if(file!="") {
      fileIntegrator->setText(file.right(13)==".mbsimint.xml"?file:file+".mbsimint.xml");
      actionSaveIntegrator->setDisabled(false);
      saveIntegrator();
    }
  }
}

void MainWindow::saveIntegrator() {
  QString file = fileIntegrator->text();
  ((Integrator*)integratorList->topLevelItem(0))->writeXMLFile(file);
}

void MainWindow::newDoubleParameter() {
  QTreeWidgetItem* parentItem = parameterList->invisibleRootItem();
  QString str;
  for(int i=1; i<10000; i++) {
    str = "a" + QString::number(i);
    if(!getChild(parentItem, str))
      break;
  }
  Parameter *parameter = new DoubleParameter(str, parentItem, -1);
  connect(parameter,SIGNAL(parameterChanged(const QString&)),this,SLOT(updateOctaveParameters()));
  updateOctaveParameters();

#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
}

void MainWindow::newParameter() {
  parameterList->clear();
  actionSaveParameter->setDisabled(true);
  //actionSaveParameterAs->setDisabled(false);
  fileParameter->setText("");
}

void MainWindow::loadParameter(const QString &file) {
  fileParameter->setText(file);
  actionSaveParameter->setDisabled(true);
  if(file!="") {
    //((Solver*)elementList->topLevelItem(0))->setParameterFile(file);
    parameterList->clear();
    QTreeWidgetItem* parentItem = 0;
    if(parentItem==NULL) parentItem=parameterList->invisibleRootItem();

    MBSimObjectFactory::initialize();
    TiXmlDocument doc;
    bool ret=doc.LoadFile(file.toAscii().data());
    assert(ret==true);
    TiXml_PostLoadFile(&doc);
    TiXmlElement *e=doc.FirstChildElement();
    TiXml_setLineNrFromProcessingInstruction(e);
    map<string,string> dummy;
    incorporateNamespace(doc.FirstChildElement(), dummy);
    TiXmlElement *E=e->FirstChildElement();
    vector<QString> refFrame;
    //for(int i=0; i<2; i++) {
    while(E) {
      //TiXmlElement *E=e->FirstChildElement();
      Parameter *parameter=ObjectFactory::getInstance()->createParameter(E, parentItem, -1);
      //Parameter *parameter = new DoubleParameter(E->Attribute("name"),parentItem,-1);

//    connect(parameter,SIGNAL(valueChanged()),(Solver*)elementList->topLevelItem(0),SLOT(update()));
      parameter->initializeUsingXML(E);
      connect(parameter,SIGNAL(parameterChanged(const QString&)),this,SLOT(updateOctaveParameters()));
      E=E->NextSiblingElement();
    }
    updateOctaveParameters();
    actionSaveParameter->setDisabled(false);
  }

#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
}

void MainWindow::loadParameter() {
  QString file=QFileDialog::getOpenFileName(0, "MBSim parameter files", ".", "XML files (*.mbsimparam.xml)");
  if(file!="") {
    loadParameter(file);
  }
}

void MainWindow::saveParameterAs() {
  if(parameterList->topLevelItemCount()) {
    QString file=QFileDialog::getSaveFileName(0, "MBSim parameter files", QString("./")+((Solver*)elementList->topLevelItem(0))->getName()+".mbsimparam.xml", "XML files (*.mbsimparam.xml)");
    if(file!="") {
      fileParameter->setText(file.right(15)==".mbsimparam.xml"?file:file+".mbsimparam.xml");
      actionSaveParameter->setDisabled(false);
      saveParameter();
    }
  }
}

void MainWindow::saveParameter(QString fileName) {
  TiXmlDocument doc;
  TiXmlDeclaration *decl = new TiXmlDeclaration("1.0","UTF-8","");
  doc.LinkEndChild( decl );
  TiXmlElement *ele0=new TiXmlElement(PARAMNS+string("parameter"));
  for(int i=0; i<parameterList->topLevelItemCount(); i++) {
    ((Parameter*)parameterList->topLevelItem(i))->writeXMLFile(ele0);
  }
  doc.LinkEndChild(ele0);
  map<string, string> nsprefix;
  unIncorporateNamespace(doc.FirstChildElement(), nsprefix);  
  QString file = fileParameter->text();
  doc.SaveFile(fileName.isEmpty()?file.toAscii().data():fileName.toStdString());
}

void MainWindow::updateOctaveParameters() {
  vector<MBXMLUtils::OctaveEvaluator::Param> param;
  for(int i=0; i<parameterList->topLevelItemCount(); i++) {
    Parameter *p=static_cast<Parameter*>(parameterList->invisibleRootItem()->child(i));
    param.push_back(MBXMLUtils::OctaveEvaluator::Param(p->getName().toStdString(), toStr(p->getValue()), 0));
  }
  try {
    octEval->saveAndClearCurrentParam();
    octEval->fillParam(param, false);
  }
  catch(string e) {
    cout << "An exception occurred in updateOctaveParameters: " << e << endl;
  }
}

void MainWindow::mbsimxml(int task) {
  Solver *slv=(Solver*)elementList->topLevelItem(0);
  Integrator *integ=(Integrator*)integratorList->topLevelItem(0);
  if(!slv || !integ)
    return;

  stringstream mbsimxmlName;
  mbsimxmlName << "mbsim" << task;
  stringstream openmbvName;
  openmbvName << "openmbv" << task;
  stringstream mbsimparamName;
  mbsimparamName << "mbsim" << task << ".mbsimparam.xml";

  QString mbsFile=(uniqueTempDir/mbsimxmlName.str()).c_str();
  QString saveName=slv->getName();
  slv->setName(openmbvName.str().c_str());
  slv->writeXMLFile(mbsFile);
  slv->setName(saveName);

  QString mbsParamFile=(uniqueTempDir/mbsimparamName.str()).c_str();
  saveParameter(mbsParamFile);

  QString intFile=(uniqueTempDir/mbsimxmlName.str()).c_str();
  integ->writeXMLFile(intFile);

//  QString file = fileParameter->text();
//  fileParameter->setText(".sim.mbsimparam.xml");
//  saveParameter();
//  fileParameter->setText(file);

  vector<string> arg;
  arg.push_back(MBXMLUtils::getInstallPath()+"/bin/mbsimxml");
  if(task==1)
    arg.push_back("--stopafterfirststep");
  arg.push_back("--mbsimparam");
  arg.push_back(mbsParamFile.toStdString());
  arg.push_back(mbsFile.toStdString()+".mbsim.xml");
  arg.push_back(intFile.toStdString()+".mbsimint.xml");
  bfs::path CURDIR(bfs::current_path());
  bfs::current_path(uniqueTempDir);
  runProgramSyncronous(arg);
  bfs::current_path(CURDIR);

}

void MainWindow::preview() {
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
}

void MainWindow::simulate() {
  mbsimxml(0);
  actionOpenMBV->setDisabled(false);
}

void MainWindow::openmbv() {
  const char *name = (uniqueTempDir/"openmbv0.ombv.xml").c_str();
  if(QFile::exists(name)) {
    vector<string> command;
    command.push_back(MBXMLUtils::getInstallPath()+"/bin/openmbv");
    command.push_back("--autoreload");
    command.push_back(name);
    runProgramSyncronous(command);
  }
}

void MainWindow::h5plotserie() {
  if(elementList->topLevelItemCount()) {
    QString name = ((Solver*)elementList->topLevelItem(0))->getName() + ".mbsim.h5";
    if(QFile::exists(name)) {
      string prog = "h5plotserie";
      vector<string> command;
      command.push_back(prog);
      command.push_back(name.toStdString());
      runProgramSyncronous(command);
      //int ret = system(prog.toAscii().data());
      //cout << ret << endl;
    }
  }
}

void MainWindow::selectElement(string ID) {
#ifdef INLINE_OPENMBV
  emit inlineOpenMBVMW->highlightObject(ID);
#endif
  map<string, Element*>::iterator it=Element::idEleMap.find(ID);
  if(it!=Element::idEleMap.end()) {
    elementList->setCurrentItem(it->second);
    elementListClicked();
  }
}

void MainWindow::help() {
  QMessageBox::information(this, "MBSim GUI - GUI Help", 
      "<h1>GUI Help</h1>"
      "tbd"
      );
}

void MainWindow::about() {
  QMessageBox::about(this, "About MBSim GUI",
      "<h1>MBSim GUI - A frontend for MBSim</h1>"
      "<p>Copyright &copy; Martin Foerg <tt>&lt;martin.o.foerg@googlemail.com&gt;</tt><p/>"
      "<p>Licensed under the General Public License (see file COPYING).</p>"
      "<p>This is free software; see the source for copying conditions.  There is NO warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.</p>"
      "<h2>Authors:</h2>"
      "<ul>"
      "  <li>Martin Foerg <tt>&lt;martin.o.foerg@googlemail.com&gt;</tt> (Maintainer) </li>"
      "</ul>"
      "<h2>This program uses:</h2>"
      "<ul>"
      "tbd"
      //"  <li>'Qt - A cross-platform application and UI framework' by Nokia from <tt>http://www.qtsoftware.com</tt> (License: GPL/LGPL)</li>"
      //"  <li>'mbsimflatxml - tbd <tt>http://code.google.com/p/mbsim-env</tt> (Licence: Qwt/LGPL)</li>"
      //"  <li>'OpenMBV - tbd <tt>http://code.google.com/p/openmbv</tt> (License: LGPL)</li>"
      //"  <li>'h5plotserie - tbd <tt>http://code.google.com/p/hdf5serie</tt> (License: NCSA-HDF)</li>"
      //"  <li>...</li>"
      "</ul>"
      );
}
