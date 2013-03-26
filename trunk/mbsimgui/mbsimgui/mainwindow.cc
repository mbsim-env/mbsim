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
#include "solver.h"
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
#include "widget.h"

using namespace std;

bool absolutePath = false;
QDir mbsDir;

MainWindow *mw;

bool removeDir(const QString &dirName) {
  bool result = true;
  QDir dir(dirName);

  if (dir.exists(dirName)) {
    Q_FOREACH(QFileInfo info, dir.entryInfoList(QDir::NoDotAndDotDot | QDir::System | QDir::Hidden  | QDir::AllDirs | QDir::Files, QDir::DirsFirst)) {
      if (info.isDir()) {
        result = removeDir(info.absoluteFilePath());
      }
      else {
        result = QFile::remove(info.absoluteFilePath());
      }

      if (!result) {
        return result;
      }
    }
    result = dir.rmdir(dirName);
  }

  return result;
}

MBXMLUtils::OctaveEvaluator *MainWindow::octEval=NULL;

MainWindow::MainWindow() : inlineOpenMBVMW(0) {
  mw = this;

#ifdef INLINE_OPENMBV
  initInlineOpenMBV();
#endif

  MBSimObjectFactory::initialize();
  octEval=new MBXMLUtils::OctaveEvaluator;

  statusBar()->showMessage(tr("Ready"));

  QMenu *GUIMenu=new QMenu("GUI", menuBar());
  QAction *action = GUIMenu->addAction(Utils::QIconCached(QString::fromStdString(MBXMLUtils::getInstallPath())+"/share/mbsimgui/icons/workdir.svg"),"Workdir", this, SLOT(changeWorkingDir()));
  action->setStatusTip(tr("Change working directory"));
  GUIMenu->addSeparator();
  action = GUIMenu->addAction(Utils::QIconCached(QString::fromStdString(MBXMLUtils::getInstallPath())+"/share/mbsimgui/icons/exit.svg"), "E&xit", this, SLOT(close()));
  action->setShortcuts(QKeySequence::Quit);
  action->setStatusTip(tr("Exit the application"));
  menuBar()->addMenu(GUIMenu);

  QMenu *ProjMenu=new QMenu("Project", menuBar());
  ProjMenu->addAction("New", this, SLOT(saveProjAs()));
  ProjMenu->addAction("Load", this, SLOT(loadProj()));
  //ProjMenu->addAction("Save as", this, SLOT(saveProjAs()));
  actionSaveProj = ProjMenu->addAction("Save all", this, SLOT(saveProj()));
  actionSaveProj->setDisabled(true);
  menuBar()->addMenu(ProjMenu);

  QMenu *MBSMenu=new QMenu("MBS", menuBar());
  MBSMenu->addAction("New", this, SLOT(newMBS()));
  MBSMenu->addAction("Load", this, SLOT(loadMBS()));
  MBSMenu->addAction("Save as", this, SLOT(saveMBSAs()));
  actionSaveMBS = MBSMenu->addAction("Save", this, SLOT(saveMBS()));
  actionSaveMBS->setDisabled(true);
  menuBar()->addMenu(MBSMenu);

  QMenu *integratorMenu=new QMenu("Integrator", menuBar());
  QMenu *submenu = integratorMenu->addMenu("New");

  action=new QAction(Utils::QIconCached("newobject.svg"),"DOPRI5", this);
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
  actionSimulate = toolBar->addAction(Utils::QIconCached(QString::fromStdString(MBXMLUtils::getInstallPath())+"/share/mbsimgui/icons/simulate.svg"),"Simulate");
  actionSimulate->setStatusTip(tr("Simulate the multibody system"));
  connect(actionSimulate,SIGNAL(triggered()),this,SLOT(simulate()));
  toolBar->addAction(actionSimulate);
  actionOpenMBV = toolBar->addAction(Utils::QIconCached(QString::fromStdString(MBXMLUtils::getInstallPath())+"/share/mbsimgui/icons/openmbv.svg"),"OpenMBV");
  actionOpenMBV->setDisabled(true);
  connect(actionOpenMBV,SIGNAL(triggered()),this,SLOT(openmbv()));
  toolBar->addAction(actionOpenMBV);
  actionH5plotserie = toolBar->addAction(Utils::QIconCached(QString::fromStdString(MBXMLUtils::getInstallPath())+"/share/mbsimgui/icons/h5plotserie.svg"),"H5plotserie");
  actionH5plotserie->setDisabled(true);
  connect(actionH5plotserie,SIGNAL(triggered()),this,SLOT(h5plotserie()));
  toolBar->addAction(actionH5plotserie);

  setWindowTitle("MBSim GUI");

  elementList = new QTreeWidget;
  elementList->setColumnCount(2);
  elementList->setColumnWidth(0,200);
  QStringList list;
  list << "Name" << "Type";
  elementList->setHeaderLabels(list);
  connect(elementList,SIGNAL(pressed(QModelIndex)), this, SLOT(elementListClicked()));
  connect(elementList,SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)), this, SLOT(elementListDoubleClicked()));

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
  addDockWidget(Qt::LeftDockWidgetArea,dockWidget);
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
  addDockWidget(Qt::LeftDockWidgetArea,dockWidget2);
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
  addDockWidget(Qt::LeftDockWidgetArea,dockWidget3);
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
  QList<QTabBar *> tabList = findChildren<QTabBar *>();
  QTabBar *tabBar = tabList.at(0);
  tabBar->setCurrentIndex(0);

  QWidget *centralWidget = new QWidget;  
  setCentralWidget(centralWidget);
  QHBoxLayout *mainlayout = new QHBoxLayout;
  centralWidget->setLayout(mainlayout);
  pagesWidget = new QStackedWidget;
#ifdef INLINE_OPENMBV
  //QDockWidget *dockWidget4 = new QDockWidget("OpenMBV");
  //addDockWidget(Qt::RightDockWidgetArea,dockWidget4);
  //dockWidget4->setWidget(inlineOpenMBVMW);
  //inlineOpenMBVMW->setMinimumWidth(300);
  //inlineOpenMBVMW->setMinimumHeight(300);
  mainlayout->addWidget(inlineOpenMBVMW);
#endif
  //mainlayout->addWidget(pagesWidget);
  
  newDOPRI5Integrator();
  newMBS();
  QTreeWidgetItem* parentItem = new QTreeWidgetItem;

}

void MainWindow::initInlineOpenMBV() {
//  uniqueTempDir = QDir::tempPath() + "/mbsim_XXXXXX"; 
//  char *temp = new char[uniqueTempDir.size()];
//  cout << uniqueTempDir.toStdString() << endl;
//  for(unsigned int i=0; i<uniqueTempDir.size(); i++)
//    temp[i] = uniqueTempDir[i].toAscii();
//  cout << temp << endl;
//  mkdtemp(temp);
//  uniqueTempDir = temp;

  QDir dir = QDir::temp();
  for(int i=1; i<10000; i++) {
    QString name = "mbsim_"+QString::number(i);
    if(!dir.exists(name)) {
      dir.mkdir(name);
      uniqueTempDir = QDir::tempPath() + "/" + name;
      break;
    }
  }

  QFile::copy(QString::fromStdString(MBXMLUtils::getInstallPath()+"/share/mbsimgui/empty.ombv.xml"),uniqueTempDir+"/out1.ombv.xml");
  QFile::copy(QString::fromStdString(MBXMLUtils::getInstallPath()+"/share/mbsimgui/empty.ombv.h5"),uniqueTempDir+"/out1.ombv.h5");
  std::list<string> arg;
  arg.push_back("--wst");
  arg.push_back(MBXMLUtils::getInstallPath()+"/share/mbsimgui/inlineopenmbv.ombv.wst");
  arg.push_back("--autoreload");
  arg.push_back((uniqueTempDir+"/out1.ombv.xml").toStdString());
  inlineOpenMBVMW=new OpenMBVGUI::MainWindow(arg);

  connect(inlineOpenMBVMW, SIGNAL(objectSelected(std::string, Object*)), this, SLOT(selectElement(std::string)));
  connect(inlineOpenMBVMW, SIGNAL(objectDoubleClicked(std::string, Object*)), this, SLOT(openPropertyDialog(std::string)));
  connect(inlineOpenMBVMW, SIGNAL(fileReloaded()), this, SLOT(elementListClicked()));
}

MainWindow::~MainWindow() {
  delete inlineOpenMBVMW;

  removeDir(uniqueTempDir);
}

void MainWindow::changeWorkingDir() {
  QString dir = QFileDialog::getExistingDirectory (0, "Working directory", ".");
  if(dir != "") {
    QDir::setCurrent(dir);
    fileMBS->setText(QDir::current().relativeFilePath(absoluteMBSFilePath));
  }
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

void MainWindow::openPropertyDialog(string ID) {
  Element *element=dynamic_cast<Element*>(elementList->currentItem());
  if(element)
    element->openPropertyDialog();
}

void MainWindow::elementListClicked() {
  if(QApplication::mouseButtons()==Qt::RightButton) {
    Element *element=dynamic_cast<Element*>(elementList->currentItem());
    if(element) {
      QMenu* menu=element->getContextMenu();
      menu->exec(QCursor::pos());
    }
  } 
//  else if(QApplication::mouseButtons()==Qt::LeftButton) {
//    Element *element=dynamic_cast<Element*>(elementList->currentItem());
//    if(element) {
//      pagesWidget->insertWidget(0,element->getPropertyWidget());
//      pagesWidget->setCurrentWidget(element->getPropertyWidget());
//    }
//  }

  Element *element=dynamic_cast<Element*>(elementList->currentItem());
#ifdef INLINE_OPENMBV
  if(element)
    inlineOpenMBVMW->highlightObject(element->getID());
  else
    inlineOpenMBVMW->highlightObject("");
#endif
}

void MainWindow::elementListDoubleClicked() {
  Element *element=dynamic_cast<Element*>(elementList->currentItem());
  if(element)
    element->openPropertyDialog();
}

void MainWindow::integratorListClicked() {
  if(QApplication::mouseButtons()==Qt::RightButton) {
    Integrator *integrator=(Integrator*)integratorList->currentItem();
    if(integrator) {
      QMenu* menu=integrator->getContextMenu();
      menu->exec(QCursor::pos());
    }
  } 
//  else if(QApplication::mouseButtons()==Qt::LeftButton) {
//    Integrator *integrator=(Integrator*)integratorList->currentItem();
//    pagesWidget->insertWidget(0,integrator->getPropertyWidget());
//    pagesWidget->setCurrentWidget(integrator->getPropertyWidget());
//  }
}

void MainWindow::parameterListClicked() {
  if(QApplication::mouseButtons()==Qt::RightButton) {
    Parameter *parameter=dynamic_cast<Parameter*>(parameterList->currentItem());
    if(parameter) {
      QMenu* menu=parameter->getContextMenu();
      menu->exec(QCursor::pos());
    }
  } 
//  else if(QApplication::mouseButtons()==Qt::LeftButton) {
//    Parameter *parameter=(Parameter*)parameterList->currentItem();
//    pagesWidget->insertWidget(0,parameter->getPropertyWidget());
//    pagesWidget->setCurrentWidget(parameter->getPropertyWidget());
//  }
}

//void MainWindow::parameterListClicked(const QPoint &pos) {
//  QMenu menu(this);
//  menu.addMenu(newParameterMenu);
//  menu.exec(QCursor::pos());
//}

void MainWindow::loadProj(const QString &file) {
  loadParameter(file+"/MBS.mbsimparam.xml");
  loadIntegrator(file+"/MBS.mbsimint.xml");
  loadMBS(file+"/MBS.mbsim.xml");
  actionSaveProj->setDisabled(false);
}

void MainWindow::loadProj() {
  QString file=QFileDialog::getExistingDirectory(0, "Project directory");
  if(file!="") {
    loadProj(file);
  }
}

void MainWindow::saveProjAs() {
  if(elementList->topLevelItemCount()) {
    QString file=QFileDialog::getExistingDirectory(0, "Project directory");
    if(file!="") {
      fileMBS->setText(file+"/MBS.mbsim.xml");
      actionSaveMBS->setDisabled(false);
  //    saveMBS();
      fileIntegrator->setText(file+"/MBS.mbsimint.xml");
      actionSaveIntegrator->setDisabled(false);
  //    saveIntegrator();
      fileParameter->setText(file+"/MBS.mbsimparam.xml");
      actionSaveParameter->setDisabled(false);
  //    saveParameter();
      actionSaveProj->setDisabled(false);
    }
  }
}

void MainWindow::saveProj() {
  saveMBS();
  saveIntegrator();
  saveParameter();
}

void MainWindow::newMBS() {
  mbsDir = QDir::current();
  actionOpenMBV->setDisabled(true);
  actionH5plotserie->setDisabled(true);
  elementList->clear();
  QTreeWidgetItem* parentItem = elementList->invisibleRootItem();
  Solver *sys=new Solver("MBS", parentItem,1);
  parentItem->addChild(sys);
  ((Integrator*)integratorList->topLevelItem(0))->setSolver(sys);

  actionSaveMBS->setDisabled(true);
  fileMBS->setText("");
  absoluteMBSFilePath="";

#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
}

void MainWindow::loadMBS(const QString &file) {
  mbsDir = QFileInfo(file).absolutePath();
  absoluteMBSFilePath=file;
  fileMBS->setText(QDir::current().relativeFilePath(absoluteMBSFilePath));
  actionOpenMBV->setDisabled(true);
  actionH5plotserie->setDisabled(true);
  actionSaveMBS->setDisabled(true);
  if(file!="") {
    elementList->clear();
    QTreeWidgetItem* parentItem = elementList->invisibleRootItem();
    Solver *sys = Solver::readXMLFile(file,parentItem);
    sys->updateWidget();
    ((Integrator*)integratorList->topLevelItem(0))->setSolver(sys);
    actionSaveMBS->setDisabled(false);
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
  mbsDir = QFileInfo(file).absolutePath();
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

//    connect(parameter,SIGNAL(valueChanged()),(Solver*)elementList->topLevelItem(0),SLOT(updateWidget()));
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
  unIncorporateNamespace(doc.FirstChildElement(), Utils::getMBSimNamespacePrefixMapping());  
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
  absolutePath = true;
  Solver *slv=(Solver*)elementList->topLevelItem(0);
  Integrator *integ=(Integrator*)integratorList->topLevelItem(0);
  if(!slv || !integ)
    return;

  QString sTask = QString::number(task); 
  QString mbsFile=uniqueTempDir+"/in"+sTask+".mbsim.xml";
  QString saveName=slv->getName();
  slv->setName(QString("out")+sTask);
  slv->writeXMLFile(mbsFile);
  slv->setName(saveName);

  QString mbsParamFile=uniqueTempDir+"/in"+sTask+".mbsimparam.xml";
  saveParameter(mbsParamFile);

  QString intFile=uniqueTempDir+"/in"+sTask+".mbsimint.xml";
  integ->writeXMLFile(intFile);

  QStringList arg;
  if(task==1)
    arg.append("--stopafterfirststep");
  arg.append("--mbsimparam");
  arg.append(mbsParamFile);
  arg.append(mbsFile);
  arg.append(intFile);
  QProcess::startDetached((MBXMLUtils::getInstallPath()+"/bin/mbsimxml").c_str(), arg, uniqueTempDir);
  absolutePath = false;
}

void MainWindow::simulate() {
  mbsimxml(0);
  actionOpenMBV->setDisabled(false);
  actionH5plotserie->setDisabled(false);
}

void MainWindow::openmbv() {
  QString name = uniqueTempDir+"/out0.ombv.xml";
  if(QFile::exists(name)) {
    QStringList arg;
    arg.append("--autoreload");
    arg.append(name);
    QProcess::startDetached((MBXMLUtils::getInstallPath()+"/bin/openmbv").c_str(), arg);
  }
}

void MainWindow::h5plotserie() {
  QString name = uniqueTempDir+"/out0.mbsim.h5";
  if(QFile::exists(name)) {
    QStringList arg;
    arg.append(name);
    QProcess::startDetached((MBXMLUtils::getInstallPath()+"/bin/h5plotserie").c_str(), arg);
  }
}

void MainWindow::selectElement(string ID) {
  map<string, Element*>::iterator it=Element::idEleMap.find(ID);
  if(it!=Element::idEleMap.end())
    elementList->setCurrentItem(it->second);
#ifdef INLINE_OPENMBV
  inlineOpenMBVMW->highlightObject(ID);
#endif
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
