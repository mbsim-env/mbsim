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
#include "editors.h"
#include "utils.h"
#include "objectfactory.h"
#include <QtGui>
#include "frame.h"
#include "rigidbody.h"
#include "parameter.h"
#include "octaveutils.h"
#ifdef _WIN32 // Windows
#  include <windows.h>
#  include <process.h>
#else
#  include <spawn.h>
#  include <sys/wait.h>
#endif

using namespace std;

int digits;
bool saveNumeric;

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

ElementItem::ElementItem(const QString &str, QTreeWidget* elementList_) : elementList(elementList_) {
  setText(0,str);
  contextMenu=new QMenu("Context Menu");
  if(str=="frames") {
    addChild(new FrameItem("Frame",elementList));
  }
  if(str=="groups") {
    addChild(new GroupItem("Group",elementList));
  }
  if(str=="objects") {
    addChild(new ObjectItem("RigidBody",elementList));
    addChild(new ObjectItem("JointConstraint",elementList));
  }
  if(str=="links") {
    addChild(new LinkItem("KineticExcitation",elementList));
    addChild(new LinkItem("Joint",elementList));
    addChild(new LinkItem("SpringDamper",elementList));
  }
}

void ElementItem::add() {
}

FrameItem::FrameItem(const QString &str, QTreeWidget* elementlist) : ElementItem(str, elementlist) {
  QAction *action=new QAction(Utils::QIconCached("newobject.svg"),"Add", this);
  connect(action,SIGNAL(triggered()),this,SLOT(add()));
  contextMenu->addAction(action);
}

void FrameItem::add() {
  Group *group = dynamic_cast<Group*>(elementList->currentItem());
  RigidBody *body = dynamic_cast<RigidBody*>(elementList->currentItem());
  Container *container = dynamic_cast<Container*>(elementList->currentItem());
  if(group) {
    new Frame(group->newName(group->getContainerFrame(),"P"), group->getContainerFrame(), -1);
    ((Element*)elementList->topLevelItem(0))->update();
  } else if(body) {
    new Frame(body->newName(body->getContainerFrame(),"P"), body->getContainerFrame(), -1);
    ((Element*)elementList->topLevelItem(0))->update();
  } else if(container && container->text(0)=="frames") {
    new Frame(((Element*)container->parent())->newName(container,"P"), container, -1);
    ((Element*)elementList->topLevelItem(0))->update();
  }
}

GroupItem::GroupItem(const QString &str, QTreeWidget* elementlist) : ElementItem(str, elementlist) {
  QAction *action=new QAction(Utils::QIconCached("newobject.svg"),"Add", this);
  connect(action,SIGNAL(triggered()),this,SLOT(add()));
  contextMenu->addAction(action);
}

void GroupItem::add() {
  Group *group = dynamic_cast<Group*>(elementList->currentItem());
  if(group) {
    TiXmlElement* e = new TiXmlElement(MBSIMNS+getName().toStdString());
    e->SetAttribute("name",group->newName(group->getContainerGroup(),getName()).toStdString());
    ObjectFactory::getInstance()->createGroup(e, group->getContainerGroup(), -1);
    ((Element*)elementList->topLevelItem(0))->update();
  }
}

ObjectItem::ObjectItem(const QString &str, QTreeWidget* elementlist) : ElementItem(str, elementlist) {
  QAction *action=new QAction(Utils::QIconCached("newobject.svg"),"Add", this);
  connect(action,SIGNAL(triggered()),this,SLOT(add()));
  contextMenu->addAction(action);
}

void ObjectItem::add() {
  Group *group = dynamic_cast<Group*>(elementList->currentItem());
  if(group) {
    TiXmlElement* e = new TiXmlElement(MBSIMNS+getName().toStdString());
    e->SetAttribute("name",group->newName(group->getContainerObject(),getName()).toStdString());
    ObjectFactory::getInstance()->createObject(e, group->getContainerObject(), -1);
    ((Element*)elementList->topLevelItem(0))->update();
  }
}

LinkItem::LinkItem(const QString &str, QTreeWidget* elementlist) : ElementItem(str, elementlist) {
  QAction *action=new QAction(Utils::QIconCached("newobject.svg"),"Add", this);
  connect(action,SIGNAL(triggered()),this,SLOT(add()));
  contextMenu->addAction(action);
}

void LinkItem::add() {
  Group *group = dynamic_cast<Group*>(elementList->currentItem());
  if(group) {
    TiXmlElement* e = new TiXmlElement(MBSIMNS+getName().toStdString());
    e->SetAttribute("name",group->newName(group->getContainerLink(),getName()).toStdString());
    ObjectFactory::getInstance()->createLink(e, group->getContainerLink(), -1);
    ((Element*)elementList->topLevelItem(0))->update();
  }
}

FileItem::FileItem(const QString &str, QTreeWidget* elementlist) : ElementItem(str, elementlist) {
  QAction *action=new QAction(Utils::QIconCached("newobject.svg"),"Add", this);
  connect(action,SIGNAL(triggered()),this,SLOT(add()));
  contextMenu->addAction(action);
}

void FileItem::add() {
  Group *group = dynamic_cast<Group*>(elementList->currentItem());
  if(group) {
    group->addFromFile();
  }
}

MBXMLUtils::OctaveEvaluator *MainWindow::octEval=NULL;

MainWindow::MainWindow() {
  MBSimObjectFactory::initialize();
  octEval=new MBXMLUtils::OctaveEvaluator;

  digits = 10;
  saveNumeric = false;
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
  QGridLayout* mainlayout = new QGridLayout;

  centralWidget = new QWidget;  
  setCentralWidget(centralWidget);
  centralWidget->setLayout(mainlayout);

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

  pagesWidget = new QStackedWidget;

  sourceList = new QTreeWidget;
  sourceList->setHeaderLabel("Elements");
  sourceList->addTopLevelItem(new ElementItem("frames",elementList));
  sourceList->addTopLevelItem(new ElementItem("contours",elementList));
  sourceList->addTopLevelItem(new ElementItem("groups",elementList));
  sourceList->addTopLevelItem(new ElementItem("objects",elementList));
  sourceList->addTopLevelItem(new ElementItem("links",elementList));
  sourceList->addTopLevelItem(new FileItem("File",elementList));
  connect(sourceList,SIGNAL(pressed(QModelIndex)), this, SLOT(sourceListClicked()));
  connect(sourceList,SIGNAL(doubleClicked(QModelIndex)), this, SLOT(sourceListDoubleClicked()));

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

  dockWidget = new QDockWidget("Integrator");
  addDockWidget(Qt::LeftDockWidgetArea,dockWidget);
  box = new QWidget;
  dockWidget->setWidget(box);
  gl = new QGridLayout;
  box->setLayout(gl);
  fileIntegrator = new QLineEdit("");
  fileIntegrator->setReadOnly(true);
  gl->addWidget(new QLabel("File:"),0,0);
  gl->addWidget(fileIntegrator,0,1);
  gl->addWidget(integratorList,1,0,1,2);

  dockWidget = new QDockWidget("Parameter");
  addDockWidget(Qt::LeftDockWidgetArea,dockWidget);
  box = new QWidget;
  dockWidget->setWidget(box);
  gl = new QGridLayout;
  box->setLayout(gl);
  fileParameter = new QLineEdit("");
  fileParameter->setReadOnly(true);
  gl->addWidget(new QLabel("File:"),0,0);
  gl->addWidget(fileParameter,0,1);
  gl->addWidget(parameterList,1,0,1,2);

  mainlayout->addWidget(pagesWidget,0,0);

  newMBS();
  newDOPRI5Integrator();
  QTreeWidgetItem* parentItem = new QTreeWidgetItem;
  Integrator *previewIntegrator = new DOPRI5Integrator("DOPRI5",parentItem, 1);
  previewIntegrator->setEndTime(1e-10);
  previewIntegrator->writeXMLFile(".preview");
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
      pagesWidget->insertWidget(0,element->getPropertyDialog());
      pagesWidget->setCurrentWidget(element->getPropertyDialog());
    }
  }
}

void MainWindow::sourceListClicked() {
  if(QApplication::mouseButtons()==Qt::RightButton) {
    ElementItem *element=(ElementItem*)sourceList->currentItem();
    QMenu* menu=element->getContextMenu();
    menu->exec(QCursor::pos());
  } 
}

void MainWindow::sourceListDoubleClicked() {
  ((ElementItem*)sourceList->currentItem())->add();
}


void MainWindow::integratorListClicked() {
  if(QApplication::mouseButtons()==Qt::RightButton) {
//    Element *element=(Element*)elementList->currentItem();
//    QMenu* menu=element->getContextMenu();
//    menu->exec(QCursor::pos());
  } 
  else if(QApplication::mouseButtons()==Qt::LeftButton) {
    Integrator *integrator=(Integrator*)integratorList->currentItem();
    pagesWidget->insertWidget(0,integrator->getPropertyDialog());
    pagesWidget->setCurrentWidget(integrator->getPropertyDialog());
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
    pagesWidget->insertWidget(0,parameter->getPropertyDialog());
    pagesWidget->setCurrentWidget(parameter->getPropertyDialog());
  }
}

//void MainWindow::parameterListClicked(const QPoint &pos) {
//  QMenu menu(this);
//  menu.addMenu(newParameterMenu);
//  menu.exec(QCursor::pos());
//}

void MainWindow::resizeVariables() {
  Solver *solver = (Solver*)elementList->topLevelItem(0);
  solver->resizeVariables();
  Integrator *integrator = (Integrator*)integratorList->topLevelItem(0);
  integrator->resizeVariables();
}

void MainWindow::newMBS() {
  actionOpenMBV->setDisabled(true);
  actionH5plotserie->setDisabled(true);
  elementList->clear();
  QTreeWidgetItem* parentItem = elementList->invisibleRootItem();
  Solver *sys=new Solver("MBS", parentItem,1);
  parentItem->addChild(sys);
  //if(QFile::exists("Parameter.mbsimparam.xml")) {
    //loadParameter("Parameter.mbsimparam.xml");
  //}
  
  actionSaveMBS->setDisabled(true);
  actionSaveMBSAs->setDisabled(false);
  fileMBS->setText("");
}

void MainWindow::loadMBS(const QString &file) {
  fileMBS->setText(file);
  actionOpenMBV->setDisabled(true);
  actionH5plotserie->setDisabled(true);
  actionSaveMBS->setDisabled(true);
  if(file!="") {
    elementList->clear();
    QTreeWidgetItem* parentItem = elementList->invisibleRootItem();
    Solver *sys = Solver::readXMLFile(file,parentItem);
    sys->update();
    actionSaveMBS->setDisabled(false);
    actionSaveMBSAs->setDisabled(false);
    //if(QFile::exists("Parameter.mbsimparam.xml")) {
      //loadParameter("Parameter.mbsimparam.xml");
    //}
  }
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
      fileMBS->setText(file);
      actionSaveMBS->setDisabled(false);
      saveMBS();
    }
  }
}

void MainWindow::saveMBS() {
  QString file = fileMBS->text();
  if(file.contains(".mbsim.xml"))
    file.chop(10);
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
  parentItem->addChild(integrator);
  fileIntegrator->setText("");
}

void MainWindow::newRADAU5Integrator() {
  actionSaveIntegrator->setDisabled(true);
  integratorList->clear();
  QTreeWidgetItem* parentItem = integratorList->invisibleRootItem();
  Integrator *integrator = new RADAU5Integrator("RADAU5",parentItem, 1);
  parentItem->addChild(integrator);
  fileIntegrator->setText("");
}

void MainWindow::newLSODEIntegrator() {
  integratorList->clear();
  QTreeWidgetItem* parentItem = integratorList->invisibleRootItem();
  Integrator *integrator = new LSODEIntegrator("LSODE",parentItem, 1);
  parentItem->addChild(integrator);
  fileIntegrator->setText("");
}

void MainWindow::newLSODARIntegrator() {
  integratorList->clear();
  QTreeWidgetItem* parentItem = integratorList->invisibleRootItem();
  Integrator *integrator = new LSODARIntegrator("LSODAR",parentItem, 1);
  parentItem->addChild(integrator);
  fileIntegrator->setText("");
}

void MainWindow::newTimeSteppingIntegrator() {
  integratorList->clear();
  QTreeWidgetItem* parentItem = integratorList->invisibleRootItem();
  Integrator *integrator = new TimeSteppingIntegrator("Time stepping",parentItem, 1);
  parentItem->addChild(integrator);
  fileIntegrator->setText("");
}

void MainWindow::newEulerExplicitIntegrator() {
  integratorList->clear();
  QTreeWidgetItem* parentItem = integratorList->invisibleRootItem();
  Integrator *integrator = new EulerExplicitIntegrator("Euler explicit",parentItem, 1);
  parentItem->addChild(integrator);
  fileIntegrator->setText("");
}

void MainWindow::newRKSuiteIntegrator() {
  integratorList->clear();
  QTreeWidgetItem* parentItem = integratorList->invisibleRootItem();
  Integrator *integrator = new RKSuiteIntegrator("RKSuite",parentItem, 1);
  parentItem->addChild(integrator);
  fileIntegrator->setText("");
}

void MainWindow::loadIntegrator(const QString &file) {
  fileIntegrator->setText(file);
  actionSaveIntegrator->setDisabled(true);
  if(file!="") {
    integratorList->clear();
    QTreeWidgetItem* parentItem = 0;
    if(parentItem==NULL) parentItem=integratorList->invisibleRootItem();
    Integrator::readXMLFile(file,parentItem);
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
    QString file=QFileDialog::getSaveFileName(0, "MBSim integrator files", QString("./")+((Integrator*)integratorList->topLevelItem(0))->getType()+".mbsimint.xml", "XML files (*.mbsimint.xml)");
    if(file!="") {
      fileIntegrator->setText(file);
      actionSaveIntegrator->setDisabled(false);
      saveIntegrator();
    }
  }
}

void MainWindow::saveIntegrator() {
  QString file = fileIntegrator->text();
  if(file.contains(".mbsimint.xml"))
    file.chop(13);
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
}

void MainWindow::loadParameter() {
  QString file=QFileDialog::getOpenFileName(0, "MBSim parameter files", ".", "XML files (*.mbsimparam.xml)");
  if(file!="") {
    loadParameter(file);
  }
}

void MainWindow::saveParameterAs() {
  if(parameterList->topLevelItemCount()) {
    QString file=QFileDialog::getSaveFileName(0, "MBSim parameter files", QString("./")+"Parameter.mbsimparam.xml", "XML files (*.mbsimparam.xml)");
    if(file!="") {
    fileParameter->setText(file);
      actionSaveParameter->setDisabled(false);
      saveParameter();
    }
  }
}

void MainWindow::saveParameter() {
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
  doc.SaveFile(file.toAscii().data());
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

void MainWindow::preview() {
  Solver *solver = ((Solver*)elementList->topLevelItem(0));
  solver->writeXMLFile(".sim");
  ((Integrator*)integratorList->topLevelItem(0))->writeXMLFile(".sim");
  QString file = fileParameter->text();
  fileParameter->setText(".sim.mbsimparam.xml");
  saveParameter();
  fileParameter->setText(file);

  string prog = "mbsimxml";
  vector<string> command;
  command.push_back(prog);
  command.push_back("--mbsimparam");
  command.push_back(".sim.mbsimparam.xml");
  command.push_back(".sim.mbsim.xml");
  command.push_back(".preview.mbsimint.xml");
  string str = command[0];
  for(unsigned int i=1; i<command.size(); i++)
    str += " " + command[i];

  cout << str.c_str() << endl;
  int ret = system(str.c_str());
  cout << ret << endl;
  //runProgramSyncronous(command);
#if !defined MBSIMGUI_MINGW
  {
    QString name1 = ((Solver*)elementList->topLevelItem(0))->getName() + ".ombv.xml";
    QString name2 = ((Solver*)elementList->topLevelItem(0))->getName() + ".ombv.h5";
    //string dir = "/usr/bin/";
    string prog = "touch";
    vector<string> command;
    command.push_back(prog);
    command.push_back(name1.toStdString());
    command.push_back(name2.toStdString());
    cout << (command[0] + " " + command[1] + " " + command[2]).c_str()<< endl;
    int ret = system((command[0] + " " + command[1] + " " + command[2]).c_str());
    cout << ret << endl;
  }
#endif
  actionOpenMBV->setDisabled(false);
}

void MainWindow::simulate() {
  Solver *solver = ((Solver*)elementList->topLevelItem(0));
  solver->writeXMLFile(".sim");
  ((Integrator*)integratorList->topLevelItem(0))->writeXMLFile(".sim");
  QString file = fileParameter->text();
  fileParameter->setText(".sim.mbsimparam.xml");
  saveParameter();
  fileParameter->setText(file);

  string prog = "mbsimxml";
  vector<string> command;
  command.push_back(prog);
  command.push_back("--mbsimparam");
  command.push_back(".sim.mbsimparam.xml");
  command.push_back(".sim.mbsim.xml");
  command.push_back(".sim.mbsimint.xml");
  string str = command[0];
  for(unsigned int i=1; i<command.size(); i++)
    str += " " + command[i];

  cout << str.c_str() << endl;
  int ret = system(str.c_str());
  cout << ret << endl;
  //runProgramSyncronous(command);
#if !defined MBSIMGUI_MINGW
  {
    QString name1 = ((Solver*)elementList->topLevelItem(0))->getName() + ".ombv.xml";
    QString name2 = ((Solver*)elementList->topLevelItem(0))->getName() + ".ombv.h5";
    //string dir = "/usr/bin/";
    string prog = "touch";
    vector<string> command;
    command.push_back(prog);
    command.push_back(name1.toStdString());
    command.push_back(name2.toStdString());
    cout << (command[0] + " " + command[1] + " " + command[2]).c_str()<< endl;
    int ret = system((command[0] + " " + command[1] + " " + command[2]).c_str());
    cout << ret << endl;
  }
#endif
  actionOpenMBV->setDisabled(false);
  actionH5plotserie->setDisabled(false);
}

void MainWindow::openmbv() {
  if(elementList->topLevelItemCount()) {
    QString name = ((Solver*)elementList->topLevelItem(0))->getName() + ".ombv.xml";
    if(QFile::exists(name)) {
      //string dir = "/home/foerg/Downloads/openmbv/bin/";
      string prog = "openmbv";
      vector<string> command;
      //command.push_back(dir+prog);
      command.push_back(prog);
      command.push_back("--autoreload");
      command.push_back(name.toStdString());
      runProgramSyncronous(command);
    }
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
