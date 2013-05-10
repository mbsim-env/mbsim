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
#include "frame.h"
#include "contour.h"
#include "rigidbody.h"
#include "integrator.h"
#include "objectfactory.h"
#include "parameter.h"
#include "octaveutils.h"
#include "widget.h"
#include "treemodel.h"
#include "treeitem.h"
#include "element_view.h"
#include "parameter_view.h"
#include "integrator_view.h"
#include <mbxmlutils/utils.h>
#include <mbxmlutilstinyxml/getinstallpath.h>
#include <openmbv/mainwindow.h>
#include <QtGui>

using namespace std;
using namespace MBXMLUtils;

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

  QDir dir = QDir::temp();
  for(int i=1; i<10000; i++) {
    QString name = "mbsim_"+QString::number(i);
    if(!dir.exists(name)) {
      dir.mkdir(name);
      uniqueTempDir = QDir::tempPath() + "/" + name;
      break;
    }
  }

#ifdef INLINE_OPENMBV
  initInlineOpenMBV();
#endif

  MBSimObjectFactory::initialize();
  octEval=new MBXMLUtils::OctaveEvaluator;

  QMenu *GUIMenu=new QMenu("GUI", menuBar());
  QAction *action = GUIMenu->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_DirHomeIcon)),"Workdir", this, SLOT(changeWorkingDir()));
  action->setStatusTip(tr("Change working directory"));
  GUIMenu->addSeparator();
  action = GUIMenu->addAction(Utils::QIconCached(QString::fromStdString(MBXMLUtils::getInstallPath())+"/share/mbsimgui/icons/exit.svg"), "E&xit", this, SLOT(close()));
  action->setShortcuts(QKeySequence::Quit);
  action->setStatusTip(tr("Exit the application"));
  menuBar()->addMenu(GUIMenu);

  QMenu *ProjMenu=new QMenu("Project", menuBar());
  ProjMenu->addAction("New", this, SLOT(saveProjAs()));
  ProjMenu->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_DirOpenIcon)),"Load", this, SLOT(loadProj()));
  //ProjMenu->addAction("Save as", this, SLOT(saveProjAs()));
  actionSaveProj = ProjMenu->addAction("Save all", this, SLOT(saveProj()));
  actionSaveProj->setDisabled(true);
  menuBar()->addMenu(ProjMenu);

  QMenu *menu=new QMenu("MBS", menuBar());
  menu->addAction("New", this, SLOT(newMBS()));
  menu->addAction("Load", this, SLOT(loadMBS()));
  menu->addAction("Save as", this, SLOT(saveMBSAs()));
  actionSaveMBS = menu->addAction("Save", this, SLOT(saveMBS()));
  actionSaveMBS->setDisabled(true);
  menuBar()->addMenu(menu);

  menu=new QMenu("Integrator", menuBar());
  menu->addAction("Select", this, SLOT(selectIntegrator()));

  menu->addAction("Load", this, SLOT(loadIntegrator()));
  menu->addAction("Save as", this, SLOT(saveIntegratorAs()));
  actionSaveIntegrator = menu->addAction("Save", this, SLOT(saveIntegrator()));
  actionSaveIntegrator->setDisabled(true);
  menuBar()->addMenu(menu);

  menu=new QMenu("Parameter list", menuBar());
  menu->addAction("New", this, SLOT(newParameterList()));
  menu->addAction("Load", this, SLOT(loadParameterList()));
  menu->addAction("Save as", this, SLOT(saveParameterListAs()));
  actionSaveParameterList = menu->addAction("Save", this, SLOT(saveParameterList()));
  actionSaveParameterList->setDisabled(true);
  menuBar()->addMenu(menu);

  menu=new QMenu("Export", menuBar());
  actionSaveDataAs = menu->addAction("Export all data", this, SLOT(saveDataAs()));
  actionSaveMBSimH5DataAs = menu->addAction("Export MBSim data file", this, SLOT(saveMBSimH5DataAs()));
  actionSaveOpenMBVDataAs = menu->addAction("Export OpenMBV data", this, SLOT(saveOpenMBVDataAs()));
  actionSaveDataAs->setDisabled(true);
  actionSaveMBSimH5DataAs->setDisabled(true);
  actionSaveOpenMBVDataAs->setDisabled(true);
  menuBar()->addMenu(menu);

  menuBar()->addSeparator();
  QMenu *helpMenu=new QMenu("Help", menuBar());
  helpMenu->addAction("GUI Help...", this, SLOT(help()));
  helpMenu->addAction("About MBSim GUI", this, SLOT(about()));
  menuBar()->addMenu(helpMenu);

  QToolBar *toolBar = addToolBar("Tasks");
  actionSimulate = toolBar->addAction(Utils::QIconCached(QString::fromStdString(MBXMLUtils::getInstallPath())+"/share/mbsimgui/icons/simulate.svg"),"Simulate");
  //actionSimulate->setStatusTip(tr("Simulate the multibody system"));
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

  elementList = new ElementView;
  elementList->setModel(new ElementTreeModel);
  //elementList->setItemDelegate(new ElementDelegate);
  elementList->setColumnWidth(0,250);
  elementList->setColumnWidth(1,200);

  parameterList = new ParameterView;
  parameterList->setModel(new ParameterListModel);
  //parameterList->setItemDelegate(new ParameterDelegate);
  parameterList->setColumnWidth(0,75);
  parameterList->setColumnWidth(1,125);

  integratorView = new IntegratorView;

  action = new QAction("Add scalar parameter", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addScalarParameter()));
  parameterList->insertAction(0,action);
  action = new QAction("Add vector parameter", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addVectorParameter()));
  parameterList->insertAction(0,action);
  action = new QAction("Add matrix parameter", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addMatrixParameter()));
  parameterList->insertAction(0,action);
  parameterList->setContextMenuPolicy(Qt::ActionsContextMenu);

  connect(elementList,SIGNAL(pressed(QModelIndex)), this, SLOT(elementListClicked()));
  connect(parameterList,SIGNAL(pressed(QModelIndex)), this, SLOT(parameterListClicked()));

  QDockWidget *dockWidget1 = new QDockWidget("Multibody system");
  addDockWidget(Qt::LeftDockWidgetArea,dockWidget1);
  fileMBS = new QLineEdit("");
  fileMBS->setReadOnly(true);
  dockWidget1->setWidget(elementList);

  QDockWidget *dockWidget3 = new QDockWidget("Parameter list");
  addDockWidget(Qt::LeftDockWidgetArea,dockWidget3);
  fileParameter = new QLineEdit("");
  fileParameter->setReadOnly(true);
  dockWidget3->setWidget(parameterList);

  QDockWidget *dockWidget2 = new QDockWidget("Integrator");
  addDockWidget(Qt::LeftDockWidgetArea,dockWidget2);
  fileIntegrator = new QLineEdit("");
  fileIntegrator->setReadOnly(true);
  dockWidget2->setWidget(integratorView);
 
  //tabifyDockWidget(dockWidget1,dockWidget2);
  //tabifyDockWidget(dockWidget2,dockWidget3);
  //QList<QTabBar *> tabList = findChildren<QTabBar *>();
  //tabBar = tabList.at(0);

  QWidget *centralWidget = new QWidget;  
  setCentralWidget(centralWidget);
  QHBoxLayout *mainlayout = new QHBoxLayout;
  centralWidget->setLayout(mainlayout);
#ifdef INLINE_OPENMBV
  mainlayout->addWidget(inlineOpenMBVMW);
#endif

  QDockWidget *mbsimDW = new QDockWidget("MBSim Echo Area", this);
  addDockWidget(Qt::BottomDockWidgetArea, mbsimDW);
  mbsim=new Process(this);
  QProcessEnvironment env=QProcessEnvironment::systemEnvironment();
  env.insert("MBXMLUTILS_XMLOUTPUT", "1");
  mbsim->getProcess()->setProcessEnvironment(env);
  mbsimDW->setWidget(mbsim); 
  connect(mbsim->getProcess(),SIGNAL(finished(int,QProcess::ExitStatus)),this,SLOT(simulationFinished(int)));

  setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
  
  selectDOPRI5Integrator();
  newMBS(false);
}

void MainWindow::simulationFinished(int exitCode) {
  actionSimulate->setDisabled(false);
  statusBar()->showMessage(tr("Ready"));
  mbsim->setCurrentIndex(exitCode);
}

void MainWindow::openPropertyDialog() {
  QModelIndex index = elementList->selectionModel()->currentIndex();
  elementList->openEditor();
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

  QFile::copy(QString::fromStdString(MBXMLUtils::getInstallPath()+"/share/mbsimgui/empty.ombv.xml"),uniqueTempDir+"/out1.ombv.xml");
  QFile::copy(QString::fromStdString(MBXMLUtils::getInstallPath()+"/share/mbsimgui/empty.ombv.h5"),uniqueTempDir+"/out1.ombv.h5");
  std::list<string> arg;
  arg.push_back("--wst");
  arg.push_back(MBXMLUtils::getInstallPath()+"/share/mbsimgui/inlineopenmbv.ombv.wst");
  arg.push_back("--autoreload");
  arg.push_back((uniqueTempDir+"/out1.ombv.xml").toStdString());
  inlineOpenMBVMW=new OpenMBVGUI::MainWindow(arg);

  connect(inlineOpenMBVMW, SIGNAL(objectSelected(std::string, Object*)), this, SLOT(selectElement(std::string)));
  connect(inlineOpenMBVMW, SIGNAL(objectDoubleClicked(std::string, Object*)), this, SLOT(openPropertyDialog()));
  connect(inlineOpenMBVMW, SIGNAL(fileReloaded()), this, SLOT(selectionChanged()));
}

MainWindow::~MainWindow() {
  //delete inlineOpenMBVMW;

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
  QMessageBox::StandardButton ret = QMessageBox::warning(this, tr("Application"),
      tr("MBS, parameter list or integrator may have been modified.\n"
        "Do you want to save your changes?"),
      QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
  if(ret == QMessageBox::Save) {
    if(actionSaveMBS->isEnabled())
      saveMBS();
    else
      saveMBSAs();
    if(actionSaveIntegrator->isEnabled())
      saveIntegrator();
    else
      saveIntegratorAs();
    if(actionSaveParameterList->isEnabled())
      saveParameterList();
    else
      saveParameterListAs();
    event->accept();
  } 
  else if(ret == QMessageBox::Discard) 
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

void MainWindow::selectionChanged() {
  QModelIndex index = elementList->selectionModel()->currentIndex();
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  Element *element=dynamic_cast<Element*>(model->getItem(index)->getItemData());
#ifdef INLINE_OPENMBV
  if(element)
    inlineOpenMBVMW->highlightObject(element->getID());
  else
    inlineOpenMBVMW->highlightObject("");
#endif
}

void MainWindow::elementListClicked() {
  selectionChanged();
  if(QApplication::mouseButtons()==Qt::RightButton) {
    QModelIndex index = elementList->selectionModel()->currentIndex();
    Element *element = dynamic_cast<Element*>(static_cast<ElementTreeModel*>(elementList->model())->getItem(index)->getItemData());
    if(element && index.column()==0) {
      QMenu *menu = element->createContextMenu();
      menu->exec(QCursor::pos());
      delete menu;
    } 
  }
}

void MainWindow::parameterListClicked() {
  if(QApplication::mouseButtons()==Qt::RightButton) {
    QModelIndex index = parameterList->selectionModel()->currentIndex();
    if(index.column()==0) {
      Parameter *parameter = static_cast<Parameter*>(static_cast<ParameterListModel*>(parameterList->model())->getItem(index)->getItemData());
      QMenu *menu = parameter->createContextMenu();
      menu->exec(QCursor::pos());
      delete menu;
    } 
  }
}

void MainWindow::loadProj(const QString &file) {
  loadParameterList(file+"/MBS.mbsimparam.xml");
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
//  if(elementList->topLevelItemCount()) {
    QString file=QFileDialog::getExistingDirectory(0, "Project directory");
    if(file!="") {
      fileMBS->setText(file+"/MBS.mbsim.xml");
      actionSaveMBS->setDisabled(false);
  //    saveMBS();
      fileIntegrator->setText(file+"/MBS.mbsimint.xml");
      actionSaveIntegrator->setDisabled(false);
  //    saveIntegrator();
      fileParameter->setText(file+"/MBS.mbsimparam.xml");
      actionSaveParameterList->setDisabled(false);
  //    saveParameter();
      actionSaveProj->setDisabled(false);
    }
//  }
}

void MainWindow::saveProj() {
  saveMBS();
  saveIntegrator();
  saveParameterList();
}

void MainWindow::newMBS(bool ask) {
  //tabBar->setCurrentIndex(0);
  QMessageBox::StandardButton ret = QMessageBox::Ok;
  if(ask) 
    ret = QMessageBox::warning(this, "New MBS", "Current MBS will be deleted", QMessageBox::Ok | QMessageBox::Cancel);
  if(ret == QMessageBox::Ok) {
    mbsDir = QDir::current();
    actionOpenMBV->setDisabled(true);
    actionH5plotserie->setDisabled(true);
    actionSaveDataAs->setDisabled(true);
    actionSaveMBSimH5DataAs->setDisabled(true);
    actionSaveOpenMBVDataAs->setDisabled(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = model->index(0,0);
    model->removeRow(index.row(), index.parent());
    Solver *solver = new Solver("MBS",0);
    model->createGroupItem(solver,QModelIndex());

    //((Integrator*)integratorView->topLevelItem(0))->setSolver(0);

    actionSaveMBS->setDisabled(true);
    fileMBS->setText("");
    absoluteMBSFilePath="";

#ifdef INLINE_OPENMBV
    mbsimxml(1);
#endif
  }
}

void MainWindow::loadMBS(const QString &file) {
  //tabBar->setCurrentIndex(0);
  mbsDir = QFileInfo(file).absolutePath();
  absoluteMBSFilePath=file;
  fileMBS->setText(QDir::current().relativeFilePath(absoluteMBSFilePath));
  actionOpenMBV->setDisabled(true);
  actionH5plotserie->setDisabled(true);
  actionSaveMBS->setDisabled(true);
  actionSaveDataAs->setDisabled(true);
  actionSaveMBSimH5DataAs->setDisabled(true);
  actionSaveOpenMBVDataAs->setDisabled(true);
  if(file!="") {
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = model->index(0,0);
    model->removeRow(index.row(), index.parent());
    Solver *sys = Solver::readXMLFile(file.toStdString());
    model->createGroupItem(sys);
    //((Integrator*)integratorView->topLevelItem(0))->setSolver(sys);
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
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = model->index(0,0);
  QString file=QFileDialog::getSaveFileName(0, "XML model files", QString("./")+QString::fromStdString(model->getItem(index)->getItemData()->getName())+".mbsim.xml", "XML files (*.mbsim.xml)");
  if(file!="") {
    fileMBS->setText(file.right(10)==".mbsim.xml"?file:file+".mbsim.xml");
    actionSaveMBS->setDisabled(false);
    saveMBS();
  }
}

void MainWindow::saveMBS() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = model->index(0,0);
  Solver *solver = static_cast<Solver*>(model->getItem(index)->getItemData());
  QString file = fileMBS->text();
  mbsDir = QFileInfo(file).absolutePath();
  solver->writeXMLFile(file.toStdString());
}

void MainWindow::selectIntegrator() {
  QMenu *menu = integratorView->createContextMenu();
  menu->exec(QCursor::pos());
  delete menu;
}

void MainWindow::selectDOPRI5Integrator() {
  actionSaveIntegrator->setDisabled(true);
  integratorView->setIntegrator(0);
  fileIntegrator->setText("");
}

void MainWindow::selectRADAU5Integrator() {
  actionSaveIntegrator->setDisabled(true);
  integratorView->setIntegrator(1);
  fileIntegrator->setText("");
}

void MainWindow::selectLSODEIntegrator() {
  actionSaveIntegrator->setDisabled(true);
  integratorView->setIntegrator(2);
  fileIntegrator->setText("");
}

void MainWindow::selectLSODARIntegrator() {
  actionSaveIntegrator->setDisabled(true);
  integratorView->setIntegrator(3);
  fileIntegrator->setText("");
}

void MainWindow::selectTimeSteppingIntegrator() {
  actionSaveIntegrator->setDisabled(true);
  integratorView->setIntegrator(4);
  fileIntegrator->setText("");
}

void MainWindow::selectEulerExplicitIntegrator() {
  actionSaveIntegrator->setDisabled(true);
  integratorView->setIntegrator(5);
  fileIntegrator->setText("");
}

void MainWindow::selectRKSuiteIntegrator() {
  actionSaveIntegrator->setDisabled(true);
  integratorView->setIntegrator(6);
  fileIntegrator->setText("");
}

void MainWindow::loadIntegrator(const QString &file) {
  fileIntegrator->setText(file);
  actionSaveIntegrator->setDisabled(true);
  if(file!="") {
    integratorView->setIntegrator(Integrator::readXMLFile(file.toStdString()));
    actionSaveIntegrator->setDisabled(false);
  }
}

void MainWindow::loadIntegrator() {
  QString file=QFileDialog::getOpenFileName(0, "MBSim integrator files", ".", "XML files (*.mbsimint.xml)");
  if(file!="")
    loadIntegrator(file);
}

void MainWindow::saveIntegratorAs() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = model->index(0,0);
  QString file=QFileDialog::getSaveFileName(0, "MBSim integrator files", QString("./")+QString::fromStdString(model->getItem(index)->getItemData()->getName())+".mbsimint.xml", "XML files (*.mbsimint.xml)");
  if(file!="") {
    fileIntegrator->setText(file.right(13)==".mbsimint.xml"?file:file+".mbsimint.xml");
    actionSaveIntegrator->setDisabled(false);
    saveIntegrator();
  }
}

void MainWindow::saveIntegrator() {
  QString file = fileIntegrator->text();
  integratorView->getIntegrator()->writeXMLFile(file.toStdString());
}

void MainWindow::removeParameter() {
  ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
  QModelIndex index = parameterList->selectionModel()->currentIndex();
  model->removeRow(index.row(), index.parent());
  updateOctaveParameters();
}

void MainWindow::addScalarParameter() {
  ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
  QModelIndex index = QModelIndex();
  ScalarParameter *parameter = new ScalarParameter("a"+toStr(model->getItem(index)->getID()));
  model->createParameterItem(parameter,index);
  updateOctaveParameters();
}

void MainWindow::addVectorParameter() {
  ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
  QModelIndex index = QModelIndex();
  VectorParameter *parameter = new VectorParameter("a"+toStr(model->getItem(index)->getID()));
  model->createParameterItem(parameter,index);
  updateOctaveParameters();
}

void MainWindow::addMatrixParameter() {
  ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
  QModelIndex index = QModelIndex();
  MatrixParameter *parameter = new MatrixParameter("a"+toStr(model->getItem(index)->getID()));
  model->createParameterItem(parameter,index);
  updateOctaveParameters();
}

void MainWindow::newParameterList() {
  ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
  QModelIndex index = model->index(0,0);
  if(index.isValid()) {
    QMessageBox::StandardButton ret = QMessageBox::warning(this, "New parameter list", "Current parameters will be deleted", QMessageBox::Ok | QMessageBox::Cancel);
    if(ret == QMessageBox::Ok) {
      ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
      QModelIndex index = model->index(0,0);
      model->removeRows(index.row(), model->rowCount(QModelIndex()), index.parent());
      updateOctaveParameters();
    }
  }
}

void MainWindow::loadParameterList(const QString &file) {
  //tabBar->setCurrentIndex(2);
  ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
  QModelIndex index = model->index(0,0);
  model->removeRows(index.row(), model->rowCount(QModelIndex()), index.parent());
  fileParameter->setText(file);
  actionSaveParameterList->setDisabled(true);
  if(file!="") {
    ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
    QModelIndex index = model->index(0,0);
    model->removeRow(index.row(), index.parent());

    MBSimObjectFactory::initialize();
    TiXmlDocument doc;
    if(doc.LoadFile(file.toAscii().data())) {
      TiXml_PostLoadFile(&doc);
      TiXmlElement *e=doc.FirstChildElement();
      TiXml_setLineNrFromProcessingInstruction(e);
      map<string,string> dummy;
      incorporateNamespace(doc.FirstChildElement(), dummy);
      TiXmlElement *E=e->FirstChildElement();
      vector<QString> refFrame;
      while(E) {
        Parameter *parameter=ObjectFactory::getInstance()->createParameter(E);
        parameter->initializeUsingXML(E);
        model->createParameterItem(parameter);
        E=E->NextSiblingElement();
      }
      updateOctaveParameters();
      actionSaveParameterList->setDisabled(false);
    }
  }

#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
}

void MainWindow::loadParameterList() {
  QString file=QFileDialog::getOpenFileName(0, "MBSim parameter files", ".", "XML files (*.mbsimparam.xml)");
  if(file!="") {
    loadParameterList(file);
  }
}

void MainWindow::saveParameterListAs() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = model->index(0,0);
  QString file=QFileDialog::getSaveFileName(0, "MBSim parameter files", QString("./")+QString::fromStdString(model->getItem(index)->getItemData()->getName())+".mbsimparam.xml", "XML files (*.mbsimparam.xml)");
  if(file!="") {
    fileParameter->setText(file.right(15)==".mbsimparam.xml"?file:file+".mbsimparam.xml");
    actionSaveParameterList->setDisabled(false);
    saveParameterList();
  }
}

void MainWindow::saveParameterList(const QString &fileName) {
  ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
  QModelIndex index = model->index(0,0);

  TiXmlDocument doc;
  TiXmlDeclaration *decl = new TiXmlDeclaration("1.0","UTF-8","");
  doc.LinkEndChild( decl );
  TiXmlElement *ele0=new TiXmlElement(PARAMNS+string("parameter"));
  for(int i=0; i<model->rowCount(QModelIndex()); i++)
    static_cast<Parameter*>(model->getItem(index.sibling(i,0))->getItemData())->writeXMLFile(ele0);
  doc.LinkEndChild(ele0);
  unIncorporateNamespace(doc.FirstChildElement(), Utils::getMBSimNamespacePrefixMapping());  
  QString file = fileParameter->text();
  doc.SaveFile(fileName.isEmpty()?file.toAscii().data():fileName.toStdString());
}

void MainWindow::updateOctaveParameters(const ParameterList &paramList) {
  vector<MBXMLUtils::OctaveEvaluator::Param> param;
  ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
  QModelIndex index = model->index(0,0);
  for(int i=0; i<model->rowCount(QModelIndex()); i++) {
    Parameter *p=static_cast<Parameter*>(model->getItem(index.sibling(i,0))->getItemData());
    param.push_back(MBXMLUtils::OctaveEvaluator::Param(p->getName(), toStr(p->getValue()), 0));
  }
  for(int i=0; i<paramList.getSize(); i++)
    param.push_back(MBXMLUtils::OctaveEvaluator::Param(paramList.getParameterName(i), paramList.getParameterValue(i), 0));
  try {
    octEval->saveAndClearCurrentParam();
    octEval->fillParam(param, false);
  }
  catch(string e) {
    cout << "An exception occurred in updateOctaveParameters: " << e << endl;
  }
}

void MainWindow::saveDataAs() {
  QString dir = QFileDialog::getExistingDirectory (0, "Export simulation data", ".");
  if(dir != "") {
    QDir directory(dir);
    QMessageBox::StandardButton ret = QMessageBox::Ok;
    if(directory.count()>2)
      ret = QMessageBox::warning(this, tr("Application"), tr("Directory not empty. Overwrite existing files?"), QMessageBox::Ok | QMessageBox::Cancel);
    if(ret == QMessageBox::Ok) {
      saveMBSimH5Data(dir+"/MBS.mbsim.h5");
      saveOpenMBVXMLData(dir+"/MBS.ombv.xml");
      saveOpenMBVH5Data(dir+"/MBS.ombv.h5");
    }
  }
}

void MainWindow::saveMBSimH5DataAs() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = model->index(0,0);
  QString file=QFileDialog::getSaveFileName(0, "Export MBSim H5 file", QString("./")+QString::fromStdString(model->getItem(index)->getItemData()->getName())+".mbsim.h5", "H5 files (*.mbsim.h5)");
  if(file!="") {
    saveMBSimH5Data(file);
  }
}

void MainWindow::saveMBSimH5Data(const QString &file) {
  if(QFile::exists(file))
    QFile::remove(file);
  QFile::copy(uniqueTempDir+"/out0.mbsim.h5",file);
}

void MainWindow::saveOpenMBVDataAs() {
  QString dir = QFileDialog::getExistingDirectory (0, "Export OpenMBV data", ".");
  if(dir != "") {
    QDir directory(dir);
    QMessageBox::StandardButton ret = QMessageBox::Ok;
    if(directory.count()>2)
      ret = QMessageBox::warning(this, tr("Application"), tr("Directory not empty. Overwrite existing files?"), QMessageBox::Ok | QMessageBox::Cancel);
    if(ret == QMessageBox::Ok) {
      saveOpenMBVXMLData(dir+"/MBS.ombv.xml");
      saveOpenMBVH5Data(dir+"/MBS.ombv.h5");
    }
  }
}

void MainWindow::saveOpenMBVXMLData(const QString &file) {
  if(QFile::exists(file))
    QFile::remove(file);
  QFile::copy(uniqueTempDir+"/out0.ombv.xml",file);
}

void MainWindow::saveOpenMBVH5Data(const QString &file) {
  if(QFile::exists(file))
    QFile::remove(file);
  QFile::copy(uniqueTempDir+"/out0.ombv.h5",file);
}

void MainWindow::mbsimxml(int task) {
  absolutePath = true;
  QModelIndex index = elementList->model()->index(0,0);
  Solver *slv=dynamic_cast<Solver*>(static_cast<ElementTreeModel*>(elementList->model())->getItem(index)->getItemData());
  //Integrator *integ=(Integrator*)integratorView->topLevelItem(0);
  Integrator *integ=integratorView->getIntegrator();
  if(!slv || !integ)
    return;

  QString sTask = QString::number(task); 
  QString mbsFile=uniqueTempDir+"/in"+sTask+".mbsim.xml";
  string saveName=slv->getName();
  slv->setName("out"+sTask.toStdString());
  slv->writeXMLFile(mbsFile.toStdString());
  slv->setName(saveName);

  QString mbsParamFile=uniqueTempDir+"/in"+sTask+".mbsimparam.xml";
  saveParameterList(mbsParamFile);

  QString intFile=uniqueTempDir+"/in"+sTask+".mbsimint.xml";
  integ->writeXMLFile(intFile.toStdString());

  QStringList arg;
  if(task==1)
    arg.append("--stopafterfirststep");
  arg.append("--mbsimparam");
  arg.append(mbsParamFile);
  arg.append(mbsFile);
  arg.append(intFile);
  mbsim->getProcess()->setWorkingDirectory(uniqueTempDir);
  mbsim->clearOutputAndStart((MBXMLUtils::getInstallPath()+"/bin/mbsimxml").c_str(), arg);
  absolutePath = false;
}

void MainWindow::simulate() {
  mbsimxml(0);
  actionOpenMBV->setDisabled(false);
  actionH5plotserie->setDisabled(false);
  actionSaveDataAs->setDisabled(false);
  actionSaveMBSimH5DataAs->setDisabled(false);
  actionSaveOpenMBVDataAs->setDisabled(false);
  actionSimulate->setDisabled(true);
  statusBar()->showMessage(tr("Simulating"));
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
  //map<string, Element*>::iterator it=Element::idEleMap.find(ID);
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  map<string, QModelIndex>::iterator it=model->idEleMap.find(ID);
  if(it!=model->idEleMap.end()) {
   //QModelIndex index = elementList->selectionModel()->currentIndex();
   elementList->selectionModel()->setCurrentIndex(it->second,QItemSelectionModel::ClearAndSelect);
   //elementList->selectionModel()->setCurrentIndex(it->second.sibling(it->second.row(),1),QItemSelectionModel::Select);
  }
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

Process::Process(QWidget *parent) : QTabWidget(parent) {
  process=new QProcess(this);
  out=new QTextBrowser(this);
  err=new QTextBrowser(this);
  out->setOpenLinks(false);
  err->setOpenLinks(false);
  connect(out, SIGNAL(anchorClicked(const QUrl &)), this, SLOT(outLinkClicked(const QUrl &)));
  connect(err, SIGNAL(anchorClicked(const QUrl &)), this, SLOT(errLinkClicked(const QUrl &)));
  addTab(out, "Out");
  addTab(err, "Err");
  setCurrentIndex(1);
  setMinimumHeight(80);
  setTabPosition(QTabWidget::West);
  connect(process, SIGNAL(readyReadStandardOutput()), this, SLOT(output()));
  connect(process, SIGNAL(readyReadStandardError()), this, SLOT(error()));
}

void Process::clearOutputAndStart(const QString &program, const QStringList &arguments) {
  outText="";
  errText="";
  out->clear();
  err->clear();
  process->start(program, arguments);
}

void Process::output() {
  QString newText=process->readAllStandardOutput().data();
  convertToHtml(newText);
  outText+=newText;
  out->setHtml(outText);
  out->moveCursor(QTextCursor::End);
}

void Process::error() {
  QString newText=process->readAllStandardError().data();
  convertToHtml(newText);
  errText+=newText;
  err->setHtml(errText);
  err->moveCursor(QTextCursor::Start);
}

void Process::convertToHtml(QString &text) {
  // newlines to html
  text.replace("\n", "<br/>");
  // replace <FILE ...>
  static QRegExp fileRE("<FILE path=\"(.+)\" line=\"([0-9]+)\">(.+)</FILE>");
  text.replace(fileRE, "<a href=\"\\1?line=\\2\">\\3</a>");
}

QSize Process::sizeHint() const {
  QSize size=QTabWidget::sizeHint();
  size.setHeight(80);
  return size;
}

QSize Process::minimumSizeHint() const {
  QSize size=QTabWidget::minimumSizeHint();
  size.setHeight(80);
  return size;
}

void Process::linkClicked(const QUrl &link, QTextBrowser *std) {
  static QString editorCommand=QProcessEnvironment::systemEnvironment().value("MBSIMGUI_EDITOR", "gvim -R %1 +%2");
  cout<<"Opening file using command '"<<editorCommand.toStdString()<<"'. "<<
        "Where %1 is replaced by the filename and %2 by the line number. "<<
        "Use the environment variable MBSIMGUI_EDITOR to overwrite this command."<<endl;
  QString comm=editorCommand.arg(link.path()).arg(link.queryItemValue("line").toInt());
  QProcess::startDetached(comm);
}

void Process::outLinkClicked(const QUrl &link) {
  linkClicked(link, out);
}

void Process::errLinkClicked(const QUrl &link) {
  linkClicked(link, err);
}

void MainWindow::removeElement() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  Element *element = static_cast<Element*>(model->getItem(index)->getItemData());
  element->getParent()->removeElement(element);
  model->removeRow(index.row(), index.parent());
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
}

void MainWindow::saveElementAs() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  QString file=QFileDialog::getSaveFileName(0, "XML model files", QString("./")+QString::fromStdString(model->getItem(index)->getItemData()->getName())+".xml", "XML files (*.xml)");
  if(file!="")
    static_cast<Element*>(model->getItem(index)->getItemData())->writeXMLFile(file.toStdString());
}
