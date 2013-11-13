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
#include "object.h"
#include "link.h"
#include "observer.h"
#include "integrator.h"
#include "objectfactory.h"
#include "parameter.h"
#include "widget.h"
#include "treemodel.h"
#include "treeitem.h"
#include "element_view.h"
#include "parameter_view.h"
#include "integrator_view.h"
#include "property_view.h"
#include <mbxmlutilstinyxml/getinstallpath.h>
#include <openmbv/mainwindow.h>
#include <utime.h>
#include <QtGui>
#include <mbxmlutils/octeval.h>

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

MBXMLUtils::OctEval *MainWindow::octEval=NULL;
MBXMLUtils::NewParamLevel *MainWindow::octEvalParamLevel=NULL;

MainWindow::MainWindow(QStringList &arg) : inlineOpenMBVMW(0) {
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
  octEval=new MBXMLUtils::OctEval;
  octEvalParamLevel=new NewParamLevel(*octEval);

  QMenu *GUIMenu=new QMenu("GUI", menuBar());
  menuBar()->addMenu(GUIMenu);

  QAction *action = GUIMenu->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_DirHomeIcon)),"Workdir", this, SLOT(changeWorkingDir()));
  action->setStatusTip(tr("Change working directory"));

  action = GUIMenu->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_DirHomeIcon)),"Octave m-files path", this, SLOT(changeOctavePath()));
  action->setStatusTip(tr("Change path to the octave m-files directory"));

  GUIMenu->addSeparator();

  action = GUIMenu->addAction(Utils::QIconCached(QString::fromStdString(MBXMLUtils::getInstallPath())+"/share/mbsimgui/icons/exit.svg"), "E&xit", this, SLOT(close()));
  action->setShortcuts(QKeySequence::Quit);
  action->setStatusTip(tr("Exit the application"));

  for (int i=0; i<maxRecentFiles; i++) {
    recentProjectFileActs[i] = new QAction(this);
    recentProjectFileActs[i]->setVisible(false);
    connect(recentProjectFileActs[i], SIGNAL(triggered()), this, SLOT(openRecentProjectFile()));
  }
  QMenu *menu=new QMenu("Project", menuBar());
  menu->addAction("New", this, SLOT(newProject()));
  menu->addAction("Load", this, SLOT(loadProject()));
  menu->addAction("Save as", this, SLOT(saveProjectAs()));
  actionSaveProject = menu->addAction("Save", this, SLOT(saveProject()));
  //ProjMenu->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_DirOpenIcon)),"Load", this, SLOT(loadProj()));
  actionSaveProject->setDisabled(true);
  menu->addSeparator();
  //separatorAct = menu->addSeparator();
  for (int i = 0; i < maxRecentFiles; ++i)
    menu->addAction(recentProjectFileActs[i]);
  updateRecentProjectFileActions();
  menu->addSeparator();
  menuBar()->addMenu(menu);

  for (int i=0; i<maxRecentFiles; i++) {
    recentMBSFileActs[i] = new QAction(this);
    recentMBSFileActs[i]->setVisible(false);
    connect(recentMBSFileActs[i], SIGNAL(triggered()), this, SLOT(openRecentMBSFile()));
  }
  menu=new QMenu("MBS", menuBar());
  menu->addAction("New", this, SLOT(newMBS()));
  menu->addAction("Load", this, SLOT(loadMBS()));
  menu->addAction("Save as", this, SLOT(saveMBSAs()));
  actionSaveMBS = menu->addAction("Save", this, SLOT(saveMBS()));
  actionSaveMBS->setDisabled(true);
  menu->addSeparator();
  //separatorAct = menu->addSeparator();
  for (int i = 0; i < maxRecentFiles; ++i)
    menu->addAction(recentMBSFileActs[i]);
  updateRecentMBSFileActions();
  menu->addSeparator();
  menuBar()->addMenu(menu);

  for (int i=0; i<maxRecentFiles; i++) {
    recentParameterFileActs[i] = new QAction(this);
    recentParameterFileActs[i]->setVisible(false);
    connect(recentParameterFileActs[i], SIGNAL(triggered()), this, SLOT(openRecentParameterFile()));
  }
  menu=new QMenu("Parameters", menuBar());
  menu->addAction("New", this, SLOT(newParameterList()));
  menu->addAction("Load", this, SLOT(loadParameterList()));
  menu->addAction("Save as", this, SLOT(saveParameterListAs()));
  actionSaveParameterList = menu->addAction("Save", this, SLOT(saveParameterList()));
  actionSaveParameterList->setDisabled(true);
  menu->addSeparator();
  for (int i = 0; i < maxRecentFiles; ++i)
    menu->addAction(recentParameterFileActs[i]);
  updateRecentParameterFileActions();
  menu->addSeparator();
  menuBar()->addMenu(menu);

  for (int i=0; i<maxRecentFiles; i++) {
    recentIntegratorFileActs[i] = new QAction(this);
    recentIntegratorFileActs[i]->setVisible(false);
    connect(recentIntegratorFileActs[i], SIGNAL(triggered()), this, SLOT(openRecentIntegratorFile()));
  }
  menu=new QMenu("Integrator", menuBar());
  menu->addAction("Select", this, SLOT(selectIntegrator()));

  menu->addAction("Load", this, SLOT(loadIntegrator()));
  menu->addAction("Save as", this, SLOT(saveIntegratorAs()));
  actionSaveIntegrator = menu->addAction("Save", this, SLOT(saveIntegrator()));
  actionSaveIntegrator->setDisabled(true);
  menu->addSeparator();
  for (int i = 0; i < maxRecentFiles; ++i)
    menu->addAction(recentIntegratorFileActs[i]);
  updateRecentIntegratorFileActions();
  menu->addSeparator();
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
  //toolBar->addAction(actionSimulate);
  actionOpenMBV = toolBar->addAction(Utils::QIconCached(QString::fromStdString(MBXMLUtils::getInstallPath())+"/share/mbsimgui/icons/openmbv.svg"),"OpenMBV");
  actionOpenMBV->setDisabled(true);
  connect(actionOpenMBV,SIGNAL(triggered()),this,SLOT(openmbv()));
  //toolBar->addAction(actionOpenMBV);
  actionH5plotserie = toolBar->addAction(Utils::QIconCached(QString::fromStdString(MBXMLUtils::getInstallPath())+"/share/mbsimgui/icons/h5plotserie.svg"),"H5plotserie");
  actionH5plotserie->setDisabled(true);
  connect(actionH5plotserie,SIGNAL(triggered()),this,SLOT(h5plotserie()));
 // toolBar->addAction(actionH5plotserie);
  QAction *actionUpdate3DView = toolBar->addAction(Utils::QIconCached(QString::fromStdString(MBXMLUtils::getInstallPath())+"/share/mbsimgui/icons/h5plotserie.svg"),"Update 3D view");
  connect(actionUpdate3DView,SIGNAL(triggered()),this,SLOT(update3DView()));
//  toolBar->addAction(actionUpdate3DView);

  setWindowTitle("MBSim GUI");

  elementList = new PropertyView;
  elementList->setModel(new PropertyTreeModel);
  elementList->setColumnWidth(0,75);
  elementList->setColumnWidth(1,75);
  elementList->setColumnWidth(2,75);
  elementList->setColumnWidth(3,75);
  connect(elementList,SIGNAL(pressed(QModelIndex)), this, SLOT(elementListClicked()));

  parameterList = new ParameterView;
  parameterList->setModel(new ParameterListModel);
  parameterList->setColumnWidth(0,75);
  parameterList->setColumnWidth(1,125);
  connect(parameterList,SIGNAL(pressed(QModelIndex)), this, SLOT(parameterListClicked()));

  integratorView = new IntegratorView;

  propertyList = new PropertyView;
  propertyList->setModel(new PropertyTreeModel);
  propertyList->setColumnWidth(0,125);
  propertyList->setColumnWidth(1,75);
  propertyList->setColumnWidth(2,75);
  propertyList->setColumnWidth(3,75);
  connect(propertyList,SIGNAL(pressed(QModelIndex)), this, SLOT(propertyListClicked()));
  propertyList->setContextMenuPolicy(Qt::ActionsContextMenu);

  action = new QAction("Add string parameter", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addStringParameter()));
  parameterList->insertAction(0,action);
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

  fileProject = new QLineEdit("");
  fileProject->setReadOnly(true);

  QDockWidget *dockWidget1 = new QDockWidget("Multibody system");
  addDockWidget(Qt::LeftDockWidgetArea,dockWidget1);
  fileMBS = new QLineEdit("");
  fileMBS->setReadOnly(true);
  dockWidget1->setWidget(elementList);

  QDockWidget *dockWidget3 = new QDockWidget("Parameters");
  addDockWidget(Qt::LeftDockWidgetArea,dockWidget3);
  fileParameter = new QLineEdit("");
  fileParameter->setReadOnly(true);
  dockWidget3->setWidget(parameterList);

  QDockWidget *dockWidget2 = new QDockWidget("Integrator");
  addDockWidget(Qt::LeftDockWidgetArea,dockWidget2);
  fileIntegrator = new QLineEdit("");
  fileIntegrator->setReadOnly(true);
  dockWidget2->setWidget(integratorView);

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
  connect(mbsim->getProcess(),SIGNAL(finished(int,QProcess::ExitStatus)),this,SLOT(simulationFinished(int,QProcess::ExitStatus)));

//  QDockWidget *dockWidget4 = new QDockWidget("Properties");
//  addDockWidget(Qt::BottomDockWidgetArea,dockWidget4);
//  dockWidget4->setWidget(propertyList);
 
//  tabifyDockWidget(mbsimDW,dockWidget4);
  //QList<QTabBar *> tabList = findChildren<QTabBar *>();
  //tabBar = tabList.at(0);


  setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);

  QStringList::iterator i, i2;
  
  if((i=std::find(arg.begin(), arg.end(), "--maximized"))!=arg.end()) {
    showMaximized();
    arg.erase(i);
  }

  QString fileMBS, fileParam, fileInteg;
  QRegExp filterMBS(".+\\.mbsim\\.xml"), filterParam(".+\\.mbsimparam\\.xml"), filterInteg(".+\\.mbsimint\\.xml");
  dir.setFilter(QDir::Files);
  i=arg.begin();
  while(i!=arg.end() and (*i)[0]!='-') {
    dir.setPath(*i);
    if(dir.exists()) {
      QStringList file=dir.entryList();
      for(int j=0; j<file.size(); j++) {
        if(fileMBS.isEmpty() and filterMBS.exactMatch(file[j]))
          fileMBS = dir.path()+"/"+file[j];
        else if(fileParam.isEmpty() and filterParam.exactMatch(file[j]))
          fileParam = dir.path()+"/"+file[j];
        else if(fileInteg.isEmpty() and filterInteg.exactMatch(file[j]))
          fileInteg = dir.path()+"/"+file[j];
      }
      i2=i; i++; arg.erase(i2);
      continue;
    }
    if(QFile::exists(*i)) {
      if(fileMBS.isEmpty() and filterMBS.exactMatch(*i))
        fileMBS = *i;
      else if(fileParam.isEmpty() and filterParam.exactMatch(*i))
        fileParam = *i;
      else if(fileInteg.isEmpty() and filterInteg.exactMatch(*i))
        fileInteg = *i;
      i2=i; i++; arg.erase(i2);
      continue;
    }
    i++;
  }
  if(fileMBS.size())
    loadMBS(QDir::current().absoluteFilePath(fileMBS));
  else
    newMBS(false);
  if(fileParam.size())
    loadParameterList(QDir::current().absoluteFilePath(fileParam));
  if(fileInteg.size())
    loadIntegrator(QDir::current().absoluteFilePath(fileInteg));
  else
    selectDOPRI5Integrator();

  setAcceptDrops(true);

  QTimer *timer = new QTimer(this);
  timer->setSingleShot(true);
  connect(timer, SIGNAL(timeout()), this, SLOT(timeout()));
  timer->start(1000);

//  timer = new QTimer(this);
//  timer->setSingleShot(true);
//  connect(timer, SIGNAL(timeout()), this, SLOT(timeout2()));
//  timer->start(1300);
}

void MainWindow::timeout() {
  cout << "utime" << endl;
  QString str = uniqueTempDir+"/out1.ombv.xml";
  utime(str.toStdString().c_str(),0);
  str = uniqueTempDir+"/out1.ombv.h5";
  utime(str.toStdString().c_str(),0);
}
void MainWindow::timeout2() {
//  cout << "view top" << endl;
//  inlineOpenMBVMW->viewTopSlot();
}

void MainWindow::simulationFinished(int exitCode, QProcess::ExitStatus exitStatus) {
  actionSimulate->setDisabled(false);
  actionOpenMBV->setDisabled(false);
  actionH5plotserie->setDisabled(false);
  statusBar()->showMessage(tr("Ready"));
}

void MainWindow::openPropertyDialog() {
//  QModelIndex index = elementList->selectionModel()->currentIndex();
  propertyList->openEditor();
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
    updateRecentProjectFileActions();
    updateRecentMBSFileActions();
    updateRecentParameterFileActions();
    updateRecentIntegratorFileActions();
  }
}

void MainWindow::changeOctavePath() {
  QString path = QFileDialog::getExistingDirectory (0, "Path to octave m-files path", ".");
  if(not(path.isEmpty())) mPath << path;
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

void MainWindow::highlightObject(const string &ID) {
  currentID = ID;
  inlineOpenMBVMW->highlightObject(ID);
}

void MainWindow::selectionChanged() {
  cout << "selection Changed" << endl;
  QModelIndex index = elementList->selectionModel()->currentIndex();
  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
  Element *element=dynamic_cast<Element*>(model->getItem(index)->getItemData());
#ifdef INLINE_OPENMBV
  if(element)
    highlightObject(element->getID());
  else
    highlightObject("");
#endif
}

void MainWindow::updatePropertyTree() {
//  cout << "update property tree" << endl;
//  QModelIndex index = elementList->selectionModel()->currentIndex();
//  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
//  Element *element=dynamic_cast<Element*>(model->getItem(index)->getItemData());
//  PropertyTreeModel *propertyModel = static_cast<PropertyTreeModel*>(propertyList->model());
//  QModelIndex propertyIndex = propertyModel->index(0,0);
//  propertyModel->removeRows(propertyIndex.row(), propertyModel->rowCount(QModelIndex()), propertyIndex.parent());
//  if(element) {
//    for(int i=0; i<element->getNumberOfProperties(); i++) {
//      propertyModel->createPropertyItem(element->getProperty(i));
//    }
//  }
}

void MainWindow::elementListClicked() {
  updatePropertyTree();
  selectionChanged();
//  if(QApplication::mouseButtons()==Qt::LeftButton) {
//    updatePropertyTree();
////    for(int i=0; i<propertyModel->rowCount(QModelIndex()); i++)
////      delete propertyModel->getItem(propertyIndex.sibling(i,0))->getItemData();
//
//    return;
//  }
  if(QApplication::mouseButtons()==Qt::RightButton) {
    QModelIndex index = elementList->selectionModel()->currentIndex();
    TreeItemData *itemData = static_cast<PropertyTreeModel*>(elementList->model())->getItem(index)->getItemData();
    if(itemData && index.column()==0) {
      QMenu *menu = itemData->createContextMenu();
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

void MainWindow::propertyListClicked() {
  if(QApplication::mouseButtons()==Qt::RightButton) {
    QModelIndex index = propertyList->selectionModel()->currentIndex();
    Property *property = static_cast<Property*>(static_cast<PropertyTreeModel*>(propertyList->model())->getItem(index)->getItemData());
    QMenu *menu = property->createContextMenu();
    menu->exec(QCursor::pos());
    delete menu;
  }
}

void MainWindow::newProject(bool ask) {
  QMessageBox::StandardButton ret = QMessageBox::Ok;
  if(ask) 
    ret = QMessageBox::warning(this, "New Project", "Current project will be deleted", QMessageBox::Ok | QMessageBox::Cancel);
  if(ret == QMessageBox::Ok) {
    newMBS(false);
    newParameterList(false);
    selectDOPRI5Integrator();
    actionSaveProject->setDisabled(true);
    fileProject->setText("");
    mPath.clear();
  }
}

void MainWindow::loadProject(const QString &file) {
  if(file!="") {
    setCurrentProjectFile(file);
    fileProject->setText(file);
    mPath.clear();
    MBSimObjectFactory::initialize();
    TiXmlDocument doc;
    if(doc.LoadFile(file.toStdString())) {
      TiXml_PostLoadFile(&doc);
      TiXmlElement *e=doc.FirstChildElement();
      map<string,string> dummy;
      TiXmlElement *ele0 = doc.FirstChildElement();
      incorporateNamespace(ele0, dummy);

      TiXmlElement *ele1 = ele0->FirstChildElement(MBSIMNS"dynamicSystemSolver");
      TiXmlElement *ele2 = ele1->FirstChildElement();
      Solver *solver=dynamic_cast<Solver*>(ObjectFactory::getInstance()->createGroup(ele2,0));
      solver->initializeUsingXML(ele2);
      solver->initialize();
      PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
      QModelIndex index = model->index(0,0);
      if(model->rowCount(index))
        delete model->getItem(index)->getItemData();
      model->removeRow(index.row(), index.parent());
      model->createGroupItem(solver);

      ele1 = ele0->FirstChildElement(MBSIMNS"parameters");
      ele2 = ele1->FirstChildElement();
      ParameterListModel *pmodel = static_cast<ParameterListModel*>(parameterList->model());
      index = pmodel->index(0,0);
      for(int i=0; i<pmodel->rowCount(QModelIndex()); i++)
        delete pmodel->getItem(index.sibling(i,0))->getItemData();
      pmodel->removeRows(index.row(), pmodel->rowCount(QModelIndex()), index.parent());
      TiXmlElement *E=ele2->FirstChildElement();
      vector<QString> refFrame;
      while(E) {
        Parameter *parameter=ObjectFactory::getInstance()->createParameter(E);
        parameter->initializeUsingXML(E);
        pmodel->createParameterItem(parameter);
        E=E->NextSiblingElement();
      }
      updateOctaveParameters();

      ele1 = ele0->FirstChildElement(MBSIMNS"integrator");
      ele2 = ele1->FirstChildElement();
      Integrator *integrator=ObjectFactory::getInstance()->createIntegrator(ele2);
      integrator->initializeUsingXML(ele2);
      integratorView->setIntegrator(integrator);

      ele1 = ele0->FirstChildElement(MBSIMNS"mPaths");
      ele2=ele1->FirstChildElement();
      while(ele2) {
        mPath << ele2->Attribute("href");
        ele2=ele2->NextSiblingElement();
      }

      actionSaveProject->setDisabled(false);

#ifdef INLINE_OPENMBV
      mbsimxml(1);
#endif
    }
  }
}

void MainWindow::loadProject() {
  QString file=QFileDialog::getOpenFileName(0, "XML project files", ".", "XML files (*.mbsimproj.xml)");
  if(file!="")
    loadProject(file);
}

void MainWindow::saveProjectAs() {
  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
  QModelIndex index = model->index(0,0);
  QString file=QFileDialog::getSaveFileName(0, "XML project files", QString("./")+QString::fromStdString(model->getItem(index)->getItemData()->getName())+".mbsimproj.xml", "XML files (*.mbsimproj.xml)");
  if(file!="") {
    setCurrentProjectFile(file);
    fileProject->setText(file.right(14)==".mbsimproj.xml"?file:file+".mbsimproj.xml");
    actionSaveProject->setDisabled(false);
    saveProject();
  }
}

void MainWindow::saveProject() {
  TiXmlDocument doc;
  TiXmlDeclaration *decl = new TiXmlDeclaration("1.0","UTF-8","");
  doc.LinkEndChild( decl );
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS"MBSimProject");
  doc.LinkEndChild(ele0);
  ele0->SetAttribute("xmlns", "http://mbsim.berlios.de/MBSim");
  ele0->SetAttribute("xmlns:ombv", "http://openmbv.berlios.de/OpenMBV");

  //TiXmlElement *ele1 = ele0;
  TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"dynamicSystemSolver" );
  //TiXmlElement *ele2 = new TiXmlElement( PVNS"embed" );
  TiXmlElement *ele2 = ele1;
  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
  QModelIndex index = model->index(0,0);
  Solver *solver = static_cast<Solver*>(model->getItem(index)->getItemData());
  solver->writeXMLFile(ele2);
  //ele1->LinkEndChild(ele2);
  ele0->LinkEndChild(ele1);
  ele1 = new TiXmlElement( MBSIMNS"parameters" );
  //ele2 = new TiXmlElement( PVNS"embed" );
  ele2 = ele1;
  TiXmlElement *ele3=writeParameterList();
  ele2->LinkEndChild(ele3);
  //ele1->LinkEndChild(ele2);
  ele0->LinkEndChild(ele1);
  ele1 = new TiXmlElement( MBSIMNS"integrator" );
  //ele2 = new TiXmlElement( PVNS"embed" );
  ele2 = ele1;
  integratorView->getIntegrator()->writeXMLFile(ele2);
  //ele1->LinkEndChild(ele2);
  ele0->LinkEndChild(ele1);
  ele1 = new TiXmlElement( MBSIMNS"mPaths" );
  for(int i=0; i<mPath.size(); i++) {
    ele2 = new TiXmlElement( MBSIMNS"mPath" );
    ele2->SetAttribute("href",mPath.at(i).toStdString());
    ele1->LinkEndChild(ele2);
  }
  ele0->LinkEndChild(ele1);
  unIncorporateNamespace(doc.FirstChildElement(), Utils::getMBSimNamespacePrefixMapping());  
  QString file = fileProject->text();
  string name = (file.right(14)==".mbsimproj.xml"?file:file+".mbsimproj.xml").toStdString();
  doc.SaveFile((name.length()>14 && name.substr(name.length()-14,14)==".mbsimproj.xml")?name:name+".mbsimproj.xml");
  actionSaveProject->setDisabled(false);
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
    PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
    QModelIndex index = model->index(0,0);
    if(model->rowCount(index))
      delete model->getItem(index)->getItemData();
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
    setCurrentMBSFile(file);
    PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
    QModelIndex index = model->index(0,0);
    if(model->rowCount(index))
      delete model->getItem(index)->getItemData();
    model->removeRow(index.row(), index.parent());
    Solver *sys = Solver::readXMLFile(file.toStdString());
    model->createGroupItem(sys);
    //((Integrator*)integratorView->topLevelItem(0))->setSolver(sys);
    actionSaveMBS->setDisabled(false);
    elementList->setCurrentIndex(model->index(0,0));
    updatePropertyTree();
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
  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
  QModelIndex index = model->index(0,0);
  QString file=QFileDialog::getSaveFileName(0, "XML model files", QString("./")+QString::fromStdString(model->getItem(index)->getItemData()->getName())+".mbsim.xml", "XML files (*.mbsim.xml)");
  if(file!="") {
    setCurrentMBSFile(file);
    fileMBS->setText(file.right(10)==".mbsim.xml"?file:file+".mbsim.xml");
    actionSaveMBS->setDisabled(false);
    saveMBS();
  }
}

void MainWindow::saveMBS() {
  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
  QModelIndex index = model->index(0,0);
  Solver *solver = static_cast<Solver*>(model->getItem(index)->getItemData());
  QString file = fileMBS->text();
  mbsDir = QFileInfo(file).absolutePath();
  solver->writeXMLFile(file.toStdString());
  // Test cloning 
//  Solver solver2(*solver);
//  solver2.writeXMLFile("SolverTestFile.mbsim.xml");
//  Solver solver3("Name",0);
//  solver3 = solver2;
//  solver3.writeXMLFile("SolverTestFile2.mbsim.xml");
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
    setCurrentIntegratorFile(file);
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
  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
  QModelIndex index = model->index(0,0);
  QString file=QFileDialog::getSaveFileName(0, "MBSim integrator files", QString("./")+QString::fromStdString(model->getItem(index)->getItemData()->getName())+".mbsimint.xml", "XML files (*.mbsimint.xml)");
  if(file!="") {
    setCurrentIntegratorFile(file);
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
  delete model->getItem(index)->getItemData();
  model->removeRow(index.row(), index.parent());
  updateOctaveParameters();
}

void MainWindow::addStringParameter() {
  ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
  QModelIndex index = QModelIndex();
  StringParameter *parameter = new StringParameter("a"+toStr(model->getItem(index)->getID()));
  model->createParameterItem(parameter,index);
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

void MainWindow::newParameterList(bool ask) {
  QMessageBox::StandardButton ret = QMessageBox::Ok;
  if(ask) 
    ret = QMessageBox::warning(this, "New parameter list", "Current parameters will be deleted", QMessageBox::Ok | QMessageBox::Cancel);
  if(ret == QMessageBox::Ok) {
    ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
    QModelIndex index = model->index(0,0);
    if(index.isValid()) {
      ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
      QModelIndex index = model->index(0,0);
      for(int i=0; i<model->rowCount(QModelIndex()); i++)
        delete model->getItem(index.sibling(i,0))->getItemData();
      model->removeRows(index.row(), model->rowCount(QModelIndex()), index.parent());
      updateOctaveParameters();
    }
  }
}

void MainWindow::loadParameterList(const QString &file) {
  //tabBar->setCurrentIndex(2);
  ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
  QModelIndex index = model->index(0,0);
  for(int i=0; i<model->rowCount(QModelIndex()); i++)
    delete model->getItem(index.sibling(i,0))->getItemData();
  model->removeRows(index.row(), model->rowCount(QModelIndex()), index.parent());
  fileParameter->setText(file);
  actionSaveParameterList->setDisabled(true);
  if(file!="") {
    setCurrentParameterFile(file);
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
  QString file=QFileDialog::getOpenFileName(0, "MBSim parameter files", ".", "XML files (*.mbsimparam.xml parameter.mbsim.xml)");
  if(file!="") {
    loadParameterList(file);
  }
}

void MainWindow::saveParameterListAs() {
  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
  QModelIndex index = model->index(0,0);
  QString file=QFileDialog::getSaveFileName(0, "MBSim parameter files", QString("./")+QString::fromStdString(model->getItem(index)->getItemData()->getName())+".mbsimparam.xml", "XML files (*.mbsimparam.xml)");
  if(file!="") {
    setCurrentParameterFile(file);
    fileParameter->setText(file.right(15)==".mbsimparam.xml"?file:file+".mbsimparam.xml");
    actionSaveParameterList->setDisabled(false);
    saveParameterList();
  }
}

// write model paramters to XML structure
// The resource owner of the returned TiXmlElement is the caller!
TiXmlElement* MainWindow::writeParameterList() {
  TiXmlElement *ele0=new TiXmlElement(PARAMNS+string("parameter"));
  ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
  QModelIndex index = model->index(0,0);
  for(int i=0; i<model->rowCount(QModelIndex()); i++)
    static_cast<Parameter*>(model->getItem(index.sibling(i,0))->getItemData())->writeXMLFile(ele0);
  return ele0;
}

// write model parameters to file (using writeParameterList)
void MainWindow::saveParameterList(const QString &fileName) {
  TiXmlDocument doc;
  TiXmlDeclaration *decl = new TiXmlDeclaration("1.0","UTF-8","");
  doc.LinkEndChild( decl );
  TiXmlElement *ele0=NULL;
  ele0=writeParameterList();
  doc.LinkEndChild(ele0);
  unIncorporateNamespace(doc.FirstChildElement(), Utils::getMBSimNamespacePrefixMapping());  
  QString file = fileParameter->text();
  doc.SaveFile(fileName.isEmpty()?file.toAscii().data():fileName.toStdString());
}

// update model parameters including additional paramters from paramList
void MainWindow::updateOctaveParameters(const ParameterList &paramList) {
  // write model paramters to XML structure
  TiXmlElement *ele0=NULL;
  ele0=writeParameterList();

  paramList.writeXMLFile(ele0);

  try {
    // remove all parameters from octave using delete and new NewParamLevel
    // (this will not work for nested parameters in embed!???)
    delete octEvalParamLevel;
    octEvalParamLevel=new NewParamLevel(*octEval);
    // add parameter
    octEval->addParamSet(ele0);
    delete ele0;
  }
  catch(string e) {
    cout << "An exception occurred in updateOctaveParameters: " << e << endl;
    delete ele0;
  }
  catch(...) {
    cout << "An unknown exception occurred in updateOctaveParameters." << endl;
    delete ele0;
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
  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
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
  Solver *slv=dynamic_cast<Solver*>(static_cast<PropertyTreeModel*>(elementList->model())->getItem(index)->getItemData());
  //Integrator *integ=(Integrator*)integratorView->topLevelItem(0);
  Integrator *integ=integratorView->getIntegrator();
  if(!slv || !integ)
    return;

  QString sTask = QString::number(task); 
  QString mbsFile=uniqueTempDir+"/in"+sTask+".mbsim.xml";
  string saveName=slv->getValue();
  slv->setValue("out"+sTask.toStdString());
  slv->writeXMLFile(mbsFile.toStdString());
  slv->setValue(saveName);

  QString paramFile=uniqueTempDir+"/in"+sTask+".mbsimparam.xml";
  saveParameterList(paramFile);

  QString intFile=uniqueTempDir+"/in"+sTask+".mbsimint.xml";
  integ->writeXMLFile(intFile.toStdString());

  QStringList arg;
  if(task==1)
    arg.append("--stopafterfirststep");
  for(int i=0; i<mPath.size(); i++) {
    arg.append("--mpath");
    arg.append(mPath.at(i));
  }
  arg.append("--mbsimparam");
  arg.append(paramFile);
  arg.append(mbsFile);
  arg.append(intFile);
  mbsim->getProcess()->setWorkingDirectory(uniqueTempDir);
  mbsim->clearOutputAndStart((MBXMLUtils::getInstallPath()+"/bin/mbsimxml").c_str(), arg);
  absolutePath = false;
}

void MainWindow::simulate() {
  mbsimxml(0);
  actionSaveDataAs->setDisabled(false);
  actionSaveMBSimH5DataAs->setDisabled(false);
  actionSaveOpenMBVDataAs->setDisabled(false);
  actionSimulate->setDisabled(true);
  actionOpenMBV->setDisabled(true);
  actionH5plotserie->setDisabled(true);
  statusBar()->showMessage(tr("Simulating"));
}

void MainWindow::update3DView() {
  mbsimxml(1);
}

void MainWindow::openmbv() {
  QString name = uniqueTempDir+"/out0.ombv.xml";
  cout << "starting openmbv 1" << endl;
  cout << name.toStdString() << endl;
  if(QFile::exists(name)) {
    QStringList arg;
    arg.append("--autoreload");
    arg.append(name);
    cout << "starting openmbv 2" << endl;
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
  cout << "select Element" << endl;
  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
  map<string, QModelIndex>::iterator it=model->idEleMap.find(ID);
  if(it!=model->idEleMap.end()) {
   elementList->setCurrentIndex(it->second);
   updatePropertyTree();
  }
#ifdef INLINE_OPENMBV
  highlightObject(ID);
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
  connect(process, SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(processFinished(int,QProcess::ExitStatus)));
  addTab(out, "Out");
  addTab(err, "Err");
  setCurrentIndex(0);
  setMinimumHeight(80);
  setTabPosition(QTabWidget::West);
  connect(&timer, SIGNAL(timeout()), this, SLOT(updateOutputAndError()));
}

void Process::clearOutputAndStart(const QString &program, const QStringList &arguments) {
  outText="";
  errText="";
  out->clear();
  err->clear();
  setCurrentIndex(0);
  process->start(program, arguments);
  timer.start(250);
}

void Process::updateOutputAndError() {
  QByteArray outArray=process->readAllStandardOutput();
  if(outArray.size()!=0) {
    outText+=outArray.data();
    out->setHtml(convertToHtml(outText));
    out->moveCursor(QTextCursor::End);
  }

  QByteArray errArray=process->readAllStandardError();
  if(errArray.size()!=0) {
    errText+=errArray.data();
    err->setHtml(convertToHtml(errText));
    err->moveCursor(QTextCursor::Start);
  }
}

QString Process::convertToHtml(QString &text) {
  // the following operations modify the original text

#ifdef WIN32
  // convert windows line ending to linux line ending (they are later replaced to html line ending)
  text.replace("\x0D\x0A", "\x0A");
#endif
  // from now on use linux line ending => do not use \n, \r in string literals but \x0A, \x0D

  // remove all lines but the last ending with carriage return '\x0D'
  static QRegExp carriageReturn("(^|\x0A)[^\x0A]*\x0D([^\x0A\x0D]*\x0D)");
  text.replace(carriageReturn, "\\1\\2");

  // replace <FILE ...> to html reference
  static QRegExp fileRE("<FILE path=\"([^\"]+)\" line=\"([0-9]+)\">([^<]+)</FILE>");
  text.replace(fileRE, "<a href=\"\\1?line=\\2\">\\3</a>");

  // the following operations modify only the QString return value
  QString ret=text;

  // newlines '\x0A' to html
  ret.replace("\x0A", "<br/>");

  return ret;
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

void Process::processFinished(int exitCode, QProcess::ExitStatus exitStatus) {
  timer.stop();
  updateOutputAndError();
  if(exitStatus==QProcess::NormalExit && exitCode==0)
    setCurrentIndex(0);
  else
    setCurrentIndex(1);
}

void MainWindow::removeElement() {
  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  Element *element = static_cast<Element*>(model->getItem(index)->getItemData());
  element->getParent()->removeElement(element);
  model->removeRow(index.row(), index.parent());
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
}

void MainWindow::saveElementAs() {
  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  QString file=QFileDialog::getSaveFileName(0, "XML model files", QString("./")+QString::fromStdString(model->getItem(index)->getItemData()->getName())+".xml", "XML files (*.xml)");
  if(file!="")
    static_cast<Element*>(model->getItem(index)->getItemData())->writeXMLFile(file.toStdString());
}

void MainWindow::addFrame(Frame *frame) {
  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  QModelIndex containerIndex = index.data().toString()=="frames"?index:index.child(0,0); 
  frame->setName(frame->getName()+toStr(model->getItem(containerIndex)->getID()));
  frame->getParent()->addFrame(frame);
  model->createFrameItem(frame,containerIndex);
  QModelIndex currentIndex = containerIndex.child(model->rowCount(containerIndex)-1,0);
  elementList->setCurrentIndex(currentIndex);
  updatePropertyTree();
  mbsimxml(1);
//  elementList->openEditor();
}

void MainWindow::addContour(Contour *contour) {
  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  QModelIndex containerIndex = (index.row()==0)?index.child(1,0):index;
  contour->setName(contour->getName()+toStr(model->getItem(containerIndex)->getID()));
  contour->getParent()->addContour(contour);
  model->createContourItem(contour,containerIndex);
  QModelIndex currentIndex = containerIndex.child(model->rowCount(containerIndex)-1,0);
  elementList->setCurrentIndex(currentIndex);
//  elementList->openEditor();
}

void MainWindow::addGroup(Group *group) {
  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  QModelIndex containerIndex = (index.row()==0)?index.child(2,0):index;
  group->setName(group->getName()+toStr(model->getItem(containerIndex)->getID()));
  group->getParent()->addGroup(group);
  model->createGroupItem(group,containerIndex);
  QModelIndex currentIndex = containerIndex.child(model->rowCount(containerIndex)-1,0);
  elementList->setCurrentIndex(currentIndex);
//  elementList->openEditor();
}

void MainWindow::addObject(Object *object) {
  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  QModelIndex containerIndex = (index.row()==0)?index.child(3,0):index;
  object->setName(object->getName()+toStr(model->getItem(containerIndex)->getID()));
  object->getParent()->addObject(object);
  model->createObjectItem(object,containerIndex);
  QModelIndex currentIndex = containerIndex.child(model->rowCount(containerIndex)-1,0);
  elementList->setCurrentIndex(currentIndex);
//  elementList->openEditor();
}

void MainWindow::addLink(Link *link) {
  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  QModelIndex containerIndex = (index.row()==0)?index.child(4,0):index;
  link->setName(link->getName()+toStr(model->getItem(containerIndex)->getID()));
  link->getParent()->addLink(link);
  model->createLinkItem(link,containerIndex);
  QModelIndex currentIndex = containerIndex.child(model->rowCount(containerIndex)-1,0);
  elementList->setCurrentIndex(currentIndex);
//  elementList->openEditor();
}

void MainWindow::addObserver(Observer *observer) {
  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  QModelIndex containerIndex = (index.row()==0)?index.child(5,0):index;
  observer->setName(observer->getName()+toStr(model->getItem(containerIndex)->getID()));
  observer->getParent()->addObserver(observer);
  model->createObserverItem(observer,containerIndex);
  QModelIndex currentIndex = containerIndex.child(model->rowCount(containerIndex)-1,0);
  elementList->setCurrentIndex(currentIndex);
//  elementList->openEditor();
}

void MainWindow::dragEnterEvent(QDragEnterEvent *event) {
  if (event->mimeData()->hasUrls()) {
    event->acceptProposedAction();
  }
}

void MainWindow::dropEvent(QDropEvent *event) {
  for (int i = 0; i < event->mimeData()->urls().size(); i++) {
    QString path = event->mimeData()->urls()[i].toLocalFile().toLocal8Bit().data();
    if(path.endsWith("mbsim.xml")) {
      QFile Fout(path);
      if (Fout.exists())
        loadMBS(Fout.fileName());
    }
    else if(path.endsWith("mbsimparam.xml")) {
      QFile Fout(path);
      if (Fout.exists())
        loadParameterList(Fout.fileName());
    }
    else if(path.endsWith("mbsimint.xml")) {
      QFile Fout(path);
      if (Fout.exists())
        loadIntegrator(Fout.fileName());
    }
  }
}

void MainWindow::openRecentProjectFile() {
  QAction *action = qobject_cast<QAction *>(sender());
  if (action)
    loadProject(action->data().toString());
}

void MainWindow::setCurrentProjectFile(const QString &fileName) {

  QSettings settings;
  QStringList files = settings.value("recentProjectFileList").toStringList();
  files.removeAll(fileName);
  files.prepend(fileName);
  while (files.size() > maxRecentFiles)
    files.removeLast();

  settings.setValue("recentProjectFileList", files);

  foreach (QWidget *widget, QApplication::topLevelWidgets()) {
    MainWindow *mainWin = qobject_cast<MainWindow *>(widget);
    if (mainWin)
      mainWin->updateRecentProjectFileActions();
  }
}

void MainWindow::updateRecentProjectFileActions() {
  QSettings settings;
  QStringList files = settings.value("recentProjectFileList").toStringList();

  int numRecentFiles = qMin(files.size(), (int)maxRecentFiles);

  for (int i = 0; i < numRecentFiles; ++i) {
    QString text = QDir::current().relativeFilePath(files[i]);
//    QString text = tr("&%1 %2").arg(i + 1).arg(QFileInfo(files[i]).fileName());
    recentProjectFileActs[i]->setText(text);
    recentProjectFileActs[i]->setData(files[i]);
    recentProjectFileActs[i]->setVisible(true);
  }
  for (int j = numRecentFiles; j < maxRecentFiles; ++j)
    recentProjectFileActs[j]->setVisible(false);

  //separatorAct->setVisible(numRecentFiles > 0);
}

void MainWindow::openRecentMBSFile() {
  QAction *action = qobject_cast<QAction *>(sender());
  if (action)
    loadMBS(action->data().toString());
}

void MainWindow::setCurrentMBSFile(const QString &fileName) {

  QSettings settings;
  QStringList files = settings.value("recentMBSFileList").toStringList();
  files.removeAll(fileName);
  files.prepend(fileName);
  while (files.size() > maxRecentFiles)
    files.removeLast();

  settings.setValue("recentMBSFileList", files);

  foreach (QWidget *widget, QApplication::topLevelWidgets()) {
    MainWindow *mainWin = qobject_cast<MainWindow *>(widget);
    if (mainWin)
      mainWin->updateRecentMBSFileActions();
  }
}

void MainWindow::updateRecentMBSFileActions() {
  QSettings settings;
  QStringList files = settings.value("recentMBSFileList").toStringList();

  int numRecentFiles = qMin(files.size(), (int)maxRecentFiles);

  for (int i = 0; i < numRecentFiles; ++i) {
    QString text = QDir::current().relativeFilePath(files[i]);
//    QString text = tr("&%1 %2").arg(i + 1).arg(QFileInfo(files[i]).fileName());
    recentMBSFileActs[i]->setText(text);
    recentMBSFileActs[i]->setData(files[i]);
    recentMBSFileActs[i]->setVisible(true);
  }
  for (int j = numRecentFiles; j < maxRecentFiles; ++j)
    recentMBSFileActs[j]->setVisible(false);

  //separatorAct->setVisible(numRecentFiles > 0);
}

void MainWindow::openRecentParameterFile() {
  QAction *action = qobject_cast<QAction *>(sender());
  if (action)
    loadParameterList(action->data().toString());
}

void MainWindow::setCurrentParameterFile(const QString &fileName) {

  QSettings settings;
  QStringList files = settings.value("recentParameterFileList").toStringList();
  files.removeAll(fileName);
  files.prepend(fileName);
  while (files.size() > maxRecentFiles)
    files.removeLast();

  settings.setValue("recentParameterFileList", files);

  foreach (QWidget *widget, QApplication::topLevelWidgets()) {
    MainWindow *mainWin = qobject_cast<MainWindow *>(widget);
    if (mainWin)
      mainWin->updateRecentParameterFileActions();
  }
}

void MainWindow::updateRecentParameterFileActions() {
  QSettings settings;
  QStringList files = settings.value("recentParameterFileList").toStringList();

  int numRecentFiles = qMin(files.size(), (int)maxRecentFiles);

  for (int i = 0; i < numRecentFiles; ++i) {
    QString text = QDir::current().relativeFilePath(files[i]);
    recentParameterFileActs[i]->setText(text);
    recentParameterFileActs[i]->setData(files[i]);
    recentParameterFileActs[i]->setVisible(true);
  }
  for (int j = numRecentFiles; j < maxRecentFiles; ++j)
    recentParameterFileActs[j]->setVisible(false);
}

void MainWindow::openRecentIntegratorFile() {
  QAction *action = qobject_cast<QAction *>(sender());
  if (action)
    loadIntegrator(action->data().toString());
}

void MainWindow::setCurrentIntegratorFile(const QString &fileName) {

  QSettings settings;
  QStringList files = settings.value("recentIntegratorFileList").toStringList();
  files.removeAll(fileName);
  files.prepend(fileName);
  while (files.size() > maxRecentFiles)
    files.removeLast();

  settings.setValue("recentIntegratorFileList", files);

  foreach (QWidget *widget, QApplication::topLevelWidgets()) {
    MainWindow *mainWin = qobject_cast<MainWindow *>(widget);
    if (mainWin)
      mainWin->updateRecentIntegratorFileActions();
  }
}

void MainWindow::updateRecentIntegratorFileActions() {
  QSettings settings;
  QStringList files = settings.value("recentIntegratorFileList").toStringList();

  int numRecentFiles = qMin(files.size(), (int)maxRecentFiles);

  for (int i = 0; i < numRecentFiles; ++i) {
    QString text = QDir::current().relativeFilePath(files[i]);
    recentIntegratorFileActs[i]->setText(text);
    recentIntegratorFileActs[i]->setData(files[i]);
    recentIntegratorFileActs[i]->setVisible(true);
  }
  for (int j = numRecentFiles; j < maxRecentFiles; ++j)
    recentIntegratorFileActs[j]->setVisible(false);
}

void MainWindow::changePropertyItem(Property *property) {
  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(propertyList->model());
  QModelIndex index = propertyList->selectionModel()->currentIndex();
  QModelIndex parentIndex = index.parent();
//  Property *property = static_cast<Property*>(model->getItem(index)->getItemData());
  model->removeRow(index.row(), parentIndex);
  model->createPropertyItem(property,parentIndex);
}

void MainWindow::removeProperty() {
  PropertyTreeModel *model = static_cast<PropertyTreeModel*>(propertyList->model());
  QModelIndex index = propertyList->selectionModel()->currentIndex();
//  Property *property = static_cast<Property*>(model->getItem(index)->getItemData());
  //property->getParent()->removeProperty(property);
  model->removeRow(index.row(), index.parent());

  //delete model->getItem(index)->getItemData();
  //model->removeRow(index.row(), index.parent());
  //updateOctaveParameters();
}


