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
#include "treemodel.h"
#include "delegate.h"
#include "treeitem.h"

using namespace std;

bool absolutePath = false;
QDir mbsDir;

MainWindow *mw;

class IntegratorView : public QLineEdit {
  public:
    IntegratorView() : integrator(0) {}
    ~IntegratorView() {
      delete integrator;
    }
    void newDOPRI5Integrator() {
      delete integrator;
      integrator = new DOPRI5Integrator;
      setText("DOPRI5");
    }
    void newRADAU5Integrator() {
      delete integrator;
      integrator = new RADAU5Integrator;
      setText("RADAU5");
    }
    void newLSODEIntegrator() {
      delete integrator;
      integrator = new LSODEIntegrator;
      setText("LSODE");
    }
    void newLSODARIntegrator() {
      delete integrator;
      integrator = new LSODARIntegrator;
      setText("LSODAR");
    }
    void newTimeSteppingIntegrator() {
      delete integrator;
      integrator = new TimeSteppingIntegrator;
      setText("Time stepping");
    }
    void newEulerExplicitIntegrator() {
      delete integrator;
      integrator = new EulerExplicitIntegrator;
      setText("Euler explicit");
    }
    void newRKSuiteIntegrator() {
      delete integrator;
      integrator = new RKSuiteIntegrator;
      setText("RKSuite");
    }
    Integrator* getIntegrator() {return integrator;}
    void setIntegrator(Integrator *integrator_) {integrator = integrator_;}
  protected:
    Integrator *integrator;
};

bool IntegratorMouseEvent::eventFilter(QObject *obj, QEvent *event) {
  if (event->type() == QEvent::MouseButtonDblClick) {
    dialog = view->getIntegrator()->createPropertyDialog();
    dialog->toWidget(view->getIntegrator());
    connect(dialog,SIGNAL(ok(QWidget*)),this,SLOT(commitDataAndClose()));
    connect(dialog,SIGNAL(apply(QWidget*)),this,SLOT(commitData()));
    connect(dialog,SIGNAL(cancel(QWidget*)),this,SLOT(rejectDataAndClose()));
    dialog->exec();
    delete dialog;
    return true;
  } else
    return QObject::eventFilter(obj, event);
}

void IntegratorMouseEvent::commitData() {
  dialog->fromWidget(view->getIntegrator());
}

void IntegratorMouseEvent::commitDataAndClose() {
  dialog->fromWidget(view->getIntegrator());
  dialog->accept();
}

void IntegratorMouseEvent::rejectDataAndClose() {
  dialog->reject();
}

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

  statusBar()->showMessage(tr("Ready"));

  QMenu *GUIMenu=new QMenu("GUI", menuBar());
  QAction *action = GUIMenu->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_DirHomeIcon)),"Workdir", this, SLOT(changeWorkingDir()));
  action->setStatusTip(tr("Change working directory"));
  GUIMenu->addSeparator();
  action = GUIMenu->addAction(Utils::QIconCached(QString::fromStdString(MBXMLUtils::getInstallPath())+"/share/mbsimgui/icons/exit.svg"), "E&xit", this, SLOT(close()));
  action->setShortcuts(QKeySequence::Quit);
  action->setStatusTip(tr("Exit the application"));
  menuBar()->addMenu(GUIMenu);

  addFrameAction=new QAction("Add frame", this);
  connect(addFrameAction,SIGNAL(triggered()),this,SLOT(addFrame()));
  addContourAction=new QAction("Add contour", this);
  connect(addContourAction,SIGNAL(triggered()),this,SLOT(addContour()));
  addPointAction=new QAction("Add point", this);
  connect(addPointAction,SIGNAL(triggered()),this,SLOT(addPoint()));
  addLineAction=new QAction("Add line", this);
  connect(addLineAction,SIGNAL(triggered()),this,SLOT(addLine()));
  addPlaneAction=new QAction("Add plane", this);
  connect(addPlaneAction,SIGNAL(triggered()),this,SLOT(addPlane()));
  addSphereAction=new QAction("Add sphere", this);
  connect(addSphereAction,SIGNAL(triggered()),this,SLOT(addSphere()));
  addGroupAction=new QAction("Add group", this);
  connect(addGroupAction,SIGNAL(triggered()),this,SLOT(addGroup()));
  addObjectAction=new QAction("Add object", this);
  connect(addObjectAction,SIGNAL(triggered()),this,SLOT(addObject()));
  addRigidBodyAction=new QAction("Add rigid body", this);
  connect(addRigidBodyAction,SIGNAL(triggered()),this,SLOT(addRigidBody()));
  addKinematicConstraintAction=new QAction("Add kinematic constraint", this);
  connect(addKinematicConstraintAction,SIGNAL(triggered()),this,SLOT(addKinematicConstraint()));
  addGearConstraintAction=new QAction("Add gear constraint", this);
  connect(addGearConstraintAction,SIGNAL(triggered()),this,SLOT(addGearConstraint()));
  addJointConstraintAction=new QAction("Add joint constraint", this);
  connect(addJointConstraintAction,SIGNAL(triggered()),this,SLOT(addJointConstraint()));
  addLinkAction=new QAction("Add link", this);
  connect(addLinkAction,SIGNAL(triggered()),this,SLOT(addLink()));
  addKineticExcitationAction=new QAction("Add kinetic excitation", this);
  connect(addKineticExcitationAction,SIGNAL(triggered()),this,SLOT(addKineticExcitation()));
  addSpringDamperAction=new QAction("Add spring damper", this);
  connect(addSpringDamperAction,SIGNAL(triggered()),this,SLOT(addSpringDamper()));
  addJointAction=new QAction("Add joint", this);
  connect(addJointAction,SIGNAL(triggered()),this,SLOT(addJoint()));
  addContactAction=new QAction("Add contact", this);
  connect(addContactAction,SIGNAL(triggered()),this,SLOT(addContact()));
  addObserverAction=new QAction("Add observer", this);
  connect(addObserverAction,SIGNAL(triggered()),this,SLOT(addObserver()));
  addAbsoluteKinematicsObserverAction=new QAction("Add absolute kinematics observer", this);
  connect(addAbsoluteKinematicsObserverAction,SIGNAL(triggered()),this,SLOT(addAbsoluteKinematicsObserver()));
  removeElementAction = new QAction(this);
  removeElementAction->setObjectName(QString::fromUtf8("removeElementAction"));
  connect(removeElementAction, SIGNAL(triggered()), this, SLOT(removeElement()));
  removeElementAction->setText(QApplication::translate("MainWindow", "Remove", 0, QApplication::UnicodeUTF8));
  removeElementAction->setShortcut(QApplication::translate("MainWindow", "Ctrl+R, R", 0, QApplication::UnicodeUTF8));

  addParameterAction=new QAction("Add parameter", this);
  connect(addParameterAction,SIGNAL(triggered()),this,SLOT(addParameter()));
  removeParameterAction = new QAction(this);
  connect(removeParameterAction, SIGNAL(triggered()), this, SLOT(removeParameter()));
  removeParameterAction->setText(QApplication::translate("MainWindow", "Remove", 0, QApplication::UnicodeUTF8));
  removeParameterAction->setShortcut(QApplication::translate("MainWindow", "Ctrl+R, R", 0, QApplication::UnicodeUTF8));

  QMenu *ProjMenu=new QMenu("Project", menuBar());
  ProjMenu->addAction("New", this, SLOT(saveProjAs()));
  ProjMenu->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_DirOpenIcon)),"Load", this, SLOT(loadProj()));
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

  newDOPRI5IntegratorAction=new QAction(Utils::QIconCached("newobject.svg"),"DOPRI5 integrator", this);
  connect(newDOPRI5IntegratorAction,SIGNAL(triggered()),this,SLOT(newDOPRI5Integrator()));
  submenu->addAction(newDOPRI5IntegratorAction);
  
  newRADAU5IntegratorAction=new QAction(Utils::QIconCached("newobject.svg"),"RADAU5 integrator", this);
  connect(newRADAU5IntegratorAction,SIGNAL(triggered()),this,SLOT(newRADAU5Integrator()));
  submenu->addAction(newRADAU5IntegratorAction);

  newLSODEIntegratorAction=new QAction(Utils::QIconCached("newobject.svg"),"LSODE integrator", this);
  connect(newLSODEIntegratorAction,SIGNAL(triggered()),this,SLOT(newLSODEIntegrator()));
  submenu->addAction(newLSODEIntegratorAction);

  newLSODARIntegratorAction=new QAction(Utils::QIconCached("newobject.svg"),"LSODAR integrator", this);
  connect(newLSODARIntegratorAction,SIGNAL(triggered()),this,SLOT(newLSODARIntegrator()));
  submenu->addAction(newLSODARIntegratorAction);

  newTimeSteppingIntegratorAction=new QAction(Utils::QIconCached("newobject.svg"),"Time stepping integrator", this);
  connect(newTimeSteppingIntegratorAction,SIGNAL(triggered()),this,SLOT(newTimeSteppingIntegrator()));
  submenu->addAction(newTimeSteppingIntegratorAction);

  newEulerExplicitIntegratorAction=new QAction(Utils::QIconCached("newobject.svg"),"Euler explicit integrator", this);
  connect(newEulerExplicitIntegratorAction,SIGNAL(triggered()),this,SLOT(newEulerExplicitIntegrator()));
  submenu->addAction(newEulerExplicitIntegratorAction);

  newRKSuiteIntegratorAction=new QAction(Utils::QIconCached("newobject.svg"),"RKSuite integrator", this);
  connect(newRKSuiteIntegratorAction,SIGNAL(triggered()),this,SLOT(newRKSuiteIntegrator()));
  submenu->addAction(newRKSuiteIntegratorAction);

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
  parameterMenu->addAction(addParameterAction);
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

  elementList = new QTreeView;
  elementList->setModel(new ElementTreeModel);
  elementList->setItemDelegate(new ElementDelegate);
  elementList->setColumnWidth(0,250);
  elementList->setColumnWidth(1,200);

  parameterList = new QTreeView;
  parameterList->setModel(new ParameterListModel);
  parameterList->setItemDelegate(new ParameterDelegate);
  parameterList->setColumnWidth(0,75);
  parameterList->setColumnWidth(1,125);

  parameterList->insertAction(0,addParameterAction);
  parameterList->setContextMenuPolicy(Qt::ActionsContextMenu);

  connect(elementList,SIGNAL(pressed(QModelIndex)), this, SLOT(elementListClicked()));
  connect(parameterList,SIGNAL(pressed(QModelIndex)), this, SLOT(parameterListClicked()));

  QDockWidget *dockWidget1 = new QDockWidget("MBS");
  addDockWidget(Qt::LeftDockWidgetArea,dockWidget1);
  QWidget *box = new QWidget;
  dockWidget1->setWidget(box);
  fileMBS = new QLineEdit("");
  fileMBS->setReadOnly(true);
  QVBoxLayout *gl = new QVBoxLayout;
  box->setLayout(gl);
  gl->addWidget(elementList);

  QDockWidget *dockWidget3 = new QDockWidget("Parameter");
  addDockWidget(Qt::LeftDockWidgetArea,dockWidget3);
  box = new QWidget;
  dockWidget3->setWidget(box);
  gl = new QVBoxLayout;
  box->setLayout(gl);
  fileParameter = new QLineEdit("");
  fileParameter->setReadOnly(true);
  gl->addWidget(parameterList);

  QDockWidget *dockWidget2 = new QDockWidget("Integrator");
  addDockWidget(Qt::BottomDockWidgetArea,dockWidget2);
  box = new QWidget;
  dockWidget2->setWidget(box);
  gl = new QVBoxLayout;
  box->setLayout(gl);
  fileIntegrator = new QLineEdit("");
  fileIntegrator->setReadOnly(true);
  integratorList = new IntegratorView;
  integratorList->installEventFilter(new IntegratorMouseEvent(integratorList));
  QList<QAction*> actionList;
  actionList << newDOPRI5IntegratorAction << newRADAU5IntegratorAction << newLSODEIntegratorAction << newLSODARIntegratorAction << newTimeSteppingIntegratorAction << newEulerExplicitIntegratorAction << newRKSuiteIntegratorAction;
  integratorList->insertActions(0,actionList);
  integratorList->setContextMenuPolicy(Qt::ActionsContextMenu);
  integratorList->setReadOnly(true);
  gl->addWidget(integratorList);
 
  //tabifyDockWidget(dockWidget1,dockWidget2);
  //tabifyDockWidget(dockWidget2,dockWidget3);
  //QList<QTabBar *> tabList = findChildren<QTabBar *>();
  //tabBar = tabList.at(0);

  QWidget *centralWidget = new QWidget;  
  setCentralWidget(centralWidget);
  QHBoxLayout *mainlayout = new QHBoxLayout;
  centralWidget->setLayout(mainlayout);
  pagesWidget = new QStackedWidget;
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

  setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
  
  newDOPRI5Integrator();
  newMBS();
}

void MainWindow::openPropertyDialog() {
  QModelIndex index = elementList->selectionModel()->currentIndex();
  elementList->edit(index);
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
    if(index.column()==0) {
      ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
      QMenu menu("Context Menu");
      if(dynamic_cast<Group*>(model->getItem(index)->getItemData())) {
        menu.addSeparator();
        menu.addAction(addFrameAction);
        menu.addAction(addContourAction);
        menu.addAction(addGroupAction);
        menu.addAction(addObjectAction);
        menu.addAction(addLinkAction);
        menu.addAction(addObserverAction);
      }
      else if(dynamic_cast<Body*>(model->getItem(index)->getItemData())) {
        menu.addAction(addFrameAction);
        menu.addAction(addContourAction);
      }
      menu.addSeparator();
      if(model->getItem(index)->isRemovable()) {
        //menu.insertAction(propertiesAction);
        menu.addAction(removeElementAction);
      }
      menu.exec(QCursor::pos());
    } 
  }
}

void MainWindow::integratorListClicked() {
  cout << "integratorListClicked" << endl;
//  if(QApplication::mouseButtons()==Qt::RightButton) {
//    Integrator *integrator=(Integrator*)integratorList->currentItem();
//    if(integrator) {
//      QMenu* menu=integrator->getContextMenu();
//      menu->exec(QCursor::pos());
//    }
//  } 
//  else if(QApplication::mouseButtons()==Qt::LeftButton) {
//    Integrator *integrator=(Integrator*)integratorList->currentItem();
//    pagesWidget->insertWidget(0,integrator->getPropertyWidget());
//    pagesWidget->setCurrentWidget(integrator->getPropertyWidget());
//  }
}

void MainWindow::parameterListClicked() {
  if(QApplication::mouseButtons()==Qt::RightButton) {
    QModelIndex index = parameterList->selectionModel()->currentIndex();
    if(index.column()==0) {
      ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
      QMenu menu("Context Menu");
      menu.addAction(removeParameterAction);
      menu.exec(QCursor::pos());
    } 
  }
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
      actionSaveParameter->setDisabled(false);
  //    saveParameter();
      actionSaveProj->setDisabled(false);
    }
//  }
}

void MainWindow::saveProj() {
  saveMBS();
  saveIntegrator();
  saveParameter();
}

void MainWindow::newMBS() {
  //tabBar->setCurrentIndex(0);
  mbsDir = QDir::current();
  actionOpenMBV->setDisabled(true);
  actionH5plotserie->setDisabled(true);
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = model->index(0,0);
  model->removeRow(index.row(), index.parent());
  model->addSolver();

  //((Integrator*)integratorList->topLevelItem(0))->setSolver(0);

  actionSaveMBS->setDisabled(true);
  fileMBS->setText("");
  absoluteMBSFilePath="";

#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
}

void MainWindow::loadMBS(const QString &file) {
  //tabBar->setCurrentIndex(0);
  mbsDir = QFileInfo(file).absolutePath();
  absoluteMBSFilePath=file;
  fileMBS->setText(QDir::current().relativeFilePath(absoluteMBSFilePath));
  actionOpenMBV->setDisabled(true);
  actionH5plotserie->setDisabled(true);
  actionSaveMBS->setDisabled(true);
  if(file!="") {
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = model->index(0,0);
    model->removeRow(index.row(), index.parent());
    Solver *sys = Solver::readXMLFile(file.toStdString());
    model->createGroupItem(sys);
    //((Integrator*)integratorList->topLevelItem(0))->setSolver(sys);
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

void MainWindow::newDOPRI5Integrator() {
  actionSaveIntegrator->setDisabled(true);
  integratorList->newDOPRI5Integrator();
  fileIntegrator->setText("");
}

void MainWindow::newRADAU5Integrator() {
  actionSaveIntegrator->setDisabled(true);
  integratorList->newRADAU5Integrator();
  fileIntegrator->setText("");
}

void MainWindow::newLSODEIntegrator() {
  actionSaveIntegrator->setDisabled(true);
  integratorList->newLSODEIntegrator();
  fileIntegrator->setText("");
}

void MainWindow::newLSODARIntegrator() {
  actionSaveIntegrator->setDisabled(true);
  integratorList->newLSODARIntegrator();
  fileIntegrator->setText("");
}

void MainWindow::newTimeSteppingIntegrator() {
  actionSaveIntegrator->setDisabled(true);
  integratorList->newTimeSteppingIntegrator();
  fileIntegrator->setText("");
}

void MainWindow::newEulerExplicitIntegrator() {
  actionSaveIntegrator->setDisabled(true);
  integratorList->newEulerExplicitIntegrator();
  fileIntegrator->setText("");
}

void MainWindow::newRKSuiteIntegrator() {
  actionSaveIntegrator->setDisabled(true);
  integratorList->newRKSuiteIntegrator();
  fileIntegrator->setText("");
}

void MainWindow::loadIntegrator(const QString &file) {
  fileIntegrator->setText(file);
  actionSaveIntegrator->setDisabled(true);
  if(file!="") {
    integratorList->setIntegrator(Integrator::readXMLFile(file.toStdString()));
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
  integratorList->getIntegrator()->writeXMLFile(file.toStdString());
}

void MainWindow::removeParameter() {
  ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
  QModelIndex index = parameterList->selectionModel()->currentIndex();
  model->removeParameter(index);
}

void MainWindow::addParameter() {
  //tabBar->setCurrentIndex(2);
  ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
  model->addParameter();
  updateOctaveParameters();
}

void MainWindow::newParameter() {
  ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
  QModelIndex index = model->index(0,0);
  model->removeRows(index.row(), model->rowCount(QModelIndex()), index.parent());
}

void MainWindow::loadParameter(const QString &file) {
  //tabBar->setCurrentIndex(2);
  ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
  QModelIndex index = model->index(0,0);
  model->removeRows(index.row(), model->rowCount(QModelIndex()), index.parent());
  fileParameter->setText(file);
  actionSaveParameter->setDisabled(true);
  if(file!="") {
    ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
    QModelIndex index = model->index(0,0);
    model->removeRow(index.row(), index.parent());

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
    while(E) {
      Parameter *parameter=ObjectFactory::getInstance()->createParameter(E);
      parameter->initializeUsingXML(E);
      model->createParameterItem(parameter);
      //connect(parameter,SIGNAL(parameterChanged(const QString&)),this,SLOT(updateOctaveParameters()));
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
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = model->index(0,0);
  QString file=QFileDialog::getSaveFileName(0, "MBSim parameter files", QString("./")+QString::fromStdString(model->getItem(index)->getItemData()->getName())+".mbsimparam.xml", "XML files (*.mbsimparam.xml)");
  if(file!="") {
    fileParameter->setText(file.right(15)==".mbsimparam.xml"?file:file+".mbsimparam.xml");
    actionSaveParameter->setDisabled(false);
    saveParameter();
  }
}

void MainWindow::saveParameter(QString fileName) {
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

void MainWindow::updateOctaveParameters() {
  vector<MBXMLUtils::OctaveEvaluator::Param> param;
  ParameterListModel *model = static_cast<ParameterListModel*>(parameterList->model());
  QModelIndex index = model->index(0,0);
  for(int i=0; i<model->rowCount(QModelIndex()); i++) {
    Parameter *p=static_cast<Parameter*>(model->getItem(index.sibling(i,0))->getItemData());
    param.push_back(MBXMLUtils::OctaveEvaluator::Param(p->getName(), toStr(p->getValue()), 0));
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
  QModelIndex index = elementList->model()->index(0,0);
  Solver *slv=dynamic_cast<Solver*>(static_cast<ElementTreeModel*>(elementList->model())->getItem(index)->getItemData());
  //Integrator *integ=(Integrator*)integratorList->topLevelItem(0);
  Integrator *integ=integratorList->getIntegrator();
  if(!slv || !integ)
    return;

  QString sTask = QString::number(task); 
  QString mbsFile=uniqueTempDir+"/in"+sTask+".mbsim.xml";
  string saveName=slv->getName();
  slv->setName("out"+sTask.toStdString());
  slv->writeXMLFile(mbsFile.toStdString());
  slv->setName(saveName);

  QString mbsParamFile=uniqueTempDir+"/in"+sTask+".mbsimparam.xml";
  saveParameter(mbsParamFile);

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
  model->removeElement(index);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
}

void MainWindow::addGroup() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  model->addGroup(index);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex containerIndex = model->index(2, 0, index);
  QModelIndex currentIndex = model->index(model->rowCount(containerIndex)-1,0,containerIndex);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  //elementList->selectionModel()->setCurrentIndex(currentIndex.sibling(currentIndex.row(),1),QItemSelectionModel::Select);
}

void MainWindow::addContour() {
  QMenu menu("Context Menu");
  menu.addAction(addPointAction);
  menu.addAction(addLineAction);
  menu.addAction(addPlaneAction);
  menu.addAction(addSphereAction);
  menu.exec(QCursor::pos());
}

void MainWindow::addObject() {
  QMenu menu("Context Menu");
  menu.addAction(addRigidBodyAction);
  menu.addAction(addGearConstraintAction);
  menu.addAction(addKinematicConstraintAction);
  menu.addAction(addJointConstraintAction);
  menu.exec(QCursor::pos());
}

void MainWindow::addLink() {
  QMenu menu("Context Menu");
  menu.addAction(addKineticExcitationAction);
  menu.addAction(addSpringDamperAction);
  menu.addAction(addJointAction);
  menu.addAction(addContactAction);
  menu.exec(QCursor::pos());
}

void MainWindow::addObserver() {
  QMenu menu("Context Menu");
  menu.addAction(addAbsoluteKinematicsObserverAction);
  menu.exec(QCursor::pos());
}

void MainWindow::addRigidBody() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  model->addRigidBody(index);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex containerIndex = model->index(3, 0, index);
  QModelIndex currentIndex = model->index(model->rowCount(containerIndex)-1,0,containerIndex);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  //elementList->selectionModel()->setCurrentIndex(currentIndex.sibling(currentIndex.row(),1),QItemSelectionModel::Select);
}

void MainWindow::addGearConstraint() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  model->addGearConstraint(index);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex containerIndex = model->index(3, 0, index);
  QModelIndex currentIndex = model->index(model->rowCount(containerIndex)-1,0,containerIndex);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  //elementList->selectionModel()->setCurrentIndex(currentIndex.sibling(currentIndex.row(),1),QItemSelectionModel::Select);
}

void MainWindow::addKinematicConstraint() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  model->addKinematicConstraint(index);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex containerIndex = model->index(3, 0, index);
  QModelIndex currentIndex = model->index(model->rowCount(containerIndex)-1,0,containerIndex);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  //elementList->selectionModel()->setCurrentIndex(currentIndex.sibling(currentIndex.row(),1),QItemSelectionModel::Select);
}

void MainWindow::addJointConstraint() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  model->addJointConstraint(index);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex containerIndex = model->index(3, 0, index);
  QModelIndex currentIndex = model->index(model->rowCount(containerIndex)-1,0,containerIndex);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  //elementList->selectionModel()->setCurrentIndex(currentIndex.sibling(currentIndex.row(),1),QItemSelectionModel::Select);
}


void MainWindow::addFrame() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  model->addFrame(index);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex containerIndex = model->index(0, 0, index);
  QModelIndex currentIndex = model->index(model->rowCount(containerIndex)-1,0,containerIndex);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  //elementList->selectionModel()->setCurrentIndex(currentIndex.sibling(currentIndex.row(),1),QItemSelectionModel::Select);
}

void MainWindow::addPoint() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  model->addPoint(index);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex containerIndex = model->index(1, 0, index);
  QModelIndex currentIndex = model->index(model->rowCount(containerIndex)-1,0,containerIndex);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  //elementList->selectionModel()->setCurrentIndex(currentIndex.sibling(currentIndex.row(),1),QItemSelectionModel::Select);
}

void MainWindow::addLine() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  model->addLine(index);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex containerIndex = model->index(1, 0, index);
  QModelIndex currentIndex = model->index(model->rowCount(containerIndex)-1,0,containerIndex);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  //elementList->selectionModel()->setCurrentIndex(currentIndex.sibling(currentIndex.row(),1),QItemSelectionModel::Select);
}

void MainWindow::addPlane() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  model->addPlane(index);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex containerIndex = model->index(1, 0, index);
  QModelIndex currentIndex = model->index(model->rowCount(containerIndex)-1,0,containerIndex);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  //elementList->selectionModel()->setCurrentIndex(currentIndex.sibling(currentIndex.row(),1),QItemSelectionModel::Select);
}

void MainWindow::addSphere() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  model->addSphere(index);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex containerIndex = model->index(1, 0, index);
  QModelIndex currentIndex = model->index(model->rowCount(containerIndex)-1,0,containerIndex);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  //elementList->selectionModel()->setCurrentIndex(currentIndex.sibling(currentIndex.row(),1),QItemSelectionModel::Select);
}

void MainWindow::addKineticExcitation() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  model->addKineticExcitation(index);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex containerIndex = model->index(4, 0, index);
  QModelIndex currentIndex = model->index(model->rowCount(containerIndex)-1,0,containerIndex);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  //elementList->selectionModel()->setCurrentIndex(currentIndex.sibling(currentIndex.row(),1),QItemSelectionModel::Select);
}

void MainWindow::addSpringDamper() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  model->addSpringDamper(index);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex containerIndex = model->index(4, 0, index);
  QModelIndex currentIndex = model->index(model->rowCount(containerIndex)-1,0,containerIndex);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  //elementList->selectionModel()->setCurrentIndex(currentIndex.sibling(currentIndex.row(),1),QItemSelectionModel::Select);
}

void MainWindow::addJoint() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  model->addJoint(index);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex containerIndex = model->index(4, 0, index);
  QModelIndex currentIndex = model->index(model->rowCount(containerIndex)-1,0,containerIndex);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  //elementList->selectionModel()->setCurrentIndex(currentIndex.sibling(currentIndex.row(),1),QItemSelectionModel::Select);
}

void MainWindow::addContact() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  model->addContact(index);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex containerIndex = model->index(4, 0, index);
  QModelIndex currentIndex = model->index(model->rowCount(containerIndex)-1,0,containerIndex);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  //elementList->selectionModel()->setCurrentIndex(currentIndex.sibling(currentIndex.row(),1),QItemSelectionModel::Select);
}

void MainWindow::addAbsoluteKinematicsObserver() {
  ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
  QModelIndex index = elementList->selectionModel()->currentIndex();
  model->addAbsoluteKinematicsObserver(index);
#ifdef INLINE_OPENMBV
  mbsimxml(1);
#endif
  QModelIndex containerIndex = model->index(5, 0, index);
  QModelIndex currentIndex = model->index(model->rowCount(containerIndex)-1,0,containerIndex);
  elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
  //elementList->selectionModel()->setCurrentIndex(currentIndex.sibling(currentIndex.row(),1),QItemSelectionModel::Select);
}
