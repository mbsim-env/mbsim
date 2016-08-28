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
#include "options.h"
#include "dynamic_system_solver.h"
#include "frame.h"
#include "contour.h"
#include "object.h"
#include "link.h"
#include "constraint.h"
#include "observer.h"
#include "integrator.h"
#include "objectfactory.h"
#include "parameter.h"
#include "widget.h"
#include "treemodel.h"
#include "treeitem.h"
#include "element_view.h"
#include "embedding_view.h"
#include "solver_view.h"
#include "embed.h"
#include "mbsim_process.h"
#include "project_property_dialog.h"
#include <openmbv/mainwindow.h>
#include <utime.h>
#include <QtGui>
#include <mbxmlutils/eval.h>
#include <mbxmlutils/preprocess.h>
#include <mbxmlutilshelper/getinstallpath.h>
#include <mbxmlutilshelper/dom.h>
#include <xercesc/dom/DOMProcessingInstruction.hpp>
#include <xercesc/dom/DOMException.hpp>
#ifdef WIN32
# define putenv _putenv
#endif
#include "function_properties.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;
namespace bfs=boost::filesystem;

namespace MBSimGUI {

  bool currentTask;
  bool absolutePath = false;
  QDir mbsDir;

  MainWindow *mw;

  vector<boost::filesystem::path> dependencies;

  QDialog *MainWindow::helpDialog = NULL;
  QWebView *MainWindow::helpViewer = NULL;

  MainWindow::MainWindow(QStringList &arg) : inlineOpenMBVMW(0), autoSave(true), autoExport(false), saveFinalStateVector(false), autoSaveInterval(5), autoExportDir("./"), evalSelect(0,false) {
    // use html output of MBXMLUtils
    putenv(const_cast<char*>("MBXMLUTILS_HTMLOUTPUT=1"));

    evalSelect.setProperty(new TextProperty("octave", PV%"evaluator", true));
    
    mw = this;

    if(bfs::is_directory("/dev/shm"))
      uniqueTempDir=bfs::unique_path("/dev/shm/mbsimgui_%%%%-%%%%-%%%%-%%%%");
    else
      uniqueTempDir=bfs::unique_path(bfs::temp_directory_path()/"mbsimgui_%%%%-%%%%-%%%%-%%%%");
    bfs::create_directories(uniqueTempDir);

    mbsim = new Process(this);

    mbsimThread = new MBSimThread(this);
    parser=DOMParser::create(true);
    mbsimThread->setParser(parser);
    connect(mbsimThread, SIGNAL(resultReady(int)), this, SLOT(preprocessFinished(int)));

    QString program = (MBXMLUtils::getInstallPath()/"bin"/"mbsimxml").string().c_str();
    QStringList arguments;
    arguments << "--onlyGenerateSchema" << (uniqueTempDir/"mbsimxml.xsd").string().c_str();
    mbsim->getProcess()->execute(program,arguments);
    parser->loadGrammar(arguments[1].toStdString());

    elementList = new ElementView;

    initInlineOpenMBV();

    MBSimObjectFactory::initialize();
    eval=Eval::createEvaluator("octave", &dependencies);

    QMenu *GUIMenu=new QMenu("GUI", menuBar());
    menuBar()->addMenu(GUIMenu);

    QAction *action = GUIMenu->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_DirHomeIcon)),"Workdir", this, SLOT(changeWorkingDir()));
    action->setStatusTip(tr("Change working directory"));

    action = GUIMenu->addAction("Options", this, SLOT(openOptionsMenu()));
    action->setStatusTip(tr("Open options menu"));

    GUIMenu->addSeparator();

    GUIMenu->addSeparator();

    action = GUIMenu->addAction(Utils::QIconCached(QString::fromStdString((MBXMLUtils::getInstallPath()/"share"/"mbsimgui"/"icons"/"exit.svg").string())), "E&xit", this, SLOT(close()));
    action->setShortcuts(QKeySequence::Quit);
    action->setStatusTip(tr("Exit the application"));

    for (int i=0; i<maxRecentFiles; i++) {
      recentProjectFileActs[i] = new QAction(this);
      recentProjectFileActs[i]->setVisible(false);
      connect(recentProjectFileActs[i], SIGNAL(triggered()), this, SLOT(openRecentProjectFile()));
    }
    QMenu *menu=new QMenu("Project", menuBar());
    action = menu->addAction("New", this, SLOT(newProject()));
    action->setShortcuts(QKeySequence::New);
    action = menu->addAction("Load", this, SLOT(loadProject()));
    action->setShortcuts(QKeySequence::Open);
    action = menu->addAction("Save as", this, SLOT(saveProjectAs()));
    action->setShortcuts(QKeySequence::SaveAs);
    actionSaveProject = menu->addAction("Save", this, SLOT(saveProject()));
    actionSaveProject->setShortcuts(QKeySequence::Save);
   //ProjMenu->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_DirOpenIcon)),"Load", this, SLOT(loadProj()));
    actionSaveProject->setDisabled(true);
    menu->addSeparator();
    //separatorAct = menu->addSeparator();
    for (int i = 0; i < maxRecentFiles; ++i)
      menu->addAction(recentProjectFileActs[i]);
    updateRecentProjectFileActions();
    menu->addSeparator();
    menuBar()->addMenu(menu);
    menu->addSeparator();
    action = menu->addAction("Settings", this, SLOT(projectSettings()));

    menu=new QMenu("Export", menuBar());
    actionSaveDataAs = menu->addAction("Export all data", this, SLOT(saveDataAs()));
    actionSaveMBSimH5DataAs = menu->addAction("Export MBSim data file", this, SLOT(saveMBSimH5DataAs()));
    actionSaveOpenMBVDataAs = menu->addAction("Export OpenMBV data", this, SLOT(saveOpenMBVDataAs()));
    actionSaveStateVectorAs = menu->addAction("Export state vector", this, SLOT(saveStateVectorAs()));
    actionSaveEigenanalysisAs = menu->addAction("Export eigenanalysis", this, SLOT(saveEigenanalysisAs()));
    actionSaveDataAs->setDisabled(true);
    actionSaveMBSimH5DataAs->setDisabled(true);
    actionSaveOpenMBVDataAs->setDisabled(true);
    actionSaveStateVectorAs->setDisabled(true);
    actionSaveEigenanalysisAs->setDisabled(true);
    menuBar()->addMenu(menu);

    menuBar()->addSeparator();
    QMenu *helpMenu=new QMenu("Help", menuBar());
    helpMenu->addAction("GUI Help...", this, SLOT(help()));
    helpMenu->addAction("XML Help...", this, SLOT(xmlHelp()));
    helpMenu->addAction("About MBSim GUI", this, SLOT(about()));
    menuBar()->addMenu(helpMenu);

    QToolBar *toolBar = addToolBar("Tasks");
    actionSimulate = toolBar->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_MediaPlay)),"Start simulation");
    //actionSimulate->setStatusTip(tr("Simulate the multibody system"));
    actionSimulate->setStatusTip(tr("Simulate the multibody system"));
    connect(actionSimulate,SIGNAL(triggered()),this,SLOT(simulate()));
    toolBar->addAction(actionSimulate);
    QAction *actionInterrupt = toolBar->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_MediaStop)),"Interrupt simulation");
//    actionInterrupt->setDisabled(true);
    connect(actionInterrupt,SIGNAL(triggered()),mbsim,SLOT(interrupt()));
    toolBar->addAction(actionInterrupt);
    actionRefresh = toolBar->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_BrowserReload)),"Refresh 3D view");
//    actionRefresh->setDisabled(true);
    connect(actionRefresh,SIGNAL(triggered()),this,SLOT(refresh()));
    toolBar->addAction(actionRefresh);
    actionOpenMBV = toolBar->addAction(Utils::QIconCached(QString::fromStdString((MBXMLUtils::getInstallPath()/"share"/"mbsimgui"/"icons"/"openmbv.svg").string())),"OpenMBV");
    actionOpenMBV->setDisabled(true);
    connect(actionOpenMBV,SIGNAL(triggered()),this,SLOT(openmbv()));
    toolBar->addAction(actionOpenMBV);
    actionH5plotserie = toolBar->addAction(Utils::QIconCached(QString::fromStdString((MBXMLUtils::getInstallPath()/"share"/"mbsimgui"/"icons"/"h5plotserie.svg").string())),"H5plotserie");
    actionH5plotserie->setDisabled(true);
    connect(actionH5plotserie,SIGNAL(triggered()),this,SLOT(h5plotserie()));
    toolBar->addAction(actionH5plotserie);
    actionEigenanalysis = toolBar->addAction(Utils::QIconCached(QString::fromStdString((MBXMLUtils::getInstallPath()/"share"/"mbsimgui"/"icons"/"eigenanalysis.svg").string())),"Eigenanalysis");
    actionEigenanalysis->setDisabled(true);
    connect(actionEigenanalysis,SIGNAL(triggered()),this,SLOT(eigenanalysis()));
    toolBar->addAction(actionEigenanalysis);

    elementList->setModel(new ElementTreeModel);
    elementList->setColumnWidth(0,250);
    elementList->setColumnWidth(1,200);
    elementList->hideColumn(1);

    embeddingList = new EmbeddingView;
    embeddingList->setModel(new EmbeddingTreeModel);
    embeddingList->setColumnWidth(0,150);
    embeddingList->setColumnWidth(1,200);

    solverView = new SolverView;

    connect(elementList,SIGNAL(pressed(QModelIndex)), this, SLOT(elementListClicked()));
    connect(embeddingList,SIGNAL(pressed(QModelIndex)), this, SLOT(parameterListClicked()));
    connect(elementList->selectionModel(),SIGNAL(currentChanged(const QModelIndex&,const QModelIndex&)), this, SLOT(selectionChanged(const QModelIndex&,const QModelIndex&)));

    QDockWidget *dockWidget1 = new QDockWidget("Multibody system");
    addDockWidget(Qt::LeftDockWidgetArea,dockWidget1);
    QWidget *widget1=new QWidget(dockWidget1);
    dockWidget1->setWidget(widget1);
    QGridLayout *widgetLayout1=new QGridLayout(widget1);
    widgetLayout1->setContentsMargins(0,0,0,0);
    widget1->setLayout(widgetLayout1);
    OpenMBVGUI::AbstractViewFilter *elementListFilter=new OpenMBVGUI::AbstractViewFilter(elementList, 0, 1);
    widgetLayout1->addWidget(elementListFilter, 0, 0);
    widgetLayout1->addWidget(elementList, 1, 0);

    QDockWidget *dockWidget3 = new QDockWidget("Embeddings");
    addDockWidget(Qt::LeftDockWidgetArea,dockWidget3);
    QWidget *widget3=new QWidget(dockWidget3);
    dockWidget3->setWidget(widget3);
    QGridLayout *widgetLayout3=new QGridLayout(widget3);
    widgetLayout3->setContentsMargins(0,0,0,0);
    widget3->setLayout(widgetLayout3);
    OpenMBVGUI::AbstractViewFilter *parameterListFilter=new OpenMBVGUI::AbstractViewFilter(embeddingList, 0, -2);
    widgetLayout3->addWidget(parameterListFilter, 0, 0);
    widgetLayout3->addWidget(embeddingList, 1, 0);

    QDockWidget *dockWidget2 = new QDockWidget("Solver");
    addDockWidget(Qt::LeftDockWidgetArea,dockWidget2);
    dockWidget2->setWidget(solverView);

    //tabifyDockWidget(dockWidget3,dockWidget4);
    //tabifyDockWidget(dockWidget2,dockWidget3);
    //QList<QTabBar *> tabList = findChildren<QTabBar *>();
    //tabBar = tabList.at(0);

    QWidget *centralWidget = new QWidget;  
    setCentralWidget(centralWidget);
    QHBoxLayout *mainlayout = new QHBoxLayout;
    centralWidget->setLayout(mainlayout);
    mainlayout->addWidget(inlineOpenMBVMW);

    QDockWidget *mbsimDW = new QDockWidget("MBSim Echo Area", this);
    addDockWidget(Qt::BottomDockWidgetArea, mbsimDW);
    mbsimDW->setWidget(mbsim); 
    connect(mbsim->getProcess(),SIGNAL(finished(int,QProcess::ExitStatus)),this,SLOT(simulationFinished(int,QProcess::ExitStatus)));

    setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);

    QStringList::iterator i, i2;

    if((i=std::find(arg.begin(), arg.end(), "--maximized"))!=arg.end()) {
      showMaximized();
      arg.erase(i);
    }

    QString fileProject;
    QRegExp filterProject(".+\\.mbsimprj\\.xml");
    QDir dir;
    dir.setFilter(QDir::Files);
    i=arg.begin();
    while(i!=arg.end() and (*i)[0]!='-') {
      dir.setPath(*i);
      if(dir.exists()) {
        QStringList file=dir.entryList();
        for(int j=0; j<file.size(); j++) {
          if(fileProject.isEmpty() and filterProject.exactMatch(file[j]))
            fileProject = dir.path()+"/"+file[j];
        }
        i2=i; i++; arg.erase(i2);
        continue;
      }
      if(QFile::exists(*i)) {
        if(fileProject.isEmpty() and filterProject.exactMatch(*i))
          fileProject = *i;
        i2=i; i++; arg.erase(i2);
        continue;
      }
      i++;
    }
    if(fileProject.size())
      loadProject(QDir::current().absoluteFilePath(fileProject));
    else
      newProject(false);

    setAcceptDrops(true);

    autoSaveTimer = new QTimer(this);
    connect(autoSaveTimer, SIGNAL(timeout()), this, SLOT(autoSaveProject()));
    autoSaveTimer->start(autoSaveInterval*60000);
  }

  void MainWindow::autoSaveProject() {
    saveProject("./.MBS.mbsimprj.xml");
  }

  void MainWindow::simulationFinished(int exitCode, QProcess::ExitStatus exitStatus) {
    if(currentTask==1) {
      inlineOpenMBVMW->openFile(uniqueTempDir.generic_string()+"/out1.ombv.xml");
      QModelIndex index = elementList->selectionModel()->currentIndex();
      ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
      Element *element=dynamic_cast<Element*>(model->getItem(index)->getItemData());
      if(element)
        highlightObject(element->getID());
    }
    else if(autoExport) {
      saveMBSimH5Data(autoExportDir+"/MBS.mbsim.h5");
      saveOpenMBVXMLData(autoExportDir+"/MBS.ombv.xml");
      saveOpenMBVH5Data(autoExportDir+"/MBS.ombv.h5");
      if(saveFinalStateVector)
        saveStateVector(autoExportDir+"/statevector.asc");
    }
    actionSimulate->setDisabled(false);
    actionOpenMBV->setDisabled(false);
    actionH5plotserie->setDisabled(false);
    actionEigenanalysis->setDisabled(false);
    actionRefresh->setDisabled(false);
    statusBar()->showMessage(tr("Ready"));
  }

  void MainWindow::initInlineOpenMBV() {
    std::list<string> arg;
    arg.push_back("--wst");
    arg.push_back((MBXMLUtils::getInstallPath()/"share"/"mbsimgui"/"inlineopenmbv.ombv.wst").string());
    arg.push_back("/home/foerg/tmp/openmbv");
    inlineOpenMBVMW=new OpenMBVGUI::MainWindow(arg);

    connect(inlineOpenMBVMW, SIGNAL(objectSelected(std::string, Object*)), this, SLOT(selectElement(std::string)));
    connect(inlineOpenMBVMW, SIGNAL(objectDoubleClicked(std::string, Object*)), elementList, SLOT(openEditor()));
  }

  MainWindow::~MainWindow() {
    delete mbsim;
    delete mbsimThread;
    bfs::remove_all(uniqueTempDir);
    bfs::remove("./.MBS.mbsimprj.xml");
  }

  void MainWindow::setProjectChanged(bool changed) { 
    setWindowModified(changed);
  }
  
  bool MainWindow::maybeSave() {
    if(isWindowModified()) {
      QMessageBox::StandardButton ret = QMessageBox::warning(this, tr("Application"),
          tr("Project has been modified.\n"
            "Do you want to save your changes?"),
          QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
      if(ret == QMessageBox::Save) {
        if(actionSaveProject->isEnabled())
          return saveProject();
        else
          return saveProjectAs();
      } 
      else if(ret == QMessageBox::Cancel) 
        return false;
    }
    return true;
  }

  void MainWindow::changeWorkingDir() {
    QString dir = QFileDialog::getExistingDirectory (0, "Working directory", ".");
    if(not(dir.isEmpty())) {
      QString absoluteMBSFilePath = QDir::current().absoluteFilePath(fileProject);
      QDir::setCurrent(dir);
      mbsDir = QFileInfo(absoluteMBSFilePath).absolutePath();
      fileProject = QDir::current().relativeFilePath(absoluteMBSFilePath);
      updateRecentProjectFileActions();
    }
  }

  void MainWindow::openOptionsMenu() {
    OptionsDialog menu;
    menu.setAutoSave(autoSave);
    menu.setAutoSaveInterval(autoSaveInterval);
    menu.setAutoExport(autoExport);
    menu.setAutoExportDir(autoExportDir);
    menu.setSaveStateVector(saveFinalStateVector);
    int res = menu.exec();
    if(res == 1) {
      autoSave = menu.getAutoSave();
      autoSaveInterval = menu.getAutoSaveInterval();
      autoExport = menu.getAutoExport();
      autoExportDir = menu.getAutoExportDir();
      saveFinalStateVector = menu.getSaveStateVector();
      if(not(saveFinalStateVector)) 
        actionSaveStateVectorAs->setDisabled(true);
      if(autoSave)
        autoSaveTimer->start(autoSaveInterval*60000);
      else
        autoSaveTimer->stop();
    }
  }

  void MainWindow::closeEvent(QCloseEvent *event) {
    if(maybeSave())
      event->accept();
    else
      event->ignore();
  }

  void MainWindow::highlightObject(const string &ID) {
    currentID = ID;
    inlineOpenMBVMW->highlightObject(ID);
  }

  void MainWindow::selectionChanged(const QModelIndex &current, const QModelIndex &previous) {
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    Element *element=dynamic_cast<Element*>(model->getItem(current)->getItemData());

    if(element) {

      EmbeddingTreeModel *emodel = static_cast<EmbeddingTreeModel*>(embeddingList->model());
      vector<Element*> parents = element->getParents();
      QModelIndex index = emodel->index(0,0);
      emodel->removeRow(index.row(), index.parent());
      if(parents.size()) {
        index = emodel->createEmbeddingItem(parents[0]);
        for(size_t i=0; i<parents.size()-1; i++)
          index = emodel->createEmbeddingItem(parents[i+1],index);
        emodel->createEmbeddingItem(element,index);
      }
      else
        index = emodel->createEmbeddingItem(element);
      embeddingList->expandAll();
      embeddingList->scrollTo(index.child(emodel->rowCount(index)-1,0),QAbstractItemView::PositionAtTop);

      highlightObject(element->getID());
    }
    else
      highlightObject("");
  }

  void MainWindow::elementListClicked() {
    if(QApplication::mouseButtons()==Qt::RightButton) {
      QModelIndex index = elementList->selectionModel()->currentIndex();
      TreeItemData *itemData = static_cast<ElementTreeModel*>(elementList->model())->getItem(index)->getItemData();
      if(itemData && index.column()==0) {
        QMenu *menu = itemData->createContextMenu();
        menu->exec(QCursor::pos());
        delete menu;
      } 
    }
  }

  void MainWindow::parameterListClicked() {
    if(QApplication::mouseButtons()==Qt::RightButton) {
      QModelIndex index = embeddingList->selectionModel()->currentIndex();
      if(index.column()==0) {
        Parameter *parameter = dynamic_cast<Parameter*>(static_cast<EmbeddingTreeModel*>(embeddingList->model())->getItem(index)->getItemData());
        if(parameter) {
          QMenu *menu = parameter->createContextMenu();
          menu->exec(QCursor::pos());
          delete menu;
          return;
        }
        Element *element = dynamic_cast<Element*>(static_cast<EmbeddingTreeModel*>(embeddingList->model())->getItem(index)->getItemData());
        if(element) {
          QMenu *menu = element->createEmbeddingContextMenu();
          menu->exec(QCursor::pos());
          delete menu;
        }
      } 
    }
  }

  void MainWindow::newProject(bool ask) {
    if(maybeSave()) {
      setProjectChanged(false);
      mbsDir = QDir::current();
      actionOpenMBV->setDisabled(true);
      actionH5plotserie->setDisabled(true);
      actionEigenanalysis->setDisabled(true);
      actionSaveDataAs->setDisabled(true);
      actionSaveMBSimH5DataAs->setDisabled(true);
      actionSaveOpenMBVDataAs->setDisabled(true);
      actionSaveStateVectorAs->setDisabled(true);
      actionSaveEigenanalysisAs->setDisabled(true);

      EmbeddingTreeModel *pmodel = static_cast<EmbeddingTreeModel*>(embeddingList->model());
      QModelIndex index = pmodel->index(0,0);
      pmodel->removeRows(index.row(), pmodel->rowCount(QModelIndex()), index.parent());

      ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
      index = model->index(0,0);
      if(model->rowCount(index))
        delete model->getItem(index)->getItemData();
      model->removeRow(index.row(), index.parent());
      DynamicSystemSolver *dss = new DynamicSystemSolver("MBS",0);
      model->createGroupItem(dss,QModelIndex());

      elementList->selectionModel()->setCurrentIndex(model->index(0,0), QItemSelectionModel::ClearAndSelect);
      solverView->setSolver(new DOPRI5Integrator);
      actionSaveProject->setDisabled(true);
      fileProject="";
      mbsimxml(1);
      setWindowTitle("MBS.mbsimprj.xml[*]");
    }
  }

  void MainWindow::loadProject(const QString &file) {
    if(not(file.isEmpty())) {
      setProjectChanged(false);
      mbsDir = QFileInfo(file).absolutePath();
      QDir::setCurrent(QFileInfo(file).absolutePath());
      fileProject=QDir::current().relativeFilePath(file);
      setCurrentProjectFile(file);
      MBSimObjectFactory::initialize();
      shared_ptr<xercesc::DOMDocument> doc;
      std::string message;
      try { 
        doc=MainWindow::parser->parse(file.toStdString());
      }
      catch(const std::exception &ex) {
        message = ex.what();
      }
      catch(...) {
        message = "Unknown exception.";
      }
      if(not(message.empty())) {
        QMessageBox::StandardButton ret = QMessageBox::warning(this, tr("Application"), QString("Model file not valid: ")+QString::fromStdString(message), QMessageBox::Ignore | QMessageBox::Cancel);
        if(ret == QMessageBox::Ignore) {
          shared_ptr<DOMParser> parser=DOMParser::create(false);
          doc=parser->parse(file.toStdString());
        }
        else {
          newProject(false); 
          return;
        }
      }
      DOMElement *ele0=doc->getDocumentElement();
      //setWindowTitle(QString::fromStdString(E(ele0)->getAttribute("name")));
      setWindowTitle(fileProject+"[*]");

      evalSelect.initializeUsingXML(ele0);

      DOMElement *ele1 = ele0->getFirstElementChild();
      if(evalSelect.isActive()) ele1 = ele1->getNextElementSibling();

      DynamicSystemSolver *dss=Embed<DynamicSystemSolver>::createAndInit(ele1,0);
      dss->initialize();

      EmbeddingTreeModel *pmodel = static_cast<EmbeddingTreeModel*>(embeddingList->model());
      QModelIndex index = pmodel->index(0,0);
      pmodel->removeRows(index.row(), pmodel->rowCount(QModelIndex()), index.parent());
        
      ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
      index = model->index(0,0);
      if(model->rowCount(index))
        delete model->getItem(index)->getItemData();
      model->removeRow(index.row(), index.parent());
      model->createGroupItem(dss);

      elementList->selectionModel()->setCurrentIndex(model->index(0,0), QItemSelectionModel::ClearAndSelect);

      ele1 = ele1->getNextElementSibling();

      Solver *solver;
      if(E(ele1)->getTagName()==PV%"Embed") {
        DOMElement *ele2 = 0;
        if(E(ele1)->hasAttribute("href"))
          solver=Solver::readXMLFile(E(ele1)->getAttribute("href"));
        else {
          ele2 = ele1->getFirstElementChild();
          solver=ObjectFactory::getInstance()->createSolver(ele2);
        }
        solver->initializeUsingXMLEmbed(ele1);
        if(ele2)
          solver->initializeUsingXML(ele2);
      } else {
        solver=ObjectFactory::getInstance()->createSolver(ele1);
        solver->initializeUsingXML(ele1);
      }
      solverView->setSolver(solver);

      actionSaveProject->setDisabled(false);

      mbsimxml(1);
    }
  }

  void MainWindow::loadProject() {
    if(maybeSave()) {
      QString file=QFileDialog::getOpenFileName(0, "XML project files", ".", "XML files (*.mbsimprj.xml)");
      if(file!="")
        loadProject(file);
    }
  }

  bool MainWindow::saveProjectAs() {
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = model->index(0,0);
    QString file=QFileDialog::getSaveFileName(0, "XML project files", QString("./")+QString::fromStdString(model->getItem(index)->getItemData()->getName())+".mbsimprj.xml", "XML files (*.mbsimprj.xml)");
    if(not(file.isEmpty())) {
      file = (file.length()>13 and file.right(13)==".mbsimprj.xml")?file:file+".mbsimprj.xml";
      mbsDir = QFileInfo(file).absolutePath();
      QDir::setCurrent(QFileInfo(file).absolutePath());
      fileProject=QDir::current().relativeFilePath(file);
      setCurrentProjectFile(file);
      setWindowTitle(fileProject+"[*]");
      actionSaveProject->setDisabled(false);
      return saveProject();
    }
    return false;
  }

  DOMElement* MainWindow::writeProject(shared_ptr<xercesc::DOMDocument> &doc) {
    string message;
    try {
      DOMElement *ele0=D(doc)->createElement(MBSIMXML%"MBSimProject");
      doc->insertBefore(ele0, NULL);
      E(ele0)->setAttribute("name", "Project");

      evalSelect.writeXMLFile(ele0);

      ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
      QModelIndex index = model->index(0,0);
      DynamicSystemSolver *dss = static_cast<DynamicSystemSolver*>(model->getItem(index)->getItemData());

      Embed<DynamicSystemSolver>::writeXML(dss,ele0);
      Solver *solver = solverView->getSolver();
      if(not(absolutePath) and solver->isEmbedded())
        solver->writeXMLFileEmbed(ele0);
      else
        solver->writeXMLFile(ele0);

      return ele0;
    }
    catch(const std::exception &ex) {
      message = ex.what();
    }
    catch(const DOMException &ex) {
      message = "DOM exception: " + X()%ex.getMessage();
    }
    catch(...) {
      message = "Unknown exception.";
    }
    cout << message << endl;
    return 0;
  }

  bool MainWindow::saveProject(const QString &fileName) {
    setProjectChanged(false);
    shared_ptr<xercesc::DOMDocument> doc=MainWindow::parser->createDocument();
    DOMElement *ele0=writeProject(doc);
    if(ele0) {
      DOMParser::serialize(doc.get(), fileName.isEmpty()?fileProject.toStdString():fileName.toStdString());
      return true;
    }
    return false;
  }

  void MainWindow::selectIntegrator() {
    QMenu *menu = solverView->createContextMenu();
    menu->exec(QCursor::pos());
    delete menu;
  }

  void MainWindow::selectDOPRI5Integrator() {
    setProjectChanged(true);
    solverView->setSolver(0);
  }

  void MainWindow::selectRADAU5Integrator() {
    setProjectChanged(true);
    solverView->setSolver(1);
  }

  void MainWindow::selectLSODEIntegrator() {
    setProjectChanged(true);
    solverView->setSolver(2);
  }

  void MainWindow::selectLSODARIntegrator() {
    setProjectChanged(true);
    solverView->setSolver(3);
  }

  void MainWindow::selectTimeSteppingIntegrator() {
    setProjectChanged(true);
    solverView->setSolver(4);
  }

  void MainWindow::selectEulerExplicitIntegrator() {
    setProjectChanged(true);
    solverView->setSolver(5);
  }

  void MainWindow::selectRKSuiteIntegrator() {
    setProjectChanged(true);
    solverView->setSolver(6);
  }

  void MainWindow::selectEigenanalyser() {
    setProjectChanged(true);
    solverView->setSolver(7);
  }

  // update model parameters including additional paramters from paramList
  void MainWindow::updateParameters(Element *element) {
    shared_ptr<xercesc::DOMDocument> doc=MainWindow::parser->createDocument();
    vector<Element*> parents = element->getParents();
    Parameters param;
    for(size_t i=0; i<parents.size(); i++)
      param.addParameters(parents[i]->getParameters());
    param.addParameters(element->getParameters());
    string counterName = element->getCounterName();
    ScalarParameter *counter = NULL;
    if(not(counterName.empty())) {
      counter = new ScalarParameter(counterName,"1");
      param.addParameter(counter);
    }
    param.writeXMLFile(doc.get());
    if(counter) delete counter;

    DOMElement *root = doc->getDocumentElement();
    if(!E(root)->getFirstProcessingInstructionChildNamed("OriginalFilename")) {
      DOMProcessingInstruction *filenamePI=doc->createProcessingInstruction(X()%"OriginalFilename",
          X()%"MBS.mbsimparam.xml");
      root->insertBefore(filenamePI, root->getFirstChild());
    }

    string message;
    try {
      D(doc)->validate();
      // create a new empty evaluator
      // Note: do not use "eval.reset(); eval=..." or something like this here since this will possibly deinit
      // static part of the evaluator and then reinit these and will be time consuming.
      eval=Eval::createEvaluator(evalSelect.isActive()?static_cast<TextProperty*>(evalSelect.getProperty())->getText():"octave", &dependencies);

      // add parameter
      eval->addParamSet(doc->getDocumentElement());
    }
    catch(const std::exception &error) {
      message = string("An exception occurred in updateParameters: ") + error.what();
    }
    catch(...) {
      message = "An unknown exception occurred in updateParameters.";
    }
    cout << message << endl;
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
        saveEigenanalysis(dir+"/MBS.eigenanalysis.mat");
        if(saveFinalStateVector)
          saveStateVector(dir+"/statevector.asc");
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
    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/out0.mbsim.h5",file);
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
    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/out0.ombv.xml",file);
  }

  void MainWindow::saveOpenMBVH5Data(const QString &file) {
    if(QFile::exists(file))
      QFile::remove(file);
    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/out0.ombv.h5",file);
  }

  void MainWindow::saveStateVectorAs() {
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = model->index(0,0);
    QString file=QFileDialog::getSaveFileName(0, "Export state vector file", "./statevector.asc", "ASCII files (*.asc)");
    if(file!="") {
      saveStateVector(file);
    }
  }

  void MainWindow::saveStateVector(const QString &file) {
    if(QFile::exists(file))
      QFile::remove(file);
    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/statevector.asc",file);
  }

  void MainWindow::saveEigenanalysisAs() {
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = model->index(0,0);
    QString file=QFileDialog::getSaveFileName(0, "Export eigenanalysis file", QString("./")+QString::fromStdString(model->getItem(index)->getItemData()->getName())+".eigenanalysis.mat", "mat files (*.eigenanalysis.mat)");
    if(file!="") {
      saveEigenanalysis(file);
    }
  }

  void MainWindow::saveEigenanalysis(const QString &file) {
    if(QFile::exists(file))
      QFile::remove(file);
    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/out0.eigenanalysis.mat",file);
  }

  void MainWindow::mbsimxml(int task) {
    absolutePath = true;
    QModelIndex index = elementList->model()->index(0,0);
    DynamicSystemSolver *dss=dynamic_cast<DynamicSystemSolver*>(static_cast<ElementTreeModel*>(elementList->model())->getItem(index)->getItemData());
    Solver *solver=solverView->getSolver();
    if(!dss || !solver)
      return;

    QString sTask = QString::number(task); 
    string saveName=dss->getName();
    dss->setName("out"+sTask.toStdString());
    QString projectFile=QString::fromStdString(uniqueTempDir.generic_string())+"/in"+sTask+".mbsimprj.xml";

    currentTask = task;

    if(task==1 and OpenMBVGUI::MainWindow::getInstance()->getObjectList()->invisibleRootItem()->childCount())
      static_cast<OpenMBVGUI::Group*>(OpenMBVGUI::MainWindow::getInstance()->getObjectList()->invisibleRootItem()->child(0))->unloadFileSlot();

    shared_ptr<xercesc::DOMDocument> doc=MainWindow::parser->createDocument();
    DOMElement *ele0=writeProject(doc);
    dss->setName(saveName);
    absolutePath = false;

    if(ele0) {
      mbsimThread->setDocument(doc);
      mbsimThread->setProjectFile(projectFile);
      mbsimThread->setEvaluator(evalSelect.isActive()?static_cast<TextProperty*>(evalSelect.getProperty())->getText():"octave");
      mbsimThread->start();
    }
  }

  void MainWindow::preprocessFinished(int result) {
    if(result==0) {
      QString uniqueTempDir_ = QString::fromStdString(uniqueTempDir.generic_string());
      QString projectFile=uniqueTempDir_+"/in"+QString::number(currentTask)+".mbsimprj.xml";
      QStringList arg;
      if(currentTask==1)
        arg.append("--stopafterfirststep");
      else if(saveFinalStateVector)
        arg.append("--savefinalstatevector");
      arg.append(projectFile);
      mbsim->getProcess()->setWorkingDirectory(uniqueTempDir_);
      mbsim->clearOutputAndStart((MBXMLUtils::getInstallPath()/"bin"/"mbsimflatxml").string().c_str(), arg);
    }
    else
      mbsim->preprocessFailed(mbsimThread->getErrorText());
  }

  void MainWindow::refresh() {
    mbsimxml(1);
  }

  void MainWindow::simulate() {
    mbsimxml(0);
    actionSaveDataAs->setDisabled(false);
    actionSaveMBSimH5DataAs->setDisabled(false);
    actionSaveOpenMBVDataAs->setDisabled(false);
    if(saveFinalStateVector)
      actionSaveStateVectorAs->setDisabled(false);
    if(solverView->getSolverNumber()==7)
      actionSaveEigenanalysisAs->setDisabled(false);
    actionSimulate->setDisabled(true);
    actionOpenMBV->setDisabled(true);
    actionH5plotserie->setDisabled(true);
    actionEigenanalysis->setDisabled(true);
    actionRefresh->setDisabled(true);
    statusBar()->showMessage(tr("Simulating"));
  }

  void MainWindow::openmbv() {
    QString name = QString::fromStdString(uniqueTempDir.generic_string()+"/out0.ombv.xml");
    if(QFile::exists(name)) {
      QStringList arg;
      arg.append("--autoreload");
      arg.append(name);
      QProcess::startDetached((MBXMLUtils::getInstallPath()/"bin"/"openmbv").string().c_str(), arg);
    }
  }

  void MainWindow::h5plotserie() {
    QString name = QString::fromStdString(uniqueTempDir.generic_string()+"/out0.mbsim.h5");
    if(QFile::exists(name)) {
      QStringList arg;
      arg.append(name);
      QProcess::startDetached((MBXMLUtils::getInstallPath()/"bin"/"h5plotserie").string().c_str(), arg);
    }
  }

  void MainWindow::eigenanalysis() {
    QString name = QString::fromStdString(uniqueTempDir.generic_string()+"/out0.eigenanalysis.mat");
    QFile file(name);
    QPlainTextEdit *edit = new QPlainTextEdit;
    edit->setWindowTitle("Eigenanalysis");
    edit->setReadOnly(true);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
      edit->appendPlainText("Eigenanalysis not yet performed!");
    else {
      QTextStream in(&file);
      while (!in.atEnd()) {
        QString line = in.readLine();
        edit->appendPlainText(line);
      }
    }
    edit->show();
  }

  void MainWindow::selectElement(string ID) {
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    map<string, QModelIndex>::iterator it=model->idEleMap.find(ID);
    if(it!=model->idEleMap.end())
      elementList->selectionModel()->setCurrentIndex(it->second,QItemSelectionModel::ClearAndSelect);
  }

  void MainWindow::help() {
    QMessageBox::information(this, "MBSim GUI - GUI Help", 
        "<h1>GUI Help</h1>"
        "tbd"
        );
  }

  void MainWindow::xmlHelp(const string &url) {
    if(!helpDialog) {
      helpDialog=new QDialog();
      QGridLayout *layout=new QGridLayout(helpDialog);
      helpDialog->setLayout(layout);
      QPushButton *home=new QPushButton("Home",helpDialog);
      connect(home, SIGNAL(clicked()), this, SLOT(xmlHelp()));
      layout->addWidget(home,0,0);
      QPushButton *helpBackward=new QPushButton("Backward",helpDialog);
      layout->addWidget(helpBackward,0,1);
      QPushButton *helpForward=new QPushButton("Forward",helpDialog);
      layout->addWidget(helpForward,0,2);
      helpViewer=new QWebView(helpDialog);
      layout->addWidget(helpViewer,1,0,1,3);
      connect(helpForward, SIGNAL(clicked()), helpViewer, SLOT(forward()));
      connect(helpBackward, SIGNAL(clicked()), helpViewer, SLOT(back()));
      helpDialog->setWindowTitle("MBSimXML - Main MBSim XML Documentation");
      helpDialog->resize(700,500);
    }
    if(url.empty())
      helpViewer->load(QUrl((MBXMLUtils::getInstallPath()/"share"/"mbxmlutils"/"doc"/
        "http___www_mbsim-env_de_MBSimXML"/"mbsimxml.html").string().c_str()));
    else
      helpViewer->load(QUrl(url.c_str()));
    helpDialog->show();
    helpDialog->raise();
    helpDialog->activateWindow();
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
        //"  <li>'mbsimflatxml - tbd <tt>http://www.mbsim-env.de/</tt> (Licence: Qwt/LGPL)</li>"
        //"  <li>'OpenMBV - tbd <tt>http://www.mbsim-env.de/</tt> (License: LGPL)</li>"
        //"  <li>'h5plotserie - tbd <tt>http://www.mbsim-env.de/</tt> (License: NCSA-HDF)</li>"
        //"  <li>...</li>"
        "</ul>"
        );
  }

  void MainWindow::removeElement() {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
    Element *element = static_cast<Element*>(model->getItem(index)->getItemData());
    element->getParent()->removeElement(element);
    model->removeRow(index.row(), index.parent());
    mbsimxml(1);
  }

  void MainWindow::saveElementAs() {
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
    QString file=QFileDialog::getSaveFileName(0, "XML model files", QString("./")+QString::fromStdString(model->getItem(index)->getItemData()->getName())+".xml", "XML files (*.xml)");
    if(file!="")
      static_cast<Element*>(model->getItem(index)->getItemData())->writeXMLFileEmbed(file.toStdString());
  }

  void MainWindow::addFrame(Frame *frame) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
    QModelIndex containerIndex = index.data().toString()=="frames"?index:index.child(0,0); 
    frame->setName(frame->getName()+toStr(model->getItem(containerIndex)->getID()));
    frame->getParent()->addFrame(frame);
    model->createFrameItem(frame,containerIndex);
    QModelIndex currentIndex = containerIndex.child(model->rowCount(containerIndex)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementList->openEditor();
  }

  void MainWindow::addContour(Contour *contour) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
    QModelIndex containerIndex = (index.row()==0)?index.child(1,0):index;
    contour->setName(contour->getName()+toStr(model->getItem(containerIndex)->getID()));
    contour->getParent()->addContour(contour);
    model->createContourItem(contour,containerIndex);
    QModelIndex currentIndex = containerIndex.child(model->rowCount(containerIndex)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementList->openEditor();
  }

  void MainWindow::addGroup(Group *group) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
    QModelIndex containerIndex = (index.row()==0)?index.child(2,0):index;
    group->setName(group->getName()+toStr(model->getItem(containerIndex)->getID()));
    group->getParent()->addGroup(group);
    model->createGroupItem(group,containerIndex);
    QModelIndex currentIndex = containerIndex.child(model->rowCount(containerIndex)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementList->openEditor();
  }

  void MainWindow::addObject(Object *object) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
    QModelIndex containerIndex = (index.row()==0)?index.child(3,0):index;
    object->setName(object->getName()+toStr(model->getItem(containerIndex)->getID()));
    object->getParent()->addObject(object);
    model->createObjectItem(object,containerIndex);
    QModelIndex currentIndex = containerIndex.child(model->rowCount(containerIndex)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementList->openEditor();
  }

  void MainWindow::addLink(Link *link) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
    QModelIndex containerIndex = (index.row()==0)?index.child(4,0):index;
    link->setName(link->getName()+toStr(model->getItem(containerIndex)->getID()));
    link->getParent()->addLink(link);
    model->createLinkItem(link,containerIndex);
    QModelIndex currentIndex = containerIndex.child(model->rowCount(containerIndex)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementList->openEditor();
  }

  void MainWindow::addConstraint(Constraint *constraint) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
    QModelIndex containerIndex = (index.row()==0)?index.child(5,0):index;
    constraint->setName(constraint->getName()+toStr(model->getItem(containerIndex)->getID()));
    constraint->getParent()->addConstraint(constraint);
    model->createConstraintItem(constraint,containerIndex);
    QModelIndex currentIndex = containerIndex.child(model->rowCount(containerIndex)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementList->openEditor();
  }

  void MainWindow::addObserver(Observer *observer) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
    QModelIndex containerIndex = (index.row()==0)?index.child(5,0):index;
    observer->setName(observer->getName()+toStr(model->getItem(containerIndex)->getID()));
    observer->getParent()->addObserver(observer);
    model->createObserverItem(observer,containerIndex);
    QModelIndex currentIndex = containerIndex.child(model->rowCount(containerIndex)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementList->openEditor();
  }

  void MainWindow::addParameter(Parameter *parameter) {
    setProjectChanged(true);
    QModelIndex index = embeddingList->selectionModel()->currentIndex();
    EmbeddingTreeModel *model = static_cast<EmbeddingTreeModel*>(embeddingList->model());
    Element *element = static_cast<Element*>(model->getItem(index)->getItemData());
    if(parameter->getName()!="import")
      parameter->setName(parameter->getName()+toStr(model->getItem(index)->getID()));
    QModelIndex newIndex = model->createParameterItem(parameter,index);
    element->addParameter(parameter);
    embeddingList->selectionModel()->setCurrentIndex(newIndex, QItemSelectionModel::ClearAndSelect);
    embeddingList->openEditor();
  }

  void MainWindow::removeParameter() {
    setProjectChanged(true);
    QModelIndex index = embeddingList->selectionModel()->currentIndex().parent();
    EmbeddingTreeModel *model = static_cast<EmbeddingTreeModel*>(embeddingList->model());
    Element *element=static_cast<Element*>(model->getItem(index)->getItemData());
    EmbeddingTreeModel *pmodel = static_cast<EmbeddingTreeModel*>(embeddingList->model());
    QModelIndex pindex = embeddingList->selectionModel()->currentIndex();
    Parameter *parameter=static_cast<Parameter*>(pmodel->getItem(pindex)->getItemData());
    element->removeParameter(parameter);
    pmodel->removeRow(pindex.row(), pindex.parent());
  }

  void MainWindow::dragEnterEvent(QDragEnterEvent *event) {
    if (event->mimeData()->hasUrls()) {
      event->acceptProposedAction();
    }
  }

  void MainWindow::dropEvent(QDropEvent *event) {
    for (int i = 0; i < event->mimeData()->urls().size(); i++) {
      QString path = event->mimeData()->urls()[i].toLocalFile().toLocal8Bit().data();
      if(path.endsWith("mbsimprj.xml")) {
        QFile Fout(path);
        if (Fout.exists())
          loadProject(Fout.fileName());
      }
    }
  }

  void MainWindow::openRecentProjectFile() {
    if(maybeSave()) {
      QAction *action = qobject_cast<QAction *>(sender());
      if (action)
        loadProject(action->data().toString());
    }
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
      recentProjectFileActs[i]->setText(text);
      recentProjectFileActs[i]->setData(files[i]);
      recentProjectFileActs[i]->setVisible(true);
    }
    for (int j = numRecentFiles; j < maxRecentFiles; ++j)
      recentProjectFileActs[j]->setVisible(false);

    //separatorAct->setVisible(numRecentFiles > 0);
  }

  void MainWindow::projectSettings() {
    ProjectPropertyDialog *editor = new ProjectPropertyDialog(this);
    editor->setAttribute(Qt::WA_DeleteOnClose);
    editor->toWidget();
    editor->show();
    connect(editor,SIGNAL(apply()),this,SLOT(applySettings()));
    connect(editor,SIGNAL(finished(int)),this,SLOT(settingsFinished(int)));
  }

  void MainWindow::settingsFinished(int result) {
    if(result != 0) {
      setProjectChanged(true);
      mbsimxml(1);
    }
  }

  void MainWindow::applySettings() {
    setProjectChanged(true);
    mbsimxml(1);
  }

}
