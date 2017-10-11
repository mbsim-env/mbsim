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
#include "file_editor.h"
#include "utils.h"
#include "basicitemdata.h"
#include <openmbv/mainwindow.h>
#include <utime.h>
#include <QtGui>
#include <mbxmlutils/eval.h>
#include <mbxmlutils/preprocess.h>
#include <mbxmlutilshelper/getinstallpath.h>
#include <mbxmlutilshelper/dom.h>
#include <xercesc/dom/DOMProcessingInstruction.hpp>
#include <xercesc/dom/DOMException.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMLSSerializer.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;
namespace bfs=boost::filesystem;

namespace MBSimGUI {

  DOMImplementation *impl=DOMImplementation::getImplementation();
  DOMLSParser *parser=impl->createLSParser(DOMImplementation::MODE_SYNCHRONOUS, 0);
  DOMLSSerializer *serializer=impl->createLSSerializer();

  bool currentTask;
  bool absolutePath = false;
  QDir mbsDir;
  unordered_map<string,pair<DOMDocument*,int> > hrefMap;

  MainWindow *mw;

  vector<boost::filesystem::path> dependencies;

  QDialog *MainWindow::helpDialog = NULL;
  QWebView *MainWindow::helpViewer = NULL;

  MainWindow::MainWindow(QStringList &arg) : inlineOpenMBVMW(0), autoSave(false), autoExport(false), saveFinalStateVector(false), autoSaveInterval(5), maxUndo(10), autoExportDir("./"), allowUndo(true), doc(NULL), elementBuffer(NULL,false), parameterBuffer(NULL,false) {
    // use html output of MBXMLUtils
    static string HTMLOUTPUT="MBXMLUTILS_HTMLOUTPUT=1";
    putenv(const_cast<char*>(HTMLOUTPUT.c_str()));

    serializer->getDomConfig()->setParameter(X()%"format-pretty-print", true);

//    evalSelect.setProperty(new TextProperty("octave", PV%"evaluator", false));
    
    mw = this;

#if _WIN32
    uniqueTempDir=bfs::unique_path(bfs::temp_directory_path()/"mbsimgui_%%%%-%%%%-%%%%-%%%%");
#else
    if(bfs::is_directory("/dev/shm"))
      uniqueTempDir=bfs::unique_path("/dev/shm/mbsimgui_%%%%-%%%%-%%%%-%%%%");
    else
      uniqueTempDir=bfs::unique_path(bfs::temp_directory_path()/"mbsimgui_%%%%-%%%%-%%%%-%%%%");
#endif
    bfs::create_directories(uniqueTempDir);

    mbsim = new Process(this);

    QString program = (MBXMLUtils::getInstallPath()/"bin"/"mbsimxml").string().c_str();
    QStringList arguments;
    arguments << "--onlyListSchemas";
    QProcess process;
    process.start(program,arguments);
    process.waitForFinished(-1);
    QStringList line=QString(process.readAllStandardOutput().data()).split("\n");
    set<bfs::path> schemas;
    for(int i=0; i<line.size(); ++i)
      if(!line.at(i).isEmpty())
        schemas.insert(line.at(i).toStdString());

    mbsimThread = new MBSimThread(this);
    parser=DOMParser::create(schemas);
    mbsimThread->setParser(parser);
    connect(mbsimThread, SIGNAL(resultReady(int)), this, SLOT(preprocessFinished(int)));

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
    action->setShortcut(QKeySequence::Quit);
    action->setStatusTip(tr("Exit the application"));

    for (int i=0; i<maxRecentFiles; i++) {
      recentProjectFileActs[i] = new QAction(this);
      recentProjectFileActs[i]->setVisible(false);
      connect(recentProjectFileActs[i], SIGNAL(triggered()), this, SLOT(openRecentProjectFile()));
    }
    QMenu *menu=new QMenu("Project", menuBar());
    action = menu->addAction("New", this, SLOT(newProject()));
    action->setShortcut(QKeySequence::New);
    action = menu->addAction("Load", this, SLOT(loadProject()));
    action->setShortcut(QKeySequence::Open);
    action = menu->addAction("Save as", this, SLOT(saveProjectAs()));
    action->setShortcut(QKeySequence::SaveAs);
    actionSaveProject = menu->addAction("Save", this, SLOT(saveProject()));
    actionSaveProject->setShortcut(QKeySequence::Save);
   //ProjMenu->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_DirOpenIcon)),"Load", this, SLOT(loadProj()));
    actionSaveProject->setDisabled(true);
    menu->addSeparator();
    //separatorAct = menu->addSeparator();
    for (int i = 0; i < maxRecentFiles; ++i)
      menu->addAction(recentProjectFileActs[i]);
    updateRecentProjectFileActions();
    menu->addSeparator();
    menu->addSeparator();
    action = menu->addAction("Settings", this, SLOT(projectSettings()));
    menuBar()->addMenu(menu);

    menu=new QMenu("Edit", menuBar());
    action = menu->addAction("Edit", elementList, SLOT(openEditor()));
    action->setShortcut(QKeySequence("Ctrl+E"));
    menu->addSeparator();
    action = menu->addAction("Undo", this, SLOT(undo()));
    action->setShortcut(QKeySequence::Undo);
    action = menu->addAction("Redo", this, SLOT(redo()));
    action->setShortcut(QKeySequence::Redo);
    menu->addSeparator();
    action = menu->addAction("Copy", this, SLOT(copy()));
    action->setShortcut(QKeySequence::Copy);
    action = menu->addAction("Cut", this, SLOT(cut()));
    action->setShortcut(QKeySequence::Cut);
    action = menu->addAction("Paste", this, SLOT(paste()));
    action->setShortcut(QKeySequence::Paste);
    menu->addSeparator();
    action = menu->addAction("Remove", this, SLOT(remove()));
    action->setShortcut(QKeySequence::Delete);
    menu->addSeparator();
    action = menu->addAction("Move up", this, SLOT(moveUp()));
    action->setShortcut(QKeySequence("Ctrl+Up"));
    action = menu->addAction("Move down", this, SLOT(moveDown()));
    action->setShortcut(QKeySequence("Ctrl+Down"));
    menuBar()->addMenu(menu);

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
    actionSimulate->setStatusTip(tr("Simulate the multibody system"));
    connect(actionSimulate,SIGNAL(triggered()),this,SLOT(simulate()));
    toolBar->addAction(actionSimulate);
    QAction *actionInterrupt = toolBar->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_MediaStop)),"Interrupt simulation");
    connect(actionInterrupt,SIGNAL(triggered()),mbsim,SLOT(interrupt()));
    toolBar->addAction(actionInterrupt);
    actionRefresh = toolBar->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_BrowserReload)),"Refresh 3D view");
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
    QAction *actionDebug = toolBar->addAction(Utils::QIconCached(QString::fromStdString((MBXMLUtils::getInstallPath()/"share"/"mbsimgui"/"icons"/"debug.svg").string())),"Debug model");
    connect(actionDebug,SIGNAL(triggered()),this,SLOT(debug()));
    toolBar->addAction(actionDebug);

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
    connect(elementList->selectionModel(),SIGNAL(currentChanged(const QModelIndex&,const QModelIndex&)), this, SLOT(selectionChanged(const QModelIndex&)));

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

    if(arg.contains("--maximized"))
      showMaximized();

    QString fileProject;
    QRegExp filterProject(".+\\.mbsimprj\\.xml");
    QDir dir;
    dir.setFilter(QDir::Files);
    for(auto it=arg.begin(); it!=arg.end(); ++it) {
      if((*it)[0]=='-') continue;
      dir.setPath(*it);
      if(dir.exists()) {
        QStringList file=dir.entryList();
        for(int j=0; j<file.size(); j++) {
          if(fileProject.isEmpty() and filterProject.exactMatch(file[j]))
            fileProject = dir.path()+"/"+file[j];
        }
        continue;
      }
      if(QFile::exists(*it)) {
        if(fileProject.isEmpty() and filterProject.exactMatch(*it))
          fileProject = *it;
        continue;
      }
    }
    if(fileProject.size())
      loadProject(QDir::current().absoluteFilePath(fileProject));
    else
      newProject(false);

    setAcceptDrops(true);

    autoSaveTimer = new QTimer(this);
    connect(autoSaveTimer, SIGNAL(timeout()), this, SLOT(autoSaveProject()));
    autoSaveTimer->start(autoSaveInterval*60000);

    setWindowIcon(Utils::QIconCached(QString::fromStdString((MBXMLUtils::getInstallPath()/"share"/"mbsimgui"/"icons"/"mbsimgui.svg").string())));
  }

  void MainWindow::autoSaveProject() {
    saveProject("./.MBS.mbsimprj.xml",true,false);
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
    // use nothrow boost::filesystem functions to avoid exceptions in this dtor
    boost::system::error_code ec;
    bfs::remove_all(uniqueTempDir, ec);
    bfs::remove("./.MBS.mbsimprj.xml", ec);
  }

  void MainWindow::setProjectChanged(bool changed) { 
    setWindowModified(changed);
    if(changed) {
      undos.push_back(static_cast<xercesc::DOMDocument*>(doc->cloneNode(true)));
      if(undos.size() > maxUndo)
        undos.pop_front();
      redos.clear();
    }
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
    menu.setMaxUndo(maxUndo);
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
      maxUndo = menu.getMaxUndo();
    }
  }

  void MainWindow::highlightObject(const QString &ID) {
    currentID = ID;
    inlineOpenMBVMW->highlightObject(ID.toStdString());
  }

  void MainWindow::selectionChanged(const QModelIndex &current) {
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    Element *element=dynamic_cast<Element*>(model->getItem(current)->getItemData());
    if(element) {
      EmbeddingTreeModel *emodel = static_cast<EmbeddingTreeModel*>(embeddingList->model());
      vector<EmbedItemData*> parents = element->getParents();
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
    else if(QApplication::mouseButtons()==Qt::LeftButton)
      selectionChanged(elementList->selectionModel()->currentIndex());
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
        else {
          EmbedItemData *item = static_cast<EmbedItemData*>(static_cast<EmbeddingTreeModel*>(embeddingList->model())->getItem(index)->getItemData());
          if(item) {
            QMenu *menu = item->createEmbeddingContextMenu();
            menu->exec(QCursor::pos());
            delete menu;
          }
        }
      } 
    }
  }

  void MainWindow::solverViewClicked() {
    EmbeddingTreeModel *emodel = static_cast<EmbeddingTreeModel*>(embeddingList->model());
    Solver *solver = solverView->getSolver();
    vector<EmbedItemData*> parents = solver->getParents();
    QModelIndex index = emodel->index(0,0);
    emodel->removeRow(index.row(), index.parent());
    if(parents.size()) {
      index = emodel->createEmbeddingItem(parents[0]);
      for(size_t i=0; i<parents.size()-1; i++)
        index = emodel->createEmbeddingItem(parents[i+1],index);
      emodel->createEmbeddingItem(solver,index);
    }
    else
      index = emodel->createEmbeddingItem(solver);
    embeddingList->expandAll();
    embeddingList->scrollTo(index.child(emodel->rowCount(index)-1,0),QAbstractItemView::PositionAtTop);
  }

  void MainWindow::newProject(bool ask) {
    if(maybeSave()) {
      hrefMap.clear();
      undos.clear();
      elementBuffer.first = NULL;
      parameterBuffer.first = NULL;
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
      DynamicSystemSolver *dss = new DynamicSystemSolver("MBS");
      model->createGroupItem(dss,QModelIndex());

      doc = impl->createDocument();

      DOMElement *ele0=D(doc)->createElement(MBSIMXML%"MBSimProject");
      doc->insertBefore(ele0, NULL);
      E(ele0)->setAttribute("name", "Project");
      dss->createXMLElement(ele0);

      elementList->selectionModel()->setCurrentIndex(model->index(0,0), QItemSelectionModel::ClearAndSelect);
      Integrator *integrator = new DOPRI5Integrator;
      integrator->createXMLElement(ele0);
      solverView->setSolver(integrator);
      actionSaveProject->setDisabled(true);
      fileProject="";
      mbsimxml(1);
      setWindowTitle("MBS.mbsimprj.xml[*]");
    }
  }

  void MainWindow::loadProject(const QString &file) {
    if(not(file.isEmpty())) {
      hrefMap.clear();
      undos.clear();
      elementBuffer.first = NULL;
      parameterBuffer.first = NULL;
      setProjectChanged(false);
      mbsDir = QFileInfo(file).absolutePath();
      QDir::setCurrent(QFileInfo(file).absolutePath());
      fileProject=QDir::current().relativeFilePath(file);
      setCurrentProjectFile(file);
      MBSimObjectFactory::initialize();
      std::string message;
      try { 
        doc = MBSimGUI::parser->parseURI(X()%file.toStdString());
      }
      catch(const std::exception &ex) {
        message = ex.what();
      }
      catch(...) {
        message = "Unknown exception.";
      }
      setWindowTitle(fileProject+"[*]");
//      evalSelect.initializeUsingXML(ele0);
      rebuildTree();
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
    QString file=QFileDialog::getSaveFileName(0, "XML project files", QString("./")+model->getItem(index)->getItemData()->getName()+".mbsimprj.xml", "XML files (*.mbsimprj.xml)");
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

  bool MainWindow::saveProject(const QString &fileName, bool processDocument, bool modifyStatus) {
    try {
      serializer->writeToURI(doc, X()%(fileName.isEmpty()?fileProject.toStdString():fileName.toStdString()));
//      cout << hrefMap.size() << endl;
      for(auto it=hrefMap.begin(); it!=hrefMap.end(); it++) {
//        std::cout << "save " << it->first << ":" << it->second.first << endl;
        serializer->writeToURI(it->second.first, X()%(it->first));
      }
      if(modifyStatus) setProjectChanged(false);
      return true;
    }
    catch(const std::exception &ex) {
      cout << ex.what() << endl;
    }
    catch(const DOMException &ex) {
      cout << "DOM exception: " + X()%ex.getMessage() << endl;
    }
    catch(...) {
      cout << "Unknown exception." << endl;
    }
    return false;
  }

  void MainWindow::selectSolver(int i) {
    setProjectChanged(true);
    DOMElement *element = solverView->getSolver()->getXMLElement();
    DOMNode *parent = element->getParentNode();
    DOMNode *ps = element->getPreviousSibling();
    if(ps and X()%ps->getNodeName()=="#text")
      parent->removeChild(ps);
    parent->removeChild(element);
    solverView->setSolver(i);
    element = solverView->getSolver()->getXMLElement();
    if(element) {
      for(int i=0; i<solverView->getSolver()->getNumberOfParameters(); i++)
        solverView->getSolver()->removeParameter(solverView->getSolver()->getParameter(i));
      DOMElement *ele = static_cast<DOMElement*>(doc->importNode(element,true));
      parent->insertBefore(ele,NULL);
      solverView->getSolver()->initializeUsingXML(ele);
    }
    else
      solverView->getSolver()->createXMLElement(parent);
    std::vector<Parameter*> param;
    DOMElement *ele = MBXMLUtils::E(static_cast<DOMElement*>(parent))->getFirstElementChildNamed(MBXMLUtils::PV%"Parameter");
    if(ele) param = Parameter::initializeParametersUsingXML(ele);
    for(size_t i=0; i<param.size(); i++)
      solverView->getSolver()->addParameter(param[i]);
    solverViewClicked();
  }

  // update model parameters including additional paramters from paramList
  void MainWindow::updateParameters(EmbedItemData *item, bool exceptLatestParameter) {
    shared_ptr<xercesc::DOMDocument> doc=MainWindow::parser->createDocument();
    DOMElement *ele0 = D(doc)->createElement(PV%"Parameter");
    doc->insertBefore(ele0,NULL);

    DOMElement *parent = static_cast<DOMElement*>(item->getXMLElement()->getParentNode());
    QString counterName = (E(parent)->getTagName()==PV%"Embed")?QString::fromStdString(E(parent)->getAttribute("counterName")):"";
    if(not(counterName.isEmpty())) {
      DOMElement *ele1=D(doc)->createElement(PV%"scalarParameter");
      E(ele1)->setAttribute("name", counterName.toStdString());
      DOMText *text = doc->createTextNode(X()%"1");
      ele1->insertBefore(text,NULL);
      ele0->insertBefore(ele1,NULL);
    }

    vector<EmbedItemData*> parents = item->getParents();
    for(size_t i=0; i<parents.size(); i++) {
      for(size_t j=0; j<parents[i]->getNumberOfParameters(); j++) {
        DOMNode *node = doc->importNode(parents[i]->getParameter(j)->getXMLElement(),true);
        ele0->insertBefore(node,NULL);
      }
    }
    for(size_t j=0; j<item->getNumberOfParameters()-exceptLatestParameter; j++) {
      DOMNode *node = doc->importNode(item->getParameter(j)->getXMLElement(),true);
      ele0->insertBefore(node,NULL);
    }

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
      eval=Eval::createEvaluator("octave", &dependencies);

      // add parameter
      eval->addParamSet(doc->getDocumentElement());
    }
    catch(const std::exception &error) {
      message = string("An exception occurred in updateParameters: ") + error.what();
      cout << message << endl;
    }
    catch(...) {
      message = "An unknown exception occurred in updateParameters.";
      cout << message << endl;
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
        saveEigenanalysis(dir+"/MBS.eigenanalysis.mat");
        if(saveFinalStateVector)
          saveStateVector(dir+"/statevector.asc");
      }
    }
  }

  void MainWindow::saveMBSimH5DataAs() {
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = model->index(0,0);
    QString file=QFileDialog::getSaveFileName(0, "Export MBSim H5 file", QString("./")+model->getItem(index)->getItemData()->getName()+".mbsim.h5", "H5 files (*.mbsim.h5)");
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
    QString file=QFileDialog::getSaveFileName(0, "Export eigenanalysis file", QString("./")+model->getItem(index)->getItemData()->getName()+".eigenanalysis.mat", "mat files (*.eigenanalysis.mat)");
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
//    shared_ptr<xercesc::DOMDocument> doc(static_cast<xercesc::DOMDocument*>(this->doc->cloneNode(true)));
    shared_ptr<xercesc::DOMDocument> doc=MainWindow::parser->createDocument();
    DOMNode *newDocElement = doc->importNode(this->doc->getDocumentElement(), true);
    doc->insertBefore(newDocElement, NULL);
    DOMElement* ele0 = doc->getDocumentElement()->getFirstElementChild();
    DOMElement* ele = (E(ele0)->getTagName()==PV%"Embed")?E(ele0)->getFirstElementChildNamed(MBSIM%"DynamicSystemSolver"):ele0;
    dss->processFileID(ele);

    E(ele)->setAttribute("name","out"+sTask.toStdString());;
    QString projectFile=QString::fromStdString(uniqueTempDir.generic_string())+"/in"+sTask+".mbsimprj.flat.xml";

    currentTask = task;

    if(task==1) {
      if(OpenMBVGUI::MainWindow::getInstance()->getObjectList()->invisibleRootItem()->childCount())
        static_cast<OpenMBVGUI::Group*>(OpenMBVGUI::MainWindow::getInstance()->getObjectList()->invisibleRootItem()->child(0))->unloadFileSlot();
      DOMElement *ele1 = D(doc)->createElement( MBSIM%"plotFeatureRecursive" );
      E(ele1)->setAttribute("feature","-plotRecursive");
      ele->insertBefore( ele1, ele->getFirstElementChild() );
    }

    ele0 = ele0->getNextElementSibling();
    ele = (E(ele0)->getTagName()==PV%"Embed")?ele0->getLastElementChild():ele0;
    solverView->getSolver()->processFileID(ele);

    absolutePath = false;

//    if(ele0) {
      mbsimThread->setDocument(doc);
      mbsimThread->setProjectFile(projectFile);
      mbsimThread->setEvaluator("octave");
      mbsimThread->start();
//    }
  }

  void MainWindow::preprocessFinished(int result) {
    if(result==0) {
      QString uniqueTempDir_ = QString::fromStdString(uniqueTempDir.generic_string());
      QString projectFile=uniqueTempDir_+"/in"+QString::number(currentTask)+".mbsimprj.flat.xml";
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

  void MainWindow::refresh() {
    mbsimxml(1);
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
    FileEditor *edit = new FileEditor("Eigenanalysis",name,1,"Eigenanalysis not yet performed!",this);
    edit->setModal(true);
    edit->show();
  }

  void MainWindow::debug() {
    QString uniqueTempDir_ = QString::fromStdString(uniqueTempDir.generic_string());
    QString projectFile=uniqueTempDir_+"/in"+QString::number(currentTask)+".mbsimprj.xml";
    saveProject(projectFile,false,false);
    QStringList arg;
    arg.append("--stopafterfirststep");
    arg.append(projectFile);
    mbsim->getProcess()->setWorkingDirectory(uniqueTempDir_);
    mbsim->clearOutputAndStart((MBXMLUtils::getInstallPath()/"bin"/"mbsimxml").string().c_str(), arg);
  }

  void MainWindow::selectElement(string ID) {
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    map<QString, QModelIndex>::iterator it=model->idEleMap.find(QString::fromStdString(ID));
    if(it!=model->idEleMap.end())
      elementList->selectionModel()->setCurrentIndex(it->second,QItemSelectionModel::ClearAndSelect);
  }

  void MainWindow::help() {
    QMessageBox::information(this, "MBSim GUI - GUI Help", 
        "<h1>GUI Help</h1>"
        "tbd"
        );
  }

  void MainWindow::xmlHelp(const QString &url) {
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
    if(url.isEmpty())
      helpViewer->load(QUrl(((MBXMLUtils::getInstallPath()/"share"/"mbxmlutils"/"doc").string()+"/http___www_mbsim-env_de_MBSim/index.html").c_str()));
    else
      helpViewer->load(QUrl(url));
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
        //"  <li>'mbsimflatxml - tbd <tt>https://www.mbsim-env.de/</tt> (Licence: Qwt/LGPL)</li>"
        //"  <li>'OpenMBV - tbd <tt>https://www.mbsim-env.de/</tt> (License: LGPL)</li>"
        //"  <li>'h5plotserie - tbd <tt>https://www.mbsim-env.de/</tt> (License: NCSA-HDF)</li>"
        //"  <li>...</li>"
        "</ul>"
        );
  }

  void MainWindow::rebuildTree() {
    DOMElement *ele0=doc->getDocumentElement();
    DOMElement *ele1 = ele0->getFirstElementChild();

    DynamicSystemSolver *dss=Embed<DynamicSystemSolver>::createAndInit(ele1);

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

    Solver *solver=Embed<Solver>::createAndInit(ele1);
    solverView->setSolver(solver);
  }

  void MainWindow::undo() {
    if(allowUndo and undos.size()) {
      elementBuffer.first = NULL;
      parameterBuffer.first = NULL;
      setWindowModified(true);
      redos.push_back(doc);
      doc = undos.back();
      undos.pop_back();
      rebuildTree();
      mbsimxml(1);
    }
  }

  void MainWindow::redo() {
    if(redos.size()) {
      undos.push_back(doc);
      doc = redos.back();
      redos.pop_back();
      rebuildTree();
      mbsimxml(1);
    }
  }

  void MainWindow::removeElement() {
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
    Element *element = static_cast<Element*>(model->getItem(index)->getItemData());
    if((not dynamic_cast<DynamicSystemSolver*>(element)) and (not dynamic_cast<InternalFrame*>(element))) {
      setProjectChanged(true);
      if(element == elementBuffer.first)
        elementBuffer.first = NULL;
      element->removeXMLElement();
      element->getParent()->removeElement(element);
      model->removeRow(index.row(), index.parent());
      mbsimxml(1);
    }
  }

  void MainWindow::removeParameter() {
    setProjectChanged(true);
    EmbeddingTreeModel *model = static_cast<EmbeddingTreeModel*>(embeddingList->model());
    QModelIndex index = embeddingList->selectionModel()->currentIndex();
    Parameter *parameter = static_cast<Parameter*>(model->getItem(index)->getItemData());
    if(parameter == parameterBuffer.first)
      parameterBuffer.first = NULL;
    DOMNode *ps = parameter->getXMLElement()->getPreviousSibling();
    if(ps and X()%ps->getNodeName()=="#text")
      parameter->getXMLElement()->getParentNode()->removeChild(ps);
    parameter->getXMLElement()->getParentNode()->removeChild(parameter->getXMLElement());
    parameter->getParent()->removeParameter(parameter);
    model->removeRow(index.row(), index.parent());
  }

  void MainWindow::remove() {
    if(elementList->hasFocus())
      removeElement();
    else if(embeddingList->hasFocus())
      removeParameter();
  }

  void MainWindow::copy(bool cut) {
    if(elementList->hasFocus())
      copyElement(cut);
    else if(embeddingList->hasFocus())
      copyParameter(cut);
  }

  void MainWindow::paste() {
    if(elementList->hasFocus()) {
      ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
      QModelIndex index = elementList->selectionModel()->currentIndex();
      ContainerItemData *item = dynamic_cast<FrameItemData*>(model->getItem(index)->getItemData());
      if(item and dynamic_cast<Frame*>(getElementBuffer().first)) {
        loadFrame(item->getElement(),getElementBuffer().first);
        return;
      }
      item = dynamic_cast<ContourItemData*>(model->getItem(index)->getItemData());
      if(item and dynamic_cast<Contour*>(getElementBuffer().first)) {
        loadContour(item->getElement(),getElementBuffer().first);
        return;
      }
      item = dynamic_cast<GroupItemData*>(model->getItem(index)->getItemData());
      if(item and dynamic_cast<Group*>(getElementBuffer().first)) {
        loadGroup(item->getElement(),getElementBuffer().first);
        return;
      }
      item = dynamic_cast<ObjectItemData*>(model->getItem(index)->getItemData());
      if(item and dynamic_cast<Object*>(getElementBuffer().first)) {
        loadObject(item->getElement(),getElementBuffer().first);
        return;
      }
      item = dynamic_cast<LinkItemData*>(model->getItem(index)->getItemData());
      if(item and dynamic_cast<Link*>(getElementBuffer().first)) {
        loadLink(item->getElement(),getElementBuffer().first);
        return;
      }
      item = dynamic_cast<ConstraintItemData*>(model->getItem(index)->getItemData());
      if(item and dynamic_cast<Constraint*>(getElementBuffer().first)) {
        loadConstraint(item->getElement(),getElementBuffer().first);
        return;
      }
      item = dynamic_cast<ObserverItemData*>(model->getItem(index)->getItemData());
      if(item and dynamic_cast<Observer*>(getElementBuffer().first)) {
        loadObserver(item->getElement(),getElementBuffer().first);
        return;
      }
    }
    else if(embeddingList->hasFocus()) {
      EmbeddingTreeModel *model = static_cast<EmbeddingTreeModel*>(embeddingList->model());
      QModelIndex index = embeddingList->selectionModel()->currentIndex();
      EmbedItemData *item = dynamic_cast<EmbedItemData*>(model->getItem(index)->getItemData());
      if(item)
        loadParameter(item,getParameterBuffer().first);
    }
  }

  void MainWindow::move(bool up) {
    if(elementList->hasFocus()) {
      ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
      QModelIndex index = elementList->selectionModel()->currentIndex();
      Frame *frame = dynamic_cast<Frame*>(model->getItem(index)->getItemData());
      if(frame and (not dynamic_cast<InternalFrame*>(frame)) and (up?(frame->getParent()->getIndexOfFrame(frame)>1):(frame->getParent()->getIndexOfFrame(frame)<frame->getParent()->getNumberOfFrames()-1))) {
        moveFrame(up);
        return;
      }
      Contour *contour = dynamic_cast<Contour*>(model->getItem(index)->getItemData());
      if(contour and (up?(contour->getParent()->getIndexOfContour(contour)>0):(contour->getParent()->getIndexOfContour(contour)<contour->getParent()->getNumberOfContours()-1))) {
        moveContour(up);
        return;
      }
      Group *group = dynamic_cast<Group*>(model->getItem(index)->getItemData());
      if(group and (up?(group->getParent()->getIndexOfGroup(group)>0):(group->getParent()->getIndexOfGroup(group)<group->getParent()->getNumberOfGroups()-1))) {
        moveGroup(up);
        return;
      }
      Object *object = dynamic_cast<Object*>(model->getItem(index)->getItemData());
      if(object and (up?(object->getParent()->getIndexOfObject(object)>0):(object->getParent()->getIndexOfObject(object)<object->getParent()->getNumberOfObjects()-1))) {
        moveObject(up);
        return;
      }
      Link *link = dynamic_cast<Link*>(model->getItem(index)->getItemData());
      if(link and (up?(link->getParent()->getIndexOfLink(link)>0):(link->getParent()->getIndexOfLink(link)<link->getParent()->getNumberOfLinks()-1))) {
        moveLink(up);
        return;
      }
      Constraint *constraint = dynamic_cast<Constraint*>(model->getItem(index)->getItemData());
      if(constraint and (up?(constraint->getParent()->getIndexOfConstraint(constraint)>0):(constraint->getParent()->getIndexOfConstraint(constraint)<constraint->getParent()->getNumberOfConstraints()-1))) {
        moveConstraint(up);
        return;
      }
      Observer *observer = dynamic_cast<Observer*>(model->getItem(index)->getItemData());
      if(observer and (up?(observer->getParent()->getIndexOfObserver(observer)>0):(observer->getParent()->getIndexOfObserver(observer)<observer->getParent()->getNumberOfObservers()-1))) {
        moveObserver(up);
        return;
      }
    }
    else if(embeddingList->hasFocus()) {
      EmbeddingTreeModel *model = static_cast<EmbeddingTreeModel*>(embeddingList->model());
      QModelIndex index = embeddingList->selectionModel()->currentIndex();
      Parameter *parameter = dynamic_cast<Parameter*>(model->getItem(index)->getItemData());
      if(parameter and (up?(parameter->getParent()->getIndexOfParameter(parameter)>0):(parameter->getParent()->getIndexOfParameter(parameter)<parameter->getParent()->getNumberOfParameters()-1)))
        moveParameter(up);
    }
  }

  void MainWindow::copyElement(bool cut) {
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
    Element *element = static_cast<Element*>(model->getItem(index)->getItemData());
    if((not dynamic_cast<DynamicSystemSolver*>(element)) and (not dynamic_cast<InternalFrame*>(element)))
      elementBuffer = make_pair(element,cut);
  }

  void MainWindow::copyParameter(bool cut) {
    EmbeddingTreeModel *model = static_cast<EmbeddingTreeModel*>(embeddingList->model());
    QModelIndex index = embeddingList->selectionModel()->currentIndex();
    Parameter *parameter = static_cast<Parameter*>(model->getItem(index)->getItemData());
    parameterBuffer = make_pair(parameter,cut);
  }

  void MainWindow::moveParameter(bool up) {
    setProjectChanged(true);
    EmbeddingTreeModel *model = static_cast<EmbeddingTreeModel*>(embeddingList->model());
    QModelIndex index = embeddingList->selectionModel()->currentIndex();
    Parameter *parameter = static_cast<Parameter*>(model->getItem(index)->getItemData());
    int i = parameter->getParent()->getIndexOfParameter(parameter);
    int j = up?i-1:i+1;
    DOMElement *tmp = up?parameter->getXMLElement()->getPreviousElementSibling():parameter->getXMLElement()->getNextElementSibling()->getNextElementSibling();
    DOMNode *parent = parameter->getXMLElement()->getParentNode();
    DOMNode *ps = parameter->getXMLElement()->getPreviousSibling();
    if(ps and X()%ps->getNodeName()=="#text")
      parent->removeChild(ps);
    DOMNode *ele = parent->removeChild(parameter->getXMLElement());
    parent->insertBefore(ele,tmp);
    parameter->getParent()->setParameter(parameter->getParent()->getParameter(j),i);
    parameter->getParent()->setParameter(parameter,j);
    model->removeRows(0,parameter->getParent()->getNumberOfParameters(),index.parent());
    for(int i=0; i<parameter->getParent()->getNumberOfParameters(); i++)
      model->createParameterItem(parameter->getParent()->getParameter(i),index.parent());
    embeddingList->setCurrentIndex(index.sibling(j,0));
  }

  void MainWindow::moveFrame(bool up) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
    Frame *frame = static_cast<Frame*>(model->getItem(index)->getItemData());
    int i = frame->getParent()->getIndexOfFrame(frame);
    int j = up?i-1:i+1;
    DOMElement *ele = static_cast<DOMElement*>(X()%frame->getXMLElement()->getParentNode()->getNodeName()=="Embed"?frame->getXMLElement()->getParentNode():frame->getXMLElement());
    DOMElement *tmp = up?ele->getPreviousElementSibling():ele->getNextElementSibling()->getNextElementSibling();
    DOMNode *parent = ele->getParentNode();
    DOMNode *ps = ele->getPreviousSibling();
    if(ps and X()%ps->getNodeName()=="#text")
      parent->removeChild(ps);
    parent->removeChild(ele);
    parent->insertBefore(ele,tmp);
    frame->getParent()->setFrame(frame->getParent()->getFrame(j),i);
    frame->getParent()->setFrame(frame,j);
    model->removeRows(0,frame->getParent()->getNumberOfFrames(),index.parent());
    for(int i=0; i<frame->getParent()->getNumberOfFrames(); i++)
      model->createFrameItem(frame->getParent()->getFrame(i),index.parent());
    elementList->setCurrentIndex(index.sibling(j,0));
  }

  void MainWindow::moveContour(bool up) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
    Contour *contour = static_cast<Contour*>(model->getItem(index)->getItemData());
    int i = contour->getParent()->getIndexOfContour(contour);
    int j = up?i-1:i+1;
    DOMElement *ele = static_cast<DOMElement*>(X()%contour->getXMLElement()->getParentNode()->getNodeName()=="Embed"?contour->getXMLElement()->getParentNode():contour->getXMLElement());
    DOMElement *tmp = up?ele->getPreviousElementSibling():ele->getNextElementSibling()->getNextElementSibling();
    DOMNode *parent = ele->getParentNode();
    DOMNode *ps = ele->getPreviousSibling();
    if(ps and X()%ps->getNodeName()=="#text")
      parent->removeChild(ps);
    parent->removeChild(ele);
    parent->insertBefore(ele,tmp);
    contour->getParent()->setContour(contour->getParent()->getContour(j),i);
    contour->getParent()->setContour(contour,j);
    model->removeRows(0,contour->getParent()->getNumberOfContours(),index.parent());
    for(int i=0; i<contour->getParent()->getNumberOfContours(); i++)
      model->createContourItem(contour->getParent()->getContour(i),index.parent());
    elementList->setCurrentIndex(index.sibling(j,0));
  }

  void MainWindow::moveGroup(bool up) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
    Group *group = static_cast<Group*>(model->getItem(index)->getItemData());
    int i = group->getParent()->getIndexOfGroup(group);
    int j = up?i-1:i+1;
    DOMElement *ele = static_cast<DOMElement*>(X()%group->getXMLElement()->getParentNode()->getNodeName()=="Embed"?group->getXMLElement()->getParentNode():group->getXMLElement());
    DOMElement *tmp = up?ele->getPreviousElementSibling():ele->getNextElementSibling()->getNextElementSibling();
    DOMNode *parent = ele->getParentNode();
    DOMNode *ps = ele->getPreviousSibling();
    if(ps and X()%ps->getNodeName()=="#text")
      parent->removeChild(ps);
    parent->removeChild(ele);
    parent->insertBefore(ele,tmp);
    group->getParent()->setGroup(group->getParent()->getGroup(j),i);
    group->getParent()->setGroup(group,j);
    model->removeRows(0,group->getParent()->getNumberOfGroups(),index.parent());
    for(int i=0; i<group->getParent()->getNumberOfGroups(); i++)
      model->createGroupItem(group->getParent()->getGroup(i),index.parent(),false);
    elementList->setCurrentIndex(index.sibling(j,0));
  }

  void MainWindow::moveObject(bool up) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
    Object *object = static_cast<Object*>(model->getItem(index)->getItemData());
    int i = object->getParent()->getIndexOfObject(object);
    int j = up?i-1:i+1;
    DOMElement *ele = static_cast<DOMElement*>(X()%object->getXMLElement()->getParentNode()->getNodeName()=="Embed"?object->getXMLElement()->getParentNode():object->getXMLElement());
    DOMElement *tmp = up?ele->getPreviousElementSibling():ele->getNextElementSibling()->getNextElementSibling();
    DOMNode *parent = ele->getParentNode();
    DOMNode *ps = ele->getPreviousSibling();
    if(ps and X()%ps->getNodeName()=="#text")
      parent->removeChild(ps);
    parent->removeChild(ele);
    parent->insertBefore(ele,tmp);
    object->getParent()->setObject(object->getParent()->getObject(j),i);
    object->getParent()->setObject(object,j);
    model->removeRows(0,object->getParent()->getNumberOfObjects(),index.parent());
    for(int i=0; i<object->getParent()->getNumberOfObjects(); i++)
      model->createObjectItem(object->getParent()->getObject(i),index.parent(),false);
    elementList->setCurrentIndex(index.sibling(j,0));
  }

  void MainWindow::moveLink(bool up) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
    Link *link = static_cast<Link*>(model->getItem(index)->getItemData());
    int i = link->getParent()->getIndexOfLink(link);
    int j = up?i-1:i+1;
    DOMElement *ele = static_cast<DOMElement*>(X()%link->getXMLElement()->getParentNode()->getNodeName()=="Embed"?link->getXMLElement()->getParentNode():link->getXMLElement());
    DOMElement *tmp = up?ele->getPreviousElementSibling():ele->getNextElementSibling()->getNextElementSibling();
    DOMNode *parent = ele->getParentNode();
    DOMNode *ps = ele->getPreviousSibling();
    if(ps and X()%ps->getNodeName()=="#text")
      parent->removeChild(ps);
    parent->removeChild(ele);
    parent->insertBefore(ele,tmp);
    link->getParent()->setLink(link->getParent()->getLink(j),i);
    link->getParent()->setLink(link,j);
    model->removeRows(0,link->getParent()->getNumberOfLinks(),index.parent());
    for(int i=0; i<link->getParent()->getNumberOfLinks(); i++)
      model->createLinkItem(link->getParent()->getLink(i),index.parent());
    elementList->setCurrentIndex(index.sibling(j,0));
  }

  void MainWindow::moveConstraint(bool up) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
    Constraint *constraint = static_cast<Constraint*>(model->getItem(index)->getItemData());
    int i = constraint->getParent()->getIndexOfConstraint(constraint);
    int j = up?i-1:i+1;
    DOMElement *ele = static_cast<DOMElement*>(X()%constraint->getXMLElement()->getParentNode()->getNodeName()=="Embed"?constraint->getXMLElement()->getParentNode():constraint->getXMLElement());
    DOMElement *tmp = up?ele->getPreviousElementSibling():ele->getNextElementSibling()->getNextElementSibling();
    DOMNode *parent = ele->getParentNode();
    DOMNode *ps = ele->getPreviousSibling();
    if(ps and X()%ps->getNodeName()=="#text")
      parent->removeChild(ps);
    parent->removeChild(ele);
    parent->insertBefore(ele,tmp);
    constraint->getParent()->setConstraint(constraint->getParent()->getConstraint(j),i);
    constraint->getParent()->setConstraint(constraint,j);
    model->removeRows(0,constraint->getParent()->getNumberOfConstraints(),index.parent());
    for(int i=0; i<constraint->getParent()->getNumberOfConstraints(); i++)
      model->createConstraintItem(constraint->getParent()->getConstraint(i),index.parent());
    elementList->setCurrentIndex(index.sibling(j,0));
  }

  void MainWindow::moveObserver(bool up) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
    Observer *observer = static_cast<Observer*>(model->getItem(index)->getItemData());
    int i = observer->getParent()->getIndexOfObserver(observer);
    int j = up?i-1:i+1;
    DOMElement *ele = static_cast<DOMElement*>(X()%observer->getXMLElement()->getParentNode()->getNodeName()=="Embed"?observer->getXMLElement()->getParentNode():observer->getXMLElement());
    DOMElement *tmp = up?ele->getPreviousElementSibling():ele->getNextElementSibling()->getNextElementSibling();
    DOMNode *parent = ele->getParentNode();
    DOMNode *ps = ele->getPreviousSibling();
    if(ps and X()%ps->getNodeName()=="#text")
      parent->removeChild(ps);
    parent->removeChild(ele);
    parent->insertBefore(ele,tmp);
    observer->getParent()->setObserver(observer->getParent()->getObserver(j),i);
    observer->getParent()->setObserver(observer,j);
    model->removeRows(0,observer->getParent()->getNumberOfObservers(),index.parent());
    for(int i=0; i<observer->getParent()->getNumberOfObservers(); i++)
      model->createObserverItem(observer->getParent()->getObserver(i),index.parent());
    elementList->setCurrentIndex(index.sibling(j,0));
  }

  void MainWindow::saveElementAs() {
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
    Element *element = static_cast<Element*>(model->getItem(index)->getItemData());
    QString file=QFileDialog::getSaveFileName(0, "XML model files", QString("./")+element->getName()+".mbsim.xml", "XML files (*.xml)");
    if(not file.isEmpty()) {
      xercesc::DOMDocument *edoc = impl->createDocument();
      DOMNode *node = edoc->importNode(X()%element->getXMLElement()->getParentNode()->getNodeName()=="Embed"?element->getXMLElement()->getParentNode():element->getXMLElement(),true);
      edoc->insertBefore(node,NULL);
      serializer->writeToURI(edoc, X()%file.toStdString());
    }
  }

  void MainWindow::addFrame(Frame *frame, Element *parent) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
//    frame->setName(frame->getName()+toQStr(model->getItem(index)->getID()));
    parent->addFrame(frame);
    frame->createXMLElement(parent->getXMLFrames());
    model->createFrameItem(frame,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementList->openEditor();
  }

  void MainWindow::addContour(Contour *contour, Element *parent) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
//    contour->setName(contour->getName()+toQStr(model->getItem(index)->getID()));
    parent->addContour(contour);
    contour->createXMLElement(parent->getXMLContours());
    model->createContourItem(contour,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementList->openEditor();
  }

  void MainWindow::addGroup(Group *group, Element *parent) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
//    group->setName(group->getName()+toQStr(model->getItem(index)->getID()));
    parent->addGroup(group);
    group->createXMLElement(parent->getXMLGroups());
    model->createGroupItem(group,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementList->openEditor();
  }

  void MainWindow::addObject(Object *object, Element *parent) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
//    object->setName(object->getName()+toQStr(model->getItem(index)->getID()));
    parent->addObject(object);
    object->createXMLElement(parent->getXMLObjects());
    model->createObjectItem(object,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementList->openEditor();
  }

  void MainWindow::addLink(Link *link, Element *parent) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
//    link->setName(link->getName()+toQStr(model->getItem(index)->getID()));
    parent->addLink(link);
    link->createXMLElement(parent->getXMLLinks());
    model->createLinkItem(link,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementList->openEditor();
  }

  void MainWindow::addConstraint(Constraint *constraint, Element *parent) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
//    constraint->setName(constraint->getName()+toQStr(model->getItem(index)->getID()));
    parent->addConstraint(constraint);
    constraint->createXMLElement(parent->getXMLConstraints());
    model->createConstraintItem(constraint,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementList->openEditor();
  }

  void MainWindow::addObserver(Observer *observer, Element *parent) {
    setProjectChanged(true);
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    QModelIndex index = elementList->selectionModel()->currentIndex();
//    observer->setName(observer->getName()+toQStr(model->getItem(index)->getID()));
    parent->addObserver(observer);
    observer->createXMLElement(parent->getXMLObservers());
    model->createObserverItem(observer,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementList->openEditor();
  }

  void MainWindow::addParameter(Parameter *parameter, EmbedItemData *parent) {
    setProjectChanged(true);
    QModelIndex index = embeddingList->selectionModel()->currentIndex();
    EmbeddingTreeModel *model = static_cast<EmbeddingTreeModel*>(embeddingList->model());
    parent->addParameter(parameter);
    parameter->createXMLElement(parent->getXMLElement());
//    if(parameter->getName()!="import")
//      parameter->setName(parameter->getName()+toQStr(model->getItem(index)->getID()));
    QModelIndex newIndex = model->createParameterItem(parameter,index);
    embeddingList->selectionModel()->setCurrentIndex(newIndex, QItemSelectionModel::ClearAndSelect);
    embeddingList->openEditor();
  }

  void MainWindow::loadParameter(EmbedItemData *parent, Parameter *param) {
    setProjectChanged(true);
    DOMElement *ele = static_cast<DOMElement*>(doc->importNode(param->getXMLElement(),true));
    EmbeddingTreeModel *model = static_cast<EmbeddingTreeModel*>(embeddingList->model());
    if(parameterBuffer.second) {
      parameterBuffer.first = NULL;
      DOMNode *ps = param->getXMLElement()->getPreviousSibling();
      if(ps and X()%ps->getNodeName()=="#text")
        param->getXMLElement()->getParentNode()->removeChild(ps);
      param->getXMLElement()->getParentNode()->removeChild(param->getXMLElement());
      param->getParent()->removeParameter(param);
      QModelIndex index = model->findItem(param,model->index(0,0));
      if(index.isValid())
        model->removeRow(index.row(), index.parent());
    }
    QModelIndex index = embeddingList->selectionModel()->currentIndex();
    Parameter::insertXMLElement(ele,parent->getXMLElement());
    Parameter *parameter=ObjectFactory::getInstance()->createParameter(ele);
    parameter->initializeUsingXML(ele);
    parent->addParameter(parameter);
    QModelIndex newIndex = model->createParameterItem(parameter,index);
    embeddingList->selectionModel()->setCurrentIndex(newIndex, QItemSelectionModel::ClearAndSelect);
  }

  void MainWindow::loadFrame(Element *parent, Element *element) {
    setProjectChanged(true);
    DOMElement *ele = NULL;
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    if(element) {
      ele = static_cast<DOMElement*>(doc->importNode(X()%element->getXMLElement()->getParentNode()->getNodeName()=="Embed"?element->getXMLElement()->getParentNode():element->getXMLElement(),true));
      if(elementBuffer.second) {
        elementBuffer.first = NULL;
        element->removeXMLElement();
        element->getParent()->removeElement(element);
        QModelIndex index = model->findItem(element,model->index(0,0));
        if(index.isValid())
          model->removeRow(index.row(), index.parent());
      }
    }
    else {
      QString file=QFileDialog::getOpenFileName(0, "XML frame files", ".", "XML files (*.xml)");
      if(not file.isEmpty()) {
        xercesc::DOMDocument *doc = MBSimGUI::parser->parseURI(X()%file.toStdString());
        ele = static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
      }
      else
        return;
    }
    QModelIndex index = elementList->selectionModel()->currentIndex();
    parent->getXMLFrames()->insertBefore(ele, NULL);
    Frame *frame = Embed<Frame>::createAndInit(ele);
    if(frame) parent->addFrame(frame);
    model->createFrameItem(frame,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    mbsimxml(1);
  }

  void MainWindow::loadContour(Element *parent, Element *element) {
    setProjectChanged(true);
    DOMElement *ele = NULL;
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    if(element) {
      ele = static_cast<DOMElement*>(doc->importNode(X()%element->getXMLElement()->getParentNode()->getNodeName()=="Embed"?element->getXMLElement()->getParentNode():element->getXMLElement(),true));
      if(elementBuffer.second) {
        elementBuffer.first = NULL;
        element->removeXMLElement();
        element->getParent()->removeElement(element);
        QModelIndex index = model->findItem(element,model->index(0,0));
        if(index.isValid())
          model->removeRow(index.row(), index.parent());
      }
    }
    else {
      QString file=QFileDialog::getOpenFileName(0, "XML contour files", ".", "XML files (*.xml)");
      if(not file.isEmpty()) {
        xercesc::DOMDocument *doc = MBSimGUI::parser->parseURI(X()%file.toStdString());
        ele = static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
      }
      else
        return;
    }
    QModelIndex index = elementList->selectionModel()->currentIndex();
    parent->getXMLContours()->insertBefore(ele, NULL);
    Contour *contour = Embed<Contour>::createAndInit(ele);
    if(contour) parent->addContour(contour);
    model->createContourItem(contour,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    mbsimxml(1);
  }

  void MainWindow::loadGroup(Element *parent, Element *element) {
    setProjectChanged(true);
    DOMElement *ele = NULL;
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    if(element) {
      ele = static_cast<DOMElement*>(doc->importNode(X()%element->getXMLElement()->getParentNode()->getNodeName()=="Embed"?element->getXMLElement()->getParentNode():element->getXMLElement(),true));
      if(elementBuffer.second) {
        elementBuffer.first = NULL;
        element->removeXMLElement();
        element->getParent()->removeElement(element);
        QModelIndex index = model->findItem(element,model->index(0,0));
        if(index.isValid())
          model->removeRow(index.row(), index.parent());
      }
    }
    else {
      QString file=QFileDialog::getOpenFileName(0, "XML group files", ".", "XML files (*.xml)");
      if(not file.isEmpty()) {
        xercesc::DOMDocument *doc = MBSimGUI::parser->parseURI(X()%file.toStdString());
        ele = static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
      }
      else
        return;
    }
    QModelIndex index = elementList->selectionModel()->currentIndex();
    parent->getXMLGroups()->insertBefore(ele, NULL);
    Group *group = Embed<Group>::createAndInit(ele);
    if(group) parent->addGroup(group);
    model->createGroupItem(group,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    mbsimxml(1);
  }

  void MainWindow::loadObject(Element *parent, Element *element) {
    setProjectChanged(true);
    DOMElement *ele = NULL;
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    if(element) {
      ele = static_cast<DOMElement*>(doc->importNode(X()%element->getXMLElement()->getParentNode()->getNodeName()=="Embed"?element->getXMLElement()->getParentNode():element->getXMLElement(),true));
      if(elementBuffer.second) {
        elementBuffer.first = NULL;
        element->removeXMLElement();
        element->getParent()->removeElement(element);
        QModelIndex index = model->findItem(element,model->index(0,0));
        if(index.isValid())
          model->removeRow(index.row(), index.parent());
      }
    }
    else {
      QString file=QFileDialog::getOpenFileName(0, "XML object files", ".", "XML files (*.xml)");
      if(not file.isEmpty()) {
        xercesc::DOMDocument *doc = MBSimGUI::parser->parseURI(X()%file.toStdString());
        ele = static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
      }
      else
        return;
    }
    QModelIndex index = elementList->selectionModel()->currentIndex();
    parent->getXMLObjects()->insertBefore(ele, NULL);
    Object *object = Embed<Object>::createAndInit(ele);
    if(object) parent->addObject(object);
    model->createObjectItem(object,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    mbsimxml(1);
  }

  void MainWindow::loadLink(Element *parent, Element *element) {
    setProjectChanged(true);
    DOMElement *ele = NULL;
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    if(element) {
      ele = static_cast<DOMElement*>(doc->importNode(X()%element->getXMLElement()->getParentNode()->getNodeName()=="Embed"?element->getXMLElement()->getParentNode():element->getXMLElement(),true));
      if(elementBuffer.second) {
        elementBuffer.first = NULL;
        element->removeXMLElement();
        element->getParent()->removeElement(element);
        QModelIndex index = model->findItem(element,model->index(0,0));
        if(index.isValid())
          model->removeRow(index.row(), index.parent());
      }
    }
    else {
      QString file=QFileDialog::getOpenFileName(0, "XML link files", ".", "XML files (*.xml)");
      if(not file.isEmpty()) {
        xercesc::DOMDocument *doc = MBSimGUI::parser->parseURI(X()%file.toStdString());
        ele = static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
      }
      else
        return;
    }
    QModelIndex index = elementList->selectionModel()->currentIndex();
    parent->getXMLLinks()->insertBefore(ele, NULL);
    Link *link = Embed<Link>::createAndInit(ele);
    if(link) parent->addLink(link);
    model->createLinkItem(link,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    mbsimxml(1);
  }

  void MainWindow::loadConstraint(Element *parent, Element *element) {
    setProjectChanged(true);
    DOMElement *ele = NULL;
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    if(element) {
      ele = static_cast<DOMElement*>(doc->importNode(X()%element->getXMLElement()->getParentNode()->getNodeName()=="Embed"?element->getXMLElement()->getParentNode():element->getXMLElement(),true));
      if(elementBuffer.second) {
        elementBuffer.first = NULL;
        element->removeXMLElement();
        element->getParent()->removeElement(element);
        QModelIndex index = model->findItem(element,model->index(0,0));
        if(index.isValid())
          model->removeRow(index.row(), index.parent());
      }
    }
    else {
      QString file=QFileDialog::getOpenFileName(0, "XML constraint files", ".", "XML files (*.xml)");
      if(not file.isEmpty()) {
        xercesc::DOMDocument *doc = MBSimGUI::parser->parseURI(X()%file.toStdString());
        ele = static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
      }
      else
        return;
    }
    QModelIndex index = elementList->selectionModel()->currentIndex();
    parent->getXMLConstraints()->insertBefore(ele, NULL);
    Constraint *constraint = Embed<Constraint>::createAndInit(ele);
    if(constraint) parent->addConstraint(constraint);
    model->createConstraintItem(constraint,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    mbsimxml(1);
  }

  void MainWindow::loadObserver(Element *parent, Element *element) {
    setProjectChanged(true);
    DOMElement *ele = NULL;
    ElementTreeModel *model = static_cast<ElementTreeModel*>(elementList->model());
    if(element) {
      ele = static_cast<DOMElement*>(doc->importNode(X()%element->getXMLElement()->getParentNode()->getNodeName()=="Embed"?element->getXMLElement()->getParentNode():element->getXMLElement(),true));
      if(elementBuffer.second) {
        elementBuffer.first = NULL;
        element->removeXMLElement();
        element->getParent()->removeElement(element);
        QModelIndex index = model->findItem(element,model->index(0,0));
        if(index.isValid())
          model->removeRow(index.row(), index.parent());
      }
    }
    else {
      QString file=QFileDialog::getOpenFileName(0, "XML observer files", ".", "XML files (*.xml)");
      if(not file.isEmpty()) {
        xercesc::DOMDocument *doc = MBSimGUI::parser->parseURI(X()%file.toStdString());
        ele = static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
      }
      else
        return;
    }
    QModelIndex index = elementList->selectionModel()->currentIndex();
    parent->getXMLObservers()->insertBefore(ele, NULL);
    Observer *observer = Embed<Observer>::createAndInit(ele);
    if(observer) parent->addObserver(observer);
    model->createObserverItem(observer,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementList->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    mbsimxml(1);
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

  void MainWindow::closeEvent(QCloseEvent *event) {
    if(maybeSave())
      event->accept();
    else
      event->ignore();
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
