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
#include "project_view.h"
#include "echo_view.h"
#include "embed.h"
#include "project.h"
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
#include "dialogs.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;
namespace bfs=boost::filesystem;

namespace MBSimGUI {

  DOMImplementation *impl=DOMImplementation::getImplementation();
  DOMLSParser *parser=impl->createLSParser(DOMImplementation::MODE_SYNCHRONOUS, nullptr);
  DOMLSSerializer *serializer=impl->createLSSerializer();

  bool currentTask;
  QDir mbsDir;

  MainWindow *mw;

  vector<boost::filesystem::path> dependencies;

  WebDialog* MainWindow::xmlHelpDialog = nullptr;

  MainWindow::MainWindow(QStringList &arg) : project(nullptr), inlineOpenMBVMW(nullptr), autoSave(false), autoExport(false), saveFinalStateVector(false), autoSaveInterval(5), maxUndo(10), autoExportDir("./"), allowUndo(true), doc(nullptr), elementBuffer(NULL,false), parameterBuffer(NULL,false) {
    // use html output of MBXMLUtils
    static string HTMLOUTPUT="MBXMLUTILS_HTMLOUTPUT=1";
    putenv(const_cast<char*>(HTMLOUTPUT.c_str()));

    serializer->getDomConfig()->setParameter(X()%"format-pretty-print", true);

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

    QString program = (MBXMLUtils::getInstallPath()/"bin"/"mbsimxml").string().c_str();
    QStringList arguments;
    arguments << "--onlyListSchemas";
    process.start(program,arguments);
    process.waitForFinished(-1);
    QStringList line=QString(process.readAllStandardOutput().data()).split("\n");
    set<bfs::path> schemas;
    for(int i=0; i<line.size(); ++i)
      if(!line.at(i).isEmpty())
        schemas.insert(line.at(i).toStdString());

    parser=DOMParser::create(schemas);

    projectView = new ProjectView;
    elementView = new ElementView;
    embeddingView = new EmbeddingView;
    solverView = new SolverView;
    echoView = new EchoView(this);

    initInlineOpenMBV();

    MBSimObjectFactory::initialize();
    eval=Eval::createEvaluator("octave", &dependencies);

    QMenu *GUIMenu = new QMenu("GUI", menuBar());
    menuBar()->addMenu(GUIMenu);

    QAction *action = GUIMenu->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_DirHomeIcon)),"Workdir", this, SLOT(changeWorkingDir()));
    action->setStatusTip(tr("Change working directory"));

    action = GUIMenu->addAction(QIcon::fromTheme("document-properties"), "Options", this, SLOT(openOptionsMenu()));
    action->setStatusTip(tr("Open options menu"));

    GUIMenu->addSeparator();

    OpenMBVGUI::AbstractViewFilter *elementViewFilter = new OpenMBVGUI::AbstractViewFilter(elementView, 0, 1);
    elementViewFilter->hide();

    OpenMBVGUI::AbstractViewFilter *embeddingViewFilter = new OpenMBVGUI::AbstractViewFilter(embeddingView, 0, -2);
    embeddingViewFilter->hide();

    action = GUIMenu->addAction("Show filter");
    action->setCheckable(true);
    connect(action,SIGNAL(toggled(bool)), elementViewFilter, SLOT(setVisible(bool)));
    connect(action,SIGNAL(toggled(bool)), embeddingViewFilter, SLOT(setVisible(bool)));
    action->setStatusTip(tr("Show filter"));

    GUIMenu->addSeparator();

    action = GUIMenu->addAction(QIcon::fromTheme("application-exit"), "E&xit", this, SLOT(close()));
    action->setShortcut(QKeySequence::Quit);
    action->setStatusTip(tr("Exit the application"));

    for (auto & recentProjectFileAct : recentProjectFileActs) {
      recentProjectFileAct = new QAction(this);
      recentProjectFileAct->setVisible(false);
      connect(recentProjectFileAct, SIGNAL(triggered()), this, SLOT(openRecentProjectFile()));
    }
    QMenu *menu = new QMenu("Project", menuBar());
    action = menu->addAction(QIcon::fromTheme("document-new"), "New", this, SLOT(newProject()));
    action->setShortcut(QKeySequence::New);
    action = menu->addAction(QIcon::fromTheme("document-open"), "Load", this, SLOT(loadProject()));
    action->setShortcut(QKeySequence::Open);
    action = menu->addAction(QIcon::fromTheme("document-save-as"), "Save as", this, SLOT(saveProjectAs()));
    action->setShortcut(QKeySequence::SaveAs);
    actionSaveProject = menu->addAction(QIcon::fromTheme("document-save"), "Save", this, SLOT(saveProject()));
    actionSaveProject->setShortcut(QKeySequence::Save);
    menu->addSeparator();
    for (auto & recentProjectFileAct : recentProjectFileActs)
      menu->addAction(recentProjectFileAct);
    updateRecentProjectFileActions();
    menuBar()->addMenu(menu);

    menu = new QMenu("Edit", menuBar());
    action = menu->addAction(QIcon::fromTheme("document-properties"), "Edit", this, SLOT(edit()));
    action->setShortcut(QKeySequence("Ctrl+E"));
    menu->addSeparator();
    actionUndo = menu->addAction(QIcon::fromTheme("edit-undo"), "Undo", this, SLOT(undo()));
    actionUndo->setShortcut(QKeySequence::Undo);
    actionUndo->setDisabled(true);
    actionRedo = menu->addAction(QIcon::fromTheme("edit-redo"), "Redo", this, SLOT(redo()));
    actionRedo->setShortcut(QKeySequence::Redo);
    actionRedo->setDisabled(true);
    menu->addSeparator();
    action = menu->addAction(QIcon::fromTheme("edit-copy"), "Copy", this, SLOT(copy()));
    action->setShortcut(QKeySequence::Copy);
    action = menu->addAction(QIcon::fromTheme("edit-cut"), "Cut", this, SLOT(cut()));
    action->setShortcut(QKeySequence::Cut);
    action = menu->addAction(QIcon::fromTheme("edit-paste"), "Paste", this, SLOT(paste()));
    action->setShortcut(QKeySequence::Paste);
    menu->addSeparator();
    action = menu->addAction(QIcon::fromTheme("edit-delete"), "Remove", this, SLOT(remove()));
    action->setShortcut(QKeySequence::Delete);
    menu->addSeparator();
    action = menu->addAction(QIcon::fromTheme("go-up"), "Move up", this, SLOT(moveUp()));
    action->setShortcut(QKeySequence("Ctrl+Up"));
    action = menu->addAction(QIcon::fromTheme("go-down"), "Move down", this, SLOT(moveDown()));
    action->setShortcut(QKeySequence("Ctrl+Down"));
    menuBar()->addMenu(menu);

    menu = new QMenu("Export", menuBar());
    actionSaveDataAs = menu->addAction("Export all data", this, SLOT(saveDataAs()));
    actionSaveMBSimH5DataAs = menu->addAction("Export MBSim data file", this, SLOT(saveMBSimH5DataAs()));
    actionSaveOpenMBVDataAs = menu->addAction("Export OpenMBV data", this, SLOT(saveOpenMBVDataAs()));
    actionSaveStateVectorAs = menu->addAction("Export state vector", this, SLOT(saveStateVectorAs()));
    actionSaveEigenanalysisAs = menu->addAction("Export eigenanalysis", this, SLOT(saveEigenanalysisAs()));
    menuBar()->addMenu(menu);

    menuBar()->addSeparator();
    QMenu *helpMenu = new QMenu("Help", menuBar());
    helpMenu->addAction(QIcon::fromTheme("help-contents"), "Contents", this, SLOT(help()));
    helpMenu->addAction(QIcon::fromTheme("help-xml"), "XML Help", this, SLOT(xmlHelp()));
    helpMenu->addAction(QIcon::fromTheme("help-about"), "About", this, SLOT(about()));
    menuBar()->addMenu(helpMenu);

    QToolBar *toolBar = addToolBar("Tasks");
    actionSimulate = toolBar->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_MediaPlay)),"Start simulation");
    actionSimulate->setStatusTip(tr("Simulate the multibody system"));
    connect(actionSimulate,SIGNAL(triggered()),this,SLOT(simulate()));
    toolBar->addAction(actionSimulate);
    QAction *actionInterrupt = toolBar->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_MediaStop)),"Interrupt simulation");
    connect(actionInterrupt,SIGNAL(triggered()),this,SLOT(interrupt()));
    toolBar->addAction(actionInterrupt);
    actionRefresh = toolBar->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_BrowserReload)),"Refresh 3D view");
    connect(actionRefresh,SIGNAL(triggered()),this,SLOT(refresh()));
    toolBar->addAction(actionRefresh);
    actionOpenMBV = toolBar->addAction(Utils::QIconCached(QString::fromStdString((MBXMLUtils::getInstallPath()/"share"/"mbsimgui"/"icons"/"openmbv.svg").string())),"OpenMBV");
    connect(actionOpenMBV,SIGNAL(triggered()),this,SLOT(openmbv()));
    toolBar->addAction(actionOpenMBV);
    actionH5plotserie = toolBar->addAction(Utils::QIconCached(QString::fromStdString((MBXMLUtils::getInstallPath()/"share"/"mbsimgui"/"icons"/"h5plotserie.svg").string())),"H5plotserie");
    connect(actionH5plotserie,SIGNAL(triggered()),this,SLOT(h5plotserie()));
    toolBar->addAction(actionH5plotserie);
    actionEigenanalysis = toolBar->addAction(Utils::QIconCached(QString::fromStdString((MBXMLUtils::getInstallPath()/"share"/"mbsimgui"/"icons"/"eigenanalysis.svg").string())),"Eigenanalysis");
    connect(actionEigenanalysis,SIGNAL(triggered()),this,SLOT(eigenanalysis()));
    toolBar->addAction(actionEigenanalysis);
    actionFrequencyResponse = toolBar->addAction(Utils::QIconCached(QString::fromStdString((MBXMLUtils::getInstallPath()/"share"/"mbsimgui"/"icons"/"frequency_response.svg").string())),"Harmonic response analysis");
    connect(actionFrequencyResponse,SIGNAL(triggered()),this,SLOT(frequencyResponse()));
    actionDebug = toolBar->addAction(Utils::QIconCached(QString::fromStdString((MBXMLUtils::getInstallPath()/"share"/"mbsimgui"/"icons"/"debug.svg").string())),"Debug model");
    connect(actionDebug,SIGNAL(triggered()),this,SLOT(debug()));
    toolBar->addAction(actionDebug);
    QAction *actionKill = toolBar->addAction(Utils::QIconCached(QString::fromStdString((MBXMLUtils::getInstallPath()/"share"/"mbsimgui"/"icons"/"kill.svg").string())),"Kill simulation");
    connect(actionKill,SIGNAL(triggered()),this,SLOT(kill()));
    toolBar->addAction(actionKill);

    elementView->setModel(new ElementTreeModel);
    elementView->setColumnWidth(0,250);
    elementView->setColumnWidth(1,200);
    elementView->hideColumn(1);

    embeddingView->setModel(new EmbeddingTreeModel);
    embeddingView->setColumnWidth(0,150);
    embeddingView->setColumnWidth(1,200);

    connect(elementView,SIGNAL(pressed(QModelIndex)), this, SLOT(elementViewClicked()));
    connect(embeddingView,SIGNAL(pressed(QModelIndex)), this, SLOT(embeddingViewClicked()));
    connect(elementView->selectionModel(),SIGNAL(currentChanged(const QModelIndex&,const QModelIndex&)), this, SLOT(selectionChanged(const QModelIndex&)));

    QDockWidget *dockWidget0 = new QDockWidget("MBSim project");
    addDockWidget(Qt::LeftDockWidgetArea,dockWidget0);
    dockWidget0->setWidget(projectView);

    QDockWidget *dockWidget1 = new QDockWidget("Multibody system");
    addDockWidget(Qt::LeftDockWidgetArea,dockWidget1);
    QWidget *widget1 = new QWidget(dockWidget1);
    dockWidget1->setWidget(widget1);
    auto *widgetLayout1 = new QGridLayout(widget1);
    widgetLayout1->setContentsMargins(0,0,0,0);
    widget1->setLayout(widgetLayout1);
    widgetLayout1->addWidget(elementViewFilter, 0, 0);
    widgetLayout1->addWidget(elementView, 1, 0);

    QDockWidget *dockWidget3 = new QDockWidget("Embeddings");
    addDockWidget(Qt::LeftDockWidgetArea,dockWidget3);
    QWidget *widget3 = new QWidget(dockWidget3);
    dockWidget3->setWidget(widget3);
    auto *widgetLayout3 = new QGridLayout(widget3);
    widgetLayout3->setContentsMargins(0,0,0,0);
    widget3->setLayout(widgetLayout3);
    widgetLayout3->addWidget(embeddingViewFilter, 0, 0);
    widgetLayout3->addWidget(embeddingView, 1, 0);

    QDockWidget *dockWidget2 = new QDockWidget("Solver");
    addDockWidget(Qt::LeftDockWidgetArea,dockWidget2);
    dockWidget2->setWidget(solverView);

    //tabifyDockWidget(dockWidget3,dockWidget4);
    //tabifyDockWidget(dockWidget2,dockWidget3);
    //QList<QTabBar *> tabList = findChildren<QTabBar *>();
    //tabBar = tabList.at(0);

    QWidget *centralWidget = new QWidget;  
    setCentralWidget(centralWidget);
    auto *mainlayout = new QHBoxLayout;
    centralWidget->setLayout(mainlayout);
    mainlayout->addWidget(inlineOpenMBVMW);

    QDockWidget *mbsimDW = new QDockWidget("MBSim Echo Area", this);
    addDockWidget(Qt::BottomDockWidgetArea, mbsimDW);
    mbsimDW->setWidget(echoView);
    connect(&process,SIGNAL(finished(int,QProcess::ExitStatus)),this,SLOT(processFinished(int,QProcess::ExitStatus)));

    setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);

    if(arg.contains("--maximized"))
      showMaximized();

    QString projectFile;
    QRegExp filterProject(".+\\.mbsimprj\\.xml");
    QDir dir;
    dir.setFilter(QDir::Files);
    for(auto & it : arg) {
      if(it[0]=='-') continue;
      dir.setPath(it);
      if(dir.exists()) {
        QStringList file=dir.entryList();
        for(int j=0; j<file.size(); j++) {
          if(projectFile.isEmpty() and filterProject.exactMatch(file[j]))
            projectFile = dir.path()+"/"+file[j];
        }
        continue;
      }
      if(QFile::exists(it)) {
        if(projectFile.isEmpty() and filterProject.exactMatch(it))
          projectFile = it;
        continue;
      }
    }
    if(projectFile.size())
      loadProject(QDir::current().absoluteFilePath(projectFile));
    else
      newProject(false);

    setAcceptDrops(true);

    connect(&autoSaveTimer, SIGNAL(timeout()), this, SLOT(autoSaveProject()));
    autoSaveTimer.start(autoSaveInterval*60000);

    connect(&echoViewTimer, SIGNAL(timeout()), this, SLOT(updateEchoView()));

    setWindowIcon(Utils::QIconCached(QString::fromStdString((MBXMLUtils::getInstallPath()/"share"/"mbsimgui"/"icons"/"mbsimgui.svg").string())));
  }

  void MainWindow::autoSaveProject() {
    saveProject("./.MBS.mbsimprj.xml",true,false);
  }

  void MainWindow::processFinished(int exitCode, QProcess::ExitStatus exitStatus) {
    echoViewTimer.stop();
    updateEchoView();
    if(exitStatus==QProcess::NormalExit && exitCode==0)
      echoView->setCurrentIndex(0);
    else
      echoView->setCurrentIndex(1);
    if(currentTask==1) {
      inlineOpenMBVMW->openFile(uniqueTempDir.generic_string()+"/MBS_tmp.ombv.xml");
      QModelIndex index = elementView->selectionModel()->currentIndex();
      auto *model = static_cast<ElementTreeModel*>(elementView->model());
      auto *element=dynamic_cast<Element*>(model->getItem(index)->getItemData());
      if(element)
        highlightObject(element->getID());
    }
    else {
      if(exitStatus == QProcess::NormalExit) {
        if(autoExport) {
          saveMBSimH5Data(autoExportDir+"/MBS.mbsim.h5");
          saveOpenMBVXMLData(autoExportDir+"/MBS.ombv.xml");
          saveOpenMBVH5Data(autoExportDir+"/MBS.ombv.h5");
          if(saveFinalStateVector)
            saveStateVector(autoExportDir+"/statevector.asc");
        }
        actionSaveDataAs->setDisabled(false);
        actionSaveMBSimH5DataAs->setDisabled(false);
        actionSaveOpenMBVDataAs->setDisabled(false);
        if(saveFinalStateVector)
          actionSaveStateVectorAs->setDisabled(false);
        if(solverView->getSolverNumber()==7) {
          actionSaveEigenanalysisAs->setDisabled(false);
          actionEigenanalysis->setDisabled(false);
        }
        if(solverView->getSolverNumber()==8) {
          actionFrequencyResponse->setDisabled(false);
        }
        actionOpenMBV->setDisabled(false);
        actionH5plotserie->setDisabled(false);
      }
      else {
        actionSaveDataAs->setDisabled(true);
        actionSaveMBSimH5DataAs->setDisabled(true);
        actionSaveOpenMBVDataAs->setDisabled(true);
        actionSaveStateVectorAs->setDisabled(true);
        actionSaveEigenanalysisAs->setDisabled(true);
        actionOpenMBV->setDisabled(true);
        actionH5plotserie->setDisabled(true);
        actionEigenanalysis->setDisabled(true);
        actionFrequencyResponse->setDisabled(true);
      }
    }
    actionSimulate->setDisabled(false);
    actionRefresh->setDisabled(false);
    actionDebug->setDisabled(false);
    statusBar()->showMessage(tr("Ready"));
  }

  void MainWindow::initInlineOpenMBV() {
    std::list<string> arg;
    arg.emplace_back("--wst");
    arg.push_back((MBXMLUtils::getInstallPath()/"share"/"mbsimgui"/"inlineopenmbv.ombv.wst").string());
    arg.emplace_back("/home/foerg/tmp/openmbv");
    inlineOpenMBVMW = new OpenMBVGUI::MainWindow(arg);

    connect(inlineOpenMBVMW, SIGNAL(objectSelected(std::string, Object*)), this, SLOT(selectElement(std::string)));
    connect(inlineOpenMBVMW, SIGNAL(objectDoubleClicked(std::string, Object*)), elementView, SLOT(openEditor()));
  }

  MainWindow::~MainWindow() {
    // use nothrow boost::filesystem functions to avoid exceptions in this dtor
    boost::system::error_code ec;
    bfs::remove_all(uniqueTempDir, ec);
    bfs::remove("./.MBS.mbsimprj.xml", ec);
    delete project;
  }

  void MainWindow::setProjectChanged(bool changed) { 
    setWindowModified(changed);
    if(changed) {
      actionOpenMBV->setDisabled(true);
      actionH5plotserie->setDisabled(true);
      actionEigenanalysis->setDisabled(true);
      actionFrequencyResponse->setDisabled(true);
      undos.push_back(static_cast<xercesc::DOMDocument*>(doc->cloneNode(true)));
      if(undos.size() > maxUndo)
        undos.pop_front();
      redos.clear();
      if(allowUndo) actionUndo->setEnabled(true);
      actionRedo->setDisabled(true);
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
    QString dir = QFileDialog::getExistingDirectory (nullptr, "Working directory", ".");
    if(not(dir.isEmpty())) {
      QString absoluteMBSFilePath = QDir::current().absoluteFilePath(projectFile);
      QDir::setCurrent(dir);
      mbsDir = QFileInfo(absoluteMBSFilePath).absolutePath();
      projectFile = QDir::current().relativeFilePath(absoluteMBSFilePath);
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
        autoSaveTimer.start(autoSaveInterval*60000);
      else
        autoSaveTimer.stop();
      maxUndo = menu.getMaxUndo();
    }
  }

  void MainWindow::highlightObject(const QString &ID) {
    currentID = ID;
    inlineOpenMBVMW->highlightObject(ID.toStdString());
  }

  void MainWindow::selectionChanged(const QModelIndex &current) {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    auto *element=dynamic_cast<Element*>(model->getItem(current)->getItemData());
    if(element) {
      auto *emodel = static_cast<EmbeddingTreeModel*>(embeddingView->model());
      vector<EmbedItemData*> parents = element->getEmbedItemParents();
      QModelIndex index = emodel->index(0,0);
      emodel->removeRow(index.row(), index.parent());
      if(!parents.empty()) {
        index = emodel->createEmbeddingItem(parents[0]);
        for(size_t i=0; i<parents.size()-1; i++)
          index = emodel->createEmbeddingItem(parents[i+1],index);
        emodel->createEmbeddingItem(element,index);
      }
      else
        index = emodel->createEmbeddingItem(element);
      embeddingView->expandAll();
      highlightObject(element->getID());
    }
    else
      highlightObject("");
  }

  void MainWindow::elementViewClicked() {
    if(QApplication::mouseButtons()==Qt::RightButton) {
      QModelIndex index = elementView->selectionModel()->currentIndex();
      TreeItemData *itemData = static_cast<ElementTreeModel*>(elementView->model())->getItem(index)->getItemData();
      if(itemData && index.column()==0) {
        QMenu *menu = itemData->createContextMenu();
        menu->exec(QCursor::pos());
        delete menu;
      } 
    }
    else if(QApplication::mouseButtons()==Qt::LeftButton)
      selectionChanged(elementView->selectionModel()->currentIndex());
  }

  void MainWindow::embeddingViewClicked() {
    if(QApplication::mouseButtons()==Qt::RightButton) {
      QModelIndex index = embeddingView->selectionModel()->currentIndex();
      if(index.column()==0) {
        auto *parameter = dynamic_cast<Parameter*>(static_cast<EmbeddingTreeModel*>(embeddingView->model())->getItem(index)->getItemData());
        if(parameter) {
          QMenu *menu = parameter->createContextMenu();
          menu->exec(QCursor::pos());
          delete menu;
          return;
        }
        else {
          auto *item = static_cast<EmbedItemData*>(static_cast<EmbeddingTreeModel*>(embeddingView->model())->getItem(index)->getItemData());
          if(item and item->getXMLElement()) {
            QMenu *menu = item->createEmbeddingContextMenu();
            menu->exec(QCursor::pos());
            delete menu;
          }
        }
      } 
    }
  }

  void MainWindow::solverViewClicked() {
    auto *emodel = static_cast<EmbeddingTreeModel*>(embeddingView->model());
    vector<EmbedItemData*> parents = getProject()->getSolver()->getEmbedItemParents();
    QModelIndex index = emodel->index(0,0);
    emodel->removeRow(index.row(), index.parent());
    if(!parents.empty()) {
      index = emodel->createEmbeddingItem(parents[0]);
      for(size_t i=0; i<parents.size()-1; i++)
        index = emodel->createEmbeddingItem(parents[i+1],index);
      emodel->createEmbeddingItem(getProject()->getSolver(),index);
    }
    else
      index = emodel->createEmbeddingItem(getProject()->getSolver());
    embeddingView->expandAll();
    embeddingView->scrollTo(index.child(emodel->rowCount(index)-1,0),QAbstractItemView::PositionAtTop);
  }

  void MainWindow::projectViewClicked() {
    auto *emodel = static_cast<EmbeddingTreeModel*>(embeddingView->model());
    QModelIndex index = emodel->index(0,0);
    emodel->removeRow(index.row(), index.parent());
    emodel->createEmbeddingItem(projectView->getProject());
    embeddingView->expandAll();
    embeddingView->scrollTo(index.child(emodel->rowCount(index)-1,0),QAbstractItemView::PositionAtTop);
  }

  void MainWindow::newProject(bool ask) {
    if(maybeSave()) {
      undos.clear();
      elementBuffer.first = NULL;
      parameterBuffer.first = NULL;
      setProjectChanged(false);
      mbsDir = QDir::current();
      actionOpenMBV->setDisabled(true);
      actionH5plotserie->setDisabled(true);
      actionEigenanalysis->setDisabled(true);
      actionFrequencyResponse->setDisabled(true);
      actionSaveDataAs->setDisabled(true);
      actionSaveMBSimH5DataAs->setDisabled(true);
      actionSaveOpenMBVDataAs->setDisabled(true);
      actionSaveStateVectorAs->setDisabled(true);
      actionSaveEigenanalysisAs->setDisabled(true);
      actionSaveProject->setDisabled(true);

      auto *pmodel = static_cast<EmbeddingTreeModel*>(embeddingView->model());
      QModelIndex index = pmodel->index(0,0);
      pmodel->removeRows(index.row(), pmodel->rowCount(QModelIndex()), index.parent());

      auto *model = static_cast<ElementTreeModel*>(elementView->model());
      index = model->index(0,0);
      model->removeRow(index.row(), index.parent());

      delete project;

      doc = impl->createDocument();

      project = new Project;
      project->createXMLElement(doc);
      projectView->setProject(project);
      projectView->updateName();

      model->createGroupItem(project->getDynamicSystemSolver(),QModelIndex());
      elementView->selectionModel()->setCurrentIndex(model->index(0,0), QItemSelectionModel::ClearAndSelect);

      solverView->setSolver(0);

      projectFile="";
      mbsimxml(1);
      setWindowTitle("MBS.mbsimprj.xml[*]");
    }
  }

  void MainWindow::loadProject(const QString &file) {
    if(not(file.isEmpty())) {
      undos.clear();
      elementBuffer.first = NULL;
      parameterBuffer.first = NULL;
      setProjectChanged(false);
      mbsDir = QFileInfo(file).absolutePath();
      actionOpenMBV->setDisabled(true);
      actionH5plotserie->setDisabled(true);
      actionEigenanalysis->setDisabled(true);
      actionFrequencyResponse->setDisabled(true);
      actionSaveDataAs->setDisabled(true);
      actionSaveMBSimH5DataAs->setDisabled(true);
      actionSaveOpenMBVDataAs->setDisabled(true);
      actionSaveStateVectorAs->setDisabled(true);
      actionSaveEigenanalysisAs->setDisabled(true);
      actionSaveProject->setDisabled(false);
      QDir::setCurrent(QFileInfo(file).absolutePath());
      projectFile=QDir::current().relativeFilePath(file);
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
      setWindowTitle(projectFile+"[*]");
      rebuildTree();
      mbsimxml(1);
    }
  }

  void MainWindow::loadProject() {
    if(maybeSave()) {
      QString file=QFileDialog::getOpenFileName(nullptr, "XML project files", ".", "XML files (*.mbsimprj.xml)");
      if(file!="")
        loadProject(file);
    }
  }

  bool MainWindow::saveProjectAs() {
    QString file=QFileDialog::getSaveFileName(nullptr, "XML project files", project->getName()+".mbsimprj.xml", "XML files (*.mbsimprj.xml)");
    if(not(file.isEmpty())) {
      file = (file.length()>13 and file.right(13)==".mbsimprj.xml")?file:file+".mbsimprj.xml";
      mbsDir = QFileInfo(file).absolutePath();
      QDir::setCurrent(QFileInfo(file).absolutePath());
      projectFile=QDir::current().relativeFilePath(file);
      setCurrentProjectFile(file);
      setWindowTitle(projectFile+"[*]");
      actionSaveProject->setDisabled(false);
      return saveProject();
    }
    return false;
  }

  bool MainWindow::saveProject(const QString &fileName, bool processDocument, bool modifyStatus) {
    try {
      serializer->writeToURI(doc, X()%(fileName.isEmpty()?projectFile.toStdString():fileName.toStdString()));
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
    DOMNode *parent = getProject()->getSolver()->getXMLElement()->getParentNode();
    getProject()->getSolver()->removeXMLElement(false);
    projectView->getProject()->setSolver(solverView->createSolver(i));
    getProject()->getSolver()->createXMLElement(parent);
    std::vector<Parameter*> param;
    DOMElement *ele = MBXMLUtils::E(static_cast<DOMElement*>(parent))->getFirstElementChildNamed(MBXMLUtils::PV%"Parameter");
    if(ele) param = Parameter::initializeParametersUsingXML(ele);
    for(auto & i : param)
      getProject()->getSolver()->addParameter(i);
    solverViewClicked();
  }

  // update model parameters including additional paramters from paramList
  void MainWindow::updateParameters(EmbedItemData *item, bool exceptLatestParameter) {
    shared_ptr<xercesc::DOMDocument> doc=MainWindow::parser->createDocument();
    doc->setDocumentURI(X()%"file://Project.mbsimprj.xml");
    DOMElement *ele0 = D(doc)->createElement(PV%"Parameter");
    doc->insertBefore(ele0,nullptr);

    if(item->getXMLElement()) {
      QString counterName = item->getEmbedXMLElement()?QString::fromStdString(E(item->getEmbedXMLElement())->getAttribute("counterName")):"";
      if(not(counterName.isEmpty())) {
        DOMElement *ele1=D(doc)->createElement(PV%"scalarParameter");
        E(ele1)->setAttribute("name", counterName.toStdString());
        DOMText *text = doc->createTextNode(X()%"1");
        ele1->insertBefore(text,nullptr);
        ele0->insertBefore(ele1,nullptr);
      }
    }

    vector<EmbedItemData*> parents = item->getEmbedItemParents();
    for(auto & parent : parents) {
      for(size_t j=0; j<parent->getNumberOfParameters(); j++) {
        DOMNode *node = doc->importNode(parent->getParameter(j)->getXMLElement(),true);
        ele0->insertBefore(node,nullptr);
      }
    }
    for(size_t j=0; j<item->getNumberOfParameters()-exceptLatestParameter; j++) {
      DOMNode *node = doc->importNode(item->getParameter(j)->getXMLElement(),true);
      ele0->insertBefore(node,nullptr);
    }

    string message;
    try {
      D(doc)->validate();
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
    QString dir = QFileDialog::getExistingDirectory (nullptr, "Export simulation data", ".");
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
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = model->index(0,0);
    QString file=QFileDialog::getSaveFileName(nullptr, "Export MBSim H5 file", QString("./")+model->getItem(index)->getItemData()->getName()+".mbsim.h5", "H5 files (*.mbsim.h5)");
    if(file!="") {
      saveMBSimH5Data(file);
    }
  }

  void MainWindow::saveMBSimH5Data(const QString &file) {
    if(QFile::exists(file))
      QFile::remove(file);
    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/"+project->getDynamicSystemSolver()->getName()+".mbsim.h5",file);
  }

  void MainWindow::saveOpenMBVDataAs() {
    QString dir = QFileDialog::getExistingDirectory (nullptr, "Export OpenMBV data", ".");
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
    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/"+project->getDynamicSystemSolver()->getName()+".ombv.xml",file);
  }

  void MainWindow::saveOpenMBVH5Data(const QString &file) {
    if(QFile::exists(file))
      QFile::remove(file);
    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/"+project->getDynamicSystemSolver()->getName()+".ombv.h5",file);
  }

  void MainWindow::saveStateVectorAs() {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = model->index(0,0);
    QString file=QFileDialog::getSaveFileName(nullptr, "Export state vector file", "./statevector.asc", "ASCII files (*.asc)");
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
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = model->index(0,0);
    QString file=QFileDialog::getSaveFileName(nullptr, "Export eigenanalysis file", QString("./")+model->getItem(index)->getItemData()->getName()+".eigenanalysis.mat", "mat files (*.eigenanalysis.mat)");
    if(file!="") {
      saveEigenanalysis(file);
    }
  }

  void MainWindow::saveEigenanalysis(const QString &file) {
    if(QFile::exists(file))
      QFile::remove(file);
    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/"+project->getDynamicSystemSolver()->getName()+".eigenanalysis.mat",file);
  }

  void MainWindow::mbsimxml(int task) {

    currentTask = task;

    shared_ptr<xercesc::DOMDocument> doc=MainWindow::parser->createDocument();
    doc->setDocumentURI(X()%"file://Project.mbsimprj.xml");
    auto *newDocElement = static_cast<DOMElement*>(doc->importNode(this->doc->getDocumentElement(), true));
    doc->insertBefore(newDocElement, nullptr);
    projectView->getProject()->processIDAndHref(newDocElement);

    QString uniqueTempDir_ = QString::fromStdString(uniqueTempDir.generic_string());
    QString projectFile;

    projectFile=uniqueTempDir_+"/Project.mbsimprj.flat.xml";

    if(task==1) {
      if(OpenMBVGUI::MainWindow::getInstance()->getObjectList()->invisibleRootItem()->childCount())
        static_cast<OpenMBVGUI::Group*>(OpenMBVGUI::MainWindow::getInstance()->getObjectList()->invisibleRootItem()->child(0))->unloadFileSlot();
    }

    DOMElement *root;
    QString errorText;
    try {
      D(doc)->validate();
      root = doc->getDocumentElement();
      Preprocess::preprocess(parser, eval, dependencies, root);
    }
    catch(exception &ex) {
      errorText = ex.what();
    }
    catch(...) {
      errorText = "Unknown error";
    }
    if(not errorText.isEmpty()) {
      echoView->setErrorText(errorText);
      echoView->updateOutputAndError();
      echoView->setCurrentIndex(1);
      return;
    }
    // adapt the evaluator in the dom
    DOMElement *evaluator=D(doc)->createElement(PV%"evaluator");
    evaluator->appendChild(doc->createTextNode(X()%"xmlflat"));
    root->insertBefore(evaluator, root->getFirstChild());
    DOMParser::serialize(doc.get(), projectFile.toStdString(), false);
    QStringList arg;
    if(currentTask==1)
      arg.append("--stopafterfirststep");
    else if(saveFinalStateVector)
      arg.append("--savefinalstatevector");
    arg.append(projectFile);
    echoView->clearOutputAndError();
    echoViewTimer.start(250);
    process.setWorkingDirectory(uniqueTempDir_);
    process.start((MBXMLUtils::getInstallPath()/"bin"/"mbsimflatxml").string().c_str(), arg);
  }

  void MainWindow::simulate() {
    actionSimulate->setDisabled(true);
    statusBar()->showMessage(tr("Simulate"));
    mbsimxml(0);
  }

  void MainWindow::refresh() {
    actionRefresh->setDisabled(true);
    statusBar()->showMessage(tr("Refresh"));
    mbsimxml(1);
  }

  void MainWindow::openmbv() {
    QString name = QString::fromStdString(uniqueTempDir.generic_string())+"/"+project->getDynamicSystemSolver()->getName()+".ombv.xml";
    if(QFile::exists(name)) {
      QStringList arg;
      arg.append("--autoreload");
      arg.append(name);
      QProcess::startDetached((MBXMLUtils::getInstallPath()/"bin"/"openmbv").string().c_str(), arg);
    }
  }

  void MainWindow::h5plotserie() {
    QString name = QString::fromStdString(uniqueTempDir.generic_string())+"/"+project->getDynamicSystemSolver()->getName()+".mbsim.h5";
    if(QFile::exists(name)) {
      QStringList arg;
      arg.append(name);
      QProcess::startDetached((MBXMLUtils::getInstallPath()/"bin"/"h5plotserie").string().c_str(), arg);
    }
  }

  void MainWindow::eigenanalysis() {
    QString name = QString::fromStdString(uniqueTempDir.generic_string())+"/"+project->getDynamicSystemSolver()->getName()+".eigenanalysis.mat";
    if(QFile::exists(name)) {
      EigenanalysisDialog *dialog = new EigenanalysisDialog(name,this);
      dialog->show();
    }
  }

  void MainWindow::frequencyResponse() {
    QString name = QString::fromStdString(uniqueTempDir.generic_string())+"/"+project->getDynamicSystemSolver()->getName()+".harmonic_response_analysis.mat";
    if(QFile::exists(name)) {
      HarmonicResponseDialog *dialog = new HarmonicResponseDialog(name,this);
      dialog->show();
    }
  }

  void MainWindow::debug() {
    currentTask = 0;
    auto *newdoc = static_cast<xercesc::DOMDocument*>(doc->cloneNode(true));
    projectView->getProject()->processHref(newdoc->getDocumentElement());
    QString uniqueTempDir_ = QString::fromStdString(uniqueTempDir.generic_string());
    QString projectFile = uniqueTempDir_+"/Project.mbsimprj.xml";
    serializer->writeToURI(newdoc, X()%projectFile.toStdString());
    QStringList arg;
    arg.append("--stopafterfirststep");
    arg.append(projectFile);
    echoView->clearOutputAndError();
    process.setWorkingDirectory(uniqueTempDir_);
    process.start((MBXMLUtils::getInstallPath()/"bin"/"mbsimxml").string().c_str(), arg);
  }

  void MainWindow::selectElement(const string& ID) {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    auto it=model->idEleMap.find(QString::fromStdString(ID));
    if(it!=model->idEleMap.end())
      elementView->selectionModel()->setCurrentIndex(it->second,QItemSelectionModel::ClearAndSelect);
  }

  void MainWindow::help() {
    QMessageBox::information(this, tr("MBSimGUI - Help"), tr("<p>Please visit <a href=\"https://www.mbsim-env.de\">MBSim-Environment</a> for documentation.</p>"));
  }

  void MainWindow::xmlHelp(const QString &url) {
    if(not xmlHelpDialog) {
      xmlHelpDialog = new WebDialog(this);
      xmlHelpDialog->setWindowTitle("MBSimGUI - MBSim XML Documentation");
    }
    if(url.isEmpty())
      xmlHelpDialog->load(QUrl(((MBXMLUtils::getInstallPath()/"share"/"mbxmlutils"/"doc").string()+"/http___www_mbsim-env_de_MBSim/index.html").c_str()));
    else
      xmlHelpDialog->load(QUrl(url));
    xmlHelpDialog->show();
  }

  void MainWindow::about() {
     QMessageBox::about(this, tr("About MBSimGUI"), (tr("<p><b>MBSimGUI %1</b></p><p>MBSimGUI is a graphical user interface for the multibody simulation software MBSim.</p>").arg(VERSION)
           + tr("<p>See <a href=\"https://www.mbsim-env.de\">MBSim-Environment</a> for more information.</p>"
             "<p>Copyright &copy; Martin Foerg <tt>&lt;martin.o.foerg@googlemail.com&gt;</tt><p/>"
             "<p>Licensed under the General Public License (see file COPYING).</p>"
             "<p>This is free software; see the source for copying conditions. There is NO warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.</p>")));
  }

  void MainWindow::rebuildTree() {

    auto *pmodel = static_cast<EmbeddingTreeModel*>(embeddingView->model());
    QModelIndex index = pmodel->index(0,0);
    pmodel->removeRows(index.row(), pmodel->rowCount(QModelIndex()), index.parent());

    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    index = model->index(0,0);
    model->removeRow(index.row(), index.parent());

    delete project;

    project=Embed<Project>::createAndInit(doc->getDocumentElement());
    projectView->setProject(project);
    projectView->updateName();

    model->createGroupItem(project->getDynamicSystemSolver());
    elementView->selectionModel()->setCurrentIndex(model->index(0,0), QItemSelectionModel::ClearAndSelect);

    solverView->setSolver(getProject()->getSolver());
  }

  void MainWindow::edit() {
    if(elementView->hasFocus())
      elementView->openEditor();
    else if(embeddingView->hasFocus())
      embeddingView->openEditor();
    else if(solverView->hasFocus())
      solverView->openEditor();
    else if(projectView->hasFocus())
      projectView->openEditor();
  }

  void MainWindow::undo() {
    elementBuffer.first = NULL;
    parameterBuffer.first = NULL;
    setWindowModified(true);
    redos.push_back(doc);
    doc = undos.back();
    undos.pop_back();
    rebuildTree();
    mbsimxml(1);
    actionUndo->setDisabled(undos.empty());
    actionRedo->setEnabled(true);
  }

  void MainWindow::redo() {
    undos.push_back(doc);
    doc = redos.back();
    redos.pop_back();
    rebuildTree();
    mbsimxml(1);
    actionRedo->setDisabled(redos.empty());
    actionUndo->setEnabled(true);
  }

  void MainWindow::removeElement() {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    auto *element = dynamic_cast<Element*>(model->getItem(index)->getItemData());
    if(element and (not dynamic_cast<DynamicSystemSolver*>(element)) and (not dynamic_cast<InternalFrame*>(element))) {
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
    auto *model = static_cast<EmbeddingTreeModel*>(embeddingView->model());
    QModelIndex index = embeddingView->selectionModel()->currentIndex();
    auto *parameter = dynamic_cast<Parameter*>(model->getItem(index)->getItemData());
    if(parameter) {
      setProjectChanged(true);
      if(parameter == parameterBuffer.first)
        parameterBuffer.first = NULL;
      DOMNode *ps = parameter->getXMLElement()->getPreviousSibling();
      if(ps and X()%ps->getNodeName()=="#text")
        parameter->getXMLElement()->getParentNode()->removeChild(ps);
      parameter->getXMLElement()->getParentNode()->removeChild(parameter->getXMLElement());
      parameter->getParent()->removeParameter(parameter);
      parameter->getParent()->maybeRemoveEmbedXMLElement();
      model->removeRow(index.row(), index.parent());
      mbsimxml(1);
    }
  }

  void MainWindow::remove() {
    if(elementView->hasFocus())
      removeElement();
    else if(embeddingView->hasFocus())
      removeParameter();
  }

  void MainWindow::copy(bool cut) {
    if(elementView->hasFocus())
      copyElement(cut);
    else if(embeddingView->hasFocus())
      copyParameter(cut);
  }

  void MainWindow::paste() {
    if(elementView->hasFocus()) {
      auto *model = static_cast<ElementTreeModel*>(elementView->model());
      QModelIndex index = elementView->selectionModel()->currentIndex();
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
    else if(embeddingView->hasFocus()) {
      auto *model = static_cast<EmbeddingTreeModel*>(embeddingView->model());
      QModelIndex index = embeddingView->selectionModel()->currentIndex();
      auto *item = dynamic_cast<EmbedItemData*>(model->getItem(index)->getItemData());
      if(item)
        loadParameter(item,getParameterBuffer().first);
    }
  }

  void MainWindow::move(bool up) {
    if(elementView->hasFocus()) {
      auto *model = static_cast<ElementTreeModel*>(elementView->model());
      QModelIndex index = elementView->selectionModel()->currentIndex();
      auto *frame = dynamic_cast<Frame*>(model->getItem(index)->getItemData());
      if(frame and (not dynamic_cast<InternalFrame*>(frame)) and (up?(frame->getParent()->getIndexOfFrame(frame)>1):(frame->getParent()->getIndexOfFrame(frame)<frame->getParent()->getNumberOfFrames()-1))) {
        moveFrame(up);
        return;
      }
      auto *contour = dynamic_cast<Contour*>(model->getItem(index)->getItemData());
      if(contour and (up?(contour->getParent()->getIndexOfContour(contour)>0):(contour->getParent()->getIndexOfContour(contour)<contour->getParent()->getNumberOfContours()-1))) {
        moveContour(up);
        return;
      }
      auto *group = dynamic_cast<Group*>(model->getItem(index)->getItemData());
      if(group and (up?(group->getParent()->getIndexOfGroup(group)>0):(group->getParent()->getIndexOfGroup(group)<group->getParent()->getNumberOfGroups()-1))) {
        moveGroup(up);
        return;
      }
      auto *object = dynamic_cast<Object*>(model->getItem(index)->getItemData());
      if(object and (up?(object->getParent()->getIndexOfObject(object)>0):(object->getParent()->getIndexOfObject(object)<object->getParent()->getNumberOfObjects()-1))) {
        moveObject(up);
        return;
      }
      auto *link = dynamic_cast<Link*>(model->getItem(index)->getItemData());
      if(link and (up?(link->getParent()->getIndexOfLink(link)>0):(link->getParent()->getIndexOfLink(link)<link->getParent()->getNumberOfLinks()-1))) {
        moveLink(up);
        return;
      }
      auto *constraint = dynamic_cast<Constraint*>(model->getItem(index)->getItemData());
      if(constraint and (up?(constraint->getParent()->getIndexOfConstraint(constraint)>0):(constraint->getParent()->getIndexOfConstraint(constraint)<constraint->getParent()->getNumberOfConstraints()-1))) {
        moveConstraint(up);
        return;
      }
      auto *observer = dynamic_cast<Observer*>(model->getItem(index)->getItemData());
      if(observer and (up?(observer->getParent()->getIndexOfObserver(observer)>0):(observer->getParent()->getIndexOfObserver(observer)<observer->getParent()->getNumberOfObservers()-1))) {
        moveObserver(up);
        return;
      }
    }
    else if(embeddingView->hasFocus()) {
      auto *model = static_cast<EmbeddingTreeModel*>(embeddingView->model());
      QModelIndex index = embeddingView->selectionModel()->currentIndex();
      auto *parameter = dynamic_cast<Parameter*>(model->getItem(index)->getItemData());
      if(parameter and (up?(parameter->getParent()->getIndexOfParameter(parameter)>0):(parameter->getParent()->getIndexOfParameter(parameter)<parameter->getParent()->getNumberOfParameters()-1)))
        moveParameter(up);
    }
  }

  void MainWindow::copyElement(bool cut) {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    auto *element = static_cast<Element*>(model->getItem(index)->getItemData());
    if((not dynamic_cast<DynamicSystemSolver*>(element)) and (not dynamic_cast<InternalFrame*>(element)))
      elementBuffer = make_pair(element,cut);
  }

  void MainWindow::copyParameter(bool cut) {
    auto *model = static_cast<EmbeddingTreeModel*>(embeddingView->model());
    QModelIndex index = embeddingView->selectionModel()->currentIndex();
    auto *parameter = static_cast<Parameter*>(model->getItem(index)->getItemData());
    parameterBuffer = make_pair(parameter,cut);
  }

  void MainWindow::moveParameter(bool up) {
    setProjectChanged(true);
    auto *model = static_cast<EmbeddingTreeModel*>(embeddingView->model());
    QModelIndex index = embeddingView->selectionModel()->currentIndex();
    auto *parameter = static_cast<Parameter*>(model->getItem(index)->getItemData());
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
    embeddingView->setCurrentIndex(index.sibling(j,0));
  }

  void MainWindow::moveFrame(bool up) {
    setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    auto *frame = static_cast<Frame*>(model->getItem(index)->getItemData());
    int i = frame->getParent()->getIndexOfFrame(frame);
    int j = up?i-1:i+1;
    DOMElement *ele = static_cast<DOMElement*>(frame->getEmbedXMLElement()?frame->getEmbedXMLElement():frame->getXMLElement());
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
    elementView->setCurrentIndex(index.sibling(j,0));
  }

  void MainWindow::moveContour(bool up) {
    setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    auto *contour = static_cast<Contour*>(model->getItem(index)->getItemData());
    int i = contour->getParent()->getIndexOfContour(contour);
    int j = up?i-1:i+1;
    DOMElement *ele = static_cast<DOMElement*>(contour->getEmbedXMLElement()?contour->getEmbedXMLElement():contour->getXMLElement());
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
    elementView->setCurrentIndex(index.sibling(j,0));
  }

  void MainWindow::moveGroup(bool up) {
    setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    auto *group = static_cast<Group*>(model->getItem(index)->getItemData());
    int i = group->getParent()->getIndexOfGroup(group);
    int j = up?i-1:i+1;
    DOMElement *ele = static_cast<DOMElement*>(group->getEmbedXMLElement()?group->getEmbedXMLElement():group->getXMLElement());
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
      model->createGroupItem(group->getParent()->getGroup(i),index.parent());
    elementView->setCurrentIndex(index.sibling(j,0));
  }

  void MainWindow::moveObject(bool up) {
    setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    auto *object = static_cast<Object*>(model->getItem(index)->getItemData());
    int i = object->getParent()->getIndexOfObject(object);
    int j = up?i-1:i+1;
    DOMElement *ele = static_cast<DOMElement*>(object->getEmbedXMLElement()?object->getEmbedXMLElement():object->getXMLElement());
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
      model->createObjectItem(object->getParent()->getObject(i),index.parent());
    elementView->setCurrentIndex(index.sibling(j,0));
  }

  void MainWindow::moveLink(bool up) {
    setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    auto *link = static_cast<Link*>(model->getItem(index)->getItemData());
    int i = link->getParent()->getIndexOfLink(link);
    int j = up?i-1:i+1;
    DOMElement *ele = static_cast<DOMElement*>(link->getEmbedXMLElement()?link->getEmbedXMLElement():link->getXMLElement());
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
    elementView->setCurrentIndex(index.sibling(j,0));
  }

  void MainWindow::moveConstraint(bool up) {
    setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    auto *constraint = static_cast<Constraint*>(model->getItem(index)->getItemData());
    int i = constraint->getParent()->getIndexOfConstraint(constraint);
    int j = up?i-1:i+1;
    DOMElement *ele = static_cast<DOMElement*>(constraint->getEmbedXMLElement()?constraint->getEmbedXMLElement():constraint->getXMLElement());
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
    elementView->setCurrentIndex(index.sibling(j,0));
  }

  void MainWindow::moveObserver(bool up) {
    setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    auto *observer = static_cast<Observer*>(model->getItem(index)->getItemData());
    int i = observer->getParent()->getIndexOfObserver(observer);
    int j = up?i-1:i+1;
    DOMElement *ele = static_cast<DOMElement*>(observer->getEmbedXMLElement()?observer->getEmbedXMLElement():observer->getXMLElement());
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
    elementView->setCurrentIndex(index.sibling(j,0));
  }

  void MainWindow::saveElementAs() {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    auto *element = static_cast<Element*>(model->getItem(index)->getItemData());
    bool includeParameter = false;
    if(element->getEmbedXMLElement()) {
      SaveDialog saveDialog;
      saveDialog.exec();
      includeParameter = saveDialog.includeParameter();
    }
    QString file=QFileDialog::getSaveFileName(0, "XML model files", QString("./")+element->getName()+".mbsimele.xml", "XML files (*.xml)");
    if(not file.isEmpty()) {
      xercesc::DOMDocument *edoc = impl->createDocument();
      DOMNode *node = edoc->importNode(includeParameter?element->getEmbedXMLElement():element->getXMLElement(),true);
      edoc->insertBefore(node,nullptr);
      serializer->writeToURI(edoc, X()%file.toStdString());
    }
  }

  void MainWindow::saveSolverAs() {
    Solver *solver = project->getSolver();
    bool includeParameter = false;
    if(solver->getEmbedXMLElement()) {
      SaveDialog saveDialog;
      saveDialog.exec();
      includeParameter = saveDialog.includeParameter();
    }
    QString file=QFileDialog::getSaveFileName(0, "XML model files", QString("./")+solver->getName()+".mbsimslv.xml", "XML files (*.xml)");
    if(not file.isEmpty()) {
      xercesc::DOMDocument *edoc = impl->createDocument();
      DOMNode *node = edoc->importNode(includeParameter?solver->getEmbedXMLElement():solver->getXMLElement(),true);
      edoc->insertBefore(node,nullptr);
      serializer->writeToURI(edoc, X()%file.toStdString());
    }
  }

  void MainWindow::saveEmbeddingAs() {
    EmbeddingTreeModel *model = static_cast<EmbeddingTreeModel*>(embeddingView->model());
    QModelIndex index = embeddingView->selectionModel()->currentIndex();
    EmbedItemData *item = static_cast<EmbedItemData*>(model->getItem(index)->getItemData());
    QString file=QFileDialog::getSaveFileName(0, "XML model files", QString("./")+item->getName()+".mbsimembed.xml", "XML files (*.xml)");
    if(not file.isEmpty()) {
      xercesc::DOMDocument *edoc = impl->createDocument();
      DOMNode *node;
      node = edoc->importNode(item->getEmbedXMLElement()->getFirstElementChild(),true);
      edoc->insertBefore(node,nullptr);
      serializer->writeToURI(edoc, X()%file.toStdString());
    }
  }

  void MainWindow::addFrame(Frame *frame, Element *parent) {
    setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
//    frame->setName(frame->getName()+toQStr(model->getItem(index)->getID()));
    parent->addFrame(frame);
    frame->createXMLElement(parent->getXMLFrames());
    model->createFrameItem(frame,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementView->openEditor();
  }

  void MainWindow::addContour(Contour *contour, Element *parent) {
    setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
//    contour->setName(contour->getName()+toQStr(model->getItem(index)->getID()));
    parent->addContour(contour);
    contour->createXMLElement(parent->getXMLContours());
    model->createContourItem(contour,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementView->openEditor();
  }

  void MainWindow::addGroup(Group *group, Element *parent) {
    setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
//    group->setName(group->getName()+toQStr(model->getItem(index)->getID()));
    parent->addGroup(group);
    group->createXMLElement(parent->getXMLGroups());
    model->createGroupItem(group,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementView->openEditor();
  }

  void MainWindow::addObject(Object *object, Element *parent) {
    setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
//    object->setName(object->getName()+toQStr(model->getItem(index)->getID()));
    parent->addObject(object);
    object->createXMLElement(parent->getXMLObjects());
    model->createObjectItem(object,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementView->openEditor();
  }

  void MainWindow::addLink(Link *link, Element *parent) {
    setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
//    link->setName(link->getName()+toQStr(model->getItem(index)->getID()));
    parent->addLink(link);
    link->createXMLElement(parent->getXMLLinks());
    model->createLinkItem(link,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementView->openEditor();
  }

  void MainWindow::addConstraint(Constraint *constraint, Element *parent) {
    setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
//    constraint->setName(constraint->getName()+toQStr(model->getItem(index)->getID()));
    parent->addConstraint(constraint);
    constraint->createXMLElement(parent->getXMLConstraints());
    model->createConstraintItem(constraint,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementView->openEditor();
  }

  void MainWindow::addObserver(Observer *observer, Element *parent) {
    setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
//    observer->setName(observer->getName()+toQStr(model->getItem(index)->getID()));
    parent->addObserver(observer);
    observer->createXMLElement(parent->getXMLObservers());
    model->createObserverItem(observer,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    elementView->openEditor();
  }

  void MainWindow::addParameter(Parameter *parameter, EmbedItemData *parent) {
    setProjectChanged(true);
    QModelIndex index = embeddingView->selectionModel()->currentIndex();
    auto *model = static_cast<EmbeddingTreeModel*>(embeddingView->model());
    parent->addParameter(parameter);
    parameter->createXMLElement(parent->createParameterXMLElement());
//    if(parameter->getName()!="import")
//      parameter->setName(parameter->getName()+toQStr(model->getItem(index)->getID()));
    QModelIndex newIndex = model->createParameterItem(parameter,index);
    embeddingView->selectionModel()->setCurrentIndex(newIndex, QItemSelectionModel::ClearAndSelect);
    embeddingView->openEditor();
  }

  void MainWindow::loadParameter(EmbedItemData *parent, Parameter *param, bool embed) {
    setProjectChanged(true);
    vector<DOMElement*> elements;
    QString file;
    auto *model = static_cast<EmbeddingTreeModel*>(embeddingView->model());
    QModelIndex index = embeddingView->selectionModel()->currentIndex();
    if(param) {
      elements.push_back(static_cast<DOMElement*>(doc->importNode(param->getXMLElement(),true)));
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
    }
    else {
      file=QFileDialog::getOpenFileName(nullptr, "XML frame files", ".", "XML files (*.xml)");
      if(not file.isEmpty()) {
        xercesc::DOMDocument *doc = MBSimGUI::parser->parseURI(X()%file.toStdString());
        DOMElement *ele = embed?doc->getDocumentElement()->getFirstElementChild():static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true))->getFirstElementChild();
        while(ele) {
          elements.push_back(ele);
          ele = ele->getNextElementSibling();
        }
      }
      else
        return;
    }
    QModelIndex newIndex;
    for(auto & element : elements) {
      Parameter *parameter=ObjectFactory::getInstance()->createParameter(element);
      if(not parameter) {
        QMessageBox::warning(nullptr, "Load", "Cannot load file.");
        return;
      }
      if(embed)
        E(parent->createEmbedXMLElement())->setAttribute("parameterHref",mbsDir.relativeFilePath(file).toStdString());
      else
        parent->createParameterXMLElement()->insertBefore(element,nullptr);
      parameter->initializeUsingXML(element);
      parent->addParameter(parameter);
      newIndex = model->createParameterItem(parameter,index);
    }
    embeddingView->selectionModel()->setCurrentIndex(newIndex, QItemSelectionModel::ClearAndSelect);
  }

  void MainWindow::removeParameter(EmbedItemData *parent) {
    setProjectChanged(true);
    auto *model = static_cast<EmbeddingTreeModel*>(embeddingView->model());
    QModelIndex index = embeddingView->selectionModel()->currentIndex();
    model->removeRows(0,parent->getNumberOfParameters(),index);
    int n = parent->getNumberOfParameters();
    for(int i=0; i<n; i++)
      parent->removeParameter(parent->getParameter(i));
    parent->getEmbedXMLElement()->removeAttribute(X()%"parameterHref");
    parent->maybeRemoveEmbedXMLElement();
  }

  void MainWindow::loadFrame(Element *parent, Element *element, bool embed) {
    setProjectChanged(true);
    DOMElement *ele = nullptr;
    QString file;
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    if(element) {
      ele = static_cast<DOMElement*>(doc->importNode(element->getEmbedXMLElement()?element->getEmbedXMLElement():element->getXMLElement(),true));
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
      file=QFileDialog::getOpenFileName(nullptr, "XML frame files", ".", "XML files (*.xml)");
      if(not file.isEmpty()) {
        xercesc::DOMDocument *doc = MBSimGUI::parser->parseURI(X()%file.toStdString());
        ele = embed?doc->getDocumentElement():static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
      }
      else
        return;
    }
    Frame *frame = Embed<Frame>::createAndInit(ele);
    if(not frame) {
      QMessageBox::warning(nullptr, "Load", "Cannot load file.");
      return;
    }
    if(embed) {
      frame->setEmbedXMLElement(D(doc)->createElement(PV%"Embed"));
      parent->getXMLFrames()->insertBefore(frame->getEmbedXMLElement(), nullptr);
      E(frame->getEmbedXMLElement())->setAttribute("href",mbsDir.relativeFilePath(file).toStdString());
    }
    else
      parent->getXMLFrames()->insertBefore(ele, nullptr);
    parent->addFrame(frame);
    QModelIndex index = elementView->selectionModel()->currentIndex();
    model->createFrameItem(frame,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    mbsimxml(1);
  }

  void MainWindow::loadContour(Element *parent, Element *element, bool embed) {
    setProjectChanged(true);
    DOMElement *ele = nullptr;
    QString file;
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    if(element) {
      ele = static_cast<DOMElement*>(doc->importNode(element->getEmbedXMLElement()?element->getEmbedXMLElement():element->getXMLElement(),true));
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
      file=QFileDialog::getOpenFileName(nullptr, "XML contour files", ".", "XML files (*.xml)");
      if(not file.isEmpty()) {
        xercesc::DOMDocument *doc = MBSimGUI::parser->parseURI(X()%file.toStdString());
        ele = embed?doc->getDocumentElement():static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
      }
      else
        return;
    }
    Contour *contour = Embed<Contour>::createAndInit(ele);
    if(not contour) {
      QMessageBox::warning(nullptr, "Load", "Cannot load file.");
      return;
    }
    if(embed) {
      contour->setEmbedXMLElement(D(doc)->createElement(PV%"Embed"));
      parent->getXMLContours()->insertBefore(contour->getEmbedXMLElement(), nullptr);
      E(contour->getEmbedXMLElement())->setAttribute("href",mbsDir.relativeFilePath(file).toStdString());
    }
    else
      parent->getXMLContours()->insertBefore(ele, nullptr);
    parent->addContour(contour);
    QModelIndex index = elementView->selectionModel()->currentIndex();
    model->createContourItem(contour,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    mbsimxml(1);
  }

  void MainWindow::loadGroup(Element *parent, Element *element, bool embed) {
    setProjectChanged(true);
    DOMElement *ele = nullptr;
    QString file;
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    if(element) {
      ele = static_cast<DOMElement*>(doc->importNode(element->getEmbedXMLElement()?element->getEmbedXMLElement():element->getXMLElement(),true));
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
      file=QFileDialog::getOpenFileName(nullptr, "XML group files", ".", "XML files (*.xml)");
      if(not file.isEmpty()) {
        xercesc::DOMDocument *doc = MBSimGUI::parser->parseURI(X()%file.toStdString());
        ele = embed?doc->getDocumentElement():static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
      }
      else
        return;
    }
    Group *group = Embed<Group>::createAndInit(ele);
    if(not group) {
      QMessageBox::warning(nullptr, "Load", "Cannot load file.");
      return;
    }
    if(embed) {
      group->setEmbedXMLElement(D(doc)->createElement(PV%"Embed"));
      parent->getXMLGroups()->insertBefore(group->getEmbedXMLElement(), nullptr);
      E(group->getEmbedXMLElement())->setAttribute("href",mbsDir.relativeFilePath(file).toStdString());
    }
    else
      parent->getXMLGroups()->insertBefore(ele, nullptr);
    parent->addGroup(group);
    QModelIndex index = elementView->selectionModel()->currentIndex();
    model->createGroupItem(group,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    mbsimxml(1);
  }

  void MainWindow::loadObject(Element *parent, Element *element, bool embed) {
    setProjectChanged(true);
    DOMElement *ele = nullptr;
    QString file;
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    if(element) {
      ele = static_cast<DOMElement*>(doc->importNode(element->getEmbedXMLElement()?element->getEmbedXMLElement():element->getXMLElement(),true));
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
      file=QFileDialog::getOpenFileName(nullptr, "XML object files", ".", "XML files (*.xml)");
      if(not file.isEmpty()) {
        xercesc::DOMDocument *doc = MBSimGUI::parser->parseURI(X()%file.toStdString());
        ele = embed?doc->getDocumentElement():static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
      }
      else
        return;
    }
    Object *object = Embed<Object>::createAndInit(ele);
    if(not object) {
      QMessageBox::warning(nullptr, "Load", "Cannot load file.");
      return;
    }
    if(embed) {
      object->setEmbedXMLElement(D(doc)->createElement(PV%"Embed"));
      parent->getXMLObjects()->insertBefore(object->getEmbedXMLElement(), nullptr);
      E(object->getEmbedXMLElement())->setAttribute("href",mbsDir.relativeFilePath(file).toStdString());
    }
    else
      parent->getXMLObjects()->insertBefore(ele, nullptr);
    parent->addObject(object);
    QModelIndex index = elementView->selectionModel()->currentIndex();
    model->createObjectItem(object,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    mbsimxml(1);
  }

  void MainWindow::loadLink(Element *parent, Element *element, bool embed) {
    setProjectChanged(true);
    DOMElement *ele = nullptr;
    QString file;
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    if(element) {
      ele = static_cast<DOMElement*>(doc->importNode(element->getEmbedXMLElement()?element->getEmbedXMLElement():element->getXMLElement(),true));
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
      file=QFileDialog::getOpenFileName(nullptr, "XML link files", ".", "XML files (*.xml)");
      if(not file.isEmpty()) {
        xercesc::DOMDocument *doc = MBSimGUI::parser->parseURI(X()%file.toStdString());
        ele = embed?doc->getDocumentElement():static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
      }
      else
        return;
    }
    Link *link = Embed<Link>::createAndInit(ele);
    if(not link) {
      QMessageBox::warning(nullptr, "Load", "Cannot load file.");
      return;
    }
    if(embed) {
      link->setEmbedXMLElement(D(doc)->createElement(PV%"Embed"));
      parent->getXMLLinks()->insertBefore(link->getEmbedXMLElement(), nullptr);
      E(link->getEmbedXMLElement())->setAttribute("href",mbsDir.relativeFilePath(file).toStdString());
    }
    else
      parent->getXMLLinks()->insertBefore(ele, nullptr);
    parent->addLink(link);
    QModelIndex index = elementView->selectionModel()->currentIndex();
    model->createLinkItem(link,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    mbsimxml(1);
  }

  void MainWindow::loadConstraint(Element *parent, Element *element, bool embed) {
    setProjectChanged(true);
    DOMElement *ele = nullptr;
    QString file;
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    if(element) {
      ele = static_cast<DOMElement*>(doc->importNode(element->getEmbedXMLElement()?element->getEmbedXMLElement():element->getXMLElement(),true));
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
      file=QFileDialog::getOpenFileName(nullptr, "XML constraint files", ".", "XML files (*.xml)");
      if(not file.isEmpty()) {
        xercesc::DOMDocument *doc = MBSimGUI::parser->parseURI(X()%file.toStdString());
        ele = embed?doc->getDocumentElement():static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
      }
      else
        return;
    }
    Constraint *constraint = Embed<Constraint>::createAndInit(ele);
    if(not constraint) {
      QMessageBox::warning(nullptr, "Load", "Cannot load file.");
      return;
    }
    if(embed) {
      constraint->setEmbedXMLElement(D(doc)->createElement(PV%"Embed"));
      parent->getXMLConstraints()->insertBefore(constraint->getEmbedXMLElement(), nullptr);
      E(constraint->getEmbedXMLElement())->setAttribute("href",mbsDir.relativeFilePath(file).toStdString());
    }
    else
      parent->getXMLConstraints()->insertBefore(ele, nullptr);
    parent->addConstraint(constraint);
    QModelIndex index = elementView->selectionModel()->currentIndex();
    model->createConstraintItem(constraint,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    mbsimxml(1);
  }

  void MainWindow::loadObserver(Element *parent, Element *element, bool embed) {
    setProjectChanged(true);
    DOMElement *ele = nullptr;
    QString file;
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    if(element) {
      ele = static_cast<DOMElement*>(doc->importNode(element->getEmbedXMLElement()?element->getEmbedXMLElement():element->getXMLElement(),true));
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
      file=QFileDialog::getOpenFileName(nullptr, "XML observer files", ".", "XML files (*.xml)");
      if(not file.isEmpty()) {
        xercesc::DOMDocument *doc = MBSimGUI::parser->parseURI(X()%file.toStdString());
        ele = embed?doc->getDocumentElement():static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
      }
      else
        return;
    }
    Observer *observer = Embed<Observer>::createAndInit(ele);
    if(not observer) {
      QMessageBox::warning(nullptr, "Load", "Cannot load file.");
      return;
    }
    if(embed) {
      observer->setEmbedXMLElement(D(doc)->createElement(PV%"Embed"));
      parent->getXMLObservers()->insertBefore(observer->getEmbedXMLElement(), nullptr);
      E(observer->getEmbedXMLElement())->setAttribute("href",mbsDir.relativeFilePath(file).toStdString());
    }
    else
      parent->getXMLObservers()->insertBefore(ele, nullptr);
    parent->addObserver(observer);
    QModelIndex index = elementView->selectionModel()->currentIndex();
    model->createObserverItem(observer,index);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    mbsimxml(1);
  }

  void MainWindow::loadSolver() {
    setProjectChanged(true);
    project->getSolver()->removeXMLElement();
    DOMElement *ele = NULL;
    QString file=QFileDialog::getOpenFileName(0, "XML frame files", ".", "XML files (*.xml)");
    if(not file.isEmpty()) {
      xercesc::DOMDocument *doc = MBSimGUI::parser->parseURI(X()%file.toStdString());
      ele = static_cast<DOMElement*>(project->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
    }
    else
      return;
    Solver *solver = Embed<Solver>::createAndInit(ele);
    if(not solver) {
      QMessageBox::warning(0, "Load", "Cannot load file.");
      return;
    }
    project->getXMLElement()->insertBefore(ele, NULL);
    project->setSolver(solver);
    solverView->setSolver(solver);
    solverViewClicked();
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
      auto *action = qobject_cast<QAction *>(sender());
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
      auto *mainWin = qobject_cast<MainWindow *>(widget);
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

  void MainWindow::interrupt() {
    echoView->setErrorText("Simulation interrupted\n");
    process.terminate();
  }

  void MainWindow::kill() {
    echoView->setErrorText("Simulation killed\n");
    process.kill();
  }

  void MainWindow::updateEchoView() {
    echoView->addOutputText(process.readAllStandardOutput().data());
    echoView->addErrorText(process.readAllStandardError().data());
    echoView->updateOutputAndError();
  }

  void MainWindow::setAllowUndo(bool allowUndo_) {
    allowUndo = allowUndo_;
    actionUndo->setEnabled(allowUndo);
  }

}
