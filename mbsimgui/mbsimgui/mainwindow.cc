/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

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
#include "mainwindow.h"
#include "options.h"
#include "dynamic_system_solver.h"
#include "frame.h"
#include "contour.h"
#include "object.h"
#include "link_.h"
#include "constraint.h"
#include "observer.h"
#include "integrator.h"
#include "analyzer.h"
#include "objectfactory.h"
#include "parameter.h"
#include "widget.h"
#include "treemodel.h"
#include "treeitem.h"
#include "element_view.h"
#include "parameter_view.h"
#include "file_view.h"
#include "echo_view.h"
#include "embed.h"
#include "project.h"
#include "project_property_dialog.h"
#include "xml_property_dialog.h"
#include "clone_property_dialog.h"
#include "file_editor.h"
#include "utils.h"
#include "basicitemdata.h"
#include <openmbv/mainwindow.h>
#include <openmbvcppinterface/compoundrigidbody.h>
#include <utime.h>
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QDockWidget>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QFileDialog>
#include <QDragEnterEvent>
#include <QMimeData>
#include <QSettings>
#include <QHeaderView>
#include <QtWidgets/QDesktopWidget>
#include <QDesktopServices>
#include <QDialogButtonBox>
#include <mbxmlutils/eval.h>
#include <mbxmlutils/preprocess.h>
#include <boost/dll.hpp>
#include <boost/filesystem/fstream.hpp>
#include <mbxmlutilshelper/dom.h>
#include <xercesc/dom/DOMProcessingInstruction.hpp>
#include <xercesc/dom/DOMException.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMLSSerializer.hpp>
#include <xercesc/dom/DOMNodeList.hpp>
#include "dialogs.h"
#include "wizards.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;
namespace bfs=boost::filesystem;

namespace MBSimGUI {

  bool currentTask;

  MainWindow *mw;

  bool MainWindow::exitOK = true;

  vector<boost::filesystem::path> dependencies;

  MainWindow::MainWindow(QStringList &arg) : project(nullptr), inlineOpenMBVMW(nullptr), allowUndo(true), maxUndo(10), autoRefresh(true), statusUpdate(true), doc(nullptr), elementBuffer(nullptr,false), parameterBuffer(nullptr,false), installPath(boost::dll::program_location().parent_path().parent_path()) {
    QSettings settings;

    impl=DOMImplementation::getImplementation();
    parser=impl->createLSParser(DOMImplementation::MODE_SYNCHRONOUS, nullptr);
    serializer=impl->createLSSerializer();
    basicSerializer=impl->createLSSerializer();

    // use html output of MBXMLUtils
    static string HTMLOUTPUT="MBXMLUTILS_ERROROUTPUT=HTMLXPATH";
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

    QString program = QString::fromStdString((boost::dll::program_location().parent_path().parent_path()/"bin"/"mbsimxml").string());
    QStringList arguments;
    arguments << "--dumpXMLCatalog";
    arguments << (uniqueTempDir/".mbsimxml.catalog.xml").string().c_str();
    if(QProcess::execute(program,arguments)!=0)
      throw runtime_error("Failed to call mbsimxml --dumpXMLCatalog <file>.");

    mbxmlparser=DOMParser::create(uniqueTempDir/".mbsimxml.catalog.xml");

    echoView = new EchoView(this);
    fileView = new FileView;

    initInlineOpenMBV();

    // initialize streams
    auto f=[this](const string &s){
      echoView->addOutputText(QString::fromStdString(s));
    };
    debugStreamFlag=std::make_shared<bool>(false);
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Info      , std::make_shared<bool>(true),
      make_shared<fmatvec::PrePostfixedStream>("<span class=\"MBSIMGUI_INFO\">", "</span>", f));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Warn      , std::make_shared<bool>(true),
      make_shared<fmatvec::PrePostfixedStream>("<span class=\"MBSIMGUI_WARN\">", "</span>", f));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Debug     , debugStreamFlag             ,
      make_shared<fmatvec::PrePostfixedStream>("<span class=\"MBSIMGUI_DEBUG\">", "</span>", f));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Error     , std::make_shared<bool>(true),
      make_shared<fmatvec::PrePostfixedStream>("<span class=\"MBSIMGUI_ERROR\">", "</span>", f));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Deprecated, std::make_shared<bool>(true),
      make_shared<fmatvec::PrePostfixedStream>("<span class=\"MBSIMGUI_DEPRECATED\">", "</span>", f));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Status    , std::make_shared<bool>(true),
      make_shared<fmatvec::PrePostfixedStream>("", "", [this](const string &s){
        // call this function only every 0.25 sec
        if(getStatusTime().elapsed()<250) return;
        getStatusTime().restart();
        // print to status bar
        statusBar()->showMessage(QString::fromStdString(s));
      }));

    auto referencedFilesDialog = new QDialog(this);
    auto *layout = new QVBoxLayout;
    referencedFilesDialog->setLayout(layout);
    layout->addWidget(fileView);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    buttonBox->addButton(QDialogButtonBox::Close);
    layout->addWidget(buttonBox);
    connect(buttonBox, &QDialogButtonBox::rejected, referencedFilesDialog, &QDialog::hide);

    QMenu *fileMenu = new QMenu("File", menuBar());
    menuBar()->addMenu(fileMenu);

    QAction *action;
    action = fileMenu->addAction(QIcon::fromTheme("document-new"), "New", this, &MainWindow::newProject);
    action->setShortcut(QKeySequence::New);
    action->setStatusTip("New project");
    action = fileMenu->addAction(QIcon::fromTheme("document-open"), "Open ...", this, QOverload<>::of(&MainWindow::loadProject));
    action->setShortcut(QKeySequence::Open);
    action->setStatusTip("Open project");
    action = fileMenu->addAction(QIcon::fromTheme("document-save"), "Save", this, [=](){ saveProject(); for(size_t i=0; i<file.size(); i++) if(file[i]->getModified()) saveReferencedFile(i); });
    action->setShortcut(QKeySequence::Save);
    action->setStatusTip("Save project and all references");
    actionSaveProject = fileMenu->addAction(QIcon::fromTheme("document-save"), "Save project only", this, [=](){ this->saveProject(); });
    actionSaveProject->setStatusTip("Save project (but not the references)");
    action = fileMenu->addAction(QIcon::fromTheme("document-save-as"), "Save project as ...", this, &MainWindow::saveProjectAs);
    action->setShortcut(QKeySequence::SaveAs);
    action->setStatusTip("Save project as new file");

    fileMenu->addSeparator();

    action = fileMenu->addAction(QIcon::fromTheme("document-open"), "Show referenced files ...", referencedFilesDialog, &QDialog::show);
    action->setStatusTip("Show a list of all referenced files");
    action = fileMenu->addAction(QIcon::fromTheme("document-properties"), "Settings ...", this, &MainWindow::openOptionsMenu);
    action->setStatusTip(tr("Show settings menu"));

    fileMenu->addSeparator();

    for (auto & recentProjectFileAct : recentProjectFileActs) {
      recentProjectFileAct = new QAction(this);
      recentProjectFileAct->setVisible(false);
      connect(recentProjectFileAct, &QAction::triggered, this, &MainWindow::openRecentProjectFile);
    }
    for (auto & recentProjectFileAct : recentProjectFileActs)
      fileMenu->addAction(recentProjectFileAct);
    updateRecentProjectFileActions();

    fileMenu->addSeparator();

    action = fileMenu->addAction(QIcon::fromTheme("application-exit"), "E&xit", this, &MainWindow::close);
    action->setShortcut(QKeySequence::Quit);
    action->setStatusTip(tr("Exit the application"));

    // filter settings
    OpenMBVGUI::AbstractViewFilter::setFilterType(static_cast<OpenMBVGUI::AbstractViewFilter::FilterType>(settings.value("mainwindow/filter/type", 0).toInt()));
    OpenMBVGUI::AbstractViewFilter::setCaseSensitive(settings.value("mainwindow/filter/casesensitivity", false).toBool());
    // We cannot use the new Qt function pointer-based connection mechanism here since this seems not to work
    // on Windows if the signal and slot lives in different DLLs as it is for the following two connections.
    // Hence, we keep here the old macro base mechanism and use Q_OBJECT and moc for this class.
    connect(OpenMBVGUI::AbstractViewFilter::staticObject(), SIGNAL(optionsChanged()), this, SLOT(abstractViewFilterOptionsChanged()));

    elementView = new ElementView;
    elementView->setModel(new ElementTreeModel(this));
    elementView->setColumnWidth(0,250);
    elementView->setColumnWidth(1,200);
    elementView->hideColumn(1);
    elementViewFilter = new OpenMBVGUI::AbstractViewFilter(elementView, 0, 1);
    elementViewFilter->hide();
    // if new rows get insert update the item in the AbstractViewFilter
    connect(elementView->model(), &QAbstractItemModel::rowsInserted, [this](const QModelIndex &parent, int first, int last) {
      for(int r=first; r<=last; ++r)
        elementViewFilter->updateItem(elementView->model()->index(r,0,parent));
    });

    parameterView = new ParameterView;
    parameterView->setModel(new ParameterTreeModel(this));
    parameterView->setColumnWidth(0,150);
    parameterView->setColumnWidth(1,200);
    parameterViewFilter = new OpenMBVGUI::AbstractViewFilter(parameterView, 0, 2);
    parameterViewFilter->hide();

    auto editMenu = new QMenu("Edit", menuBar());
    action = editMenu->addAction(QIcon::fromTheme("document-properties"), "Edit", this, &MainWindow::edit);
    action->setShortcut(QKeySequence("Ctrl+E"));
    editMenu->addSeparator();
    actionUndo = editMenu->addAction(QIcon::fromTheme("edit-undo"), "Undo", this, &MainWindow::undo);
    actionUndo->setShortcut(QKeySequence::Undo);
    actionUndo->setDisabled(true);
    actionRedo = editMenu->addAction(QIcon::fromTheme("edit-redo"), "Redo", this, &MainWindow::redo);
    actionRedo->setShortcut(QKeySequence::Redo);
    actionRedo->setDisabled(true);
    editMenu->addSeparator();
    action = editMenu->addAction(QIcon::fromTheme("edit-copy"), "Copy", this, &MainWindow::copy);
    action->setShortcut(QKeySequence::Copy);
    action = editMenu->addAction(QIcon::fromTheme("edit-cut"), "Cut", this, &MainWindow::cut);
    action->setShortcut(QKeySequence::Cut);
    action = editMenu->addAction(QIcon::fromTheme("edit-paste"), "Paste", this, &MainWindow::paste);
    action->setShortcut(QKeySequence::Paste);
    editMenu->addSeparator();
    action = editMenu->addAction(QIcon::fromTheme("edit-delete"), "Remove", this, &MainWindow::remove);
    action->setShortcut(QKeySequence::Delete);
    editMenu->addSeparator();
    action = editMenu->addAction(QIcon::fromTheme("go-up"), "Move up", this, &MainWindow::moveUp);
    action->setShortcut(QKeySequence("Ctrl+Up"));
    action = editMenu->addAction(QIcon::fromTheme("go-down"), "Move down", this, &MainWindow::moveDown);
    action->setShortcut(QKeySequence("Ctrl+Down"));
    menuBar()->addMenu(editMenu);

    auto runMenu = new QMenu("Run", menuBar());
    menuBar()->addMenu(runMenu);

    auto sceneViewMenu = inlineOpenMBVMW->getSceneViewMenu();
    sceneViewMenu->removeAction(sceneViewMenu->findChild<QAction*>("MainWindow::sceneViewMenu::releaseCamera"));
    menuBar()->addMenu(sceneViewMenu);

    auto miscMenu = new QMenu("Misc", menuBar());
    menuBar()->addMenu(miscMenu);

    auto exportMenu = new QMenu("Data Export", menuBar());
    actionSaveDataAs = exportMenu->addAction("Export all data", this, &MainWindow::saveDataAs);
    actionSaveMBSimH5DataAs = exportMenu->addAction("Export MBSim data file", this, &MainWindow::saveMBSimH5DataAs);
    actionSaveOpenMBVDataAs = exportMenu->addAction("Export OpenMBV data", this, &MainWindow::saveOpenMBVDataAs);
    actionSaveStateVectorAs = exportMenu->addAction("Export state vector", this, &MainWindow::saveStateVectorAs);
    actionSaveStateTableAs = exportMenu->addAction("Export state table", this, &MainWindow::saveStateTableAs);
    actionSaveLinearSystemAnalysisAs = exportMenu->addAction("Export linear system analysis", this, &MainWindow::saveLinearSystemAnalysisAs);
    menuBar()->addMenu(exportMenu);

    auto *dockMenu=new QMenu("Docks", menuBar());
    menuBar()->addMenu(dockMenu);

    QMenu *toolMenu = new QMenu("Tools", menuBar());
    menuBar()->addMenu(toolMenu);

    QMenu *helpMenu = new QMenu("Help", menuBar());
    helpMenu->addAction(QIcon::fromTheme("help-contents"), "Contents", this, &MainWindow::help);
    helpMenu->addAction(QIcon::fromTheme("help-xml"), "XML Help", this, [=](){ this->xmlHelp(); });
    helpMenu->addAction(QIcon::fromTheme("help-about"), "About", this, &MainWindow::about);
    menuBar()->addMenu(helpMenu);

    QToolBar *runBar = addToolBar("Run Toolbar");
    toolMenu->addAction(runBar->toggleViewAction());
    runBar->setObjectName("toolbar/run");
    actionSimulate = runBar->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_MediaPlay)),"Start simulation");
    runMenu->addAction(actionSimulate);
    actionSimulate->setStatusTip(tr("Simulate the multibody system"));
    connect(actionSimulate,&QAction::triggered,this,&MainWindow::simulate);
    QAction *actionInterrupt = runBar->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_MediaStop)),"Interrupt simulation");
    runMenu->addAction(actionInterrupt);
    connect(actionInterrupt,&QAction::triggered,this,&MainWindow::interrupt);
    QAction *actionKill = runBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"kill.svg").string())),"Kill simulation");
    runMenu->addAction(actionKill);
    connect(actionKill,&QAction::triggered,this,&MainWindow::kill);
    actionOpenMBV = runBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"openmbv.svg").string())),"OpenMBV");
    runMenu->addAction(actionOpenMBV);
    connect(actionOpenMBV,&QAction::triggered,this,&MainWindow::openmbv);
    actionH5plotserie = runBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"h5plotserie.svg").string())),"H5plotserie");
    runMenu->addAction(actionH5plotserie);
    connect(actionH5plotserie,&QAction::triggered,this,&MainWindow::h5plotserie);
    actionLinearSystemAnalysis = runBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"eigenanalysis.svg").string())),"Linear system analysis");
    runMenu->addAction(actionLinearSystemAnalysis);
    connect(actionLinearSystemAnalysis,&QAction::triggered,this,&MainWindow::linearSystemAnalysis);
    actionStateTable = runBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"state_table.svg").string())),"Show state table");
    runMenu->addAction(actionStateTable);
    connect(actionStateTable,&QAction::triggered,this,&MainWindow::showStateTable);

    QToolBar *miscBar = addToolBar("Misc Toolbar");
    miscBar->setObjectName("toolbar/misc");
    toolMenu->addAction(miscBar->toggleViewAction());
    QAction *actionCreateFMU = miscBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"FMI_bare.svg").string())),"Create FMU");
    miscMenu->addAction(actionCreateFMU);
    connect(actionCreateFMU,&QAction::triggered,this,&MainWindow::createFMU);
    QAction *actionFem = miscBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"fbt.svg").string())),"Flexible body tool");
    miscMenu->addAction(actionFem);
    connect(actionFem,&QAction::triggered,this,&MainWindow::flexibleBodyTool);
    actionDebug = miscBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"debug.svg").string())),"Debug model");
    miscMenu->addAction(actionDebug);
    connect(actionDebug,&QAction::triggered,this,&MainWindow::debug);
    QAction *actionConvert = miscBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"convert.svg").string())),"Convert file");
    miscMenu->addAction(actionConvert);
    connect(actionConvert,&QAction::triggered,this,&MainWindow::convertDocument);

    QToolBar *sceneViewToolBar = inlineOpenMBVMW->getSceneViewToolBar();
    addToolBar(sceneViewToolBar);
    sceneViewToolBar->show();
    sceneViewToolBar->setObjectName("toolbar/sceneview");
    toolMenu->addAction(sceneViewToolBar->toggleViewAction());
    sceneViewToolBar->insertSeparator(sceneViewToolBar->actions()[0]);
    actionRefresh = new QAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_BrowserReload)),"Refresh scene view", this);
    sceneViewToolBar->insertAction(sceneViewToolBar->actions()[0], actionRefresh);

    sceneViewMenu->insertSeparator(sceneViewMenu->actions()[0]);
    sceneViewMenu->insertAction(sceneViewMenu->actions()[0], actionRefresh);
    connect(actionRefresh,&QAction::triggered,this,&MainWindow::refresh);

    fileView->setModel(new FileTreeModel(this));
    fileView->setColumnWidth(0,250);
    fileView->setColumnWidth(1,50);
    fileView->hideColumn(3);

    connect(elementView->selectionModel(), &QItemSelectionModel::currentRowChanged, this, &MainWindow::elementChanged);
    connect(elementView, &ElementView::pressed, this, &MainWindow::elementViewClicked);
    connect(parameterView, &ParameterView::pressed, this, &MainWindow::parameterViewClicked);

    QDockWidget *dockModelTree = new QDockWidget("Model Tree", this);
    dockMenu->addAction(dockModelTree->toggleViewAction());
    dockModelTree->setFeatures(dockModelTree->features() | QDockWidget::DockWidgetVerticalTitleBar);
    dockModelTree->setObjectName("dockWidget/mbs");
    addDockWidget(Qt::LeftDockWidgetArea,dockModelTree);
    QWidget *widget1 = new QWidget(dockModelTree);
    dockModelTree->setWidget(widget1);
    auto *widgetLayout1 = new QVBoxLayout(widget1);
    widgetLayout1->setContentsMargins(0,0,0,0);
    widget1->setLayout(widgetLayout1);
    widgetLayout1->addWidget(elementViewFilter);
    widgetLayout1->addWidget(elementView);

    QDockWidget *dockParameterTree = new QDockWidget("Parameter Tree (of selected object)", this);
    dockMenu->addAction(dockParameterTree->toggleViewAction());
    dockParameterTree->setFeatures(dockParameterTree->features() | QDockWidget::DockWidgetVerticalTitleBar);
    dockParameterTree->setObjectName("dockWidget/parameters");
    addDockWidget(Qt::LeftDockWidgetArea,dockParameterTree);
    QWidget *widget3 = new QWidget(dockParameterTree);
    dockParameterTree->setWidget(widget3);
    auto *widgetLayout3 = new QVBoxLayout(widget3);
    widgetLayout3->setContentsMargins(0,0,0,0);
    widget3->setLayout(widgetLayout3);
    widgetLayout3->addWidget(parameterViewFilter);
    widgetLayout3->addWidget(parameterView);

    QDockWidget *dockEchoArea = new QDockWidget("MBSim Echo Area", this);
    dockMenu->addAction(dockEchoArea->toggleViewAction());
    dockEchoArea->setFeatures(dockEchoArea->features() | QDockWidget::DockWidgetVerticalTitleBar);
    dockEchoArea->setObjectName("dockWidget/echoArea");
    addDockWidget(Qt::BottomDockWidgetArea, dockEchoArea);
    dockEchoArea->setWidget(echoView);

    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);
    auto *mainlayout = new QHBoxLayout;
    centralWidget->setLayout(mainlayout);
    mainlayout->addWidget(inlineOpenMBVMW);

    connect(&process,QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),this,&MainWindow::processFinished);
    connect(&process,&QProcess::readyReadStandardOutput,this,&MainWindow::updateEchoView);
    connect(&process,&QProcess::readyReadStandardError,this,&MainWindow::updateStatus);

    setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);

    if(arg.contains("--maximized"))
      showMaximized();

    QString projectFile;
    QRegExp filterProject(".+\\.mbsx");
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
        if(projectFile.isEmpty())
          projectFile = it;
        continue;
      }
    }

    // auto exit if everything is finished
    if(arg.contains("--autoExit")) {
      process.setProcessChannelMode(QProcess::ForwardedChannels);
      auto timer=new QTimer(this);
      connect(timer, &QTimer::timeout, [this, timer](){
        if(process.state()==QProcess::NotRunning) {
          timer->stop();
          if(!close())
            timer->start(100);
        }
      });
      timer->start(100);
    }

    if(projectFile.size())
      loadProject(QDir::current().absoluteFilePath(projectFile));
    else
      newProject();

    setAcceptDrops(true);

    connect(&autoSaveTimer, &QTimer::timeout, this, &MainWindow::autoSaveProject);
    autoSaveTimer.start(settings.value("mainwindow/options/autosaveinterval", 5).toInt()*60000);
    statusTime.start();

    setWindowIcon(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"mbsimgui.svg").string())));

    openOptionsMenu(true);
  }

  void MainWindow::autoSaveProject() {
    saveProject("./.Project.mbsx",false);
  }

  void MainWindow::processFinished(int exitCode, QProcess::ExitStatus exitStatus) {
    updateEchoView();
    if(currentTask==1) {
      QModelIndex index = elementView->selectionModel()->currentIndex();
      auto *model = static_cast<ElementTreeModel*>(elementView->model());
      auto *element=dynamic_cast<Element*>(model->getItem(index)->getItemData());
      if(element)
        highlightObject(element->getID());
    }
    else {
      if(exitStatus == QProcess::NormalExit) {
        QSettings settings;
        bool saveFinalStateVector = settings.value("mainwindow/options/savestatevector", false).toBool();
        if(settings.value("mainwindow/options/autoexport", false).toBool()) {
	  QString dir = settings.value("mainwindow/options/autoexportdir", "./").toString();
	  saveMBSimH5Data(dir+"/MBS.mbsh5");
	  saveOpenMBVXMLData(dir+"/MBS.ombvx");
	  saveOpenMBVH5Data(dir+"/MBS.ombvh5");
	  saveLinearSystemAnalysis(dir+"/eigenanalysis.mat");
	  saveInputTable(dir+"/inputtable.asc");
	  saveOutputTable(dir+"/outputtable.asc");
	  if(saveFinalStateVector)
	    saveStateVector(dir+"/statevector.asc");
	  saveStateTable(dir+"/statetable.asc");
        }
        actionSaveDataAs->setDisabled(false);
	actionSaveStateTableAs->setDisabled(false);
	actionStateTable->setDisabled(false);
	actionSaveMBSimH5DataAs->setDisabled(false);
	actionSaveOpenMBVDataAs->setDisabled(false);
	if(saveFinalStateVector)
	  actionSaveStateVectorAs->setDisabled(false);
	if(dynamic_cast<LinearSystemAnalyzer*>(project->getSolver())) {
	  actionSaveLinearSystemAnalysisAs->setDisabled(false);
	  actionLinearSystemAnalysis->setDisabled(false);
	}
      }
      else {
        setExitBad();
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
    arg.push_back((installPath/"share"/"mbsimgui"/"inlineopenmbv.ombvwst").string());
    arg.emplace_back("--hdf5RefreshDelta");
    arg.emplace_back("0");
    inlineOpenMBVMW = new OpenMBVGUI::MainWindow(arg, true);

    // We cannot use the new Qt function pointer-based connection mechanism here since this seems not to work
    // on Windows if the signal and slot lives in different DLLs as it is for the following two connections.
    // Hence, we keep here the old macro base mechanism and use Q_OBJECT and moc for this class.
    connect(inlineOpenMBVMW, SIGNAL(objectSelected(std::string, OpenMBVGUI::Object*)), this, SLOT(selectElement(const std::string&, OpenMBVGUI::Object*)));
    connect(inlineOpenMBVMW, SIGNAL(objectDoubleClicked(std::string, OpenMBVGUI::Object*)), this, SLOT(openElementEditor()));

    // bugfix version for bfs::copy_file which is at least buggy in boost 1.74
    auto bfs__copy_file = [](const bfs::path &src, const bfs::path &dst, bfs::copy_option options) {
      bfs::ifstream s(src, ios_base::binary);
      bfs::ofstream d(dst, ios_base::binary);
      char buffer[10240];
      while(true) {
        s.read(buffer, 10240);
        auto size=s.gcount();
        if(size==0)
          break;
        d.write(buffer, size);
      }
    };
    bfs__copy_file(installPath/"share"/"mbsimgui"/"MBS_tmp.ombvx",  uniqueTempDir/"MBS_tmp.ombvx",  bfs::copy_option::overwrite_if_exists);
    bfs__copy_file(installPath/"share"/"mbsimgui"/"MBS_tmp.ombvh5", uniqueTempDir/"MBS_tmp.ombvh5", bfs::copy_option::overwrite_if_exists);
    inlineOpenMBVMW->openFile(uniqueTempDir.generic_string()+"/MBS_tmp.ombvx");
    connect(inlineOpenMBVMW, &OpenMBVGUI::MainWindow::fileReloaded, [this](){
      if(callViewAllAfterFileReloaded)
        inlineOpenMBVMW->viewAllSlot();
      callViewAllAfterFileReloaded = false;
    });
  }

  MainWindow::~MainWindow() {
    process.waitForFinished(-1);
    if(process.exitStatus()!=QProcess::NormalExit || process.exitCode()!=0)
      setExitBad();
    if(lsa) delete lsa;
    if(st) delete st;
    centralWidget()->layout()->removeWidget(inlineOpenMBVMW);
    delete inlineOpenMBVMW;
    // use nothrow boost::filesystem functions to avoid exceptions in this dtor
    boost::system::error_code ec;
    bfs::remove_all(uniqueTempDir, ec);
    bfs::remove("./.Project.mbsx", ec);
    auto *pmodel = static_cast<ParameterTreeModel*>(parameterView->model());
    pmodel->removeRows(pmodel->index(0,0).row(), pmodel->rowCount(QModelIndex()), QModelIndex());
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    model->removeRows(model->index(0,0).row(), model->rowCount(QModelIndex()), QModelIndex());
    auto *fmodel = static_cast<FileTreeModel*>(fileView->model());
    fmodel->removeRows(fmodel->index(0,0).row(), fmodel->rowCount(QModelIndex()), QModelIndex());
    delete project;
    parser->release();
    serializer->release();
    basicSerializer->release();
  }

  void MainWindow::updateUndos() {
    auto *oldDoc = static_cast<DOMDocument*>(doc->cloneNode(true));
    oldDoc->setDocumentURI(doc->getDocumentURI());
    auto u = vector<DOMDocument*>(1+file.size());
    u[0] = oldDoc;
    for(int i=0; i<file.size();i++) {
      auto *oldDoc = static_cast<DOMDocument*>(file[i]->getXMLDocument()->cloneNode(true));
      oldDoc->setDocumentURI(file[i]->getXMLDocument()->getDocumentURI());
      u[i+1] = oldDoc;
    }
    undos.push_back(u);
    if(undos.size() > maxUndo)
      undos.pop_front();
    redos.clear();
    if(allowUndo) actionUndo->setEnabled(true);
    actionRedo->setDisabled(true);
  }
  
  bool MainWindow::maybeSave() {
    //bool referencedFileModified = false;
    vector<int> modifiedFiles;
    for(size_t i=0; i<file.size(); i++) {
      if(file[i]->getModified())
        modifiedFiles.push_back(i);
    }
    if(isWindowModified() or modifiedFiles.size()) {
      QString text;
      if(not modifiedFiles.size())
        text = "Project has been modified.";
      else if(not isWindowModified())
        text = "Referenced file has been modified.";
      else
        text = "Project and referenced file have been modified.";
      QMessageBox::StandardButton ret = QMessageBox::warning(this, "Application", text+"\nDo you want to save your changes?", QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
      if(ret == QMessageBox::Save) {
        if(isWindowModified()) {
          if(actionSaveProject->isEnabled())
            return saveProject();
          else
            return saveProjectAs();
        }
        for(size_t i=0; i<modifiedFiles.size(); i++)
          saveReferencedFile(modifiedFiles[i]);
      } 
      else if(ret == QMessageBox::Cancel) 
        return false;
    }
    return true;
  }

  void MainWindow::openOptionsMenu(bool justSetOptions) {
    QSettings settings;
    OptionsDialog menu(this);
    menu.setAutoSave(settings.value("mainwindow/options/autosave", false).toBool());
    menu.setAutoSaveInterval(settings.value("mainwindow/options/autosaveinterval", 5).toInt());
    menu.setAutoExport(settings.value("mainwindow/options/autoexport", false).toBool());
    menu.setAutoExportDir(settings.value("mainwindow/options/autoexportdir", "./").toString());
    menu.setSaveStateVector(settings.value("mainwindow/options/savestatevector", false).toBool());
    menu.setMaxUndo(settings.value("mainwindow/options/maxundo", 10).toInt());
    menu.setShowFilters(settings.value("mainwindow/options/showfilters", true).toBool());
    menu.setShowHiddenElements(settings.value("mainwindow/options/showhiddenelements", false).toBool());
    bool oldShowHiddenElement=menu.getShowHiddenElements();
    menu.setAutoRefresh(settings.value("mainwindow/options/autorefresh", true).toBool());
    menu.setStatusUpdate(settings.value("mainwindow/options/statusupdate", true).toBool());
    menu.setPlugins(settings.value("mainwindow/options/plugins", QString()).toString());
    menu.setBaseIndexForPlot(settings.value("mainwindow/options/baseindexforplot", 0).toInt());

#ifdef _WIN32
    QFile file(qgetenv("APPDATA")+"/mbsim-env/mbsimxml.modulepath");
#else
    QFile file(qgetenv("HOME")+"/.config/mbsim-env/mbsimxml.modulepath");
#endif
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    menu.setModulePath(file.readAll());
    file.close();

    auto oldPlugins=menu.getPlugins();

    int res = 1;
    if(!justSetOptions)
      res = menu.exec();
    if(res == 1) {
      settings.setValue("mainwindow/options/autosave"          , menu.getAutoSave());
      settings.setValue("mainwindow/options/autosaveinterval"  , menu.getAutoSaveInterval());
      settings.setValue("mainwindow/options/autoexport"        , menu.getAutoExport());
      settings.setValue("mainwindow/options/autoexportdir"     , menu.getAutoExportDir());
      settings.setValue("mainwindow/options/savestatevector"   , menu.getSaveStateVector());
      settings.setValue("mainwindow/options/maxundo"           , menu.getMaxUndo());
      settings.setValue("mainwindow/options/showfilters"       , menu.getShowFilters());
      settings.setValue("mainwindow/options/showhiddenelements", menu.getShowHiddenElements());
      settings.setValue("mainwindow/options/autorefresh"       , menu.getAutoRefresh());
      settings.setValue("mainwindow/options/statusupdate"      , menu.getStatusUpdate());
      settings.setValue("mainwindow/options/plugins"           , menu.getPlugins());
      settings.setValue("mainwindow/options/baseindexforplot"  , menu.getBaseIndexForPlot());

      file.open(QIODevice::WriteOnly | QIODevice::Text);
      file.write(menu.getModulePath().toUtf8());
      file.close();

      bool autoSave = menu.getAutoSave();
      int autoSaveInterval = menu.getAutoSaveInterval();
      if(not(menu.getSaveStateVector())) 
        actionSaveStateVectorAs->setDisabled(true);
      if(autoSave)
        autoSaveTimer.start(autoSaveInterval*60000);
      else
        autoSaveTimer.stop();
      maxUndo = menu.getMaxUndo();

      bool showFilters = menu.getShowFilters();
      elementViewFilter->setVisible(showFilters);
      parameterViewFilter->setVisible(showFilters);

      if(menu.getShowHiddenElements()!=oldShowHiddenElement)
        elementChanged(elementView->currentIndex()); // this update the parameters (calls Parameter::updateValue())

      autoRefresh = menu.getAutoRefresh();
      statusUpdate = menu.getStatusUpdate();

      if(oldPlugins!=menu.getPlugins())
        QMessageBox::information(this, "Program restart required!",
          "The MBSimGUI plugin search path has changed.\nThis needs a restart of MBSimGUI to take effect.");
    }
  }

  void MainWindow::highlightObject(const string &ID) {
    currentID = ID;
    inlineOpenMBVMW->highlightObject(ID);
  }

  void MainWindow::elementChanged(const QModelIndex &current) {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    auto *item = model->getItem(current)->getItemData();
    auto *embeditem = dynamic_cast<EmbedItemData*>(item);
    if(not embeditem) {
      auto *container = dynamic_cast<ContainerItemData*>(item);
      if(container) embeditem = container->getElement();
    }
    if(embeditem) {
      auto *pmodel = static_cast<ParameterTreeModel*>(parameterView->model());
      vector<EmbedItemData*> parents = embeditem->getEmbedItemParents();
      pmodel->removeRow(pmodel->index(0,0).row(), QModelIndex());
      if(!parents.empty()) {
        pmodel->createParameterItem(parents[0]->getParameters());
        for(size_t i=0; i<parents.size()-1; i++)
          pmodel->createParameterItem(parents[i+1]->getParameters(),parents[i]->getParameters()->getModelIndex());
        pmodel->createParameterItem(embeditem->getParameters(),parents[parents.size()-1]->getParameters()->getModelIndex());
      }
      else
        pmodel->createParameterItem(embeditem->getParameters());
      parameterView->expandAll();
      parameterView->scrollToBottom();
      auto *element = dynamic_cast<Element*>(item);
      if(element)
        highlightObject(element->getID());
      else
        highlightObject("");
    }
  }

  void MainWindow::elementViewClicked(const QModelIndex &current) {
    if(QApplication::mouseButtons()==Qt::RightButton) {
      TreeItemData *itemData = static_cast<ElementTreeModel*>(elementView->model())->getItem(current)->getItemData();
      QMenu *menu = itemData->createContextMenu();
      menu->exec(QCursor::pos());
      delete menu;
    }
  }

  void MainWindow::parameterViewClicked(const QModelIndex &current) {
    if(QApplication::mouseButtons()==Qt::RightButton) {
      auto *item = dynamic_cast<ParameterItem*>(static_cast<ParameterTreeModel*>(parameterView->model())->getItem(current)->getItemData());
      QMenu *menu = item->createContextMenu();
      menu->exec(QCursor::pos());
      delete menu;
    }
  }

  void MainWindow::newProject() {
    if(maybeSave()) {
      undos.clear();
      actionUndo->setDisabled(true);
      actionRedo->setDisabled(true);
      elementBuffer.first = nullptr;
      parameterBuffer.first = nullptr;
      setWindowModified(false);
      actionOpenMBV->setDisabled(true);
      actionH5plotserie->setDisabled(true);
      actionLinearSystemAnalysis->setDisabled(true);
      actionStateTable->setDisabled(true);
      actionSaveDataAs->setDisabled(true);
      actionSaveMBSimH5DataAs->setDisabled(true);
      actionSaveOpenMBVDataAs->setDisabled(true);
      actionSaveStateVectorAs->setDisabled(true);
      actionSaveStateTableAs->setDisabled(true);
      actionSaveLinearSystemAnalysisAs->setDisabled(true);
      actionSaveProject->setDisabled(true);
      projectFile="";
      setWindowTitle("Project.mbsx[*]");

      auto *pmodel = static_cast<ParameterTreeModel*>(parameterView->model());
      pmodel->removeRows(pmodel->index(0,0).row(), pmodel->rowCount(QModelIndex()), QModelIndex());

      auto *model = static_cast<ElementTreeModel*>(elementView->model());
      model->removeRows(model->index(0,0).row(), model->rowCount(QModelIndex()), QModelIndex());

      auto *fmodel = static_cast<FileTreeModel*>(fileView->model());
      fmodel->removeRows(fmodel->index(0,0).row(), fmodel->rowCount(QModelIndex()), QModelIndex());

      delete project;

      file.clear();
      idMap.clear();
      IDcounter = 0;

      doc = impl->createDocument();
      doc->setDocumentURI(X()%QUrl::fromLocalFile(QDir::currentPath()+"/Project.mbsx").toString().toStdString());

      project = new Project;
      project->createXMLElement(doc);

      model->createProjectItem(project);

      elementView->selectionModel()->setCurrentIndex(project->getModelIndex(), QItemSelectionModel::ClearAndSelect);

      callViewAllAfterFileReloaded = true;
      refresh();
    }
  }

  void MainWindow::loadProject(const QString &fileName) {
    if(QFile::exists(fileName)) {
      undos.clear();
      actionUndo->setDisabled(true);
      actionRedo->setDisabled(true);
      elementBuffer.first = nullptr;
      parameterBuffer.first = nullptr;
      setWindowModified(false);
      actionOpenMBV->setDisabled(true);
      actionH5plotserie->setDisabled(true);
      actionLinearSystemAnalysis->setDisabled(true);
      actionStateTable->setDisabled(true);
      actionSaveDataAs->setDisabled(true);
      actionSaveMBSimH5DataAs->setDisabled(true);
      actionSaveOpenMBVDataAs->setDisabled(true);
      actionSaveStateVectorAs->setDisabled(true);
      actionSaveStateTableAs->setDisabled(true);
      actionSaveLinearSystemAnalysisAs->setDisabled(true);
      actionSaveProject->setDisabled(false);
      projectFile = QDir::current().relativeFilePath(fileName);
      setWindowTitle(projectFile+"[*]");
      setCurrentProjectFile(fileName);

      try { 
        doc = parser->parseURI(X()%fileName.toStdString());
        DOMParser::handleCDATA(doc->getDocumentElement());
      }
      catch(const std::exception &ex) {
        mw->setExitBad();
        cerr << ex.what() << endl;
        return;
      }
      catch(...) {
        mw->setExitBad();
        cerr << "Unknown exception." << endl;
        return;
      }

      auto *pmodel = static_cast<ParameterTreeModel*>(parameterView->model());
      pmodel->removeRows(pmodel->index(0,0).row(), pmodel->rowCount(QModelIndex()), QModelIndex());

      auto *model = static_cast<ElementTreeModel*>(elementView->model());
      model->removeRows(model->index(0,0).row(), model->rowCount(QModelIndex()), QModelIndex());

      auto *fmodel = static_cast<FileTreeModel*>(fileView->model());
      fmodel->removeRows(fmodel->index(0,0).row(), fmodel->rowCount(QModelIndex()), QModelIndex());

      delete project;

      file.clear();
      idMap.clear();
      IDcounter = 0;

      project=Embed<Project>::create(doc->getDocumentElement(),nullptr);
      project->create();

      model->createProjectItem(project);

      elementView->selectionModel()->setCurrentIndex(project->getModelIndex(), QItemSelectionModel::ClearAndSelect);

      callViewAllAfterFileReloaded = true;
      refresh();
    }
    else
      QMessageBox::warning(this, "Project load", "Project file does not exist.");
  }

  void MainWindow::loadProject() {
    if(maybeSave()) {
      QString file=QFileDialog::getOpenFileName(this, "Open MBSim file", QFileInfo(getProjectFilePath()).absolutePath(), "MBSim files (*.mbsx);;XML files (*.xml);;All files (*.*)");
      if(file.startsWith("//"))
        file.replace('/','\\'); // xerces-c is not able to parse files from network shares that begin with "//"
      if(not file.isEmpty())
        loadProject(file);
    }
  }

  bool MainWindow::saveProjectAs() {
    QString file=QFileDialog::getSaveFileName(this, "Save MBSim file", getProjectFilePath(), "MBSim files (*.mbsx)");
    if(not(file.isEmpty())) {
      file = file.endsWith(".mbsx")?file:file+".mbsx";
      doc->setDocumentURI(X()%QUrl::fromLocalFile(file).toString().toStdString());
      projectFile = QDir::current().relativeFilePath(file);
      setCurrentProjectFile(file);
      setWindowTitle(projectFile+"[*]");
      actionSaveProject->setDisabled(false);
      return saveProject();
    }
    return false;
  }

  bool MainWindow::saveProject(const QString &fileName, bool modifyStatus) {
    try {
      serializer->writeToURI(doc, X()%(fileName.isEmpty()?projectFile.toStdString():fileName.toStdString()));
      if(modifyStatus) setWindowModified(false);
      return true;
    }
    catch(const std::exception &ex) {
      mw->setExitBad();
      cerr << ex.what() << endl;
    }
    catch(const DOMException &ex) {
      mw->setExitBad();
      cerr << X()%ex.getMessage() << endl;
    }
    catch(...) {
      mw->setExitBad();
      cerr << "Unknown exception." << endl;
    }
    return false;
  }

  void MainWindow::selectSolver(Solver *solver) {
    updateUndos();
    setWindowModified(true);
    DOMElement *ele = nullptr;
    auto *parameterFileItem = project->getSolver()->getParameterFileItem();
    DOMElement *embed = project->getSolver()->getEmbedXMLElement();
    if(parameterFileItem)
      ele = parameterFileItem->getXMLElement();
    else if(embed)
      ele = MBXMLUtils::E(embed)->getFirstElementChildNamed(MBXMLUtils::PV%"Parameter");
    QModelIndex pindex = project->getSolver()->getParameters()->getModelIndex();
    static_cast<ParameterTreeModel*>(parameterView->model())->removeRow(pindex.row(), pindex.parent());
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    model->removeRow(project->getSolver()->getModelIndex().row(), project->getModelIndex());
    if(project->getSolver()->getFileItem())
      E(embed)->removeAttribute("href");
    else
      project->getSolver()->removeXMLElement(false);
    project->setSolver(solver);
    model->createSolverItem(solver,project->getModelIndex());
    solver->createXMLElement(embed?embed:project->getXMLElement());
    solver->setEmbedXMLElement(embed);
    elementView->selectionModel()->setCurrentIndex(solver->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(ele) {
      solver->setParameterFileItem(parameterFileItem);
      solver->createParameters();
    }
    openElementEditor(false);
  }

  // Create an new eval and fills its context with all parameters/imports which may influence item.
  // The context contains also all counterName variables where the count is set to 0 or 1 for 0-based or 1-based evaluators, respectively
  // Also a list of all parameter levels which may influence item is returned. This list contains for each parameter level:
  // - the XML representation of the parameters (paramEle)
  // - the counterName which is none empty if this parameter has a Array/Pattern with a counterName, else couterName is empty
  // - the unevaluated count string of the Array/Pattern
  // - the unevaluated onlyif string of the Array/Pattern
  vector<MainWindow::ParameterLevel> MainWindow::updateParameters(EmbedItemData *item, bool exceptLatestParameter) {
    // get evaluator (octave, python, ...)
    string evalName="octave"; // default evaluator
    if(project)
      evalName = project->getEvaluator();
    else {
      DOMElement *root = doc->getDocumentElement();
      DOMElement *evaluator;
      if(E(root)->getTagName()==PV%"Embed") {
        auto r=root->getFirstElementChild();
        if(E(r)->getTagName()==PV%"Parameter")
          r=r->getNextElementSibling();
        evaluator=E(r)->getFirstElementChildNamed(PV%"evaluator");
      }
      else
        evaluator=E(root)->getFirstElementChildNamed(PV%"evaluator");
      if(evaluator)
        evalName=X()%E(evaluator)->getFirstTextChild()->getData();
    }
    eval=Eval::createEvaluator(evalName, &dependencies);

    vector<ParameterLevel> parameterLevels;
    if(item) {
      vector<EmbedItemData*> parents = item->getEmbedItemParents();
      parents.emplace_back(item); // add the item itself to parents (to avoid code duplication for parents and item)
      // add parameters from top parameter level to buttom parameter level
      for(auto & parent : parents) {
        // all parameters
        shared_ptr<DOMDocument> doc=mbxmlparser->createDocument();
        doc->setDocumentURI(this->doc->getDocumentURI());
        DOMElement *eleP = D(doc)->createElement(PV%"Parameter");
        doc->insertBefore(eleP,nullptr);
        int skipLast = (parent==item && exceptLatestParameter) ? 1 : 0;
        for(size_t j=0; j<parent->getNumberOfParameters() - skipLast; j++) {
          DOMNode *node = doc->importNode(parent->getParameter(j)->getXMLElement(),true);
          eleP->insertBefore(node,nullptr);
          boost::filesystem::path orgFileName=E(parent->getParameter(j)->getXMLElement())->getOriginalFilename();
          DOMProcessingInstruction *filenamePI=node->getOwnerDocument()->createProcessingInstruction(X()%"OriginalFilename",
              X()%orgFileName.string());
          node->insertBefore(filenamePI, node->getFirstChild());
        }
        try {
          D(doc)->validate();
          DOMElement *ele = doc->getDocumentElement();
          parameterLevels.emplace_back(doc, ele);

          // the embed count if its a embed
          string counterName = parent->getEmbedXMLElement()?E(parent->getEmbedXMLElement())->getAttribute("counterName"):"";
          if(not counterName.empty()) {
            auto count1=eval->create(1.0);
            eval->convertIndex(count1, false);
            eval->addParam(counterName, count1);
            eval->addParam(counterName+"_count", eval->create(1.0));
            parameterLevels.back().counterName = counterName;
            parameterLevels.back().countStr = E(parent->getEmbedXMLElement())->getAttribute("count");
            parameterLevels.back().onlyIfStr = E(parent->getEmbedXMLElement())->getAttribute("onlyIf");
          }

          eval->addParamSet(ele);
        }
        catch(const std::exception &error) {
          mw->setExitBad();
          cerr << string("An exception occurred in updateParameters: ") + error.what() << endl;
        }
        catch(...) {
          mw->setExitBad();
          cerr << "An unknown exception occurred in updateParameters." << endl;
        }
      }
    }
    return parameterLevels;
  }

  pair<vector<string>, map<vector<int>, MBXMLUtils::Eval::Value>> MainWindow::evaluateForAllArrayPattern(
    const vector<ParameterLevel> &parameterLevels, const std::string &code, xercesc::DOMElement *e) {
    #define CATCH(msg) \
      catch(DOMEvalException &e) { \
        mw->setExitBad(); \
        std::cerr << msg << ": " << e.getMessage() << std::endl; \
      } \
      catch(...) { \
        mw->setExitBad(); \
        std::cerr << msg << ": Unknwon error" << std::endl; \
      }

    vector<string> counterNames;
    for(auto &x : parameterLevels)
      if(!x.counterName.empty())
        counterNames.emplace_back(x.counterName);
    map<vector<int>, Eval::Value> values;

    // evaluate the code for all possible counter values (recursively)
    // save the evaluated counter name in a set to add only unique names
    set<string> evaluatedNames;
    vector<int> levels(counterNames.size());
    function<void(vector<MainWindow::ParameterLevel>::const_iterator, vector<MainWindow::ParameterLevel>::const_iterator,
                  int level, vector<int> levels)> walk;
    walk=[&code, e, &walk, &evaluatedNames, &values](vector<MainWindow::ParameterLevel>::const_iterator start,
                                                     vector<MainWindow::ParameterLevel>::const_iterator end,
                                                     int level, vector<int> levels) {
      Eval::Value count = mw->eval->create(1.0);
      if(!start->counterName.empty())
        try { count = mw->eval->stringToValue(start->countStr, e); }
        CATCH("Cannot evaluate Array/Pattern 'count' variable");
      int countInt = 1;
      try { countInt = mw->eval->cast<int>(count); }
      CATCH("The Array/Patttern 'count' variable is not of type int");
      for(int counterValue1Based=1; counterValue1Based<=countInt; ++counterValue1Based) {
        NewParamLevel newParamLevel(mw->eval);

        if(!start->counterName.empty()) {
          // add counter parameter
          auto counterValue=mw->eval->create(static_cast<double>(counterValue1Based));
          mw->eval->convertIndex(counterValue, false);
          mw->eval->addParam(start->counterName, counterValue);
          mw->eval->addParam(start->counterName+"_count", count);
          levels[level] = mw->eval->cast<int>(counterValue);
          // add local parameters
          mw->eval->addParamSet(start->paramEle);
          // skip if onlyIf evaluates to false
          if(!start->onlyIfStr.empty()) {
            try {
              bool onlyIf = mw->eval->cast<int>(mw->eval->stringToValue(code,e));
              if(!onlyIf)
                continue;
            }
            CATCH("Failed to evaluate Array/Pattern 'onlyif' variable, not skipping this counter");
          }
        }
        else
          mw->eval->addParamSet(start->paramEle);

        auto startNext=start;
        startNext++;
        if(startNext!=end)
          walk(startNext, end, !start->counterName.empty() ? level+1 : level, levels);
        else {
          try {
            auto value = mw->eval->stringToValue(code,e,false);
            evaluatedNames.insert(mw->eval->cast<string>(value));
            values.emplace(levels, value);
          }
          CATCH("Failed to evaluate parameter");
        }
      }
    };
    walk(parameterLevels.begin(), parameterLevels.end(), 0, levels);
    return {counterNames, values};
  }

  void MainWindow::saveDataAs() {
    QString dir = QFileDialog::getExistingDirectory (this, "Export simulation data", getProjectPath());
    if(dir != "") {
      QDir directory(dir);
      QMessageBox::StandardButton ret = QMessageBox::Ok;
      if(directory.count()>2)
        ret = QMessageBox::warning(this, tr("Application"), tr("Directory not empty. Overwrite existing files?"), QMessageBox::Ok | QMessageBox::Cancel);
      if(ret == QMessageBox::Ok) {
        QSettings settings;
        saveMBSimH5Data(dir+"/MBS.mbsh5");
        saveOpenMBVXMLData(dir+"/MBS.ombvx");
        saveOpenMBVH5Data(dir+"/MBS.ombvh5");
	saveLinearSystemAnalysis(dir+"/linear_system_analysis.h5");
	saveInputTable(dir+"/inputtable.asc");
	saveOutputTable(dir+"/outputtable.asc");
        if(settings.value("mainwindow/options/savestatevector", false).toBool())
          saveStateVector(dir+"/statevector.asc");
        saveStateTable(dir+"/statetable.asc");
      }
    }
  }

  void MainWindow::saveMBSimH5DataAs() {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = model->index(0,0);
    QString file=QFileDialog::getSaveFileName(this, "Export MBSim H5 file", getProjectDir().absoluteFilePath(model->getItem(index)->getItemData()->getName()+".mbsh5"), "H5 files (*.mbsh5)");
    if(file!="") {
      saveMBSimH5Data(file);
    }
  }

  void MainWindow::saveMBSimH5Data(const QString &file) {
    if(QFile::exists(file))
      QFile::remove(file);
    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/"+project->getDynamicSystemSolver()->getName()+".mbsh5",file);
  }

  void MainWindow::saveOpenMBVDataAs() {
    QString dir = QFileDialog::getExistingDirectory(this, "Export OpenMBV data", getProjectPath());
    if(dir != "") {
      QDir directory(dir);
      QMessageBox::StandardButton ret = QMessageBox::Ok;
      if(directory.count()>2)
        ret = QMessageBox::warning(this, tr("Application"), tr("Directory not empty. Overwrite existing files?"), QMessageBox::Ok | QMessageBox::Cancel);
      if(ret == QMessageBox::Ok) {
        saveOpenMBVXMLData(dir+"/MBS.ombvx");
        saveOpenMBVH5Data(dir+"/MBS.ombvh5");
      }
    }
  }

  void MainWindow::saveOpenMBVXMLData(const QString &file) {
    if(QFile::exists(file))
      QFile::remove(file);
    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/"+project->getDynamicSystemSolver()->getName()+".ombvx",file);
  }

  void MainWindow::saveOpenMBVH5Data(const QString &file) {
    if(QFile::exists(file))
      QFile::remove(file);
    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/"+project->getDynamicSystemSolver()->getName()+".ombvh5",file);
  }

  void MainWindow::saveStateVectorAs() {
    QString file=QFileDialog::getSaveFileName(this, "Export state vector", getProjectDir().absoluteFilePath("statevector.asc"), "ASCII files (*.asc)");
    if(file!="") {
      saveStateVector(file);
    }
  }

  void MainWindow::saveStateVector(const QString &file) {
    if(QFile::exists(file))
      QFile::remove(file);
    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/statevector.asc",file);
  }

  void MainWindow::saveStateTableAs() {
    QString file=QFileDialog::getSaveFileName(this, "Export state table", getProjectDir().absoluteFilePath("statetable.asc"), "ASCII files (*.asc)");
    if(file!="") {
      saveStateTable(file);
    }
  }

  void MainWindow::saveStateTable(const QString &file) {
    if(QFile::exists(file))
      QFile::remove(file);
    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/statetable.asc",file);
  }

  void MainWindow::saveLinearSystemAnalysisAs() {
    QString dir = QFileDialog::getExistingDirectory(this, "Export linear system analysis", getProjectPath());
    if(dir != "") {
      QDir directory(dir);
      QMessageBox::StandardButton ret = QMessageBox::Ok;
      if(directory.count()>2)
        ret = QMessageBox::warning(this, tr("Application"), tr("Directory not empty. Overwrite existing files?"), QMessageBox::Ok | QMessageBox::Cancel);
      if(ret == QMessageBox::Ok) {
	saveLinearSystemAnalysis(dir+"/linear_system_analysis.h5");
	saveInputTable(dir+"/inputtable.asc");
	saveOutputTable(dir+"/outputtable.asc");
      }
    }
  }

  void MainWindow::saveInputTable(const QString &file) {
    if(QFile::exists(file))
      QFile::remove(file);
    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/inputtable.asc",file);
  }

  void MainWindow::saveOutputTable(const QString &file) {
    if(QFile::exists(file))
      QFile::remove(file);
    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/outputtable.asc",file);
  }

  void MainWindow::saveLinearSystemAnalysis(const QString &file) {
    if(QFile::exists(file))
      QFile::remove(file);
    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/linear_system_analysis.h5",file);
  }

  void MainWindow::mbsimxml(int task) {
    currentTask = task;

    shared_ptr<DOMDocument> doc=mbxmlparser->createDocument();
    doc->setDocumentURI(this->doc->getDocumentURI());
    auto *newDocElement = static_cast<DOMElement*>(doc->importNode(this->doc->getDocumentElement(), true));
    doc->insertBefore(newDocElement, nullptr);
    project->processIDAndHref(newDocElement);

    QString uniqueTempDir_ = QString::fromStdString(uniqueTempDir.generic_string());
    QString projectFile;

    projectFile=uniqueTempDir_+"/Project.flat.mbsx";

    actionSimulate->setDisabled(true);
    actionRefresh->setDisabled(true);
    if(task==0) {
      actionSaveDataAs->setDisabled(true);
      actionSaveMBSimH5DataAs->setDisabled(true);
      actionSaveOpenMBVDataAs->setDisabled(true);
      actionSaveStateVectorAs->setDisabled(true);
      actionSaveStateTableAs->setDisabled(true);
      actionSaveLinearSystemAnalysisAs->setDisabled(true);
      actionOpenMBV->setDisabled(false);
      actionH5plotserie->setDisabled(false);
      actionLinearSystemAnalysis->setDisabled(true);
    }

    echoView->clearOutput();
    echoView->showXMLCode(false);
    DOMElement *root;
    QString errorText;

    *debugStreamFlag=echoView->debugEnabled();

    try {
      fmatvec::Atom::msgStatic(fmatvec::Atom::Info)<<"Validate "<<D(doc)->getDocumentFilename().string()<<endl;
      D(doc)->validate();
      root = doc->getDocumentElement();
      vector<boost::filesystem::path> dependencies;
      shared_ptr<Eval> eval=Eval::createEvaluator(project->getEvaluator(), &dependencies);
      Preprocess::preprocess(mbxmlparser, eval, dependencies, root);
    }
    catch(exception &ex) {
      mw->setExitBad();
      errorText = ex.what();
    }
    catch(...) {
      mw->setExitBad();
      errorText = "Unknown error";
    }
    if(not errorText.isEmpty()) {
      echoView->addOutputText("<span class=\"MBSIMGUI_ERROR\">"+errorText+"</span>");
      echoView->updateOutput(true);
      actionSimulate->setDisabled(false);
      actionRefresh->setDisabled(false);
      actionDebug->setDisabled(false);
      statusBar()->showMessage(tr("Ready"));
      auto out=errorText;
      cerr<<out.remove(QRegExp("<[^>]*>")).toStdString()<<endl;
      return;
    }
    echoView->updateOutput(true);
    // adapt the evaluator in the dom
    DOMElement *evaluator=E(root)->getFirstElementChildNamed(PV%"evaluator");
    if(evaluator)
      E(evaluator)->getFirstTextChild()->setData(X()%"xmlflat");
    else {
      evaluator=D(doc)->createElement(PV%"evaluator");
      evaluator->appendChild(doc->createTextNode(X()%"xmlflat"));
      root->insertBefore(evaluator, root->getFirstChild());
    }
    E(root)->setOriginalFilename();
    basicSerializer->writeToURI(doc.get(), X()%projectFile.toStdString());
    QStringList arg;
    QSettings settings;
    if(currentTask==1)
      arg.append("--stopafterfirststep");
    else {
      arg.append("--savestatetable");
      if(settings.value("mainwindow/options/savestatevector", false).toBool())
        arg.append("--savefinalstatevector");
      arg.append("--baseindexforplot");
      arg.append(settings.value("mainwindow/options/baseindexforplot", "0").toString());
    }

    // we print everything except status messages to stdout
    arg.append("--stdout"); arg.append(R"#(info~<span class="MBSIMGUI_INFO">~</span>)#");
    arg.append("--stdout"); arg.append(R"#(warn~<span class="MBSIMGUI_WARN">~</span>)#");
    if(*debugStreamFlag) {
      arg.append("--stdout"); arg.append(R"#(debug~<span class="MBSIMGUI_DEBUG">~</span>)#");
    }
    arg.append("--stdout"); arg.append(R"#(error~<span class="MBSIMGUI_ERROR">~</span>)#");
    arg.append("--stdout"); arg.append(R"#(depr~<span class="MBSIMGUI_DEPRECATED">~</span>)#");
    // status message go to stderr
    arg.append("--stderr"); arg.append("status~~\n");

    // we change the current directory (see below) hence we need to add the current dir as modulePath
    arg.append("--modulePath");
    arg.append(QDir::currentPath());

    arg.append(projectFile);
    process.setWorkingDirectory(uniqueTempDir_);
    process.start(QString::fromStdString((installPath/"bin"/"mbsimflatxml").string()), arg);
  }

  void MainWindow::simulate() {
    statusBar()->showMessage(tr("Simulate"));
    mbsimxml(0);
  }

  void MainWindow::refresh() {
    statusBar()->showMessage(tr("Refresh"));
    mbsimxml(1);
  }

  void MainWindow::openmbv() {
    QString name = QString::fromStdString(uniqueTempDir.generic_string())+"/"+project->getDynamicSystemSolver()->getName()+".ombvx";
    if(QFile::exists(name))
      QProcess::startDetached(QString::fromStdString((installPath/"bin"/"openmbv").string()), QStringList(name));
  }

  void MainWindow::h5plotserie() {
    QString name = QString::fromStdString(uniqueTempDir.generic_string())+"/"+project->getDynamicSystemSolver()->getName()+".mbsh5";
    if(QFile::exists(name))
      QProcess::startDetached(QString::fromStdString((installPath/"bin"/"h5plotserie").string()), QStringList(name));
  }

  void MainWindow::linearSystemAnalysis() {
    QString file1 = QString::fromStdString(uniqueTempDir.generic_string())+"/linear_system_analysis.h5";
    QString file2 = QString::fromStdString(uniqueTempDir.generic_string())+"/statetable.asc";
    if(QFile::exists(file1) and QFile::exists(file2)) {
      if(not lsa) {
	lsa = new LinearSystemAnalysisDialog(this);
	connect(&process,QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),lsa,&LinearSystemAnalysisDialog::updateWidget);
      }
      lsa->show();
    }
  }

  void MainWindow::showStateTable() {
    QString file = QString::fromStdString(uniqueTempDir.generic_string())+"/statetable.asc";
    if(QFile::exists(file)) {
      if(not st) {
	st = new StateTableDialog(this);
	connect(&process,QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),st,&StateTableDialog::updateWidget);
      }
      st->show();
    }
  }

  void MainWindow::debug() {
    currentTask = 0;
    shared_ptr<DOMDocument> doc=mbxmlparser->createDocument();
    doc->setDocumentURI(this->doc->getDocumentURI());
    auto *newDocElement = static_cast<DOMElement*>(doc->importNode(this->doc->getDocumentElement(), true));
    doc->insertBefore(newDocElement, nullptr);
    project->processIDAndHref(newDocElement);
    QString uniqueTempDir_ = QString::fromStdString(uniqueTempDir.generic_string());
    QString projectFile = uniqueTempDir_+"/Project.mbsx";
    serializer->writeToURI(doc.get(), X()%projectFile.toStdString());
    QStringList arg;
    arg.append("--stopafterfirststep");

    // we print everything except status messages to stdout
    arg.append("--stdout"); arg.append(R"#(info~<span class="MBSIMGUI_INFO">~</span>)#");
    arg.append("--stdout"); arg.append(R"#(warn~<span class="MBSIMGUI_WARN">~</span>)#");
    if(*debugStreamFlag) {
      arg.append("--stdout"); arg.append(R"#(debug~<span class="MBSIMGUI_DEBUG">~</span>)#");
    }
    arg.append("--stdout"); arg.append(R"#(error~<span class="MBSIMGUI_ERROR">~</span>)#");
    arg.append("--stdout"); arg.append(R"#(depr~<span class="MBSIMGUI_DEPRECATED">~</span>)#");
    // status message go to stderr
    arg.append("--stderr"); arg.append("status~~\n");

    // we change the current directory (see below) hence we need to add the current dir as modulePath
    arg.append("--modulePath");
    arg.append(QDir::currentPath());

    arg.append(projectFile);
    echoView->clearOutput();
    echoView->showXMLCode(true);
    process.setWorkingDirectory(uniqueTempDir_);
    process.start(QString::fromStdString((installPath/"bin"/"mbsimxml").string()), arg);
    statusBar()->showMessage(tr("Debug model"));
  }

  void MainWindow::selectElement(const string& ID, OpenMBVGUI::Object *obj) {
    if(!obj)
      return;
    auto id=ID;
    // if no ID is given (obj->getObject() has no ID set) and its a RigidBody inside of a CompoundRigidBody then use the ID of the parent CompoundRigidBody.
    auto o=obj->getObject();
    while(id.empty()) {
      auto rb=dynamic_pointer_cast<OpenMBV::RigidBody>(o);
      if(!rb)
        break;
      auto crb=rb->getCompound().lock();
      if(!crb)
        break;
      id=crb->getID();
      o=crb;
    }
    Element *element = idMap[id];
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    if(element) elementView->selectionModel()->setCurrentIndex(model->findItem(element,project->getDynamicSystemSolver()->getModelIndex()),QItemSelectionModel::ClearAndSelect);
  }

  void MainWindow::abstractViewFilterOptionsChanged() {
    QSettings settings;
    settings.setValue("mainwindow/filter/type", static_cast<int>(OpenMBVGUI::AbstractViewFilter::getFilterType()));
    settings.setValue("mainwindow/filter/casesensitivity", OpenMBVGUI::AbstractViewFilter::getCaseSensitive());
  }

  void MainWindow::help() {
    QMessageBox::information(this, tr("MBSimGUI - Help"), tr("<p>Please visit <a href=\"https://www.mbsim-env.de\">MBSim-Environment</a> for documentation.</p>"));
  }

  void MainWindow::xmlHelp(const QString &url) {
    QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString((installPath/"share"/"mbxmlutils"/"doc"/"http___www_mbsim-env_de_MBSimXML"/"mbsimxml.html").string())));
  }

  void MainWindow::about() {
     QMessageBox::about(this, tr("About MBSimGUI"), (tr("<h1>MBSimGUI %1</h1><p>MBSimGUI is a graphical user interface for the multibody simulation software MBSim.</p>").arg(VERSION)
           + tr("<p>See <a href=\"https://www.mbsim-env.de\">MBSim-Environment</a> for more information.</p>"
             "<h2>Copyright</h2>"
             "<p>Copyright &copy; Martin Foerg <tt>&lt;martin.o.foerg@googlemail.com&gt;</tt><p/>"
             "<p>Licensed under the Lesser General Public License (see file COPYING).</p>"
             "<p>This is free software; see the source for copying conditions. There is NO warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.</p>"
             "<h2>Dependencies:</h2>"
             "<pre>"
#include "../NOTICE"
             "</pre>"
             "<p>A special thanks to all authors of these projects.</p>"
     )));
  }

  void MainWindow::rebuildTree() {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    auto *pmodel = static_cast<ParameterTreeModel*>(parameterView->model());
    auto *fmodel = static_cast<FileTreeModel*>(fileView->model());

    ElementView::Node root("Root",false,false);
    elementView->save(model->index(0,0),root);

    pmodel->removeRows(pmodel->index(0,0).row(), pmodel->rowCount(QModelIndex()), QModelIndex());
    model->removeRows(model->index(0,0).row(), model->rowCount(QModelIndex()), QModelIndex());
    fmodel->removeRows(fmodel->index(0,0).row(), fmodel->rowCount(QModelIndex()), QModelIndex());

    delete project;

    idMap.clear();
    IDcounter = 0;

    project=Embed<Project>::create(doc->getDocumentElement(),nullptr);
    project->create();

    model->createProjectItem(project);

    elementView->restore(model->index(0,0),root);
  }

  void MainWindow::edit() {
    if(elementView->hasFocus())
      openElementEditor();
    else if(parameterView->hasFocus())
      openParameterEditor();
  }

  void MainWindow::undo() {
    elementBuffer.first = nullptr;
    parameterBuffer.first = nullptr;
    setWindowModified(true);
    auto r = vector<DOMDocument*>(1+file.size());
    r[0] = doc;
    for(int i=0; i<file.size(); i++)
      r[i+1] = file[i]->getXMLDocument();
    redos.push_back(r);
    auto u = undos.back();
    doc = u[0];
    file.resize(u.size()-1);
    for(int i=0; i<u.size()-1; i++)
      file[i] = new FileItemData(u[i+1]);
    undos.pop_back();
    rebuildTree();
    if(getAutoRefresh()) refresh();
    actionUndo->setDisabled(undos.empty());
    actionRedo->setEnabled(true);
  }

  void MainWindow::redo() {
    auto u = vector<DOMDocument*>(1+file.size());
    u[0] = doc;
    for(int i=0; i<file.size(); i++)
      u[i+1] = file[i]->getXMLDocument();
    undos.push_back(u);
    auto r = redos.back();
    doc = r[0];
    file.resize(r.size()-1);
    for(int i=0; i<r.size()-1; i++)
      file[i] = new FileItemData(r[i+1]);
    redos.pop_back();
    rebuildTree();
    if(getAutoRefresh()) refresh();
    actionRedo->setDisabled(redos.empty());
    actionUndo->setEnabled(true);
  }

  void MainWindow::removeElement() {
    updateUndos();
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    auto *element = dynamic_cast<Element*>(model->getItem(index)->getItemData());
    if(element and (not dynamic_cast<DynamicSystemSolver*>(element)) and (not dynamic_cast<InternalFrame*>(element))) {
      if(element == elementBuffer.first)
        elementBuffer.first = nullptr;
      QModelIndex pindex = element->getParameters()->getModelIndex();
      static_cast<ParameterTreeModel*>(parameterView->model())->removeRow(pindex.row(), pindex.parent());
      model->removeRow(index.row(), index.parent());
      element->removeXMLElement();
      element->getParent()->removeElement(element);
      updateReferences(element->getParent());
      if(getAutoRefresh()) refresh();
    }
  }

  void MainWindow::removeParameter() {
    updateUndos();
    auto *model = static_cast<ParameterTreeModel*>(parameterView->model());
    QModelIndex index = parameterView->selectionModel()->currentIndex();
    auto *parameter = dynamic_cast<Parameter*>(model->getItem(index)->getItemData());
    if(parameter) {
      auto *parent = parameter->getParent();
      if(parameter == parameterBuffer.first)
        parameterBuffer.first = nullptr;
      DOMNode *ps = parameter->getXMLElement()->getPreviousSibling();
      if(ps and X()%ps->getNodeName()=="#text")
        parameter->getXMLElement()->getParentNode()->removeChild(ps);
      parameter->getXMLElement()->getParentNode()->removeChild(parameter->getXMLElement());
      parent->removeParameter(parameter);
      parent->maybeRemoveEmbedXMLElement();
      updateParameterReferences(parent);
      model->removeRow(index.row(), index.parent());
      if(getAutoRefresh()) refresh();
    }
  }

  void MainWindow::remove() {
    if(elementView->hasFocus())
      removeElement();
    else if(parameterView->hasFocus())
      removeParameter();
  }

  void MainWindow::copy(bool cut) {
    if(elementView->hasFocus())
      copyElement(cut);
    else if(parameterView->hasFocus())
      copyParameter(cut);
  }

  void MainWindow::paste() {
    if(elementView->hasFocus()) {
      auto *model = static_cast<ElementTreeModel*>(elementView->model());
      QModelIndex index = elementView->selectionModel()->currentIndex();
      ContainerItemData *item = dynamic_cast<FrameItemData*>(model->getItem(index)->getItemData());
      if(item and dynamic_cast<Frame*>(getElementBuffer().first)) {
	createFrame(pasteElement(item->getElement(),getElementBuffer().first),item->getElement());
        return;
      }
      item = dynamic_cast<ContourItemData*>(model->getItem(index)->getItemData());
      if(item and dynamic_cast<Contour*>(getElementBuffer().first)) {
	createContour(pasteElement(item->getElement(),getElementBuffer().first),item->getElement());
	return;
      }
      item = dynamic_cast<GroupItemData*>(model->getItem(index)->getItemData());
      if(item and dynamic_cast<Group*>(getElementBuffer().first)) {
	createGroup(pasteElement(item->getElement(),getElementBuffer().first),item->getElement());
	return;
      }
      item = dynamic_cast<ObjectItemData*>(model->getItem(index)->getItemData());
      if(item and dynamic_cast<Object*>(getElementBuffer().first)) {
	createObject(pasteElement(item->getElement(),getElementBuffer().first),item->getElement());
	return;
      }
      item = dynamic_cast<LinkItemData*>(model->getItem(index)->getItemData());
      if(item and dynamic_cast<Link*>(getElementBuffer().first)) {
	createLink(pasteElement(item->getElement(),getElementBuffer().first),item->getElement());
	return;
      }
      item = dynamic_cast<ConstraintItemData*>(model->getItem(index)->getItemData());
      if(item and dynamic_cast<Constraint*>(getElementBuffer().first)) {
	createConstraint(pasteElement(item->getElement(),getElementBuffer().first),item->getElement());
	return;
      }
      item = dynamic_cast<ObserverItemData*>(model->getItem(index)->getItemData());
      if(item and dynamic_cast<Observer*>(getElementBuffer().first)) {
	createObserver(pasteElement(item->getElement(),getElementBuffer().first),item->getElement());
	return;
      }
    }
    else if(parameterView->hasFocus()) {
      auto *model = static_cast<ParameterTreeModel*>(parameterView->model());
      QModelIndex index = parameterView->selectionModel()->currentIndex();
      auto *item = dynamic_cast<Parameters*>(model->getItem(index)->getItemData());
      if(item and dynamic_cast<Parameter*>(getParameterBuffer().first) and not dynamic_cast<InternalFrame*>(item->getParent()))
        pasteParameter(item->getParent(),getParameterBuffer().first);
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
    else if(parameterView->hasFocus()) {
      auto *model = static_cast<ParameterTreeModel*>(parameterView->model());
      QModelIndex index = parameterView->selectionModel()->currentIndex();
      auto *parameter = dynamic_cast<Parameter*>(model->getItem(index)->getItemData());
      if(parameter and (up?(parameter->getParent()->getIndexOfParameter(parameter)>0):(parameter->getParent()->getIndexOfParameter(parameter)<parameter->getParent()->getNumberOfParameters()-1)))
        moveParameter(up);
    }
  }

  void MainWindow::copyElement(bool cut) {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    auto *element = dynamic_cast<Element*>(model->getItem(index)->getItemData());
    if(element and (not dynamic_cast<DynamicSystemSolver*>(element)) and (not dynamic_cast<InternalFrame*>(element)))
      elementBuffer = make_pair(element,cut);
  }

  void MainWindow::copyParameter(bool cut) {
    auto *model = static_cast<ParameterTreeModel*>(parameterView->model());
    QModelIndex index = parameterView->selectionModel()->currentIndex();
    auto *parameter = dynamic_cast<Parameter*>(model->getItem(index)->getItemData());
    if(parameter)
      parameterBuffer = make_pair(parameter,cut);
  }

  void MainWindow::moveParameter(bool up) {
    auto *model = static_cast<ParameterTreeModel*>(parameterView->model());
    QModelIndex index = parameterView->selectionModel()->currentIndex();
    QModelIndex parentIndex = index.parent();
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
    model->removeRows(0,parameter->getParent()->getNumberOfParameters(),parentIndex);
    for(int i=0; i<parameter->getParent()->getNumberOfParameters(); i++)
      model->createParameterItem(parameter->getParent()->getParameter(i),parentIndex);
    parameterView->setCurrentIndex(parameter->getParent()->getParameter(j)->getModelIndex());
    updateParameterReferences(parameter->getParent());
  }

  void MainWindow::moveFrame(bool up) {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    QModelIndex parentIndex = index.parent();
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
    model->removeRows(0,frame->getParent()->getNumberOfFrames(),parentIndex);
    for(int i=0; i<frame->getParent()->getNumberOfFrames(); i++)
      model->createFrameItem(frame->getParent()->getFrame(i),parentIndex);
    elementView->setCurrentIndex(frame->getParent()->getFrame(j)->getModelIndex());
    updateReferences(frame->getParent());
  }

  void MainWindow::moveContour(bool up) {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    QModelIndex parentIndex = index.parent();
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
    model->removeRows(0,contour->getParent()->getNumberOfContours(),parentIndex);
    for(int i=0; i<contour->getParent()->getNumberOfContours(); i++)
      model->createContourItem(contour->getParent()->getContour(i),parentIndex);
    elementView->setCurrentIndex(contour->getParent()->getContour(j)->getModelIndex());
    updateReferences(contour->getParent());
  }

  void MainWindow::moveGroup(bool up) {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    QModelIndex parentIndex = index.parent();
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
    model->removeRows(0,group->getParent()->getNumberOfGroups(),parentIndex);
    for(int i=0; i<group->getParent()->getNumberOfGroups(); i++)
      model->createGroupItem(group->getParent()->getGroup(i),parentIndex);
    elementView->setCurrentIndex(group->getParent()->getGroup(j)->getModelIndex());
    updateReferences(group->getParent());
  }

  void MainWindow::moveObject(bool up) {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    QModelIndex parentIndex = index.parent();
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
    model->removeRows(0,object->getParent()->getNumberOfObjects(),parentIndex);
    for(int i=0; i<object->getParent()->getNumberOfObjects(); i++)
      model->createObjectItem(object->getParent()->getObject(i),parentIndex);
    elementView->setCurrentIndex(object->getParent()->getObject(j)->getModelIndex());
    updateReferences(object->getParent());
  }

  void MainWindow::moveLink(bool up) {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    QModelIndex parentIndex = index.parent();
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
    model->removeRows(0,link->getParent()->getNumberOfLinks(),parentIndex);
    for(int i=0; i<link->getParent()->getNumberOfLinks(); i++)
      model->createLinkItem(link->getParent()->getLink(i),parentIndex);
    elementView->setCurrentIndex(link->getParent()->getLink(j)->getModelIndex());
    updateReferences(link->getParent());
  }

  void MainWindow::moveConstraint(bool up) {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    QModelIndex parentIndex = index.parent();
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
    model->removeRows(0,constraint->getParent()->getNumberOfConstraints(),parentIndex);
    for(int i=0; i<constraint->getParent()->getNumberOfConstraints(); i++)
      model->createConstraintItem(constraint->getParent()->getConstraint(i),parentIndex);
    elementView->setCurrentIndex(constraint->getParent()->getConstraint(j)->getModelIndex());
    updateReferences(constraint->getParent());
  }

  void MainWindow::moveObserver(bool up) {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    QModelIndex parentIndex = index.parent();
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
    model->removeRows(0,observer->getParent()->getNumberOfObservers(),parentIndex);
    for(int i=0; i<observer->getParent()->getNumberOfObservers(); i++)
      model->createObserverItem(observer->getParent()->getObserver(i),parentIndex);
    elementView->setCurrentIndex(observer->getParent()->getObserver(j)->getModelIndex());
    updateReferences(observer->getParent());
  }

  void MainWindow::exportElement(const QString &title) {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    auto index = elementView->selectionModel()->currentIndex();
    auto *item = static_cast<EmbedItemData*>(model->getItem(index)->getItemData());
    SaveModelDialog dialog(title, getProjectDir().absoluteFilePath(item->getName()+".mbsmx"),item->getNumberOfParameters());
    int result = dialog.exec();
    if(result) {
      if(not dialog.getModelFileName().isEmpty()) {
	QMessageBox::StandardButton ret = QMessageBox::Yes;
	if(QFileInfo::exists(dialog.getModelFileName()))
	  ret = QMessageBox::question(this, "Replace file", "A file named " + dialog.getModelFileName() + " already exists. Do you want to replace it?", QMessageBox::Yes | QMessageBox::No);
	if(ret == QMessageBox::Yes) {
	  DOMDocument *doc = impl->createDocument();
	  DOMNode *node = doc->importNode(item->getXMLElement(),true);
	  doc->insertBefore(node,nullptr);
	  serializer->writeToURI(doc, X()%dialog.getModelFileName().toStdString());
	}
      }
      if(item->getNumberOfParameters() and not dialog.getParameterFileName().isEmpty()) {
	QMessageBox::StandardButton ret = QMessageBox::Yes;
	if(QFileInfo::exists(dialog.getParameterFileName()))
	  ret = QMessageBox::question(this, "Replace file", "A file named " + dialog.getParameterFileName() + " already exists. Do you want to replace it?", QMessageBox::Yes | QMessageBox::No);
	if(ret == QMessageBox::Yes) {
	  DOMDocument *doc = impl->createDocument();
	  DOMNode *node = doc->importNode(item->getParameter(0)->getXMLElement()->getParentNode(),true);
	  doc->insertBefore(node,nullptr);
	  serializer->writeToURI(doc, X()%dialog.getParameterFileName().toStdString());
	}
      }
    }
  }

  void MainWindow::enableElement(bool enable) {
    updateUndos();
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    auto *element = static_cast<Element*>(model->getItem(index)->getItemData());
    DOMElement *embedNode = element->getEmbedXMLElement();
    auto *fileItem = element->getDedicatedFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setWindowModified(true);
    if(enable) {
      // try to restore the count from the processing instruction EnabledCount or use 1 as count
      string count;
      auto enabledCount=E(embedNode)->getFirstProcessingInstructionChildNamed("MBSimGUI_EnabledCount");
      if(enabledCount) {
        count=X()%enabledCount->getData();
        embedNode->removeChild(enabledCount);
      }
      if(!count.empty())
        E(embedNode)->setAttribute("count",count);
      else {
        E(embedNode)->removeAttribute("count");
        E(embedNode)->removeAttribute("counterName");
      }
    }
    else {
      if(not embedNode)
        embedNode = element->createEmbedXMLElement();
      // save current count to processing instruction and set count to 0
      if(E(embedNode)->hasAttribute("count")) {
        auto enabledCount=E(embedNode)->getFirstProcessingInstructionChildNamed("MBSimGUI_EnabledCount");
        if(enabledCount)
          enabledCount->setData(X()%E(embedNode)->getAttribute("count"));
        else {
          enabledCount=embedNode->getOwnerDocument()->createProcessingInstruction(X()%"MBSimGUI_EnabledCount",
                                                                                  X()%E(embedNode)->getAttribute("count"));
          embedNode->insertBefore(enabledCount, embedNode->getFirstChild());
        }
      }
      E(embedNode)->setAttribute("count","0");
      if(!E(embedNode)->hasAttribute("counterName"))
        E(embedNode)->setAttribute("counterName","MBXMLUtils_disabled");
    }
    element->maybeRemoveEmbedXMLElement();
    element->updateStatus();
    elementViewFilter->updateItem(index);
    if(getAutoRefresh()) refresh();
  }

  void MainWindow::exportParameters() {
    auto *model = static_cast<ParameterTreeModel*>(parameterView->model());
    auto index = parameterView->selectionModel()->currentIndex();
    auto *parameters = static_cast<Parameters*>(model->getItem(index)->getItemData());
    SaveParameterDialog dialog(getProjectDir().absoluteFilePath(parameters->getParent()->getName()+".mbspx"));
    int result = dialog.exec();
    if(result and not dialog.getParameterFileName().isEmpty()) {
      QMessageBox::StandardButton ret = QMessageBox::Yes;
      if(QFileInfo::exists(dialog.getParameterFileName()))
	ret = QMessageBox::question(this, "Replace file", "A file named " + dialog.getParameterFileName() + " already exists. Do you want to replace it?", QMessageBox::Yes | QMessageBox::No);
      if(ret == QMessageBox::Yes) {
	DOMDocument *doc = impl->createDocument();
	DOMNode *node = doc->importNode(parameters->getParent()->getEmbedXMLElement()->getFirstElementChild(),true);
	doc->insertBefore(node,nullptr);
	serializer->writeToURI(doc, X()%dialog.getParameterFileName().toStdString());
      }
    }
  }

  void MainWindow::updateNames(EmbedItemData *parent) {
    auto *dedicatedParent = parent->getDedicatedItem();
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem) {
      for(int i=0; i<fileItem->getNumberOfReferences(); i++) {
        if(fileItem->getFileReference(i)!=dedicatedParent)
          fileItem->getFileReference(i)->updateNames();
      }
    }
  }

  void MainWindow::updateValues(EmbedItemData *parent) {
    auto *dedicatedParent = parent->getDedicatedItem();
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem) {
      for(int i=0; i<fileItem->getNumberOfReferences(); i++) {
        if(fileItem->getFileReference(i)!=dedicatedParent) {
          fileItem->getFileReference(i)->updateValues();
          fileItem->getFileReference(i)->updateNames();
	}
      }
    }
    fileItem = parent->getParameterFileItem();
    if(fileItem) {
      for(int i=0; i<fileItem->getNumberOfReferences(); i++) {
        if(fileItem->getFileReference(i)!=parent)
          fileItem->getFileReference(i)->updateValues();
      }
    }
  }

  void MainWindow::updateReferences(EmbedItemData *parent) {
    auto *dedicatedParent = parent->getDedicatedItem();
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem) {
      fileItem->setModified(true);
      for(int i=0; i<fileItem->getNumberOfReferences(); i++) {
        if(fileItem->getFileReference(i)!=dedicatedParent) {
          fileItem->getFileReference(i)->setXMLElement(dedicatedParent->getXMLElement());
          fileItem->getFileReference(i)->clear();
          auto *model = static_cast<ElementTreeModel*>(elementView->model());
	  QModelIndex index = model->findItem(fileItem->getFileReference(i),project->getModelIndex());
          model->removeRows(0,model->rowCount(index),index);
          fileItem->getFileReference(i)->create();
          model->updateElementItem(static_cast<Element*>(fileItem->getFileReference(i)));
        }
      }
    }
    else
      setWindowModified(true);
  }

  void MainWindow::updateParameterReferences(EmbedItemData *parent) {
    auto *element = dynamic_cast<Element*>(parent);
    if(element and element->getParent()) {
      auto *dedicatedParent = element->getParent()->getDedicatedItem();
      auto *fileItem = dedicatedParent->getFileItem();
      if(fileItem) {
	fileItem->setModified(true);
	for(int i=0; i<fileItem->getNumberOfReferences(); i++) {
	  if(fileItem->getFileReference(i)!=dedicatedParent) {
	    fileItem->getFileReference(i)->clear();
	    auto *model = static_cast<ElementTreeModel*>(elementView->model());
	    QModelIndex index = model->findItem(fileItem->getFileReference(i),project->getModelIndex());
	    model->removeRows(0,model->rowCount(index),index);
	    fileItem->getFileReference(i)->create();
	    model->updateElementItem(static_cast<Element*>(fileItem->getFileReference(i)));
	  }
	}
      }
    }
    auto* fileItem = parent->getDedicatedParameterFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setWindowModified(true);
    fileItem = parent->getParameterFileItem();
    if(fileItem) {
      for(int i=0; i<fileItem->getNumberOfReferences(); i++) {
        if(fileItem->getFileReference(i)!=parent) {
          fileItem->getFileReference(i)->clearParameters();
          fileItem->getFileReference(i)->createParameters();
        }
      }
    }
  }

  void MainWindow::addFrame(Frame *frame, Element *parent) {
    updateUndos();
    QModelIndex index = elementView->selectionModel()->currentIndex();
    parent->addFrame(frame);
    frame->createXMLElement(parent->getXMLFrames());
    static_cast<ElementTreeModel*>(elementView->model())->createFrameItem(frame,index);
    updateReferences(parent);
    elementView->selectionModel()->setCurrentIndex(frame->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    openElementEditor(false);
  }

  void MainWindow::addContour(Contour *contour, Element *parent) {
    updateUndos();
    QModelIndex index = elementView->selectionModel()->currentIndex();
    parent->addContour(contour);
    contour->createXMLElement(parent->getXMLContours());
    static_cast<ElementTreeModel*>(elementView->model())->createContourItem(contour,index);
    updateReferences(parent);
    elementView->selectionModel()->setCurrentIndex(contour->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    openElementEditor(false);
  }

  void MainWindow::addGroup(Group *group, Element *parent) {
    updateUndos();
    QModelIndex index = elementView->selectionModel()->currentIndex();
    parent->addGroup(group);
    group->createXMLElement(parent->getXMLGroups());
    static_cast<ElementTreeModel*>(elementView->model())->createGroupItem(group,index);
    updateReferences(parent);
    elementView->selectionModel()->setCurrentIndex(group->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    openElementEditor(false);
  }

  void MainWindow::addObject(Object *object, Element *parent) {
    updateUndos();
    QModelIndex index = elementView->selectionModel()->currentIndex();
    parent->addObject(object);
    object->createXMLElement(parent->getXMLObjects());
    static_cast<ElementTreeModel*>(elementView->model())->createObjectItem(object,index);
    updateReferences(parent);
    elementView->selectionModel()->setCurrentIndex(object->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    openElementEditor(false);
  }

  void MainWindow::addLink(Link *link, Element *parent) {
    updateUndos();
    QModelIndex index = elementView->selectionModel()->currentIndex();
    parent->addLink(link);
    link->createXMLElement(parent->getXMLLinks());
    static_cast<ElementTreeModel*>(elementView->model())->createLinkItem(link,index);
    updateReferences(parent);
    elementView->selectionModel()->setCurrentIndex(link->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    openElementEditor(false);
  }

  void MainWindow::addConstraint(Constraint *constraint, Element *parent) {
    updateUndos();
    QModelIndex index = elementView->selectionModel()->currentIndex();
    parent->addConstraint(constraint);
    constraint->createXMLElement(parent->getXMLConstraints());
    static_cast<ElementTreeModel*>(elementView->model())->createConstraintItem(constraint,index);
    updateReferences(parent);
    elementView->selectionModel()->setCurrentIndex(constraint->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    openElementEditor(false);
  }

  void MainWindow::addObserver(Observer *observer, Element *parent) {
    updateUndos();
    QModelIndex index = elementView->selectionModel()->currentIndex();
    parent->addObserver(observer);
    observer->createXMLElement(parent->getXMLObservers());
    static_cast<ElementTreeModel*>(elementView->model())->createObserverItem(observer,index);
    updateReferences(parent);
    elementView->selectionModel()->setCurrentIndex(observer->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    openElementEditor(false);
  }

  void MainWindow::addParameter(Parameter *parameter, EmbedItemData *parent) {
    updateUndos();
    QModelIndex index = parameterView->selectionModel()->currentIndex();
    auto *model = static_cast<ParameterTreeModel*>(parameterView->model());
    parent->addParameter(parameter);
    parameter->createXMLElement(parent->createParameterXMLElement());
    updateParameterReferences(parent);
    model->createParameterItem(parameter,index);
    parameterView->selectionModel()->setCurrentIndex(parameter->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    openParameterEditor(false);
  }

  void MainWindow::pasteParameter(EmbedItemData *parent, Parameter *param) {
    updateUndos();
    DOMElement *parentele = parent->createParameterXMLElement();
    DOMElement *pele = static_cast<DOMElement*>(parentele->getOwnerDocument()->importNode(param->getXMLElement(),true));
    if(parameterBuffer.second) {
      auto *parent = param->getParent();
      parameterBuffer.first = nullptr;
      DOMNode *ps = param->getXMLElement()->getPreviousSibling();
      if(ps and X()%ps->getNodeName()=="#text")
        param->getXMLElement()->getParentNode()->removeChild(ps);
      param->getXMLElement()->getParentNode()->removeChild(param->getXMLElement());
      auto *model = static_cast<ParameterTreeModel*>(parameterView->model());
      QModelIndex index = model->findItem(param,project->getParameters()->getModelIndex());
      if(index.isValid()) model->removeRow(index.row(), index.parent());
      parent->removeParameter(param);
      parent->maybeRemoveEmbedXMLElement();
    }
    Parameter *parameter=ObjectFactory::getInstance().create<Parameter>(pele);
    parentele->insertBefore(pele,nullptr);
    parameter->setXMLElement(pele);
    parameter->updateValue();
    parent->addParameter(parameter);
    parent->updateName();
    static_cast<ParameterTreeModel*>(parameterView->model())->createParameterItem(parameter,parent->getParameters()->getModelIndex());
    updateParameterReferences(parent);
    parameterView->selectionModel()->setCurrentIndex(parent->getParameters()->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(getAutoRefresh()) refresh();
  }

  void MainWindow::loadParameter(EmbedItemData *parent) {
    LoadParameterDialog dialog;
    int result = dialog.exec();
    if(result) {
      updateUndos();
      if(parent->getNumberOfParameters()) removeParameter(parent);
      DOMDocument *doc = nullptr;
      QString file = dialog.getParameterFileName().isEmpty()?"":getProjectDir().absoluteFilePath(dialog.getParameterFileName());
      if(QFileInfo::exists(file)) {
	if(file.startsWith("//"))
	  file.replace('/','\\'); // xerces-c is not able to parse files from network shares that begin with "//"
	auto *parentele = parent->createEmbedXMLElement();
	if(dialog.referenceParameter()) {
	  QDir parentDir = QDir(QFileInfo(QUrl(QString::fromStdString(MBXMLUtils::X()%parentele->getOwnerDocument()->getDocumentURI())).toLocalFile()).canonicalPath());
	  E(parentele)->setAttribute("parameterHref",(dialog.getAbsoluteFilePath()?parentDir.absoluteFilePath(file):parentDir.relativeFilePath(file)).toStdString());
	  parent->setParameterFileItem(addFile(file));
	}
	else {
	  doc = parser->parseURI(X()%file.toStdString());
	  DOMParser::handleCDATA(doc->getDocumentElement());
	  parentele->insertBefore(static_cast<DOMElement*>(parentele->getOwnerDocument()->importNode(doc->getDocumentElement(),true)),parent->getXMLElement());
	}
      }
      parent->createParameters();
      parent->updateName();
      for(int i=0; i<parent->getNumberOfParameters(); i++)
	static_cast<ParameterTreeModel*>(parameterView->model())->createParameterItem(parent->getParameter(i),parent->getParameters()->getModelIndex());
      parameterView->scrollToBottom();
      updateParameterReferences(parent);
      parameterView->selectionModel()->setCurrentIndex(parent->getParameters()->getModelIndex(), QItemSelectionModel::ClearAndSelect);
      if(getAutoRefresh()) refresh();
    }
  }

  void MainWindow::removeParameter(EmbedItemData *parent) {
    updateUndos();
    auto *model = static_cast<ParameterTreeModel*>(parameterView->model());
    QModelIndex index = parameterView->selectionModel()->currentIndex();
    int n = parent->getNumberOfParameters();
    if(parent->getParameterFileItem()) {
      for(int i=n-1; i>=0; i--)
        parent->removeParameter(parent->getParameter(i));
      parent->getEmbedXMLElement()->removeAttribute(X()%"parameterHref");
      parent->setParameterFileItem(nullptr);
    }
    else {
      for(int i=n-1; i>=0; i--) {
        auto *parameter = parent->getParameter(i);
        parameterBuffer.first = nullptr;
        DOMNode *ps = parameter->getXMLElement()->getPreviousSibling();
        if(ps and X()%ps->getNodeName()=="#text")
          parameter->getXMLElement()->getParentNode()->removeChild(ps);
        parameter->getXMLElement()->getParentNode()->removeChild(parameter->getXMLElement());
        parent->removeParameter(parameter);
      }
    }
    parent->maybeRemoveEmbedXMLElement();
    updateParameterReferences(parent);
    model->removeRows(0,n,index);
    if(getAutoRefresh()) refresh();
  }

  DOMElement* MainWindow::pasteElement(Element *parent, Element *element) {
    updateUndos();
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    DOMElement *ele = static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(element->getEmbedXMLElement()?element->getEmbedXMLElement():element->getXMLElement(),true));
    if(elementBuffer.second) {
      elementBuffer.first = nullptr;
      auto *parent = element->getParent();
      QModelIndex index = model->findItem(element,project->getDynamicSystemSolver()->getModelIndex());
      model->removeRow(index.row(), index.parent());
      element->removeXMLElement();
      parent->removeElement(element);
    }
    return ele;
  }

  DOMElement* MainWindow::loadEmbedItemData(EmbedItemData *parent, const QString &title) {
    DOMElement *element = nullptr;
    LoadModelDialog dialog(title);
    int result = dialog.exec();
    if(result) {
      updateUndos();
      DOMDocument *doc = nullptr;
      QString file = dialog.getParameterFileName().isEmpty()?"":getProjectDir().absoluteFilePath(dialog.getParameterFileName());
      if(QFileInfo::exists(file)) {
	if(file.startsWith("//"))
	  file.replace('/','\\'); // xerces-c is not able to parse files from network shares that begin with "//"
	element = D(parent->getXMLElement()->getOwnerDocument())->createElement(PV%"Embed");
	if(dialog.referenceParameter()) {
	  QDir parentDir = QDir(QFileInfo(QUrl(QString::fromStdString(MBXMLUtils::X()%parent->getXMLElement()->getOwnerDocument()->getDocumentURI())).toLocalFile()).canonicalPath());
	  E(element)->setAttribute("parameterHref",(dialog.getAbsoluteParameterFilePath()?parentDir.absoluteFilePath(file):parentDir.relativeFilePath(file)).toStdString());
	}
	else {
	  doc = parser->parseURI(X()%file.toStdString());
	  DOMParser::handleCDATA(doc->getDocumentElement());
	  DOMElement *ele = static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
	  element->insertBefore(ele,nullptr);
	}
      }
      file = dialog.getModelFileName().isEmpty()?"":getProjectDir().absoluteFilePath(dialog.getModelFileName());
      if(QFileInfo::exists(file)) {
	if(file.startsWith("//"))
	  file.replace('/','\\'); // xerces-c is not able to parse files from network shares that begin with "//"
	if(dialog.referenceModel()) {
	  if(not element) element = D(parent->getXMLElement()->getOwnerDocument())->createElement(PV%"Embed");
	  QDir parentDir = QDir(QFileInfo(QUrl(QString::fromStdString(MBXMLUtils::X()%parent->getXMLElement()->getOwnerDocument()->getDocumentURI())).toLocalFile()).canonicalPath());
	  E(element)->setAttribute("href",(dialog.getAbsoluteModelFilePath()?parentDir.absoluteFilePath(file):parentDir.relativeFilePath(file)).toStdString());
	}
	else {
	  doc = parser->parseURI(X()%file.toStdString());
	  DOMParser::handleCDATA(doc->getDocumentElement());
	  DOMElement *ele = static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
	  if(element) element->insertBefore(ele,nullptr);
	  else element = ele;
	}
      }
    }
    return element;
  }

  template<class Container>
  QModelIndex MainWindow::getContainerIndex(Element *parent) {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    for(int row=0; row<model->rowCount(parent->getModelIndex()); row++) {
      auto containerIndex = model->index(row, 0, parent->getModelIndex());
      auto *container = model->getItem(containerIndex)->getItemData();
      if(dynamic_cast<Container*>(container)) return containerIndex;
    }
    return QModelIndex();
  }

  bool MainWindow::createFrame(DOMElement *ele, Element *parent, bool showDialogOnError) {
    if(not ele) return false;
    Frame *frame = Embed<FixedRelativeFrame>::create(ele,parent);
    if(not frame)
      frame = Embed<NodeFrame>::create(ele,parent);
    if(not frame) {
      if(showDialogOnError) QMessageBox::warning(this, "Create frame", "Cannot create frame.");
      return false;
    }
    parent->getXMLFrames()->insertBefore(ele, nullptr);
    parent->addFrame(frame);
    frame->create();
    updateReferences(parent);
    static_cast<ElementTreeModel*>(elementView->model())->createFrameItem(frame,getContainerIndex<FrameItemData>(parent));
    elementView->selectionModel()->setCurrentIndex(frame->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(getAutoRefresh()) refresh();
    return true;
  }

  bool MainWindow::createContour(DOMElement *ele, Element *parent, bool showDialogOnError) {
    if(not ele) return false;
    Contour *contour = Embed<Contour>::create(ele,parent);
    if(not contour) {
      if(showDialogOnError) QMessageBox::warning(this, "Create contour", "Cannot create contour.");
      return false;
    }
    parent->getXMLContours()->insertBefore(ele, nullptr);
    parent->addContour(contour);
    contour->create();
    updateReferences(parent);
    static_cast<ElementTreeModel*>(elementView->model())->createContourItem(contour,getContainerIndex<ContourItemData>(parent));
    elementView->selectionModel()->setCurrentIndex(contour->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(getAutoRefresh()) refresh();
    return true;
  }

  bool MainWindow::createGroup(DOMElement *ele, Element *parent, bool showDialogOnError) {
    if(not ele) return false;
    Group *group = Embed<Group>::create(ele,parent);
    if(not group) {
      if(showDialogOnError) QMessageBox::warning(this, "Create group", "Cannot create group.");
      return false;
    }
    parent->getXMLGroups()->insertBefore(ele, nullptr);
    parent->addGroup(group);
    group->create();
    updateReferences(parent);
    static_cast<ElementTreeModel*>(elementView->model())->createGroupItem(group,getContainerIndex<GroupItemData>(parent));
    elementView->selectionModel()->setCurrentIndex(group->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(getAutoRefresh()) refresh();
    return true;
  }

  bool MainWindow::createObject(DOMElement *ele, Element *parent, bool showDialogOnError) {
    if(not ele) return false;
    Object *object = Embed<Object>::create(ele,parent);
    if(not object) {
      if(showDialogOnError) QMessageBox::warning(this, "Create object", "Cannot create object.");
      return false;
    }
    parent->getXMLObjects()->insertBefore(ele, nullptr);
    parent->addObject(object);
    object->create();
    updateReferences(parent);
    static_cast<ElementTreeModel*>(elementView->model())->createObjectItem(object,getContainerIndex<ObjectItemData>(parent));
    elementView->selectionModel()->setCurrentIndex(object->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(getAutoRefresh()) refresh();
    return true;
  }

  bool MainWindow::createLink(DOMElement *ele, Element *parent, bool showDialogOnError) {
    if(not ele) return false;
    Link *link = Embed<Link>::create(ele,parent);
    if(not link) {
      if(showDialogOnError) QMessageBox::warning(this, "Create link", "Cannot create link.");
      return false;
    }
    parent->getXMLLinks()->insertBefore(ele, nullptr);
    parent->addLink(link);
    link->create();
    updateReferences(parent);
    static_cast<ElementTreeModel*>(elementView->model())->createLinkItem(link,getContainerIndex<LinkItemData>(parent));
    elementView->selectionModel()->setCurrentIndex(link->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(getAutoRefresh()) refresh();
    return true;
  }

  bool MainWindow::createConstraint(DOMElement *ele, Element *parent, bool showDialogOnError) {
    if(not ele) return false;
    Constraint *constraint = Embed<Constraint>::create(ele,parent);
    if(not constraint) {
      if(showDialogOnError) QMessageBox::warning(this, "Create constraint", "Cannot create constraint.");
      return false;
    }
    parent->getXMLConstraints()->insertBefore(ele, nullptr);
    parent->addConstraint(constraint);
    constraint->create();
    updateReferences(parent);
    static_cast<ElementTreeModel*>(elementView->model())->createConstraintItem(constraint,getContainerIndex<ConstraintItemData>(parent));
    elementView->selectionModel()->setCurrentIndex(constraint->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(getAutoRefresh()) refresh();
    return true;
  }

  bool MainWindow::createObserver(DOMElement *ele, Element *parent, bool showDialogOnError) {
    if(not ele) return false;
    Observer *observer = Embed<Observer>::create(ele,parent);
    if(not observer) {
      if(showDialogOnError) QMessageBox::warning(this, "Create observer", "Cannot create observer.");
      return false;
    }
    parent->getXMLObservers()->insertBefore(ele, nullptr);
    parent->addObserver(observer);
    observer->create();
    updateReferences(parent);
    static_cast<ElementTreeModel*>(elementView->model())->createObserverItem(observer,getContainerIndex<ObserverItemData>(parent));
    elementView->selectionModel()->setCurrentIndex(observer->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(getAutoRefresh()) refresh();
    return true;
  }

  void MainWindow::createAny(xercesc::DOMElement *ele, Element *parent, const FQN &requestedXMLType) {
    if     (requestedXMLType==MBSIM%"Frame"     ) createFrame     (ele, parent);
    else if(requestedXMLType==MBSIM%"Contour"   ) createContour   (ele, parent);
    else if(requestedXMLType==MBSIM%"Group"     ) createGroup     (ele, parent);
    else if(requestedXMLType==MBSIM%"Object"    ) createObject    (ele, parent);
    else if(requestedXMLType==MBSIM%"Link"      ) createLink      (ele, parent);
    else if(requestedXMLType==MBSIM%"Constraint") createConstraint(ele, parent);
    else if(requestedXMLType==MBSIM%"Observer"  ) createObserver  (ele, parent);
    else
      QMessageBox::warning(this, "Create element", "Unknown container type of imported/referenced element.");
// NOTE: The below code does not work for element not known to the GUI (=Unknown<Container> type)
//       We can only make createAny to work by checking if the type of the XML "ele" is derived from the corresponding XML container type but this
//       is only possible then the GUI loads XML files with a validating parser (which is not yet done)
//    MBXMLUtils::FQN newXMLType;
//    if     (createFrame     (ele, parent, false)) { newXMLType=MBSIM%"Frame"; }
//    else if(createContour   (ele, parent, false)) { newXMLType=MBSIM%"Contour"; }
//    else if(createGroup     (ele, parent, false)) { newXMLType=MBSIM%"Group"; }
//    else if(createObject    (ele, parent, false)) { newXMLType=MBSIM%"Object"; }
//    else if(createLink      (ele, parent, false)) { newXMLType=MBSIM%"Link"; }
//    else if(createConstraint(ele, parent, false)) { newXMLType=MBSIM%"Constraint"; }
//    else if(createObserver  (ele, parent, false)) { newXMLType=MBSIM%"Observer"; }
//
//    if(newXMLType==MBXMLUtils::FQN())
//      QMessageBox::warning(this, "Create element", "Unknown container type of imported/referenced element.");
//
//    if(newXMLType != requestedXMLType)
//      QMessageBox::information(this, "Create element", ("The imported/referenced element is not of the selected container type '"+requestedXMLType.second+"'.\n"+
//                                                        "It's added to it's corresponding container type '"+newXMLType.second+"'.").c_str());
  }

  void MainWindow::createDynamicSystemSolver(DOMElement *ele) {
    if(not ele) return;
    auto pindex = project->getDynamicSystemSolver()->getParameters()->getModelIndex();
    static_cast<ParameterTreeModel*>(parameterView->model())->removeRow(pindex.row(), pindex.parent());
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    model->removeRows(0,model->rowCount(project->getModelIndex()),project->getModelIndex());
    project->getDynamicSystemSolver()->removeXMLElement(true);
    DynamicSystemSolver *dss = Embed<DynamicSystemSolver>::create(ele,project);
    if(not dss) {
      QMessageBox::warning(this, "Create dynamic system solver", "Cannot create dynamic system solver.");
      return;
    }
    project->getXMLElement()->insertBefore(ele, project->getSolver()->getEmbedXMLElement()?project->getSolver()->getEmbedXMLElement():project->getSolver()->getXMLElement());
    project->setDynamicSystemSolver(dss);
    dss->create();
    setWindowModified(true);
    model->createGroupItem(dss,project->getModelIndex());
    model->createSolverItem(project->getSolver(),project->getModelIndex());
    elementView->selectionModel()->setCurrentIndex(dss->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(getAutoRefresh()) refresh();
  }

  void MainWindow::createSolver(DOMElement *ele) {
    if(not ele) return;
    QModelIndex pindex = project->getSolver()->getParameters()->getModelIndex();
    static_cast<ParameterTreeModel*>(parameterView->model())->removeRow(pindex.row(), pindex.parent());
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    model->removeRow(project->getSolver()->getModelIndex().row(), project->getModelIndex());

    project->getSolver()->removeXMLElement(true);

    Solver *solver = Embed<Solver>::create(ele,project);
    if(not solver) {
      QMessageBox::warning(this, "Create solver", "Cannot create solver.");
      return;
    }
    project->getXMLElement()->insertBefore(ele, nullptr);
    project->setSolver(solver);
    solver->create();
    setWindowModified(true);
    model->createSolverItem(solver,project->getModelIndex());
    elementView->selectionModel()->setCurrentIndex(solver->getModelIndex(), QItemSelectionModel::ClearAndSelect);
  }

  void MainWindow::editElementSource() {
    if(not editorIsOpen()) {
      menuBar()->setDisabled(true);
      QModelIndex index = elementView->selectionModel()->currentIndex();
      auto *element = static_cast<EmbedItemData*>(static_cast<ElementTreeModel*>(elementView->model())->getItem(index)->getItemData());
      editor = new XMLPropertyDialog(element);
      editor->setAttribute(Qt::WA_DeleteOnClose);
      editor->toWidget();
      editor->show();
      connect(editor,&QDialog::finished,this,[=](){
        if(editor->result()==QDialog::Accepted) {
          editor->fromWidget();
          element->clear();
          auto *model = static_cast<ElementTreeModel*>(elementView->model());
          QModelIndex index = model->findItem(element,project->getModelIndex());
          model->removeRows(0,model->rowCount(index),index);
          element->create();
          model->updateElementItem(static_cast<Element*>(element));
          updateReferences(element);
          if(getAutoRefresh()) refresh();
        }
	menuBar()->setEnabled(true);
        editor = nullptr;
      });
      connect(editor,&ElementPropertyDialog::apply,this,[=](){
        editor->fromWidget();
        element->clear();
        auto *model = static_cast<ElementTreeModel*>(elementView->model());
	QModelIndex index = model->findItem(element,project->getModelIndex());
        model->removeRows(0,model->rowCount(index),index);
        element->create();
        model->updateElementItem(static_cast<Element*>(element));
        updateReferences(element);
        if(getAutoRefresh()) refresh();
      });
    }
  }

  void MainWindow::editParametersSource() {
    if(not editorIsOpen()) {
      menuBar()->setDisabled(true);
      QModelIndex index = parameterView->selectionModel()->currentIndex();
      auto *item = static_cast<Parameters*>(static_cast<ParameterTreeModel*>(parameterView->model())->getItem(index)->getItemData());
      EmbedItemData *parent = item->getParent();
      editor = new ParameterXMLPropertyDialog(parent);
      editor->setAttribute(Qt::WA_DeleteOnClose);
      editor->toWidget();
      editor->show();
      connect(editor,&QDialog::finished,this,[=](){
        if(editor->result()==QDialog::Accepted) {
          editor->fromWidget();
          int n = parent->getNumberOfParameters();
          for(int i=n-1; i>=0; i--) parent->removeParameter(parent->getParameter(i));
          QModelIndex index = parent->getParameters()->getModelIndex();
          auto *model = static_cast<ParameterTreeModel*>(parameterView->model());
          model->removeRows(0,n,index);
          parent->createParameters();
          model->updateParameterItem(parent->getParameters());
          updateParameterReferences(parent);
          if(getAutoRefresh()) refresh();
        }
	menuBar()->setEnabled(true);
        editor = nullptr;
      });
      connect(editor,&ElementPropertyDialog::apply,this,[=](){
        editor->fromWidget();
        int n = parent->getNumberOfParameters();
        for(int i=n-1; i>=0; i--) parent->removeParameter(parent->getParameter(i));
        QModelIndex index = parent->getParameters()->getModelIndex();
        auto *model = static_cast<ParameterTreeModel*>(parameterView->model());
        model->removeRows(0,n,index);
        parent->createParameters();
        model->updateParameterItem(parent->getParameters());
        updateParameterReferences(parent);
        if(getAutoRefresh()) refresh();
      });
    }
  }

  void MainWindow::dragEnterEvent(QDragEnterEvent *event) {
    if (event->mimeData()->hasUrls()) {
      event->acceptProposedAction();
    }
  }

  void MainWindow::dropEvent(QDropEvent *event) {
    for (int i = 0; i < event->mimeData()->urls().size(); i++) {
      QString path = event->mimeData()->urls()[i].toLocalFile().toLocal8Bit().data();
      if(path.endsWith(".mbsx")) {
        QFile Fout(path);
        if (Fout.exists())
          loadProject(Fout.fileName());
      }
    }
  }

  void MainWindow::closeEvent(QCloseEvent *event) {
    if(maybeSave()) {
      QSettings settings;
      settings.setValue("mainwindow/geometry", saveGeometry());
      settings.setValue("mainwindow/state", saveState());
      settings.setValue("mainwindow/elementview/header/state", elementView->header()->saveState());
      settings.setValue("mainwindow/parameterview/header/state", parameterView->header()->saveState());
      event->accept();
    }
    else
      event->ignore();
  }

  void MainWindow::showEvent(QShowEvent *event) {
    QSettings settings;
    restoreGeometry(settings.value("mainwindow/geometry").toByteArray());
    restoreState(settings.value("mainwindow/state").toByteArray());
    elementView->header()->restoreState(settings.value("mainwindow/elementview/header/state").toByteArray());
    parameterView->header()->restoreState(settings.value("mainwindow/parameterview/header/state").toByteArray());
    QMainWindow::showEvent(event);
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
    QStringList files = settings.value("mainwindow/recentProjectFileList").toStringList();
    files.removeAll(fileName);
    files.prepend(fileName);
    while(files.size() > maxRecentFiles)
      files.removeLast();

    settings.setValue("mainwindow/recentProjectFileList", files);

    foreach(QWidget *widget, QApplication::topLevelWidgets()) {
      auto *mainWin = dynamic_cast<MainWindow*>(widget);
      if(mainWin)
        mainWin->updateRecentProjectFileActions();
    }
  }

  void MainWindow::updateRecentProjectFileActions() {
    QSettings settings;
    QStringList files = settings.value("mainwindow/recentProjectFileList").toStringList();

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

  void MainWindow::interrupt() {
    echoView->clearOutput();
    echoView->addOutputText("<span class=\"MBSIMGUI_WARN\">Simulation interrupted</span>\n");
    echoView->updateOutput(true);
    process.terminate();
  }

  void MainWindow::kill() {
    echoView->clearOutput();
    echoView->addOutputText("<span class=\"MBSIMGUI_WARN\">Simulation killed</span>\n");
    echoView->updateOutput(true);
    process.kill();
  }

  void MainWindow::flexibleBodyTool() {
    if(not fbt) {
      fbt = new FlexibleBodyTool(this);
      updateParameters(project);
    }
    fbt->show();
  }

  void MainWindow::createFMU() {
    QFileInfo projectFile = QFileInfo(getProjectFilePath());
    CreateFMUDialog dialog(projectFile.absolutePath()+"/"+projectFile.baseName()+".fmu");
    int result = dialog.exec();
    if(result) {
      if(not dialog.getFileName().isEmpty()) {
	QMessageBox::StandardButton ret = QMessageBox::Yes;
	if(QFileInfo::exists(dialog.getFileName()))
	  ret = QMessageBox::question(this, "Replace file", "A file named " + dialog.getFileName() + " already exists. Do you want to replace it?", QMessageBox::Yes | QMessageBox::No);
	if(ret == QMessageBox::Yes) {
	  shared_ptr<DOMDocument> doc=mbxmlparser->createDocument();
	  doc->setDocumentURI(this->doc->getDocumentURI());
	  auto *newDocElement = static_cast<DOMElement*>(doc->importNode(this->doc->getDocumentElement(), true));
	  doc->insertBefore(newDocElement, nullptr);
	  project->processIDAndHref(newDocElement);
	  QString uniqueTempDir_ = QString::fromStdString(uniqueTempDir.generic_string());
	  QString projectFile = uniqueTempDir_+"/Project.mbsx";
	  serializer->writeToURI(doc.get(), X()%projectFile.toStdString());
	  QStringList arg;
	  if(dialog.cosim()) arg.append("--cosim");
	  if(dialog.nocompress()) arg.append("--nocompress");
	  arg.append(projectFile);
	  echoView->clearOutput();
	  echoView->showXMLCode(false);
	  process.setWorkingDirectory(uniqueTempDir_);
	  fmuFileName = dialog.getFileName();
	  process.start(QString::fromStdString((installPath/"bin"/"mbsimCreateFMU").string()), arg);
	  connect(&process,QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),this,[=]() {
	    if(QFile::exists(fmuFileName))
	      QFile::remove(fmuFileName);
	    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/"+"mbsim.fmu",fmuFileName);
	  });
	  statusBar()->showMessage(tr("Create FMU"));
	}
      }
    }
  }

  void MainWindow::updateEchoView() {
    auto data=process.readAllStandardOutput();
    echoView->addOutputText(data.data());
    echoView->updateOutput(true);
  }

  void MainWindow::updateStatus() {
    // call this function only every 0.25 sec
    if(statusTime.elapsed()<250)
      return;
    statusTime.restart();

    // show only last line
    auto data=process.readAllStandardError();
    string s=data.data();
    s.resize(s.length()-1);
    auto i=s.rfind('\n');
    i = i==string::npos ? 0 : i+1;
    statusBar()->showMessage(QString::fromStdString(s.substr(i)));
  }

  QString MainWindow::getProjectFilePath() const {
    return QUrl(QString::fromStdString(X()%doc->getDocumentURI())).toLocalFile();
  }

  void MainWindow::openElementEditor(bool config) {
    if(not editorIsOpen()) {
      QModelIndex index = elementView->selectionModel()->currentIndex();
      auto *element = dynamic_cast<EmbedItemData*>(static_cast<ElementTreeModel*>(elementView->model())->getItem(index)->getItemData());
      if(element) {
	menuBar()->setDisabled(true);
        updateParameters(element);
        editor = element->createPropertyDialog();
        editor->setAttribute(Qt::WA_DeleteOnClose);
        if(config)
          editor->toWidget();
        else
          editor->setCancel(false);
        editor->show();
        connect(editor,&QDialog::finished,this,[=](){
          if(editor->result()==QDialog::Accepted) {
            auto *fileItem = element->getDedicatedFileItem();
            if(fileItem)
              fileItem->setModified(true);
            else
	      setWindowModified(true);
	    if(editor->getCancel())
              updateUndos();
            editor->fromWidget();
	    updateNames(element);
            if(getAutoRefresh()) refresh();
          }
          menuBar()->setEnabled(true);
          editor = nullptr;
        });
        connect(editor,&ElementPropertyDialog::apply,this,[=](){
          auto *fileItem = element->getDedicatedFileItem();
          if(fileItem)
            fileItem->setModified(true);
          else
            setWindowModified(true);
          if(editor->getCancel())
            updateUndos();
          editor->fromWidget();
	  updateNames(element);
          if(getAutoRefresh()) refresh();
          editor->setCancel(true);
        });
//        connect(editor,&ElementPropertyDialog::showXMLHelp,this,[=](){
//          // generate url for current element
//          string url="file://"+(installPath/"share"/"mbxmlutils"/"doc").string();
//          string ns=element->getXMLType().first;
//          replace(ns.begin(), ns.end(), ':', '_');
//          replace(ns.begin(), ns.end(), '.', '_');
//          replace(ns.begin(), ns.end(), '/', '_');
//          url+="/"+ns+"/index.html#"+element->getXMLType().second;
//          // open in XML help dialog
//          xmlHelp(QString::fromStdString(url));
//        });
      }
    }
  }

  void MainWindow::openParameterEditor(bool config) {
    if(not editorIsOpen()) {
      QModelIndex index = parameterView->selectionModel()->currentIndex();
      auto *parameter = dynamic_cast<Parameter*>(static_cast<ParameterTreeModel*>(parameterView->model())->getItem(index)->getItemData());
      if(parameter) {
        menuBar()->setDisabled(true);
        updateParameters(parameter->getParent(),true);
        editor = parameter->createPropertyDialog();
        editor->setAttribute(Qt::WA_DeleteOnClose);
        if(config)
          editor->toWidget();
        else
          editor->setCancel(false);
        editor->show();
        connect(editor,&QDialog::finished,this,[=](){
          if(editor->result()==QDialog::Accepted) {
            auto* fileItem = parameter->getParent()->getDedicatedParameterFileItem();
            if(fileItem)
              fileItem->setModified(true);
            else
	      setWindowModified(true);
	    if(editor->getCancel())
              updateUndos();
            editor->fromWidget();
	    updateValues(parameter->getParent());
            if(getAutoRefresh()) refresh();
            if(getStatusUpdate()) parameter->getParent()->updateStatus();
          }
        menuBar()->setEnabled(true);
        editor = nullptr;
        });
        connect(editor,&ParameterPropertyDialog::apply,this,[=](){
          auto* fileItem = parameter->getParent()->getDedicatedParameterFileItem();
          if(fileItem)
            fileItem->setModified(true);
          else
            setWindowModified(true);
          if(editor->getCancel())
            updateUndos();
          editor->fromWidget();
	  updateValues(parameter->getParent());
          if(getAutoRefresh()) refresh();
          editor->setCancel(true);
	  if(getStatusUpdate()) parameter->getParent()->updateStatus();
        });
      }
    }
  }

  void MainWindow::openCloneEditor() {
    if(not editorIsOpen()) {
      QModelIndex index = elementView->selectionModel()->currentIndex();
      auto *element = dynamic_cast<Element*>(static_cast<ElementTreeModel*>(elementView->model())->getItem(index)->getItemData());
      if(element) {
        menuBar()->setDisabled(true);
        updateParameters(element);
        editor = new ClonePropertyDialog(element);
        editor->setAttribute(Qt::WA_DeleteOnClose);
        editor->toWidget();
        editor->show();
        connect(editor,&QDialog::finished,this,[=](){
          if(editor->result()==QDialog::Accepted) {
            auto* fileItem = element->getDedicatedFileItem();
            if(fileItem)
              fileItem->setModified(true);
            else
	      setWindowModified(true);
	    if(editor->getCancel())
              updateUndos();
            editor->fromWidget();
            if(getAutoRefresh()) refresh();
          }
	  menuBar()->setEnabled(true);
          editor = nullptr;
        });
        connect(editor,&ElementPropertyDialog::apply,this,[=](){
          auto* fileItem = element->getDedicatedFileItem();
          if(fileItem)
            fileItem->setModified(true);
          else
            setWindowModified(true);
          if(editor->getCancel())
            updateUndos();
          editor->fromWidget();
          if(getAutoRefresh()) refresh();
        });
      }
    }
  }

  FileItemData* MainWindow::addFile(const QFileInfo &fileName) {
    for(size_t i=0; i<file.size(); i++) {
      if(fileName==file[i]->getFileInfo())
        return file[i];
    }
    DOMDocument *doc = mw->parser->parseURI(MBXMLUtils::X()%fileName.absoluteFilePath().toStdString());
    MBXMLUtils::DOMParser::handleCDATA(doc->getDocumentElement());
    auto *fileItem = new FileItemData(doc);
    file.push_back(fileItem);
    static_cast<FileTreeModel*>(fileView->model())->createFileItem(fileItem);
    return fileItem;
  }

  void MainWindow::removeFile(FileItemData *fileItem) {
    auto *model = static_cast<FileTreeModel*>(fileView->model());
    QModelIndex index = model->findItem(fileItem,QModelIndex());
    fileView->model()->removeRow(index.row(),QModelIndex());
    for(auto it = file.begin(); it != file.end(); ++it) {
      if(*it==fileItem) {
        file.erase(it);
        break;
      }
    }
    delete fileItem;
  }

  void MainWindow::saveReferencedFile(int i) {
    try {
      serializer->writeToURI(file[i]->getXMLDocument(), X()%file[i]->getFileInfo().absoluteFilePath().toStdString());
      file[i]->setModified(false);
    }
    catch(const std::exception &ex) {
      mw->setExitBad();
      cerr << ex.what() << endl;
    }
    catch(const DOMException &ex) {
      mw->setExitBad();
      cerr << X()%ex.getMessage() << endl;
    }
    catch(...) {
      mw->setExitBad();
      cerr << "Unknown exception." << endl;
    }
  }

  void MainWindow::convertDocument() {
    QString file=QFileDialog::getOpenFileName(this, "Open MBSim file", getProjectFilePath(), "MBSim files (*.mbsx);;MBSim model files (*.mbsmx);;XML files (*.xml);;All files (*.*)");
    if(not(file.isEmpty())) {
      DOMDocument *doc = parser->parseURI(X()%file.toStdString());
      DOMNodeList* list = doc->getElementsByTagName(X()%"naturalModeScaleFactor");
      for(size_t j=0; j<list->getLength(); j++)
	doc->renameNode(list->item(j),X()%MBSIMCONTROL.getNamespaceURI(),X()%"normalModeScaleFactor");
      list = doc->getElementsByTagName(X()%"naturalModeScale");
      for(size_t j=0; j<list->getLength(); j++)
	doc->renameNode(list->item(j),X()%MBSIMCONTROL.getNamespaceURI(),X()%"normalModeScale");
      list = doc->getElementsByTagName(X()%"visualizeNaturalModeShapes");
      for(size_t j=0; j<list->getLength(); j++)
	doc->renameNode(list->item(j),X()%MBSIMCONTROL.getNamespaceURI(),X()%"visualizeNormalModes");
      list = doc->getElementsByTagName(X()%"SymbolicFunction");
      for(size_t j=0; j<list->getLength(); j++) {
	if(not E(static_cast<DOMElement*>(list->item(j)))->getFirstElementChildNamed(MBSIM%"definition")) {
	  auto *node = doc->renameNode(list->item(j),X()%MBSIM.getNamespaceURI(),X()%"definition");
	  DOMElement *ele = D(doc)->createElement(MBSIM%"SymbolicFunction");
	  node->getParentNode()->insertBefore(ele,node);
	  ele->insertBefore(node,nullptr);
	}
      }
      file=QFileDialog::getSaveFileName(this, "Save MBSim file", file, "MBSim files (*.mbsx);;MBSim model files (*.mbsmx);;XML files (*.xml);;All files (*.*)");
      if(not(file.isEmpty())) {
	try {
	  serializer->writeToURI(doc, X()%(file.toStdString()));
	}
	catch(const std::exception &ex) {
	  cerr << ex.what() << endl;
	}
      }
    }
  }

}
