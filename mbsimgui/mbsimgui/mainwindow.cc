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
#include <mbxmlutils/eval.h>
#include <mbxmlutils/preprocess.h>
#include <boost/dll.hpp>
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

  bool currentTask;

  MainWindow *mw;

  vector<boost::filesystem::path> dependencies;

  MainWindow::MainWindow(QStringList &arg) : project(nullptr), inlineOpenMBVMW(nullptr), allowUndo(true), maxUndo(10), autoRefresh(true), doc(nullptr), elementBuffer(nullptr,false), parameterBuffer(nullptr,false), installPath(boost::dll::program_location().parent_path().parent_path()) {
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
    arguments << "--onlyListSchemas";
    QProcess processGetSchemas;
    processGetSchemas.start(program,arguments);
    processGetSchemas.waitForFinished(-1);
    QStringList line=QString(processGetSchemas.readAllStandardOutput().data()).split("\n");
    set<bfs::path> schemas;
    for(int i=0; i<line.size(); ++i)
      if(!line.at(i).isEmpty())
        schemas.insert(line.at(i).toStdString());

    mbxmlparser=DOMParser::create(schemas);

    elementView = new ElementView;
    parameterView = new ParameterView;
    echoView = new EchoView(this);
    fileView = new FileView;

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

    initInlineOpenMBV();

    MBSimObjectFactory::initialize();

    QMenu *GUIMenu = new QMenu("GUI", menuBar());
    menuBar()->addMenu(GUIMenu);

    QAction *action = GUIMenu->addAction(QIcon::fromTheme("document-properties"), "Options", this, &MainWindow::openOptionsMenu);
    action->setStatusTip(tr("Open options menu"));

    GUIMenu->addSeparator();

    elementViewFilter = new OpenMBVGUI::AbstractViewFilter(elementView, 0, 1);
    elementViewFilter->hide();

    parameterViewFilter = new OpenMBVGUI::AbstractViewFilter(parameterView, 0, -2);
    parameterViewFilter->hide();

    GUIMenu->addSeparator();

    action = GUIMenu->addAction(QIcon::fromTheme("application-exit"), "E&xit", this, &MainWindow::close);
    action->setShortcut(QKeySequence::Quit);
    action->setStatusTip(tr("Exit the application"));

    for (auto & recentProjectFileAct : recentProjectFileActs) {
      recentProjectFileAct = new QAction(this);
      recentProjectFileAct->setVisible(false);
      connect(recentProjectFileAct, &QAction::triggered, this, &MainWindow::openRecentProjectFile);
    }
    QMenu *menu = new QMenu("Project", menuBar());
    action = menu->addAction(QIcon::fromTheme("document-new"), "New", this, &MainWindow::newProject);
    action->setShortcut(QKeySequence::New);
    action = menu->addAction(QIcon::fromTheme("document-open"), "Open", this, QOverload<>::of(&MainWindow::loadProject));
    action->setShortcut(QKeySequence::Open);
    action = menu->addAction(QIcon::fromTheme("document-save-as"), "Save as", this, &MainWindow::saveProjectAs);
    action->setShortcut(QKeySequence::SaveAs);
    actionSaveProject = menu->addAction(QIcon::fromTheme("document-save"), "Save", this, [=](){ this->saveProject(); });
    actionSaveProject->setShortcut(QKeySequence::Save);
    menu->addSeparator();
    for (auto & recentProjectFileAct : recentProjectFileActs)
      menu->addAction(recentProjectFileAct);
    updateRecentProjectFileActions();
    menuBar()->addMenu(menu);

    menu = new QMenu("Referenced files", menuBar());
    menu->addAction(QIcon::fromTheme("document-save"), "Save all", this, [=](){ for(size_t i=0; i<file.size(); i++) if(file[i]->getModified()) saveReferencedFile(i); });
    menuBar()->addMenu(menu);
//    menu->addAction(QIcon::fromTheme("document-save"), "Save", this, [=](){
//      QModelIndex index = fileView->selectionModel()->currentIndex();
//      auto *model = static_cast<FileTreeModel*>(fileView->model());
//      auto *fileItem=dynamic_cast<FileItemData*>(model->getItem(index)->getItemData());
//      });

    menu = new QMenu("Edit", menuBar());
    action = menu->addAction(QIcon::fromTheme("document-properties"), "Edit", this, &MainWindow::edit);
    action->setShortcut(QKeySequence("Ctrl+E"));
    menu->addSeparator();
    actionUndo = menu->addAction(QIcon::fromTheme("edit-undo"), "Undo", this, &MainWindow::undo);
    actionUndo->setShortcut(QKeySequence::Undo);
    actionUndo->setDisabled(true);
    actionRedo = menu->addAction(QIcon::fromTheme("edit-redo"), "Redo", this, &MainWindow::redo);
    actionRedo->setShortcut(QKeySequence::Redo);
    actionRedo->setDisabled(true);
    menu->addSeparator();
    action = menu->addAction(QIcon::fromTheme("edit-copy"), "Copy", this, &MainWindow::copy);
    action->setShortcut(QKeySequence::Copy);
    action = menu->addAction(QIcon::fromTheme("edit-cut"), "Cut", this, &MainWindow::cut);
    action->setShortcut(QKeySequence::Cut);
    action = menu->addAction(QIcon::fromTheme("edit-paste"), "Paste", this, &MainWindow::paste);
    action->setShortcut(QKeySequence::Paste);
    menu->addSeparator();
    action = menu->addAction(QIcon::fromTheme("edit-delete"), "Remove", this, &MainWindow::remove);
    action->setShortcut(QKeySequence::Delete);
    menu->addSeparator();
    action = menu->addAction(QIcon::fromTheme("go-up"), "Move up", this, &MainWindow::moveUp);
    action->setShortcut(QKeySequence("Ctrl+Up"));
    action = menu->addAction(QIcon::fromTheme("go-down"), "Move down", this, &MainWindow::moveDown);
    action->setShortcut(QKeySequence("Ctrl+Down"));
    menuBar()->addMenu(menu);

    menu = new QMenu("Export", menuBar());
    actionSaveDataAs = menu->addAction("Export all data", this, &MainWindow::saveDataAs);
    actionSaveMBSimH5DataAs = menu->addAction("Export MBSim data file", this, &MainWindow::saveMBSimH5DataAs);
    actionSaveOpenMBVDataAs = menu->addAction("Export OpenMBV data", this, &MainWindow::saveOpenMBVDataAs);
    actionSaveStateVectorAs = menu->addAction("Export state vector", this, &MainWindow::saveStateVectorAs);
    actionSaveStateTableAs = menu->addAction("Export state table", this, &MainWindow::saveStateTableAs);
    actionSaveEigenanalysisAs = menu->addAction("Export eigenanalysis", this, &MainWindow::saveEigenanalysisAs);
    actionSaveHarmonicResponseAnalysisAs = menu->addAction("Export harmonic response analysis", this, &MainWindow::saveHarmonicResponseAnalysisAs);
    menuBar()->addMenu(menu);

    menuBar()->addSeparator();
    QMenu *helpMenu = new QMenu("Help", menuBar());
    helpMenu->addAction(QIcon::fromTheme("help-contents"), "Contents", this, &MainWindow::help);
    helpMenu->addAction(QIcon::fromTheme("help-xml"), "XML Help", this, [=](){ this->xmlHelp(); });
    helpMenu->addAction(QIcon::fromTheme("help-about"), "About", this, &MainWindow::about);
    menuBar()->addMenu(helpMenu);

    QToolBar *toolBar = addToolBar("Tasks");
    toolBar->setObjectName("toolbar/tasks");
    actionSimulate = toolBar->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_MediaPlay)),"Start simulation");
    actionSimulate->setStatusTip(tr("Simulate the multibody system"));
    connect(actionSimulate,&QAction::triggered,this,&MainWindow::simulate);
    toolBar->addAction(actionSimulate);
    QAction *actionInterrupt = toolBar->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_MediaStop)),"Interrupt simulation");
    connect(actionInterrupt,&QAction::triggered,this,&MainWindow::interrupt);
    toolBar->addAction(actionInterrupt);
    actionRefresh = toolBar->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_BrowserReload)),"Refresh 3D view");
    connect(actionRefresh,&QAction::triggered,this,&MainWindow::refresh);
    toolBar->addAction(actionRefresh);
    actionOpenMBV = toolBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"openmbv.svg").string())),"OpenMBV");
    connect(actionOpenMBV,&QAction::triggered,this,&MainWindow::openmbv);
    toolBar->addAction(actionOpenMBV);
    actionH5plotserie = toolBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"h5plotserie.svg").string())),"H5plotserie");
    connect(actionH5plotserie,&QAction::triggered,this,&MainWindow::h5plotserie);
    toolBar->addAction(actionH5plotserie);
    actionEigenanalysis = toolBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"eigenanalysis.svg").string())),"Eigenanalysis");
    connect(actionEigenanalysis,&QAction::triggered,this,&MainWindow::eigenanalysis);
    toolBar->addAction(actionEigenanalysis);
    actionHarmonicResponseAnalysis = toolBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"frequency_response.svg").string())),"Harmonic response analysis");
    connect(actionHarmonicResponseAnalysis,&QAction::triggered,this,&MainWindow::harmonicResponseAnalysis);
    actionDebug = toolBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"debug.svg").string())),"Debug model");
    connect(actionDebug,&QAction::triggered,this,&MainWindow::debug);
    toolBar->addAction(actionDebug);
    QAction *actionKill = toolBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"kill.svg").string())),"Kill simulation");
    connect(actionKill,&QAction::triggered,this,&MainWindow::kill);
    toolBar->addAction(actionKill);

    elementView->setModel(new ElementTreeModel(this));
    elementView->setColumnWidth(0,250);
    elementView->setColumnWidth(1,200);
    elementView->hideColumn(1);

    parameterView->setModel(new ParameterTreeModel(this));
    parameterView->setColumnWidth(0,150);
    parameterView->setColumnWidth(1,200);

    fileView->setModel(new FileTreeModel(this));
    fileView->setColumnWidth(0,250);
    fileView->setColumnWidth(1,50);
    fileView->hideColumn(3);

    connect(elementView, &ElementView::pressed, this, &MainWindow::elementViewClicked);
    connect(parameterView, &ParameterView::pressed, this, &MainWindow::parameterViewClicked);

    QDockWidget *dockWidget5 = new QDockWidget("File list", this);
    dockWidget5->setObjectName("dockWidget/files");
    addDockWidget(Qt::LeftDockWidgetArea, dockWidget5);
    dockWidget5->setWidget(fileView);

    QDockWidget *dockWidget1 = new QDockWidget("Model tree", this);
    dockWidget1->setObjectName("dockWidget/mbs");
    addDockWidget(Qt::LeftDockWidgetArea,dockWidget1);
    QWidget *widget1 = new QWidget(dockWidget1);
    dockWidget1->setWidget(widget1);
    auto *widgetLayout1 = new QVBoxLayout(widget1);
    widgetLayout1->setContentsMargins(0,0,0,0);
    widget1->setLayout(widgetLayout1);
    widgetLayout1->addWidget(elementViewFilter);
    widgetLayout1->addWidget(elementView);

    tabifyDockWidget(dockWidget5,dockWidget1);

    QDockWidget *dockWidget3 = new QDockWidget("Parameter tree", this);
    dockWidget3->setObjectName("dockWidget/parameters");
    addDockWidget(Qt::LeftDockWidgetArea,dockWidget3);
    QWidget *widget3 = new QWidget(dockWidget3);
    dockWidget3->setWidget(widget3);
    auto *widgetLayout3 = new QVBoxLayout(widget3);
    widgetLayout3->setContentsMargins(0,0,0,0);
    widget3->setLayout(widgetLayout3);
    widgetLayout3->addWidget(parameterViewFilter);
    widgetLayout3->addWidget(parameterView);

    QDockWidget *dockWidget4 = new QDockWidget("MBSim Echo Area", this);
    dockWidget4->setObjectName("dockWidget/echoArea");
    addDockWidget(Qt::BottomDockWidgetArea, dockWidget4);
    dockWidget4->setWidget(echoView);

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
    if(projectFile.size())
      loadProject(QDir::current().absoluteFilePath(projectFile));
    else
      newProject();

    setAcceptDrops(true);

    connect(&autoSaveTimer, &QTimer::timeout, this, &MainWindow::autoSaveProject);
    autoSaveTimer.start(settings.value("mainwindow/options/autosaveinterval", 5).toInt()*60000);
    statusTime.start();

    setWindowIcon(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"mbsimgui.svg").string())));

    // auto exit if everything is finished
    if(arg.contains("--autoExit")) {
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

    openOptionsMenu(true);
  }

  void MainWindow::autoSaveProject() {
    saveProject("./.Project.mbsx",false);
  }

  void MainWindow::processFinished(int exitCode, QProcess::ExitStatus exitStatus) {
    updateEchoView();
    if(currentTask==1 && bfs::exists(uniqueTempDir.generic_string()+"/MBS_tmp.ombvx") && process.state()==QProcess::NotRunning) {
      inlineOpenMBVMW->openFile(uniqueTempDir.generic_string()+"/MBS_tmp.ombvx");
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
          QString autoExportDir = settings.value("mainwindow/options/autoexportdir", "./").toString();
          saveMBSimH5Data(autoExportDir+"/MBS.mbsh5");
          saveOpenMBVXMLData(autoExportDir+"/MBS.ombvx");
          saveOpenMBVH5Data(autoExportDir+"/MBS.ombvh5");
          if(saveFinalStateVector)
            saveStateVector(autoExportDir+"/statevector.asc");
        }
        actionSaveDataAs->setDisabled(false);
        actionSaveMBSimH5DataAs->setDisabled(false);
        actionSaveOpenMBVDataAs->setDisabled(false);
        if(saveFinalStateVector)
          actionSaveStateVectorAs->setDisabled(false);
        actionSaveStateTableAs->setDisabled(false);
        if(dynamic_cast<Eigenanalyzer*>(getProject()->getSolver())) {
          actionSaveEigenanalysisAs->setDisabled(false);
          actionEigenanalysis->setDisabled(false);
        }
        if(dynamic_cast<HarmonicResponseAnalyzer*>(getProject()->getSolver())) {
          actionSaveHarmonicResponseAnalysisAs->setDisabled(false);
          actionHarmonicResponseAnalysis->setDisabled(false);
        }
        actionOpenMBV->setDisabled(false);
        actionH5plotserie->setDisabled(false);
      }
      else {
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
    inlineOpenMBVMW = new OpenMBVGUI::MainWindow(arg);

    connect(inlineOpenMBVMW, &OpenMBVGUI::MainWindow::objectSelected, this, &MainWindow::selectElement);
    connect(inlineOpenMBVMW, &OpenMBVGUI::MainWindow::objectDoubleClicked, this, [=](){ openElementEditor(); });
  }

  MainWindow::~MainWindow() {
    process.waitForFinished(-1);
    centralWidget()->layout()->removeWidget(inlineOpenMBVMW);
    delete inlineOpenMBVMW;
    // use nothrow boost::filesystem functions to avoid exceptions in this dtor
    boost::system::error_code ec;
    bfs::remove_all(uniqueTempDir, ec);
    bfs::remove("./.Project.mbsx", ec);
    delete project;
    parser->release();
    serializer->release();
    basicSerializer->release();
  }

  void MainWindow::setProjectChanged(bool changed) { 
    setWindowModified(changed);
    if(changed) {
      QSettings settings;
      xercesc::DOMDocument* oldDoc = static_cast<xercesc::DOMDocument*>(doc->cloneNode(true));
      oldDoc->setDocumentURI(doc->getDocumentURI());
      undos.push_back(oldDoc);
      if(undos.size() > maxUndo)
        undos.pop_front();
      redos.clear();
      if(allowUndo) actionUndo->setEnabled(true);
      actionRedo->setDisabled(true);
    }
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
    menu.setShowFilters(settings.value("mainwindow/options/showfilters", false).toBool());
    menu.setAutoRefresh(settings.value("mainwindow/options/autorefresh", true).toBool());

#ifdef _WIN32
    QFile file(qgetenv("APPDATA")+"/mbsim-env/mbsimxml.modulepath");
#else
    QFile file(qgetenv("HOME")+"/.config/mbsim-env/mbsimxml.modulepath");
#endif
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    menu.setModulePath(file.readAll());
    file.close();

    int res = 1;
    if(!justSetOptions)
      res = menu.exec();
    if(res == 1) {
      settings.setValue("mainwindow/options/autosave",         menu.getAutoSave());
      settings.setValue("mainwindow/options/autosaveinterval", menu.getAutoSaveInterval());
      settings.setValue("mainwindow/options/autoexport",       menu.getAutoExport());
      settings.setValue("mainwindow/options/autoexportdir",    menu.getAutoExportDir());
      settings.setValue("mainwindow/options/savestatevector",  menu.getSaveStateVector());
      settings.setValue("mainwindow/options/maxundo",          menu.getMaxUndo());
      settings.setValue("mainwindow/options/showfilters",      menu.getShowFilters());
      settings.setValue("mainwindow/options/autorefresh",      menu.getAutoRefresh());

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
      autoRefresh = menu.getAutoRefresh();
    }
  }

  void MainWindow::highlightObject(const string &ID) {
    currentID = ID;
    inlineOpenMBVMW->highlightObject(ID);
  }

  void MainWindow::elementViewClicked(const QModelIndex &current) {
    if(allowUndo) {
      auto *model = static_cast<ElementTreeModel*>(elementView->model());
      auto *embeditem=dynamic_cast<EmbedItemData*>(model->getItem(current)->getItemData());
      if(embeditem) {
        auto *pmodel = static_cast<ParameterTreeModel*>(parameterView->model());
        vector<EmbedItemData*> parents = embeditem->getEmbedItemParents();
        QModelIndex index = pmodel->index(0,0);
        pmodel->removeRow(index.row(), index.parent());
        if(!parents.empty()) {
          pmodel->createParameterItem(parents[0]->getParameters());
          for(size_t i=0; i<parents.size()-1; i++)
            pmodel->createParameterItem(parents[i+1]->getParameters(),parents[i]->getParameters()->getModelIndex());
          pmodel->createParameterItem(embeditem->getParameters(),parents[parents.size()-1]->getParameters()->getModelIndex());
        }
        else
          pmodel->createParameterItem(embeditem->getParameters());
        parameterView->expandAll();
        auto *element=dynamic_cast<Element*>(model->getItem(current)->getItemData());
        if(element)
          highlightObject(element->getID());
        else
          highlightObject("");
      }
      else {
        parameterView->selectionModel()->clearSelection();
        fileView->selectionModel()->clearSelection();
      }
    }
    if(QApplication::mouseButtons()==Qt::RightButton) {
      TreeItemData *itemData = static_cast<ElementTreeModel*>(elementView->model())->getItem(current)->getItemData();
      QMenu *menu = itemData->createContextMenu();
      menu->exec(QCursor::pos());
      delete menu;
    }
  }

  void MainWindow::parameterViewClicked(const QModelIndex &current) {
    auto *item = dynamic_cast<ParameterItem*>(static_cast<ParameterTreeModel*>(parameterView->model())->getItem(current)->getItemData());
    if(QApplication::mouseButtons()==Qt::RightButton) {
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
      setProjectChanged(false);
      actionOpenMBV->setDisabled(true);
      actionH5plotserie->setDisabled(true);
      actionEigenanalysis->setDisabled(true);
      actionHarmonicResponseAnalysis->setDisabled(true);
      actionSaveDataAs->setDisabled(true);
      actionSaveMBSimH5DataAs->setDisabled(true);
      actionSaveOpenMBVDataAs->setDisabled(true);
      actionSaveStateVectorAs->setDisabled(true);
      actionSaveStateTableAs->setDisabled(true);
      actionSaveEigenanalysisAs->setDisabled(true);
      actionSaveHarmonicResponseAnalysisAs->setDisabled(true);
      actionSaveProject->setDisabled(true);

      auto *pmodel = static_cast<ParameterTreeModel*>(parameterView->model());
      QModelIndex index = pmodel->index(0,0);
      pmodel->removeRows(index.row(), pmodel->rowCount(QModelIndex()), index.parent());

      auto *model = static_cast<ElementTreeModel*>(elementView->model());
      index = model->index(0,0);
      model->removeRows(index.row(), model->rowCount(QModelIndex()), index.parent());

      auto *fmodel = static_cast<FileTreeModel*>(fileView->model());
      index = fmodel->index(0,0);
      fmodel->removeRows(index.row(), fmodel->rowCount(QModelIndex()), index.parent());

      delete project;

      file.clear();
      idMap.clear();
      IDcounter = 0;

      doc = impl->createDocument();
      doc->setDocumentURI(X()%QUrl::fromLocalFile(QDir::currentPath()+"/Project.mbsx").toString().toStdString());

      project = new Project;
      project->createXMLElement(doc);

      model->createProjectItem(project,QModelIndex());

//      model->createGroupItem(project->getDynamicSystemSolver(),QModelIndex());
//      model->createSolverItem(getProject()->getSolver(),model->index(1,0));

      projectFile="";
      refresh();
      setWindowTitle("Project.mbsx[*]");
    }
  }

  void MainWindow::loadProject(const QString &fileName) {
    if(QFile::exists(fileName)) {
      undos.clear();
      actionUndo->setDisabled(true);
      actionRedo->setDisabled(true);
      elementBuffer.first = nullptr;
      parameterBuffer.first = nullptr;
      setProjectChanged(false);
      actionOpenMBV->setDisabled(true);
      actionH5plotserie->setDisabled(true);
      actionEigenanalysis->setDisabled(true);
      actionHarmonicResponseAnalysis->setDisabled(true);
      actionSaveDataAs->setDisabled(true);
      actionSaveMBSimH5DataAs->setDisabled(true);
      actionSaveOpenMBVDataAs->setDisabled(true);
      actionSaveStateVectorAs->setDisabled(true);
      actionSaveStateTableAs->setDisabled(true);
      actionSaveEigenanalysisAs->setDisabled(true);
      actionSaveHarmonicResponseAnalysisAs->setDisabled(true);
      actionSaveProject->setDisabled(false);
      projectFile = QDir::current().relativeFilePath(fileName);
      setCurrentProjectFile(fileName);
      try { 
        doc = parser->parseURI(X()%fileName.toStdString());
        DOMParser::handleCDATA(doc->getDocumentElement());
      }
      catch(const std::exception &ex) {
        cout << ex.what() << endl;
        return;
      }
      catch(...) {
        cout << "Unknown exception." << endl;
        return;
      }
      setWindowTitle(projectFile+"[*]");
      rebuildTree();
      refresh();
    }
    else
      QMessageBox::warning(nullptr, "Project load", "Project file does not exist.");
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
      if(modifyStatus) setProjectChanged(false);
      return true;
    }
    catch(const std::exception &ex) {
      cout << ex.what() << endl;
    }
    catch(const DOMException &ex) {
      cout << X()%ex.getMessage() << endl;
    }
    catch(...) {
      cout << "Unknown exception." << endl;
    }
    return false;
  }

  void MainWindow::selectSolver(Solver *solver) {
    setProjectChanged(true);
    DOMElement *ele = nullptr;
    auto *parameterFileItem = getProject()->getSolver()->getParameterFileItem();
    DOMElement *embed = getProject()->getSolver()->getEmbedXMLElement();
    if(parameterFileItem)
      ele = parameterFileItem->getXMLElement();
    else if(embed)
      ele = MBXMLUtils::E(embed)->getFirstElementChildNamed(MBXMLUtils::PV%"Parameter");
    QModelIndex pindex = getProject()->getSolver()->getParameters()->getModelIndex();
    static_cast<ParameterTreeModel*>(parameterView->model())->removeRow(pindex.row(), pindex.parent());
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    model->removeRow(getProject()->getSolver()->getModelIndex().row(), project->getModelIndex());
    if(getProject()->getSolver()->getFileItem())
      E(embed)->removeAttribute("href");
    else
      getProject()->getSolver()->removeXMLElement(false);
    getProject()->setSolver(solver);
    model->createSolverItem(solver,getProject()->getModelIndex());
    elementView->selectionModel()->setCurrentIndex(solver->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    solver->createXMLElement(embed?embed:getProject()->getXMLElement());
    solver->setEmbedXMLElement(embed);
    if(ele) {
      solver->setParameterFileItem(parameterFileItem);
      std::vector<Parameter*> param = Parameter::createParameters(ele);
      for(auto & i : param)
        solver->addParameter(i);
    }
  }

  // update model parameters including additional paramters from paramList
  void MainWindow::updateParameters(EmbedItemData *item, bool exceptLatestParameter) {
    shared_ptr<xercesc::DOMDocument> doc=mbxmlparser->createDocument();
    doc->setDocumentURI(this->doc->getDocumentURI());
    DOMElement *eleE0 = D(doc)->createElement(PV%"Embed");
    doc->insertBefore(eleE0,nullptr);
    DOMElement *eleP0 = D(doc)->createElement(PV%"Parameter");
    eleE0->insertBefore(eleP0,nullptr);
    DOMElement *eleE1 = D(doc)->createElement(PV%"Embed");
    eleE0->insertBefore(eleE1,nullptr);
    DOMElement *eleP1 = D(doc)->createElement(PV%"Parameter");
    eleE1->insertBefore(eleP1,nullptr);
    string evalName="octave"; // default evaluator
    if(project)
      evalName = project->getEvaluator();
    else {
      DOMElement *root = this->doc->getDocumentElement();
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
    if(item) {
      vector<EmbedItemData*> parents = item->getEmbedItemParents();
      for(auto & parent : parents) {
        for(size_t j=0; j<parent->getNumberOfParameters(); j++) {
          DOMNode *node = doc->importNode(parent->getParameter(j)->getXMLElement(),true);
          eleP0->insertBefore(node,nullptr);
          boost::filesystem::path orgFileName=E(parent->getParameter(j)->getXMLElement())->getOriginalFilename();
          DOMProcessingInstruction *filenamePI=node->getOwnerDocument()->createProcessingInstruction(X()%"OriginalFilename",
              X()%orgFileName.string());
          node->insertBefore(filenamePI, node->getFirstChild());
        }
      }
      for(int j=0; j<item->getNumberOfParameters()-exceptLatestParameter; j++) {
        DOMNode *node = doc->importNode(item->getParameter(j)->getXMLElement(),true);
        eleP1->insertBefore(node,nullptr);
        boost::filesystem::path orgFileName=E(item->getParameter(j)->getXMLElement())->getOriginalFilename();
        DOMProcessingInstruction *filenamePI=node->getOwnerDocument()->createProcessingInstruction(X()%"OriginalFilename",
            X()%orgFileName.string());
        node->insertBefore(filenamePI, node->getFirstChild());
      }
      try {
        D(doc)->validate();
        DOMElement *ele = doc->getDocumentElement()->getFirstElementChild();
        eval->addParamSet(ele);
        if(item->getXMLElement()) {
          string counterName = item->getEmbedXMLElement()?E(item->getEmbedXMLElement())->getAttribute("counterName"):"";
          if(not counterName.empty())
            eval->addParam(eval->cast<string>(eval->stringToValue(counterName,item->getEmbedXMLElement(),false)),eval->create(1.0));
        }
        ele = ele->getNextElementSibling()->getFirstElementChild();
        eval->addParamSet(ele);
      }
      catch(const std::exception &error) {
        cout << string("An exception occurred in updateParameters: ") + error.what() << endl;
      }
      catch(...) {
        cout << "An unknown exception occurred in updateParameters." << endl;
      }
    }
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
        saveEigenanalysis(dir+"/eigenanalysis.mat");
        saveHarmonicResponseAnalysis(dir+"/harmonic_response_analysis.mat");
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

  void MainWindow::saveEigenanalysisAs() {
    QString file=QFileDialog::getSaveFileName(this, "Export eigenanalysis", getProjectDir().absoluteFilePath("eigenanalysis.mat"), "MAT files (*.mat)");
    if(file!="") {
      saveEigenanalysis(file);
    }
  }

  void MainWindow::saveEigenanalysis(const QString &file) {
    if(QFile::exists(file))
      QFile::remove(file);
    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/eigenanalysis.mat",file);
  }

  void MainWindow::saveHarmonicResponseAnalysisAs() {
    QString file=QFileDialog::getSaveFileName(this, "Export harmonic response analysis", getProjectDir().absoluteFilePath("harmonic_response_analysis.mat"), "MAT files (*.mat)");
    if(file!="") {
      saveHarmonicResponseAnalysis(file);
    }
  }

  void MainWindow::saveHarmonicResponseAnalysis(const QString &file) {
    if(QFile::exists(file))
      QFile::remove(file);
    QFile::copy(QString::fromStdString(uniqueTempDir.generic_string())+"/harmonic_response_analysis.mat",file);
  }

  void MainWindow::mbsimxml(int task) {
    currentTask = task;

    shared_ptr<xercesc::DOMDocument> doc=mbxmlparser->createDocument();
    doc->setDocumentURI(this->doc->getDocumentURI());
    auto *newDocElement = static_cast<DOMElement*>(doc->importNode(this->doc->getDocumentElement(), true));
    doc->insertBefore(newDocElement, nullptr);
    getProject()->processIDAndHref(newDocElement);

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
      actionSaveEigenanalysisAs->setDisabled(true);
      actionSaveHarmonicResponseAnalysisAs->setDisabled(true);
      actionOpenMBV->setDisabled(true);
      actionH5plotserie->setDisabled(true);
      actionEigenanalysis->setDisabled(true);
      actionHarmonicResponseAnalysis->setDisabled(true);
    }
    else if(task==1) {
      if(OpenMBVGUI::MainWindow::getInstance()->getObjectList()->invisibleRootItem()->childCount())
        static_cast<OpenMBVGUI::Group*>(OpenMBVGUI::MainWindow::getInstance()->getObjectList()->invisibleRootItem()->child(0))->unloadFileSlot();
    }

    echoView->clearOutput();
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
      errorText = ex.what();
    }
    catch(...) {
      errorText = "Unknown error";
    }
    if(not errorText.isEmpty()) {
      echoView->addOutputText("<span class=\"MBSIMGUI_ERROR\">"+errorText+"</span>");
      echoView->updateOutput(true);
      actionSimulate->setDisabled(false);
      actionRefresh->setDisabled(false);
      actionDebug->setDisabled(false);
      statusBar()->showMessage(tr("Ready"));
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
    if(QFile::exists(name)) {
      QStringList arg;
      arg.append("--autoreload");
      arg.append(name);
      QProcess::startDetached(QString::fromStdString((installPath/"bin"/"openmbv").string()), arg);
    }
  }

  void MainWindow::h5plotserie() {
    QString name = QString::fromStdString(uniqueTempDir.generic_string())+"/"+project->getDynamicSystemSolver()->getName()+".mbsh5";
    if(QFile::exists(name)) {
      QStringList arg;
      arg.append(name);
      QProcess::startDetached(QString::fromStdString((installPath/"bin"/"h5plotserie").string()), arg);
    }
  }

  void MainWindow::eigenanalysis() {
    QString file1 = QString::fromStdString(uniqueTempDir.generic_string())+"/eigenanalysis.mat";
    QString file2 = QString::fromStdString(uniqueTempDir.generic_string())+"/statetable.asc";
    if(QFile::exists(file1) and QFile::exists(file2)) {
      EigenanalysisDialog *dialog = new EigenanalysisDialog(this);
      dialog->show();
    }
  }

  void MainWindow::harmonicResponseAnalysis() {
    QString file1 = QString::fromStdString(uniqueTempDir.generic_string())+"/harmonic_response_analysis.mat";
    QString file2 = QString::fromStdString(uniqueTempDir.generic_string())+"/statetable.asc";
    if(QFile::exists(file1) and QFile::exists(file2)) {
      HarmonicResponseDialog *dialog = new HarmonicResponseDialog(this);
      dialog->show();
    }
  }

  void MainWindow::debug() {
    currentTask = 0;
    QString uniqueTempDir_ = QString::fromStdString(uniqueTempDir.generic_string());
    QString projectFile = uniqueTempDir_+"/Project.mbsx";
    serializer->writeToURI(doc, X()%projectFile.toStdString());
    QStringList arg;
    arg.append("--stopafterfirststep");
    arg.append(projectFile);
    echoView->clearOutput();
    process.setWorkingDirectory(uniqueTempDir_);
    process.start(QString::fromStdString((installPath/"bin"/"mbsimxml").string()), arg);
  }

  void MainWindow::selectElement(const string& ID) {
    Element *element = idMap[ID];
    if(element) {
      elementView->selectionModel()->setCurrentIndex(element->getModelIndex(),QItemSelectionModel::ClearAndSelect);
      elementViewClicked(element->getModelIndex());
    }
  }

  void MainWindow::help() {
    QMessageBox::information(this, tr("MBSimGUI - Help"), tr("<p>Please visit <a href=\"https://www.mbsim-env.de\">MBSim-Environment</a> for documentation.</p>"));
  }

  void MainWindow::xmlHelp(const QString &url) {
    QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString((installPath/"share"/"mbxmlutils"/"doc"/"http___www_mbsim-env_de_MBSimXML"/"mbsimxml.html").string())));
  }

  void MainWindow::about() {
     QMessageBox::about(this, tr("About MBSimGUI"), (tr("<p><b>MBSimGUI %1</b></p><p>MBSimGUI is a graphical user interface for the multibody simulation software MBSim.</p>").arg(VERSION)
           + tr("<p>See <a href=\"https://www.mbsim-env.de\">MBSim-Environment</a> for more information.</p>"
             "<p>Copyright &copy; Martin Foerg <tt>&lt;martin.o.foerg@googlemail.com&gt;</tt><p/>"
             "<p>Licensed under the General Public License (see file COPYING).</p>"
             "<p>This is free software; see the source for copying conditions. There is NO warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.</p>")));
  }

  void MainWindow::rebuildTree() {

    auto *pmodel = static_cast<ParameterTreeModel*>(parameterView->model());
    QModelIndex index = pmodel->index(0,0);
    pmodel->removeRows(index.row(), pmodel->rowCount(QModelIndex()), index.parent());

    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    index = model->index(0,0);
    model->removeRows(index.row(), model->rowCount(QModelIndex()), index.parent());

    auto *fmodel = static_cast<FileTreeModel*>(fileView->model());
    index = fmodel->index(0,0);
    fmodel->removeRows(index.row(), fmodel->rowCount(QModelIndex()), index.parent());

    delete project;

    file.clear();
    idMap.clear();
    IDcounter = 0;

    project=Embed<Project>::create(doc->getDocumentElement(),nullptr);
    project->create();

    model->createProjectItem(project);

//    model->createGroupItem(project->getDynamicSystemSolver());
//    model->createSolverItem(getProject()->getSolver(),model->index(1,0));

 //   solverView->setSolver(getProject()->getSolver());
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
    redos.push_back(doc);
    doc = undos.back();
    undos.pop_back();
    rebuildTree();
    if(getAutoRefresh()) refresh();
    actionUndo->setDisabled(undos.empty());
    actionRedo->setEnabled(true);
  }

  void MainWindow::redo() {
    undos.push_back(doc);
    doc = redos.back();
    redos.pop_back();
    rebuildTree();
    if(getAutoRefresh()) refresh();
    actionRedo->setDisabled(redos.empty());
    actionUndo->setEnabled(true);
  }

  void MainWindow::removeElement() {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    auto *element = dynamic_cast<Element*>(model->getItem(index)->getItemData());
    if(element and (not dynamic_cast<DynamicSystemSolver*>(element)) and (not dynamic_cast<InternalFrame*>(element))) {
      auto *parent = element->getParent();
      auto *dedicatedParent = static_cast<Element*>(parent->getDedicatedItem());
      auto* fileItem = dedicatedParent->getFileItem();
      if(fileItem)
        fileItem->setModified(true);
      else
        setProjectChanged(true);
      if(element == elementBuffer.first)
        elementBuffer.first = nullptr;
      QModelIndex pindex = element->getParameters()->getModelIndex();
      static_cast<ParameterTreeModel*>(parameterView->model())->removeRow(pindex.row(), pindex.parent());
      model->removeRow(index.row(), index.parent());
      element->removeXMLElement();
      parent->removeElement(element);
      updateReferences(dedicatedParent);
      if(getAutoRefresh()) refresh();
    }
  }

  void MainWindow::removeParameter() {
    auto *model = static_cast<ParameterTreeModel*>(parameterView->model());
    QModelIndex index = parameterView->selectionModel()->currentIndex();
    auto *parameter = dynamic_cast<Parameter*>(model->getItem(index)->getItemData());
    if(parameter) {
      auto *fileItem = parameter->getParent()->getDedicatedParameterFileItem();
      if(fileItem)
        fileItem->setModified(true);
      else
        setProjectChanged(true);
      if(parameter == parameterBuffer.first)
        parameterBuffer.first = nullptr;
      DOMNode *ps = parameter->getXMLElement()->getPreviousSibling();
      if(ps and X()%ps->getNodeName()=="#text")
        parameter->getXMLElement()->getParentNode()->removeChild(ps);
      parameter->getXMLElement()->getParentNode()->removeChild(parameter->getXMLElement());
      parameter->getParent()->removeParameter(parameter);
      parameter->getParent()->maybeRemoveEmbedXMLElement();
      auto *dedicatedParent = dynamic_cast<Element*>(parameter->getParent()->getDedicatedItem());
      if(dedicatedParent) updateReferences(dedicatedParent);
      updateParameterReferences(parameter->getParent());
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
    else if(parameterView->hasFocus()) {
      auto *model = static_cast<ParameterTreeModel*>(parameterView->model());
      QModelIndex index = parameterView->selectionModel()->currentIndex();
      auto *item = dynamic_cast<Parameters*>(model->getItem(index)->getItemData());
      if(item and not dynamic_cast<InternalFrame*>(item->getParent()))
        loadParameter(item->getParent(),getParameterBuffer().first);
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
    auto *element = static_cast<Element*>(model->getItem(index)->getItemData());
    if((not dynamic_cast<DynamicSystemSolver*>(element)) and (not dynamic_cast<InternalFrame*>(element)))
      elementBuffer = make_pair(element,cut);
  }

  void MainWindow::copyParameter(bool cut) {
    auto *model = static_cast<ParameterTreeModel*>(parameterView->model());
    QModelIndex index = parameterView->selectionModel()->currentIndex();
    auto *parameter = static_cast<Parameter*>(model->getItem(index)->getItemData());
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
    parameterView->setCurrentIndex(parentIndex.child(j,0));
    auto* fileItem = parameter->getParent()->getDedicatedParameterFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    auto *dedicatedParent = dynamic_cast<Element*>(parameter->getParent()->getDedicatedItem());
    if(dedicatedParent) updateReferences(dedicatedParent);
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
    elementView->setCurrentIndex(parentIndex.child(j,0));
    auto *dedicatedParent = static_cast<Element*>(frame->getParent()->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    updateReferences(dedicatedParent);
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
    elementView->setCurrentIndex(parentIndex.child(j,0));
    auto *dedicatedParent = static_cast<Element*>(contour->getParent()->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    updateReferences(dedicatedParent);
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
    elementView->setCurrentIndex(parentIndex.child(j,0));
    auto *dedicatedParent = static_cast<Element*>(group->getParent()->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    updateReferences(dedicatedParent);
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
    elementView->setCurrentIndex(parentIndex.child(j,0));
    auto *dedicatedParent = static_cast<Element*>(object->getParent()->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    updateReferences(dedicatedParent);
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
    elementView->setCurrentIndex(parentIndex.child(j,0));
    auto *dedicatedParent = static_cast<Element*>(link->getParent()->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    updateReferences(dedicatedParent);
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
    elementView->setCurrentIndex(parentIndex.child(j,0));
    auto *dedicatedParent = static_cast<Element*>(constraint->getParent()->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    updateReferences(dedicatedParent);
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
    elementView->setCurrentIndex(parentIndex.child(j,0));
    auto *dedicatedParent = static_cast<Element*>(observer->getParent()->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    updateReferences(dedicatedParent);
  }

  void MainWindow::exportElement() {
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    auto *element = static_cast<Element*>(model->getItem(index)->getItemData());
    QString file = QFileDialog::getSaveFileName(this, "Export MBSim model file", getProjectDir().absoluteFilePath(element->getName()+".mbsmx"), "MBSim model files (*.mbsmx)");
    if(not file.isEmpty()) {
      xercesc::DOMDocument *edoc = impl->createDocument();
      DOMNode *node = edoc->importNode(element->getXMLElement(),true);
      edoc->insertBefore(node,nullptr);
      serializer->writeToURI(edoc, X()%file.toStdString());
      if(element->getEmbedXMLElement()) {
        QMessageBox::StandardButton button = QMessageBox::question(this, "Question", "Export parameters?");
        if(button==QMessageBox::Yes) exportParameters();
      }
    }
  }

  void MainWindow::enableElement(bool enabled) {
    setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    auto *element = static_cast<Element*>(model->getItem(index)->getItemData());
    DOMElement *embedNode = element->getEmbedXMLElement();
    if(enabled) {
      if(element->getNumberOfParameters())
        E(embedNode)->setAttribute("count","1");
      else {
        E(embedNode)->removeAttribute("count");
        E(embedNode)->removeAttribute("counterName");
      }
    }
    else {
      if(not embedNode)
        embedNode = element->createEmbedXMLElement();
      E(embedNode)->setAttribute("counterName","n");
      E(embedNode)->setAttribute("count","0");
    }
    element->maybeRemoveEmbedXMLElement();
    element->updateStatus();
    if(getAutoRefresh()) refresh();
  }

  void MainWindow::exportParameters() {
    ParameterTreeModel *model = static_cast<ParameterTreeModel*>(parameterView->model());
    QModelIndex index = parameterView->selectionModel()->currentIndex();
    auto *item = static_cast<Parameters*>(model->getItem(index)->getItemData());
    QString file=QFileDialog::getSaveFileName(this, "Export MBSim parameter file", getProjectDir().absoluteFilePath(item->getParent()->getName()+".mbspx"), "Parameter files (*.mbspx)");
    if(not file.isEmpty()) {
      xercesc::DOMDocument *edoc = impl->createDocument();
      DOMNode *node;
      node = edoc->importNode(item->getParent()->getEmbedXMLElement()->getFirstElementChild(),true);
      edoc->insertBefore(node,nullptr);
      serializer->writeToURI(edoc, X()%file.toStdString());
    }
  }

  void MainWindow::updateReferences(Element *element) {
    auto *fileItem = element->getFileItem();
    if(fileItem) {
      for(int i=0; i<fileItem->getNumberOfReferences(); i++) {
        if(fileItem->getReference(i)!=element) {
          fileItem->getReference(i)->setXMLElement(element->getXMLElement());
          fileItem->getReference(i)->clear();
          QModelIndex index = fileItem->getReference(i)->getModelIndex();
          auto *model = static_cast<ElementTreeModel*>(elementView->model());
          model->removeRows(0,model->rowCount(index),index);
          fileItem->getReference(i)->create();
          model->updateElementItem(static_cast<Element*>(fileItem->getReference(i)));
        }
      }
    }
  }

  void MainWindow::updateParameterReferences(EmbedItemData *parent) {
    auto *fileItem = parent->getParameterFileItem();
    if(fileItem) {
      for(int i=0; i<fileItem->getNumberOfReferences(); i++) {
        if(fileItem->getReference(i)!=parent) {
          fileItem->getReference(i)->clearParameters();
          fileItem->getReference(i)->createParameters();
        }
      }
    }
  }

  void MainWindow::addFrame(Frame *frame, Element *parent) {
    auto *dedicatedParent = static_cast<Element*>(parent->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    parent->addFrame(frame);
    frame->createXMLElement(parent->getXMLFrames());
    model->createFrameItem(frame,index);
    updateReferences(dedicatedParent);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    openElementEditor(false);
  }

  void MainWindow::addContour(Contour *contour, Element *parent) {
    auto *dedicatedParent = static_cast<Element*>(parent->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    parent->addContour(contour);
    contour->createXMLElement(parent->getXMLContours());
    model->createContourItem(contour,index);
    updateReferences(dedicatedParent);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    openElementEditor(false);
  }

  void MainWindow::addGroup(Group *group, Element *parent) {
    auto *dedicatedParent = static_cast<Element*>(parent->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    parent->addGroup(group);
    group->createXMLElement(parent->getXMLGroups());
    model->createGroupItem(group,index);
    updateReferences(dedicatedParent);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    openElementEditor(false);
  }

  void MainWindow::addObject(Object *object, Element *parent) {
    auto *dedicatedParent = static_cast<Element*>(parent->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    parent->addObject(object);
    object->createXMLElement(parent->getXMLObjects());
    model->createObjectItem(object,index);
    updateReferences(dedicatedParent);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    openElementEditor(false);
  }

  void MainWindow::addLink(Link *link, Element *parent) {
    auto *dedicatedParent = static_cast<Element*>(parent->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    parent->addLink(link);
    link->createXMLElement(parent->getXMLLinks());
    model->createLinkItem(link,index);
    updateReferences(dedicatedParent);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    openElementEditor(false);
  }

  void MainWindow::addConstraint(Constraint *constraint, Element *parent) {
    auto *dedicatedParent = static_cast<Element*>(parent->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    parent->addConstraint(constraint);
    constraint->createXMLElement(parent->getXMLConstraints());
    model->createConstraintItem(constraint,index);
    updateReferences(dedicatedParent);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    openElementEditor(false);
  }

  void MainWindow::addObserver(Observer *observer, Element *parent) {
    auto *dedicatedParent = static_cast<Element*>(parent->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    parent->addObserver(observer);
    observer->createXMLElement(parent->getXMLObservers());
    model->createObserverItem(observer,index);
    updateReferences(dedicatedParent);
    QModelIndex currentIndex = index.child(model->rowCount(index)-1,0);
    elementView->selectionModel()->setCurrentIndex(currentIndex, QItemSelectionModel::ClearAndSelect);
    openElementEditor(false);
  }

  void MainWindow::addParameter(Parameter *parameter, EmbedItemData *parent) {
    auto* fileItem = parent->getDedicatedParameterFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    QModelIndex index = parameterView->selectionModel()->currentIndex();
    auto *model = static_cast<ParameterTreeModel*>(parameterView->model());
    parent->addParameter(parameter);
    parameter->createXMLElement(parent->createParameterXMLElement());
    auto *dedicatedParent = dynamic_cast<Element*>(parent->getDedicatedItem());
    if(dedicatedParent) updateReferences(dedicatedParent);
    updateParameterReferences(parent);
    model->createParameterItem(parameter,index);
    parameterView->selectionModel()->setCurrentIndex(parameter->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    openParameterEditor(false);
  }

  void MainWindow::loadParameter(EmbedItemData *parent, Parameter *param, bool embed, bool add) {
    vector<DOMElement*> elements;
    auto *model = static_cast<ParameterTreeModel*>(parameterView->model());
    FileItemData *parameterFileItem = nullptr;
    if(param) {
      DOMElement *parentele = parent->getEmbedXMLElement()?parent->getEmbedXMLElement():parent->getXMLElement();
      elements.push_back(static_cast<DOMElement*>(parentele->getOwnerDocument()->importNode(param->getXMLElement(),true)));
      if(parameterBuffer.second) {
        parameterBuffer.first = nullptr;
        DOMNode *ps = param->getXMLElement()->getPreviousSibling();
        if(ps and X()%ps->getNodeName()=="#text")
          param->getXMLElement()->getParentNode()->removeChild(ps);
        param->getXMLElement()->getParentNode()->removeChild(param->getXMLElement());
        param->getParent()->removeParameter(param);
        QModelIndex index = param->getModelIndex();
        model->removeRow(index.row(), index.parent());
        auto* fileItem = param->getParent()->getDedicatedParameterFileItem();
        if(fileItem)
          fileItem->setModified(true);
        else
          setProjectChanged(true);
        auto *dedicatedParent = dynamic_cast<Element*>(param->getParent()->getDedicatedItem());
        if(dedicatedParent) updateReferences(dedicatedParent);
        updateParameterReferences(param->getParent());
      }
    }
    else {
      if(parent->getNumberOfParameters() and not add) removeParameter(parent);
      xercesc::DOMDocument *doc = nullptr;
      QString action = add?"Add":(embed?"Embed":"Import");
      QString file = QFileDialog::getOpenFileName(this, action+" MBSim parameter file", ".", "MBSim parameter files (*.mbspx);;XML files (*.xml);;All files (*.*)");
      if(not file.isEmpty()) {
        if(file.startsWith("//"))
          file.replace('/','\\'); // xerces-c is not able to parse files from network shares that begin with "//"
        if(embed) {
          parameterFileItem = addFile(file);
          doc = parameterFileItem->getXMLDocument();
        }
        else {
          doc = parser->parseURI(X()%file.toStdString());
          DOMParser::handleCDATA(doc->getDocumentElement());
        }
        DOMElement *parentele = parent->getEmbedXMLElement()?parent->getEmbedXMLElement():parent->getXMLElement();
        DOMElement *ele = embed?doc->getDocumentElement()->getFirstElementChild():static_cast<DOMElement*>(parentele->getOwnerDocument()->importNode(doc->getDocumentElement(),true))->getFirstElementChild();
        while(ele) {
          elements.push_back(ele);
          ele = ele->getNextElementSibling();
        }
      }
      else
        return;
    }
    if(embed) {
      parent->setParameterFileItem(parameterFileItem);
      E(parent->createEmbedXMLElement())->setAttribute("parameterHref",getProjectDir().relativeFilePath(parameterFileItem->getFileInfo().absoluteFilePath()).toStdString());
    }
    for(auto & element : elements) {
      Parameter *parameter=ObjectFactory::getInstance()->createParameter(element);
      if(not parameter) {
        QMessageBox::warning(nullptr, "Import", "Cannot import file.");
        return;
      }
      if(not embed) parent->createParameterXMLElement()->insertBefore(element,nullptr);
      parameter->setXMLElement(element);
      parent->addParameter(parameter);
      model->createParameterItem(parameter,parameterView->selectionModel()->currentIndex());
    }
    auto* fileItem = parent->getDedicatedParameterFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    auto *dedicatedParent = dynamic_cast<Element*>(parent->getDedicatedItem());
    if(dedicatedParent) updateReferences(dedicatedParent);
    updateParameterReferences(parent);
    parameterView->selectionModel()->setCurrentIndex(parent->getParameters()->getModelIndex(), QItemSelectionModel::ClearAndSelect);
  }

  void MainWindow::removeParameter(EmbedItemData *parent) {
    auto *fileItem = parent->getDedicatedParameterFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
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
    auto *dedicatedParent = dynamic_cast<Element*>(parent->getDedicatedItem());
    if(dedicatedParent) updateReferences(dedicatedParent);
    updateParameterReferences(parent);
    model->removeRows(0,n,index);
    if(getAutoRefresh()) refresh();
  }

  tuple<DOMElement*, FileItemData*> MainWindow::loadElement(Element *parent, Element *element, bool embed) {
    DOMElement *ele = nullptr;
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    FileItemData *fileItem = nullptr;
    if(element) {
      ele = static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(element->getEmbedXMLElement()?element->getEmbedXMLElement():element->getXMLElement(),true));
      if(elementBuffer.second) {
        elementBuffer.first = nullptr;
        QModelIndex index = element->getModelIndex();
        model->removeRow(index.row(), index.parent());
        auto *parent = element->getParent();
        auto *dedicatedParent = static_cast<Element*>(parent->getDedicatedItem());
        auto *fileItem = dedicatedParent->getFileItem();
        if(fileItem)
          fileItem->setModified(true);
        else
          setProjectChanged(true);
        element->removeXMLElement();
        parent->removeElement(element);
        updateReferences(dedicatedParent);
      }
    }
    else {
      xercesc::DOMDocument *doc = nullptr;
      QString action = embed?"Embed":"Import";
      QString file = QFileDialog::getOpenFileName(this, action+" MBSim model file", ".", "MBSim model files (*.mbsmx);;XML files (*.xml);;All files (*.*)");
      if(not file.isEmpty()) {
        if(file.startsWith("//"))
          file.replace('/','\\'); // xerces-c is not able to parse files from network shares that begin with "//"
        if(embed) {
          fileItem = addFile(file);
          doc = fileItem->getXMLDocument();
        }
        else {
          doc = parser->parseURI(X()%file.toStdString());
          DOMParser::handleCDATA(doc->getDocumentElement());
        }
        ele = embed?doc->getDocumentElement():static_cast<DOMElement*>(parent->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
      }
    }
    return tuple<DOMElement*,FileItemData*>(ele,fileItem);
  }

  void MainWindow::loadFrame(Element *parent, Element *element, bool embed) {
    tuple<DOMElement*, FileItemData*> data = loadElement(parent,element,embed);
    if(not std::get<0>(data)) return;
    Frame *frame = Embed<Frame>::create(std::get<0>(data),parent);
    if(not frame) {
      QMessageBox::warning(nullptr, "Import", "Cannot import file.");
      return;
    }
    if(embed) {
      frame->setFileItem(std::get<1>(data));
      frame->setEmbedXMLElement(D(parent->getXMLElement()->getOwnerDocument())->createElement(PV%"Embed"));
      parent->getXMLFrames()->insertBefore(frame->getEmbedXMLElement(), nullptr);
      E(frame->getEmbedXMLElement())->setAttribute("href",getProjectDir().relativeFilePath(std::get<1>(data)->getFileInfo().absoluteFilePath()).toStdString());
    }
    else
      parent->getXMLFrames()->insertBefore(std::get<0>(data), nullptr);
    parent->addFrame(frame);
    frame->create();
    auto *dedicatedParent = static_cast<Element*>(parent->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    updateReferences(dedicatedParent);
    static_cast<ElementTreeModel*>(elementView->model())->createFrameItem(frame,elementView->selectionModel()->currentIndex());
    elementView->selectionModel()->setCurrentIndex(frame->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(not element) {
      QMessageBox::StandardButton button = QMessageBox::question(this, "Question", embed?"Embed parameters?":"Import parameters?");
      if(button==QMessageBox::Yes) loadParameter(frame,nullptr,embed);
    }
    if(getAutoRefresh()) refresh();
  }

  void MainWindow::loadContour(Element *parent, Element *element, bool embed) {
    tuple<DOMElement*, FileItemData*> data = loadElement(parent,element,embed);
    if(not std::get<0>(data)) return;
    Contour *contour = Embed<Contour>::create(std::get<0>(data),parent);
    if(not contour) {
      QMessageBox::warning(nullptr, "Import", "Cannot import file.");
      return;
    }
    if(embed) {
      contour->setFileItem(std::get<1>(data));
      contour->setEmbedXMLElement(D(parent->getXMLElement()->getOwnerDocument())->createElement(PV%"Embed"));
      parent->getXMLContours()->insertBefore(contour->getEmbedXMLElement(), nullptr);
      E(contour->getEmbedXMLElement())->setAttribute("href",getProjectDir().relativeFilePath(std::get<1>(data)->getFileInfo().absoluteFilePath()).toStdString());
    }
    else
      parent->getXMLContours()->insertBefore(std::get<0>(data), nullptr);
    parent->addContour(contour);
    contour->create();
    auto *dedicatedParent = static_cast<Element*>(parent->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    updateReferences(dedicatedParent);
    static_cast<ElementTreeModel*>(elementView->model())->createContourItem(contour,elementView->selectionModel()->currentIndex());
    elementView->selectionModel()->setCurrentIndex(contour->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(not element) {
      QMessageBox::StandardButton button = QMessageBox::question(this, "Question", embed?"Embed parameters?":"Import parameters?");
      if(button==QMessageBox::Yes) loadParameter(contour,nullptr,embed);
    }
    if(getAutoRefresh()) refresh();
  }

  void MainWindow::loadGroup(Element *parent, Element *element, bool embed) {
    tuple<DOMElement*, FileItemData*> data = loadElement(parent,element,embed);
    if(not std::get<0>(data)) return;
    Group *group = Embed<Group>::create(std::get<0>(data),parent);
    if(not group) {
      QMessageBox::warning(nullptr, "Import", "Cannot import file.");
      return;
    }
    if(embed) {
      group->setFileItem(std::get<1>(data));
      group->setEmbedXMLElement(D(parent->getXMLElement()->getOwnerDocument())->createElement(PV%"Embed"));
      parent->getXMLGroups()->insertBefore(group->getEmbedXMLElement(), nullptr);
      E(group->getEmbedXMLElement())->setAttribute("href",getProjectDir().relativeFilePath(std::get<1>(data)->getFileInfo().absoluteFilePath()).toStdString());
    }
    else
      parent->getXMLGroups()->insertBefore(std::get<0>(data), nullptr);
    parent->addGroup(group);
    group->create();
    auto *dedicatedParent = static_cast<Element*>(parent->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    updateReferences(dedicatedParent);
    static_cast<ElementTreeModel*>(elementView->model())->createGroupItem(group,elementView->selectionModel()->currentIndex());
    elementView->selectionModel()->setCurrentIndex(group->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(not element) {
      QMessageBox::StandardButton button = QMessageBox::question(this, "Question", embed?"Embed parameters?":"Import parameters?");
      if(button==QMessageBox::Yes) loadParameter(group,nullptr,embed);
    }
    if(getAutoRefresh()) refresh();
  }

  void MainWindow::loadObject(Element *parent, Element *element, bool embed) {
    tuple<DOMElement*, FileItemData*> data = loadElement(parent,element,embed);
    if(not std::get<0>(data)) return;
    Object *object = Embed<Object>::create(std::get<0>(data),parent);
    if(not object) {
      QMessageBox::warning(nullptr, "Import", "Cannot import file.");
      return;
    }
    if(embed) {
      object->setFileItem(std::get<1>(data));
      object->setEmbedXMLElement(D(parent->getXMLElement()->getOwnerDocument())->createElement(PV%"Embed"));
      parent->getXMLObjects()->insertBefore(object->getEmbedXMLElement(), nullptr);
      E(object->getEmbedXMLElement())->setAttribute("href",getProjectDir().relativeFilePath(std::get<1>(data)->getFileInfo().absoluteFilePath()).toStdString());
    }
    else
      parent->getXMLObjects()->insertBefore(std::get<0>(data), nullptr);
    parent->addObject(object);
    object->create();
    auto *dedicatedParent = static_cast<Element*>(parent->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    updateReferences(dedicatedParent);
    static_cast<ElementTreeModel*>(elementView->model())->createObjectItem(object,elementView->selectionModel()->currentIndex());
    elementView->selectionModel()->setCurrentIndex(object->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(not element) {
      QMessageBox::StandardButton button = QMessageBox::question(this, "Question", embed?"Embed parameters?":"Import parameters?");
      if(button==QMessageBox::Yes) loadParameter(object,nullptr,embed);
    }
    if(getAutoRefresh()) refresh();
  }

  void MainWindow::loadLink(Element *parent, Element *element, bool embed) {
    tuple<DOMElement*, FileItemData*> data = loadElement(parent,element,embed);
    if(not std::get<0>(data)) return;
    Link *link = Embed<Link>::create(std::get<0>(data),parent);
    if(not link) {
      QMessageBox::warning(nullptr, "Import", "Cannot import file.");
      return;
    }
    if(embed) {
      link->setFileItem(std::get<1>(data));
      link->setEmbedXMLElement(D(parent->getXMLElement()->getOwnerDocument())->createElement(PV%"Embed"));
      parent->getXMLLinks()->insertBefore(link->getEmbedXMLElement(), nullptr);
      E(link->getEmbedXMLElement())->setAttribute("href",getProjectDir().relativeFilePath(std::get<1>(data)->getFileInfo().absoluteFilePath()).toStdString());
    }
    else
      parent->getXMLLinks()->insertBefore(std::get<0>(data), nullptr);
    parent->addLink(link);
    link->create();
    auto *dedicatedParent = static_cast<Element*>(parent->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    updateReferences(dedicatedParent);
    static_cast<ElementTreeModel*>(elementView->model())->createLinkItem(link,elementView->selectionModel()->currentIndex());
    elementView->selectionModel()->setCurrentIndex(link->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(not element) {
      QMessageBox::StandardButton button = QMessageBox::question(this, "Question", embed?"Embed parameters?":"Import parameters?");
      if(button==QMessageBox::Yes) loadParameter(link,nullptr,embed);
    }
    if(getAutoRefresh()) refresh();
  }

  void MainWindow::loadConstraint(Element *parent, Element *element, bool embed) {
    tuple<DOMElement*, FileItemData*> data = loadElement(parent,element,embed);
    if(not std::get<0>(data)) return;
    Constraint *constraint = Embed<Constraint>::create(std::get<0>(data),parent);
    if(not constraint) {
      QMessageBox::warning(nullptr, "Import", "Cannot import file.");
      return;
    }
    if(embed) {
      constraint->setFileItem(std::get<1>(data));
      constraint->setEmbedXMLElement(D(parent->getXMLElement()->getOwnerDocument())->createElement(PV%"Embed"));
      parent->getXMLConstraints()->insertBefore(constraint->getEmbedXMLElement(), nullptr);
      E(constraint->getEmbedXMLElement())->setAttribute("href",getProjectDir().relativeFilePath(std::get<1>(data)->getFileInfo().absoluteFilePath()).toStdString());
    }
    else
      parent->getXMLConstraints()->insertBefore(std::get<0>(data), nullptr);
    parent->addConstraint(constraint);
    constraint->create();
    auto *dedicatedParent = static_cast<Element*>(parent->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    updateReferences(dedicatedParent);
    static_cast<ElementTreeModel*>(elementView->model())->createConstraintItem(constraint,elementView->selectionModel()->currentIndex());
    elementView->selectionModel()->setCurrentIndex(constraint->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(not element) {
      QMessageBox::StandardButton button = QMessageBox::question(this, "Question", embed?"Embed parameters?":"Import parameters?");
      if(button==QMessageBox::Yes) loadParameter(constraint,nullptr,embed);
    }
    if(getAutoRefresh()) refresh();
  }

  void MainWindow::loadObserver(Element *parent, Element *element, bool embed) {
    tuple<DOMElement*, FileItemData*> data = loadElement(parent,element,embed);
    if(not std::get<0>(data)) return;
    Observer *observer = Embed<Observer>::create(std::get<0>(data),parent);
    if(not observer) {
      QMessageBox::warning(nullptr, "Import", "Cannot import file.");
      return;
    }
    if(embed) {
      observer->setFileItem(std::get<1>(data));
      observer->setEmbedXMLElement(D(parent->getXMLElement()->getOwnerDocument())->createElement(PV%"Embed"));
      parent->getXMLObservers()->insertBefore(observer->getEmbedXMLElement(), nullptr);
      E(observer->getEmbedXMLElement())->setAttribute("href",getProjectDir().relativeFilePath(std::get<1>(data)->getFileInfo().absoluteFilePath()).toStdString());
    }
    else
      parent->getXMLObservers()->insertBefore(std::get<0>(data), nullptr);
    parent->addObserver(observer);
    observer->create();
    auto *dedicatedParent = static_cast<Element*>(parent->getDedicatedItem());
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem)
      fileItem->setModified(true);
    else
      setProjectChanged(true);
    updateReferences(dedicatedParent);
    static_cast<ElementTreeModel*>(elementView->model())->createObserverItem(observer,elementView->selectionModel()->currentIndex());
    elementView->selectionModel()->setCurrentIndex(observer->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(not element) {
      QMessageBox::StandardButton button = QMessageBox::question(this, "Question", embed?"Embed parameters?":"Import parameters?");
      if(button==QMessageBox::Yes) loadParameter(observer,nullptr,embed);
    }
    if(getAutoRefresh()) refresh();
  }

  void MainWindow::loadDynamicSystemSolver(bool embed) {
    DOMElement *ele = nullptr;
    xercesc::DOMDocument *doc = nullptr;
    FileItemData *fileItem = nullptr;
    QString action = embed?"Embed":"Import";
    QString file = QFileDialog::getOpenFileName(this, action+" MBSim dynamic system solver file", ".", "Dynamic system solver files (*.dssx);;XML files (*.xml);;All files (*.*)");
    if(not file.isEmpty()) {
      if(file.startsWith("//"))
        file.replace('/','\\'); // xerces-c is not able to parse files from network shares that begin with "//"
      if(embed) {
        fileItem = addFile(file);
        doc = fileItem->getXMLDocument();
      }
      else {
        doc = parser->parseURI(X()%file.toStdString());
        DOMParser::handleCDATA(doc->getDocumentElement());
      }
      ele = embed?doc->getDocumentElement():static_cast<DOMElement*>(project->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
    }
    else
      return;
    setProjectChanged(true);
    DOMElement *embedele = getProject()->getDynamicSystemSolver()->getEmbedXMLElement();
    DOMElement *paramele = nullptr;
    auto *parameterFileItem = getProject()->getDynamicSystemSolver()->getParameterFileItem();
    if(parameterFileItem)
      paramele = parameterFileItem->getXMLElement();
    else if(embedele)
      paramele = MBXMLUtils::E(embedele)->getFirstElementChildNamed(MBXMLUtils::PV%"Parameter");
    QModelIndex pindex = getProject()->getDynamicSystemSolver()->getParameters()->getModelIndex();
    static_cast<ParameterTreeModel*>(parameterView->model())->removeRow(pindex.row(), pindex.parent());
    if(getProject()->getDynamicSystemSolver()->getFileItem())
      E(embedele)->removeAttribute("href");
    else
      getProject()->getDynamicSystemSolver()->removeXMLElement(false);
    DynamicSystemSolver *dss = Embed<DynamicSystemSolver>::create(ele,project);
    if(not dss) {
      QMessageBox::warning(0, "Import", "Cannot import file.");
      return;
    }
    if(embed) {
      dss->setFileItem(fileItem);
      dss->setEmbedXMLElement(embedele?embedele:D(project->getXMLElement()->getOwnerDocument())->createElement(PV%"Embed"));
      project->getXMLElement()->insertBefore(dss->getEmbedXMLElement(), project->getSolver()->getEmbedXMLElement()?project->getSolver()->getEmbedXMLElement():project->getSolver()->getXMLElement());
      E(dss->getEmbedXMLElement())->setAttribute("href",getProjectDir().relativeFilePath(fileItem->getFileInfo().absoluteFilePath()).toStdString());
    }
    else {
      if(embedele)
        embedele->insertBefore(ele, nullptr);
      else
        project->getXMLElement()->insertBefore(ele, project->getSolver()->getEmbedXMLElement()?project->getSolver()->getEmbedXMLElement():project->getSolver()->getXMLElement());
    }
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    auto index = project->getDynamicSystemSolver()->getModelIndex();
    model->removeRow(index.row(), index.parent());
    project->setDynamicSystemSolver(dss);
    dss->create();
    model->createGroupItem(dss);
    elementView->selectionModel()->setCurrentIndex(dss->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(embed or not dss->getEmbedXMLElement()) {
      if(not dss->getEmbedXMLElement()) dss->setEmbedXMLElement(embedele);
      if(paramele) {
        dss->setParameterFileItem(parameterFileItem);
        std::vector<Parameter*> param = Parameter::createParameters(paramele);
        for(auto & i : param)
          dss->addParameter(i);
      }
    }
    if(getAutoRefresh()) refresh();
  }

  void MainWindow::loadSolver(bool embed) {
    DOMElement *ele = nullptr;
    xercesc::DOMDocument *doc = nullptr;
    FileItemData *fileItem = nullptr;
    QString action = embed?"Embed":"Import";
    QString file = QFileDialog::getOpenFileName(this, action+" MBSim model file", ".", "MBSim model files (*.mbsmx);;XML files (*.xml);;All files (*.*)");
    if(not file.isEmpty()) {
      if(file.startsWith("//"))
        file.replace('/','\\'); // xerces-c is not able to parse files from network shares that begin with "//"
      if(embed) {
        fileItem = addFile(file);
        doc = fileItem->getXMLDocument();
      }
      else {
        doc = parser->parseURI(X()%file.toStdString());
        DOMParser::handleCDATA(doc->getDocumentElement());
      }
      ele = embed?doc->getDocumentElement():static_cast<DOMElement*>(project->getXMLElement()->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
    }
    else
      return;
    setProjectChanged(true);
    DOMElement *embedele = getProject()->getSolver()->getEmbedXMLElement();
    DOMElement *paramele = nullptr;
    auto *parameterFileItem = getProject()->getSolver()->getParameterFileItem();
    if(parameterFileItem)
      paramele = parameterFileItem->getXMLElement();
    else if(embedele)
      paramele = MBXMLUtils::E(embedele)->getFirstElementChildNamed(MBXMLUtils::PV%"Parameter");
    QModelIndex pindex = getProject()->getSolver()->getParameters()->getModelIndex();
    static_cast<ParameterTreeModel*>(parameterView->model())->removeRow(pindex.row(), pindex.parent());
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    model->removeRow(getProject()->getSolver()->getModelIndex().row(), project->getModelIndex());
    if(getProject()->getSolver()->getFileItem())
      E(embedele)->removeAttribute("href");
    else
      getProject()->getSolver()->removeXMLElement(false);
    Solver *solver = Embed<Solver>::create(ele,project);
    if(not solver) {
      QMessageBox::warning(0, "Import", "Cannot import file.");
      return;
    }
    if(embed) {
      solver->setFileItem(fileItem);
      solver->setEmbedXMLElement(embedele?embedele:D(project->getXMLElement()->getOwnerDocument())->createElement(PV%"Embed"));
      project->getXMLElement()->insertBefore(solver->getEmbedXMLElement(), nullptr);
      E(solver->getEmbedXMLElement())->setAttribute("href",getProjectDir().relativeFilePath(fileItem->getFileInfo().absoluteFilePath()).toStdString());
    }
    else {
      if(embedele)
        embedele->insertBefore(ele, nullptr);
      else
        project->getXMLElement()->insertBefore(ele, nullptr);
    }
    project->setSolver(solver);
    solver->create();
    model->createSolverItem(solver,getProject()->getModelIndex());
    elementView->selectionModel()->setCurrentIndex(solver->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(embed or not solver->getEmbedXMLElement()) {
      if(not solver->getEmbedXMLElement()) solver->setEmbedXMLElement(embedele);
      if(paramele) {
        solver->setParameterFileItem(parameterFileItem);
        std::vector<Parameter*> param = Parameter::createParameters(paramele);
        for(auto & i : param)
          solver->addParameter(i);
      }
    }
    QMessageBox::StandardButton button = QMessageBox::question(this, "Question", embed?"Embed parameters?":"Import parameters?");
    if(button==QMessageBox::Yes) loadParameter(solver,nullptr,embed);
  }

  void MainWindow::viewProjectSource() {
    SourceDialog dialog(getProject()->getXMLElement(),this);
    dialog.exec();
  }

  void MainWindow::editElementSource() {
    if(not editorIsOpen()) {
      setAllowUndo(false);
      QModelIndex index = elementView->selectionModel()->currentIndex();
      auto *element = dynamic_cast<Element*>(static_cast<ElementTreeModel*>(elementView->model())->getItem(index)->getItemData());
      editor = new XMLPropertyDialog(element);
      editor->setAttribute(Qt::WA_DeleteOnClose);
      editor->toWidget();
      editor->show();
      connect(editor,&QDialog::finished,this,[=](){
        if(editor->result()==QDialog::Accepted) {
          auto dedicatedElement = static_cast<Element*>(element->getDedicatedItem());
          auto* fileItem = dedicatedElement->getFileItem();
          if(fileItem)
            fileItem->setModified(true);
          else
            setProjectChanged(true);
          editor->fromWidget();
          element->clear();
          QModelIndex index = element->getModelIndex();
          auto *model = static_cast<ElementTreeModel*>(elementView->model());
          model->removeRows(0,model->rowCount(index),index);
          element->create();
          model->updateElementItem(static_cast<Element*>(element));
          updateReferences(dedicatedElement);
          if(getAutoRefresh()) refresh();
        }
        setAllowUndo(true);
        editor = nullptr;
      });
      connect(editor,&ElementPropertyDialog::apply,this,[=](){
        auto dedicatedElement = static_cast<Element*>(element->getDedicatedItem());
        auto* fileItem = dedicatedElement->getFileItem();
        if(fileItem)
          fileItem->setModified(true);
        else
          setProjectChanged(true);
        editor->fromWidget();
        element->clear();
        QModelIndex index = element->getModelIndex();
        auto *model = static_cast<ElementTreeModel*>(elementView->model());
        model->removeRows(0,model->rowCount(index),index);
        element->create();
        model->updateElementItem(static_cast<Element*>(element));
        updateReferences(dedicatedElement);
        if(getAutoRefresh()) refresh();
        editor->setCancel(true);
      });
    }
  }

  void MainWindow::viewParametersSource() {
    QModelIndex index = parameterView->selectionModel()->currentIndex();
    auto *item = dynamic_cast<Parameters*>(static_cast<ElementTreeModel*>(parameterView->model())->getItem(index)->getItemData());
    if(item) {
      SourceDialog dialog(item->getParent()->getEmbedXMLElement()->getFirstElementChild(),this);
      dialog.exec();
    }
  }

  void MainWindow::viewSolverSource() {
    SourceDialog dialog(getProject()->getSolver()->getXMLElement(),this);
    dialog.exec();
  }

  void MainWindow::viewParameterSource() {
    QModelIndex index = parameterView->selectionModel()->currentIndex();
    auto *parameter = dynamic_cast<Parameter*>(static_cast<ParameterTreeModel*>(parameterView->model())->getItem(index)->getItemData());
    if(parameter) {
      SourceDialog dialog(parameter->getXMLElement(),this);
      dialog.exec();
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
      settings.setValue("mainwindow/embeddingview/state", parameterView->header()->saveState());
      event->accept();
    }
    else
      event->ignore();
  }

  void MainWindow::showEvent(QShowEvent *event) {
    QSettings settings;
    restoreGeometry(settings.value("mainwindow/geometry").toByteArray());
    restoreState(settings.value("mainwindow/state").toByteArray());
    elementView->header()->restoreState(settings.value("mainwindow/elementview/state").toByteArray());
    parameterView->header()->restoreState(settings.value("mainwindow/embeddingview/state").toByteArray());
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

  void MainWindow::settingsFinished(int result) {
    if(result != 0) {
      setProjectChanged(true);
      if(getAutoRefresh()) refresh();
    }
  }

  void MainWindow::applySettings() {
    setProjectChanged(true);
    if(getAutoRefresh()) refresh();
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

  void MainWindow::updateEchoView() {
    echoView->addOutputText(process.readAllStandardOutput().data());
    echoView->updateOutput(true);
  }

  void MainWindow::setAllowUndo(bool allowUndo_) {
    allowUndo = allowUndo_;
    actionUndo->setEnabled(allowUndo);
  }

  void MainWindow::updateStatus() {
    // call this function only every 0.25 sec
    if(statusTime.elapsed()<250)
      return;
    statusTime.restart();

    // show only last line
    string s=process.readAllStandardError().data();
    s.resize(s.length()-1);
    auto i=s.rfind('\n');
    i = i==string::npos ? 0 : i+1;
    statusBar()->showMessage(QString::fromStdString(s.substr(i)));
  }

  QString MainWindow::getProjectFilePath() const {
    return QUrl(QString::fromStdString(X()%doc->getDocumentURI())).toLocalFile();
  }

  void MainWindow::openProjectEditor() {
    if(not editorIsOpen()) {
      setAllowUndo(false);
      updateParameters(getProject());
      editor = getProject()->createPropertyDialog();
      editor->setAttribute(Qt::WA_DeleteOnClose);
      editor->toWidget();
      editor->show();
      connect(editor,&ProjectPropertyDialog::finished,this,[=](){
        if(editor->result()==QDialog::Accepted) {
          setProjectChanged(true);
          editor->fromWidget();
          if(getAutoRefresh()) refresh();
        }
        setAllowUndo(true);
        editor = nullptr;
      });
      connect(editor,&ProjectPropertyDialog::apply,this,[=](){
        setProjectChanged(true);
        editor->fromWidget();
        if(getAutoRefresh()) refresh();
      });
    }
  }

  void MainWindow::openElementEditor(bool config) {
    if(not editorIsOpen()) {
      QModelIndex index = elementView->selectionModel()->currentIndex();
      auto *element = dynamic_cast<EmbedItemData*>(static_cast<ElementTreeModel*>(elementView->model())->getItem(index)->getItemData());
      if(element) {
        setAllowUndo(false);
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
            auto* fileItem = element->getDedicatedFileItem();
            if(fileItem)
              fileItem->setModified(true);
            else if(editor->getCancel())
              setProjectChanged(true);
            editor->fromWidget();
            if(getAutoRefresh()) refresh();
          }
          setAllowUndo(true);
          editor = nullptr;
        });
        connect(editor,&ElementPropertyDialog::apply,this,[=](){
          auto* fileItem = element->getDedicatedFileItem();
          if(fileItem)
            fileItem->setModified(true);
          else if(editor->getCancel())
            setProjectChanged(true);
          editor->fromWidget();
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
        setAllowUndo(false);
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
            else if(editor->getCancel())
              setProjectChanged(true);
            editor->fromWidget();
            if(getAutoRefresh()) refresh();
            parameter->getParent()->updateStatus();
          }
        setAllowUndo(true);
        editor = nullptr;
        });
        connect(editor,&ParameterPropertyDialog::apply,this,[=](){
          auto* fileItem = parameter->getParent()->getDedicatedParameterFileItem();
          if(fileItem)
            fileItem->setModified(true);
          else if(editor->getCancel())
            setProjectChanged(true);
          editor->fromWidget();
          if(getAutoRefresh()) refresh();
          editor->setCancel(true);
          parameter->getParent()->updateStatus();
        });
      }
    }
  }

  void MainWindow::openSolverEditor() {
    if(not editorIsOpen()) {
      setAllowUndo(false);
      updateParameters(getProject()->getSolver());
      editor = getProject()->getSolver()->createPropertyDialog();
      editor->setAttribute(Qt::WA_DeleteOnClose);
      editor->toWidget();
      editor->show();
      connect(editor,&ProjectPropertyDialog::finished,this,[=](){
        if(editor->result()==QDialog::Accepted) {
          auto* fileItem = getProject()->getSolver()->getFileItem();
          if(fileItem)
            fileItem->setModified(true);
          else
            setProjectChanged(true);
          editor->fromWidget();
        }
        setAllowUndo(true);
        editor = nullptr;
      });
      connect(editor,&ProjectPropertyDialog::apply,this,[=](){
        auto* fileItem = getProject()->getSolver()->getFileItem();
        if(fileItem)
          fileItem->setModified(true);
        else
          setProjectChanged(true);
        editor->fromWidget();
      });
    }
  }

  void MainWindow::openCloneEditor() {
    if(not editorIsOpen()) {
      QModelIndex index = elementView->selectionModel()->currentIndex();
      auto *element = dynamic_cast<Element*>(static_cast<ElementTreeModel*>(elementView->model())->getItem(index)->getItemData());
      if(element) {
        setAllowUndo(false);
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
              setProjectChanged(true);
            editor->fromWidget();
            if(getAutoRefresh()) refresh();
          }
          setAllowUndo(true);
          editor = nullptr;
        });
        connect(editor,&ElementPropertyDialog::apply,this,[=](){
          auto* fileItem = element->getDedicatedFileItem();
          if(fileItem)
            fileItem->setModified(true);
          else
            setProjectChanged(true);
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
    QModelIndex index = fileItem->getModelIndex();
    fileView->model()->removeRow(index.row(), index.parent());
    for(auto it = file.begin(); it != file.end(); ++it) {
      if(*it==fileItem) {
        file.erase(it);
        break;
      }
    }
    delete fileItem;
  }

  void MainWindow::addElementView(EmbedItemData *item) {
    ElementView *elementView = new ElementView;
    itemView.push_back(elementView);
    auto *model = new ElementTreeModel(this);
    elementView->setModel(model);
    model->createElementItem(static_cast<Element*>(item),QModelIndex());
//    QDockWidget *dockWidget = new QDockWidget(item->getName(), this);
//    dockWidget->setObjectName("dockWidget/item");
//    addDockWidget(Qt::LeftDockWidgetArea,dockWidget);
//    QWidget *widget = new QWidget(dockWidget);
//    dockWidget->setWidget(widget);
//    auto *widgetLayout = new QVBoxLayout(widget);
//    widgetLayout->setContentsMargins(0,0,0,0);
//    widget->setLayout(widgetLayout);
//    widgetLayout->addWidget(elementView);
    tabWidget->addTab(elementView,item->getName());
  }

  void MainWindow::saveReferencedFile(int i) {
    try {
      serializer->writeToURI(file[i]->getXMLDocument(), X()%file[i]->getFileInfo().absoluteFilePath().toStdString());
      file[i]->setModified(false);
    }
    catch(const std::exception &ex) {
      cout << ex.what() << endl;
    }
    catch(const DOMException &ex) {
      cout << X()%ex.getMessage() << endl;
    }
    catch(...) {
      cout << "Unknown exception." << endl;
    }
  }

}
