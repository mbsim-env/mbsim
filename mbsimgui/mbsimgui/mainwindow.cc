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

    elementView = new ElementView;
    parameterView = new ParameterView;
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

    QMenu *GUIMenu = new QMenu("GUI", menuBar());
    menuBar()->addMenu(GUIMenu);

    QAction *action = GUIMenu->addAction(QIcon::fromTheme("document-properties"), "Options", this, &MainWindow::openOptionsMenu);
    action->setStatusTip(tr("Open options menu"));

    GUIMenu->addSeparator();

    elementViewFilter = new OpenMBVGUI::AbstractViewFilter(elementView, 0, 2);
    elementViewFilter->hide();

    parameterViewFilter = new OpenMBVGUI::AbstractViewFilter(parameterView, 0, 2);
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
    actionSaveLinearSystemAnalysisAs = menu->addAction("Export linear system analysis", this, &MainWindow::saveLinearSystemAnalysisAs);
    menuBar()->addMenu(menu);

    auto dialog = new QDialog(this);
    auto *layout = new QVBoxLayout;
    dialog->setLayout(layout);
    layout->addWidget(fileView);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
    buttonBox->addButton(QDialogButtonBox::Close);
    layout->addWidget(buttonBox);
    connect(buttonBox, &QDialogButtonBox::rejected, dialog, &QDialog::hide);

    menu = new QMenu("References", menuBar());
    menu->addAction(QIcon::fromTheme("document-open"), "Open list", dialog, &QDialog::show);
    menu->addAction(QIcon::fromTheme("document-save"), "Save all", this, [=](){ for(size_t i=0; i<file.size(); i++) if(file[i]->getModified()) saveReferencedFile(i); });
    menuBar()->addMenu(menu);

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
    QAction *actionInterrupt = toolBar->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_MediaStop)),"Interrupt simulation");
    connect(actionInterrupt,&QAction::triggered,this,&MainWindow::interrupt);
    QAction *actionKill = toolBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"kill.svg").string())),"Kill simulation");
    connect(actionKill,&QAction::triggered,this,&MainWindow::kill);
    actionRefresh = toolBar->addAction(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_BrowserReload)),"Refresh 3D view");
    connect(actionRefresh,&QAction::triggered,this,&MainWindow::refresh);
    actionOpenMBV = toolBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"openmbv.svg").string())),"OpenMBV");
    connect(actionOpenMBV,&QAction::triggered,this,&MainWindow::openmbv);
    actionH5plotserie = toolBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"h5plotserie.svg").string())),"H5plotserie");
    connect(actionH5plotserie,&QAction::triggered,this,&MainWindow::h5plotserie);
    actionLinearSystemAnalysis = toolBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"eigenanalysis.svg").string())),"Linear system analysis");
    connect(actionLinearSystemAnalysis,&QAction::triggered,this,&MainWindow::linearSystemAnalysis);
    actionStateTable = toolBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"state_table.svg").string())),"Show state table");
    connect(actionStateTable,&QAction::triggered,this,&MainWindow::showStateTable);
    QAction *actionCreateFMU = toolBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"FMI_bare.svg").string())),"Create FMU");
    connect(actionCreateFMU,&QAction::triggered,this,&MainWindow::createFMU);
    QAction *actionFem = toolBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"fbt.svg").string())),"Flexible body tool");
    connect(actionFem,&QAction::triggered,this,&MainWindow::flexibleBodyTool);
    actionDebug = toolBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"debug.svg").string())),"Debug model");
    connect(actionDebug,&QAction::triggered,this,&MainWindow::debug);
    QAction *actionConvert = toolBar->addAction(Utils::QIconCached(QString::fromStdString((installPath/"share"/"mbsimgui"/"icons"/"convert.svg").string())),"Convert file");
    connect(actionConvert,&QAction::triggered,this,&MainWindow::convertDocument);

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

    connect(elementView->selectionModel(), &QItemSelectionModel::currentRowChanged, this, &MainWindow::elementChanged);
    connect(elementView, &ElementView::pressed, this, &MainWindow::elementViewClicked);
    connect(parameterView, &ParameterView::pressed, this, &MainWindow::parameterViewClicked);

    QDockWidget *dockWidget1 = new QDockWidget("Model Tree", this);
    dockWidget1->setFeatures(QDockWidget::DockWidgetVerticalTitleBar);
    dockWidget1->setObjectName("dockWidget/mbs");
    addDockWidget(Qt::LeftDockWidgetArea,dockWidget1);
    QWidget *widget1 = new QWidget(dockWidget1);
    dockWidget1->setWidget(widget1);
    auto *widgetLayout1 = new QVBoxLayout(widget1);
    widgetLayout1->setContentsMargins(0,0,0,0);
    widget1->setLayout(widgetLayout1);
    widgetLayout1->addWidget(elementViewFilter);
    widgetLayout1->addWidget(elementView);

    QDockWidget *dockWidget3 = new QDockWidget("Parameter Tree (of selected object)", this);
    dockWidget3->setFeatures(QDockWidget::DockWidgetVerticalTitleBar);
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
    dockWidget4->setFeatures(QDockWidget::DockWidgetVerticalTitleBar);
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
    connect(inlineOpenMBVMW, SIGNAL(objectSelected(std::string, Object*)), this, SLOT(selectElement(std::string)));
    connect(inlineOpenMBVMW, SIGNAL(objectDoubleClicked(std::string, Object*)), this, SLOT(openElementEditor()));

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
    menu.setAutoRefresh(settings.value("mainwindow/options/autorefresh", true).toBool());
    menu.setStatusUpdate(settings.value("mainwindow/options/statusupdate", true).toBool());
    menu.setPlugins(settings.value("mainwindow/options/plugins", QString()).toString());
    menu.setBaseIndexForPlot(settings.value("mainwindow/options/baseindexforplot", 1).toInt());

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
      settings.setValue("mainwindow/options/autosave",         menu.getAutoSave());
      settings.setValue("mainwindow/options/autosaveinterval", menu.getAutoSaveInterval());
      settings.setValue("mainwindow/options/autoexport",       menu.getAutoExport());
      settings.setValue("mainwindow/options/autoexportdir",    menu.getAutoExportDir());
      settings.setValue("mainwindow/options/savestatevector",  menu.getSaveStateVector());
      settings.setValue("mainwindow/options/maxundo",          menu.getMaxUndo());
      settings.setValue("mainwindow/options/showfilters",      menu.getShowFilters());
      settings.setValue("mainwindow/options/autorefresh",      menu.getAutoRefresh());
      settings.setValue("mainwindow/options/statusupdate",     menu.getStatusUpdate());
      settings.setValue("mainwindow/options/plugins",          menu.getPlugins());
      settings.setValue("mainwindow/options/baseindexforplot", menu.getBaseIndexForPlot());

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

  // update model parameters including additional paramters from paramList
  void MainWindow::updateParameters(EmbedItemData *item, bool exceptLatestParameter) {
    shared_ptr<DOMDocument> doc=mbxmlparser->createDocument();
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
        for(auto & parent : parents) {
          string counterName = parent->getEmbedXMLElement()?E(parent->getEmbedXMLElement())->getAttribute("counterName"):"";
          if(not counterName.empty()) {
            string evaluatedCounterName(eval->cast<string>(eval->stringToValue(counterName,parent->getEmbedXMLElement(),false)));
            auto count=eval->create(1.0);
            eval->convertIndex(count, false);
            eval->addParam(evaluatedCounterName, count);
            eval->addParam(evaluatedCounterName+"_count", eval->create(1.0));
          }
        }
        DOMElement *ele = doc->getDocumentElement()->getFirstElementChild();
        eval->addParamSet(ele);
        string counterName = item->getEmbedXMLElement()?E(item->getEmbedXMLElement())->getAttribute("counterName"):"";
        if(not counterName.empty()) {
          string evaluatedCounterName(eval->cast<string>(eval->stringToValue(counterName,item->getEmbedXMLElement(),false)));
          auto count=eval->create(1.0);
          eval->convertIndex(count, false);
          eval->addParam(evaluatedCounterName, count);
          eval->addParam(evaluatedCounterName+"_count", eval->create(1.0));
        }
        ele = ele->getNextElementSibling()->getFirstElementChild();
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
      arg.append(settings.value("mainwindow/options/baseindexforplot", "1").toString());
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

    // we change the current directory (see below) hence we need to add the current dir as modulePath
    arg.append("--modulePath");
    arg.append(QDir::currentPath());

    arg.append(projectFile);
    echoView->clearOutput();
    process.setWorkingDirectory(uniqueTempDir_);
    process.start(QString::fromStdString((installPath/"bin"/"mbsimxml").string()), arg);
    statusBar()->showMessage(tr("Debug model"));
  }

  void MainWindow::selectElement(const string& ID) {
    Element *element = idMap[ID];
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    if(element) elementView->selectionModel()->setCurrentIndex(model->findItem(element,project->getDynamicSystemSolver()->getModelIndex()),QItemSelectionModel::ClearAndSelect);
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
    setWindowModified(true);
    auto *model = static_cast<ElementTreeModel*>(elementView->model());
    QModelIndex index = elementView->selectionModel()->currentIndex();
    auto *element = static_cast<Element*>(model->getItem(index)->getItemData());
    DOMElement *embedNode = element->getEmbedXMLElement();
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
        if(fileItem->getReference(i)!=dedicatedParent)
          fileItem->getReference(i)->updateNames();
      }
    }
  }

  void MainWindow::updateValues(EmbedItemData *parent) {
    auto *dedicatedParent = parent->getDedicatedItem();
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem) {
      for(int i=0; i<fileItem->getNumberOfReferences(); i++) {
        if(fileItem->getReference(i)!=dedicatedParent) {
          fileItem->getReference(i)->updateValues();
          fileItem->getReference(i)->updateNames();
	}
      }
    }
    fileItem = parent->getParameterFileItem();
    if(fileItem) {
      for(int i=0; i<fileItem->getNumberOfReferences(); i++) {
        if(fileItem->getReference(i)!=parent)
          fileItem->getReference(i)->updateValues();
      }
    }
  }

  void MainWindow::updateReferences(EmbedItemData *parent) {
    auto *dedicatedParent = parent->getDedicatedItem();
    auto *fileItem = dedicatedParent->getFileItem();
    if(fileItem) {
      fileItem->setModified(true);
      for(int i=0; i<fileItem->getNumberOfReferences(); i++) {
        if(fileItem->getReference(i)!=dedicatedParent) {
          fileItem->getReference(i)->setXMLElement(dedicatedParent->getXMLElement());
          fileItem->getReference(i)->clear();
          auto *model = static_cast<ElementTreeModel*>(elementView->model());
	  QModelIndex index = model->findItem(fileItem->getReference(i),project->getModelIndex());
          model->removeRows(0,model->rowCount(index),index);
          fileItem->getReference(i)->create();
          model->updateElementItem(static_cast<Element*>(fileItem->getReference(i)));
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
	  if(fileItem->getReference(i)!=dedicatedParent) {
	    fileItem->getReference(i)->clear();
	    auto *model = static_cast<ElementTreeModel*>(elementView->model());
	    QModelIndex index = model->findItem(fileItem->getReference(i),project->getModelIndex());
	    model->removeRows(0,model->rowCount(index),index);
	    fileItem->getReference(i)->create();
	    model->updateElementItem(static_cast<Element*>(fileItem->getReference(i)));
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
        if(fileItem->getReference(i)!=parent) {
          fileItem->getReference(i)->clearParameters();
          fileItem->getReference(i)->createParameters();
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

  void MainWindow::createFrame(DOMElement *ele, Element *parent) {
    if(not ele) return;
    Frame *frame = Embed<FixedRelativeFrame>::create(ele,parent);
    if(not frame)
      frame = Embed<NodeFrame>::create(ele,parent);
    if(not frame) {
      QMessageBox::warning(nullptr, "Create frame", "Cannot create frame.");
      return;
    }
    parent->getXMLFrames()->insertBefore(ele, nullptr);
    parent->addFrame(frame);
    frame->create();
    updateReferences(parent);
    static_cast<ElementTreeModel*>(elementView->model())->createFrameItem(frame,elementView->selectionModel()->currentIndex());
    elementView->selectionModel()->setCurrentIndex(frame->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(getAutoRefresh()) refresh();
  }

  void MainWindow::createContour(DOMElement *ele, Element *parent) {
    if(not ele) return;
    Contour *contour = Embed<Contour>::create(ele,parent);
    if(not contour) {
      QMessageBox::warning(nullptr, "Create contour", "Cannot create contour.");
      return;
    }
    parent->getXMLContours()->insertBefore(ele, nullptr);
    parent->addContour(contour);
    contour->create();
    updateReferences(parent);
    static_cast<ElementTreeModel*>(elementView->model())->createContourItem(contour,elementView->selectionModel()->currentIndex());
    elementView->selectionModel()->setCurrentIndex(contour->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(getAutoRefresh()) refresh();
  }

  void MainWindow::createGroup(DOMElement *ele, Element *parent) {
    if(not ele) return;
    Group *group = Embed<Group>::create(ele,parent);
    if(not group) {
      QMessageBox::warning(nullptr, "Create group", "Cannot create group.");
      return;
    }
    parent->getXMLGroups()->insertBefore(ele, nullptr);
    parent->addGroup(group);
    group->create();
    updateReferences(parent);
    static_cast<ElementTreeModel*>(elementView->model())->createGroupItem(group,elementView->selectionModel()->currentIndex());
    elementView->selectionModel()->setCurrentIndex(group->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(getAutoRefresh()) refresh();
  }

  void MainWindow::createObject(DOMElement *ele, Element *parent) {
    if(not ele) return;
    Object *object = Embed<Object>::create(ele,parent);
    if(not object) {
      QMessageBox::warning(nullptr, "Create object", "Cannot create object.");
      return;
    }
    parent->getXMLObjects()->insertBefore(ele, nullptr);
    parent->addObject(object);
    object->create();
    updateReferences(parent);
    static_cast<ElementTreeModel*>(elementView->model())->createObjectItem(object,elementView->selectionModel()->currentIndex());
    elementView->selectionModel()->setCurrentIndex(object->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(getAutoRefresh()) refresh();
  }

  void MainWindow::createLink(DOMElement *ele, Element *parent) {
    if(not ele) return;
    Link *link = Embed<Link>::create(ele,parent);
    if(not link) {
      QMessageBox::warning(nullptr, "Create link", "Cannot create link.");
      return;
    }
    parent->getXMLLinks()->insertBefore(ele, nullptr);
    parent->addLink(link);
    link->create();
    updateReferences(parent);
    static_cast<ElementTreeModel*>(elementView->model())->createLinkItem(link,elementView->selectionModel()->currentIndex());
    elementView->selectionModel()->setCurrentIndex(link->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(getAutoRefresh()) refresh();
  }

  void MainWindow::createConstraint(DOMElement *ele, Element *parent) {
    if(not ele) return;
    Constraint *constraint = Embed<Constraint>::create(ele,parent);
    if(not constraint) {
      QMessageBox::warning(nullptr, "Create constraint", "Cannot create constraint.");
      return;
    }
    parent->getXMLConstraints()->insertBefore(ele, nullptr);
    parent->addConstraint(constraint);
    constraint->create();
    updateReferences(parent);
    static_cast<ElementTreeModel*>(elementView->model())->createConstraintItem(constraint,elementView->selectionModel()->currentIndex());
    elementView->selectionModel()->setCurrentIndex(constraint->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(getAutoRefresh()) refresh();
  }

  void MainWindow::createObserver(DOMElement *ele, Element *parent) {
    if(not ele) return;
    Observer *observer = Embed<Observer>::create(ele,parent);
    if(not observer) {
      QMessageBox::warning(nullptr, "Create observer", "Cannot create observer.");
      return;
    }
    parent->getXMLObservers()->insertBefore(ele, nullptr);
    parent->addObserver(observer);
    observer->create();
    updateReferences(parent);
    static_cast<ElementTreeModel*>(elementView->model())->createObserverItem(observer,elementView->selectionModel()->currentIndex());
    elementView->selectionModel()->setCurrentIndex(observer->getModelIndex(), QItemSelectionModel::ClearAndSelect);
    if(getAutoRefresh()) refresh();
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
      QMessageBox::warning(0, "Create dynamic system solver", "Cannot create dynamic system solver.");
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
      QMessageBox::warning(0, "Create solver", "Cannot create solver.");
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
