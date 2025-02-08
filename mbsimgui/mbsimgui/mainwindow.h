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

#ifndef __MAINWINDOW_H_
#define __MAINWINDOW_H_

#include <QMainWindow>
#include <QProcess>
#include <QTimer>
#include <QElapsedTimer>
#include <QDir>
#include <boost/filesystem/path.hpp>
#include <xercesc/util/XercesDefs.hpp>
#include <deque>
#include <sstream>
#include <openmbv/mainwindow.h>
#include "frame.h"
#include "mbxmlutils/eval.h"

class QAction;
class QModelIndex;
class FrameItemData;
class ContourItemData;
class GroupItemData;
class ObjectItemData;
class LinkItemData;
class ConstraintItemData;
class ObserverItemData;

namespace OpenMBVGUI {
  class MainWindow;
  class AbstractViewFilter;
}

namespace MBXMLUtils {
  class Eval;
  class NewParamLevel;
  class DOMParser;
}

namespace XERCES_CPP_NAMESPACE {
  class DOMImplementation;
  class DOMLSParser;
  class DOMLSSerializer;
  class DOMDocument;
  class DOMElement;
}

namespace MBSimGUI {

  class MBSimThread;
  class ElementView;
  class ParameterView;
  class EchoView;
  class FileView;
  class PropertyDialog;
  class Element;
  class Frame;
  class Contour;
  class Group;
  class Object;
  class Link;
  class Constraint;
  class Observer;
  class Solver;
  class Parameter;
  class EmbedItemData;
  class Project;
  class FileItemData;
  class EchoStream;
  class LinearSystemAnalysisDialog;
  class FlexibleBodyTool;
  class StateTableDialog;

  class MainWindow : public QMainWindow {
    Q_OBJECT

    private:
      static bool exitOK;
      std::unordered_map<std::string,Element*> idMap;
      Project *project;
      std::vector<FileItemData*> file;
      ElementView *elementView;
      std::vector<ElementView*> itemView;
      ParameterView *parameterView;
      EchoView *echoView;
      FileView *fileView;
      QTabWidget *tabWidget;
      std::shared_ptr<bool> debugStreamFlag;
      QString projectFile;
      QProcess process;
      OpenMBVGUI::MainWindow *inlineOpenMBVMW;
      boost::filesystem::path uniqueTempDir;
      QAction *actionSave, *actionSaveProject, *actionSimulate, *actionInterrupt, *actionKill, *actionOpenMBV, *actionH5plotserie, *actionLinearSystemAnalysis, *actionStateTable, *actionSaveDataAs, *actionSaveMBSimH5DataAs, *actionSaveOpenMBVDataAs, *actionRefresh, *actionDebug, *actionSaveStateVectorAs, *actionSaveStateTableAs, *actionSaveLinearSystemAnalysisAs, *actionUndo, *actionRedo;
      OpenMBVGUI::AbstractViewFilter *elementViewFilter, *parameterViewFilter;
      QTimer autoSaveTimer;
      QElapsedTimer statusTime;
      int IDcounter{0};
      std::string currentID;
      enum { maxRecentFiles = 5 };
      QAction *recentProjectFileActs[maxRecentFiles];
      bool allowUndo;
      int maxUndo;
      bool autoRefresh;
      bool statusUpdate;
      bool callViewAllAfterFileReloaded { false };
      std::shared_ptr<xercesc::DOMDocument> doc;
      std::deque<std::vector<std::shared_ptr<xercesc::DOMDocument>>> undos, redos;
      std::pair<Element*,bool> elementBuffer;
      std::pair<Parameter*,bool> parameterBuffer;
      QString fmuFileName;
      LinearSystemAnalysisDialog *lsa{nullptr};
      FlexibleBodyTool *fbt{nullptr};
      StateTableDialog *st{nullptr};
      QString configPath;
      void initInlineOpenMBV();
      void dragEnterEvent(QDragEnterEvent *event) override;
      void dropEvent(QDropEvent *event) override;
      void closeEvent(QCloseEvent *event) override;
      void showEvent(QShowEvent *event) override;
      bool maybeSave();
      void setCurrentProjectFile(const QString &fileName);
      void updateRecentProjectFileActions();
      void move(bool up);
      void openOptionsMenu(bool justSetOptions=false);
      void manageTemplates();
      void openRecentProjectFile();
      void newProject();
      void newProjectFromTemplate();
      void loadProject(const QString &file, bool updateRecent=true);
      bool saveProject(const QString &fileName="", bool modifyStatus=true);
      void edit();
      void undo();
      void redo();
      void copy(bool cut=false);
      void cut() { copy(true); }
      void paste();
      void remove();
      void moveUp() { move(true); }
      void moveDown() { move(false); }
      void saveDataAs();
      void saveMBSimH5DataAs();
      void saveOpenMBVDataAs();
      void saveStateVectorAs();
      void saveStateTableAs();
      void saveLinearSystemAnalysisAs();
      void help();
      void about();
      void relnotes();
      void simulate();
      void interrupt();
      void openmbv();
      void h5plotserie();
      void linearSystemAnalysis();
      void showStateTable();
      void debug();
      void kill();
      void createFMU();
      void elementChanged(const QModelIndex &current);
      void elementViewClicked(const QModelIndex &current);
      void parameterViewClicked(const QModelIndex &current);
      void processFinished(int exitCode, QProcess::ExitStatus exitStatus);
      void updateStatus();
      void autoSaveProject();
      void updateNames(EmbedItemData *element);
      void updateValues(EmbedItemData *element);
      void updateReferences(EmbedItemData *element);
      void updateParameterReferences(EmbedItemData *parent);
      void saveReferencedFile(int i);
      void convertDocument();
      void createNewEvaluator();

      int openedEditors { 0 };
      int mbsimxmlQueuedTask { -1 };
    private slots:
      void selectElement(const std::string& ID, OpenMBVGUI::Object *obj);
      void abstractViewFilterOptionsChanged();

    public:
      MainWindow(QStringList &arg);
      ~MainWindow() override;
      std::shared_ptr<MBXMLUtils::DOMParser> mbxmlparser;
      std::shared_ptr<MBXMLUtils::DOMParser> mbxmlparserNoVal;
      std::shared_ptr<MBXMLUtils::Eval> eval;
      xercesc::DOMImplementation *impl;
      xercesc::DOMLSSerializer *serializer;
      void mbsimxmlQueue();
      void mbsimxml(int task);
      const boost::filesystem::path& getUniqueTempDir() const { return uniqueTempDir; }
      void addParameter(Parameter *parameter, EmbedItemData *parent);
      void addFrame(Frame *frame, Element *parent);
      void addContour(Contour *contour, Element *parent);
      void addGroup(Group *group, Element *parent);
      void addObject(Object *object, Element *parent);
      void addLink(Link *link, Element *parent);
      void addConstraint(Constraint *constraint, Element *parent);
      void addObserver(Observer *observer, Element *parent);
      template<class Base> void add(Base *base, TreeItemData *item);
      void pasteParameter(EmbedItemData *parent, Parameter *param=nullptr);
      void loadParameter(EmbedItemData *parent);
      void removeParameter(EmbedItemData *parent);
      xercesc::DOMElement* pasteElement(Element *parent, Element *element);
      xercesc::DOMElement* loadEmbedItemData(EmbedItemData *parent, const QString &title);
      void updateEchoView();
      void clearEchoView(const QString &initialText="");

      template<class Container>
      QModelIndex getContainerIndex(Element *parent);
      // create an Element based on the XML content "ele" as a child of the Element "parent"
      bool createFrame(xercesc::DOMElement *ele, Element *parent, bool showDialogOnError=true);
      bool createContour(xercesc::DOMElement *ele, Element *parent, bool showDialogOnError=true);
      bool createGroup(xercesc::DOMElement *ele, Element *parent, bool showDialogOnError=true);
      bool createObject(xercesc::DOMElement *ele, Element *parent, bool showDialogOnError=true);
      bool createLink(xercesc::DOMElement *ele, Element *parent, bool showDialogOnError=true);
      bool createConstraint(xercesc::DOMElement *ele, Element *parent, bool showDialogOnError=true);
      bool createObserver(xercesc::DOMElement *ele, Element *parent, bool showDialogOnError=true);
      // create an Element based on the XML content "ele" in the correct container of the parent of the Element parent (used by import/reference element (where the user maybe does not know the proper container))
      void createAny(xercesc::DOMElement *ele, Element *parent, const MBXMLUtils::FQN &requestedXMLType);

      void createDynamicSystemSolver(xercesc::DOMElement *ele);
      void createSolver(xercesc::DOMElement *ele);
      void highlightObject(const std::string &ID);
      const std::string& getHighlightedObject() const { return currentID; }
      ElementView* getElementView() { return elementView; }
      ParameterView* getParameterView() { return parameterView; }
      void updateUndos();
      void selectSolver(Solver *solver);
      const std::pair<Element*,bool>& getElementBuffer() const { return elementBuffer; }
      const std::pair<Parameter*,bool>& getParameterBuffer() const { return parameterBuffer; }
      Project* getProject() { return project; }
      QElapsedTimer& getStatusTime() { return statusTime; }
      QString getProjectFile() const { return projectFile; }
      QString getProjectFilePath() const;
      QString getProjectPath() const { return QFileInfo(getProjectFilePath()).canonicalPath(); }
      QDir getProjectDir() const { return QFileInfo(getProjectFilePath()).dir(); }
      bool getAutoRefresh() const { return autoRefresh; }
      bool getStatusUpdate() const { return statusUpdate; }

      bool editorIsOpen() const { return openedEditors > 0; }

      void loadProject();
      bool saveProjectAs();
      void saveProjectAsTemplate();
      void refresh();
      void xmlHelp(const QString &url="");
      void editElementSource();
      void editParametersSource();
      void exportElement(const QString &title);
      void copyElement(bool cut=false);
      void removeElement();
      void enableElement(bool enable);
      void moveFrame(bool up);
      void moveContour(bool up);
      void moveGroup(bool up);
      void moveObject(bool up);
      void moveLink(bool up);
      void moveConstraint(bool up);
      void moveObserver(bool up);
      void copyParameter(bool cut=false);
      void removeParameter();
      void moveParameter(bool up);
      void saveMBSimH5Data(const QString &file);
      void saveOpenMBVXMLData(const QString &file);
      void saveOpenMBVH5Data(const QString &file);
      void saveStateVector(const QString &file);
      void saveStateTable(const QString &file);
      void saveInputTable(const QString &file);
      void saveOutputTable(const QString &file);
      void saveLinearSystemAnalysis(const QString &file);

      struct ParameterLevel {
        ParameterLevel(std::shared_ptr<xercesc::DOMDocument> doc_, xercesc::DOMElement *paramEle_,
                       std::string counterName_={}, std::string countStr_={}, std::string onlyIfStr_={}) :
          doc(doc_), paramEle(paramEle_),
          counterName(counterName_), countStr(std::move(countStr_)), onlyIfStr(std::move(onlyIfStr_)) {}
        std::shared_ptr<xercesc::DOMDocument> doc; // just needed for lifetime handling of paramEle
        xercesc::DOMElement *paramEle; // a "pv:Parameter" XML element of parameter of this level
        std::string counterName; // may be empty if this parameter level does not contain a embed
        std::string countStr;
        std::string onlyIfStr;
      };
      // Updates/initialize the mbsimgui internal evaluator (member variable eval) with the parameters
      // required for the element item.
      // Returns a list of all possible parameter levels, even if no Embed is defined on a level.
      // (note that only embeds on container elements are considered here)
      // If exceptLatestParameter is true then the last parameter (if item is a parameter) is skipped (not added)
      // count is used as the count for the embeds, if not given 0 (0-based count) is used.
      // In rare cases it is possible that count changes the result of vector<ParameterLevel>. In this case
      // you need to call updateParameter again to get the proper result.
      std::vector<ParameterLevel> updateParameters(EmbedItemData *item, bool exceptLatestParameter=false,
                                                   const std::vector<int>& count={});

      // Evaluates the string code as full eval if set to true or as partial eval if it is false.
      // The evaluation may be done multiple times, for each possible combination of Embed count ones.
      // The possible Embed's are taken from parameterLevels which is the output of updateParameters(...).
      // If catchErrors is true all errros are catched and it is tried to continue as good as possible.
      // If trackFirstLastCall is true special boolean parameters are added before code is evaluated indicating if the
      // current call is the first or last call. Doing so will be slightly slower.
      // Returns, as first, a list of all counterNames which are relevant (none existing Embed elements or empty counterName
      // (with count=1) are not returned.
      // As second a map with all possible combinations of these counts and the corresponding evaluation is returned.
      static std::pair<std::vector<std::string>, std::map<std::vector<int>, MBXMLUtils::Eval::Value>> evaluateForAllArrayPattern(
        const std::vector<ParameterLevel> &parameterLevels, const std::string &code, xercesc::DOMElement *e,
        bool fullEval, bool skipRet, bool catchErrors, bool trackFirstLastCall=false,
        const std::function<void(const std::vector<std::string>&, const std::vector<int>&)> &preCodeFunc={});

      void rebuildTree();
      void exportParameters();
      void openParameterEditor(bool config=true);
      void openCloneEditor();
      FileItemData* addFile(const QFileInfo &file);
      void removeFile(FileItemData *fileItem);
      std::string getID(Element* element) { std::string ID = std::to_string(IDcounter++); idMap[ID] = element; return ID; }
      static int getExitOK() { return exitOK; }
      static void setExitBad() { exitOK=false; }
      static boost::filesystem::path getInstallPath();
      void flexibleBodyTool();
      FlexibleBodyTool *getFlexibleBodyTool() { return fbt; }
      void expandToDepth(int depth);

      // Prepare the MainWindow for a "quasi" modal dialog open.
      // The MainWindow will still be active (the dialog to open is none modal) but many features of the MainWindow
      // are disabled, e.g. only the 3D view and other basic stuff is still active.
      // prepareForPropertyDialogOpen/prepareForPropertyDialogClose must be called as a pair!
      void prepareForPropertyDialogOpen();
      // Prepare the MainWindow for a "quasi" modal dialog close.
      // This just reverts the actions on the MainWindow taken by prepareForPropertyDialogOpen().
      void prepareForPropertyDialogClose();
      static void updateNameOfCorrespondingElementAndItsChilds(const QModelIndex &index);
    public slots:
      void openElementEditor(bool config=true);

  };

  template<> inline void MainWindow::add<FixedRelativeFrame>(FixedRelativeFrame *base, TreeItemData *item) { addFrame(base, static_cast<Element*>(item)); }
  template<> inline void MainWindow::add<Object>(Object *base, TreeItemData *item) { addObject(base, static_cast<Element*>(item)); }
  template<> inline void MainWindow::add<Link>(Link *base, TreeItemData *item) { addLink(base, static_cast<Element*>(item)); }
  template<> inline void MainWindow::add<Observer>(Observer *base, TreeItemData *item) { addObserver(base, static_cast<Element*>(item)); }
  template<> inline void MainWindow::add<Constraint>(Constraint *base, TreeItemData *item) { addConstraint(base, static_cast<Element*>(item)); }
  template<> inline void MainWindow::add<Group>(Group *base, TreeItemData *item) { addGroup(base, static_cast<Element*>(item)); }
  template<> inline void MainWindow::add<Contour>(Contour *base, TreeItemData *item) { addContour(base, static_cast<Element*>(item)); }
  template<> inline void MainWindow::add<NodeFrame>(NodeFrame *base, TreeItemData *item) { addFrame(base, static_cast<Element*>(item)); }
  template<> inline void MainWindow::add<Solver>(Solver *base, TreeItemData *item) { selectSolver(base); }
  template<> inline void MainWindow::add<Parameter>(Parameter *base, TreeItemData *item) { addParameter(base, static_cast<EmbedItemData*>(item)); }

  extern template QModelIndex MainWindow::getContainerIndex<FrameItemData>(Element *parent);
  extern template QModelIndex MainWindow::getContainerIndex<ContourItemData>(Element *parent);
  extern template QModelIndex MainWindow::getContainerIndex<GroupItemData>(Element *parent);
  extern template QModelIndex MainWindow::getContainerIndex<ObjectItemData>(Element *parent);
  extern template QModelIndex MainWindow::getContainerIndex<LinkItemData>(Element *parent);
  extern template QModelIndex MainWindow::getContainerIndex<ConstraintItemData>(Element *parent);
  extern template QModelIndex MainWindow::getContainerIndex<ObserverItemData>(Element *parent);

}

#endif
