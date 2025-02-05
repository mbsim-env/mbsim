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
#include "element_context_menu.h"
#include "mainwindow.h"
#include "rigid_body.h"
#include "flexible_ffr_body.h"
#include "constraint.h"
#include "kinetic_excitation.h"
#include "spring_damper.h"
#include "joint.h"
#include "friction.h"
#include "clutch.h"
#include "contact.h"
#include "observer.h"
#include "frame.h"
#include "contour.h"
#include "group.h"
#include "gear.h"
#include "connection.h"
#include "structure.h"
#include "sensor.h"
#include "physics.h"
#include "element_view.h"
#include "utils.h"
#include "project.h"
#include "view_menu.h"
#include "echo_view.h"
#include <mbxmlutils/eval.h>
#include <QMessageBox>
#include <xercesc/dom/DOMProcessingInstruction.hpp>

using namespace MBXMLUtils;

namespace {
  const QString importReferenceActionText(
    "Import/Reference model-element from file...");
  const QString importReferenceTooltipText(
    "Import an model-element from a file or use the XML 'Embed' functionality to reference a external model-element file.");
  const QString importReferenceTitle(
    "Import/Reference Model-Element");
}

namespace MBSimGUI {

  extern MainWindow *mw;

  ElementContextMenu::ElementContextMenu(Element *element, QWidget *parent, bool removable, bool saveable) : QMenu(parent) {
    // Edit
    setToolTipsVisible(true);
    auto *action=new QAction(QIcon::fromTheme("document-properties"), "Edit", this);
    action->setShortcut(QKeySequence("Ctrl+E"));
    connect(action,&QAction::triggered,this,[=](){ mw->openElementEditor(); });
    addAction(action);
    if(saveable) {
      // Exit XML
      action=new QAction(QIcon::fromTheme("document-properties"), "Edit XML", this);
      connect(action,&QAction::triggered,mw,&MainWindow::editElementSource);
      addAction(action);
      addSeparator();
      // Export model-element
      action=new QAction(QIcon::fromTheme("document-save-as"), "Export model-element to file...", this);
      connect(action,&QAction::triggered,[]() { mw->exportElement("Export Model-Element"); } );
      addAction(action);
    }
    if(removable) {
      // Copy
      addSeparator();
      action=new QAction(QIcon::fromTheme("edit-copy"), "Copy", this);
      action->setShortcut(QKeySequence::Copy);
      connect(action,&QAction::triggered,this,[=](){ mw->copyElement(); });
      addAction(action);
      // Cut
      action=new QAction(QIcon::fromTheme("edit-cut"), "Cut", this);
      action->setShortcut(QKeySequence::Cut);
      connect(action,&QAction::triggered,this,[=](){ mw->copyElement(true); });
      addAction(action);
      // Remove
      addSeparator();
      action=new QAction(QIcon::fromTheme("edit-delete"), "Remove", this);
      action->setShortcut(QKeySequence::Delete);
      connect(action,&QAction::triggered,mw,&MainWindow::removeElement);
      addAction(action);
      addSeparator();

      // Embed
      bool embedActive = false;
      auto embedEle = element->getEmbedXMLElement();
      if(embedEle) {
        bool hasEmbedCount = E(embedEle)->hasAttribute("count");
        if(!hasEmbedCount || E(embedEle)->getAttribute("count")!="0")
          embedActive = true;
      }
      action=new QAction(Utils::QIconCached(QString::fromStdString((
                 MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/(embedActive ? "embed-active.svg" : "embed.svg")).string())), "Array/Pattern", this);
      action->setToolTip("Create/Edit the array/pattern properties of this element using the XML 'Embed' functionality.\n"
                         "With the array/pattern functionality an element can be duplicated to an array/pattern of N elements.");
      connect(action,&QAction::triggered,this,[=](){ mw->openCloneEditor(); });
      if(E(element->getXMLElement())->getTagName()==PV%"Embed") {
        // this is a Embed element which cannot be handled by mbsimgui -> disable the Embed action to avoid the creation
        // of a nested embed
        action->setEnabled(false);
        action->setToolTip("Array/Pattern is NOT possible using UI functionality for this element.\n"
                           "Please open the element dialog and directly edit the XML content of this Embed element.\n"
                           "(this is because the href and/or parameterHref attribute evaluates to different files\n"
                           "for dependent on the counter variable of this or parent Array/Pattern elements)");
      }
      addAction(action);

      addSeparator();

      // Enable/Disable
      action = new QAction("Enable", this);
      action->setCheckable(true);
      action->setChecked(element->getEnabled());
      action->setEnabled(element->getParent()->getEnabled());
      connect(action,&QAction::toggled,mw,&MainWindow::enableElement);
      addAction(action);
    }

    // add the context actions
    auto contextAction=ElementPropertyDialog::getMBSimGUIContextActions(element->getXMLElement()); // read from XML
    if(!contextAction.empty()) {
      addSeparator();
      for(auto &ca : contextAction) {
        addAction(ca.first.c_str(), [ca, element](){
          // this code is run when the action is triggered
          mw->clearEchoView(QString("Running context action '")+ca.first.c_str()+"':\n");
          try {
            auto parameterLevels = mw->updateParameters(element);
            auto [counterName, values]=MainWindow::evaluateForAllArrayPattern(parameterLevels, ca.second,
              nullptr, true, true, false, true, [element](){
                fmatvec::Atom::msgStatic(fmatvec::Atom::Info)<<std::endl<<std::endl;
                if(mw->eval->getName()=="python")
                  mw->eval->addParam("mbsimgui_element",
                    mw->eval->stringToValue("import mbsimgui; ret=mbsimgui.Element("+std::to_string(reinterpret_cast<size_t>(element))+")"));
              });
            mw->updateEchoView();
          }
          catch(const std::exception &ex) {
            fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<ex.what()<<std::endl;
            mw->updateEchoView();
          }
        });
      }
    }
  }

  DynamicSystemSolverContextMenu::DynamicSystemSolverContextMenu(Element *element, QWidget *parent) : ElementContextMenu(element,parent,false,true) {
    addSeparator();
    setToolTipsVisible(true);
    QAction *action = new QAction(QIcon::fromTheme("document-open"), "Import/Reference model-element from file (replacing the DSS) ...", this);
    action->setToolTip(importReferenceTooltipText);
    connect(action,&QAction::triggered,this,[=](){ mw->createDynamicSystemSolver(mw->loadEmbedItemData(element, importReferenceTitle)); });
    addAction(action);
    addSeparator();
    addMenu(new ViewMenu(this));
  }

  FrameContextMenu::FrameContextMenu(Frame *frame, QWidget *parent, bool removable) : ElementContextMenu(frame,parent,removable) {
    addSeparator();
    QAction *action=new QAction(QIcon::fromTheme("go-up"), "Move up", this);
    action->setShortcut(QKeySequence("Ctrl+Up"));
    action->setEnabled(frame->getParent()->getIndexOfFrame(frame)>1);
    connect(action,&QAction::triggered,this,[=](){ mw->moveFrame(true); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("go-down"), "Move down", this);
    action->setShortcut(QKeySequence("Ctrl+Down"));
    action->setEnabled(frame->getParent()->getIndexOfFrame(frame)<frame->getParent()->getNumberOfFrames()-1);
    connect(action,&QAction::triggered,this,[=](){ mw->moveFrame(false); });
    addAction(action);
    addSeparator();
    addMenu(new ViewMenu(this));
  }

  ContourContextMenu::ContourContextMenu(Contour *contour, QWidget *parent, bool removable) : ElementContextMenu(contour,parent,removable) {
    addSeparator();
    QAction *action=new QAction(QIcon::fromTheme("go-up"), "Move up", this);
    action->setEnabled(contour->getParent()->getIndexOfContour(contour)>0);
    action->setShortcut(QKeySequence("Ctrl+Up"));
    connect(action,&QAction::triggered,this,[=](){ mw->moveContour(true); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("go-down"), "Move down", this);
    action->setShortcut(QKeySequence("Ctrl+Down"));
    action->setEnabled(contour->getParent()->getIndexOfContour(contour)<contour->getParent()->getNumberOfContours()-1);
    connect(action,&QAction::triggered,this,[=](){ mw->moveContour(false); });
    addAction(action);
    addSeparator();
    addMenu(new ViewMenu(this));
  }

  GroupContextMenu::GroupContextMenu(Group *group, QWidget *parent, bool removable) : ElementContextMenu(group,parent,removable) {
    addSeparator();
    QAction *action=new QAction(QIcon::fromTheme("go-up"), "Move up", this);
    action->setShortcut(QKeySequence("Ctrl+Up"));
    action->setEnabled(group->getParent()->getIndexOfGroup(group)>0);
    connect(action,&QAction::triggered,this,[=](){ mw->moveGroup(true); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("go-down"), "Move down", this);
    action->setShortcut(QKeySequence("Ctrl+Down"));
    action->setEnabled(group->getParent()->getIndexOfGroup(group)<group->getParent()->getNumberOfGroups()-1);
    connect(action,&QAction::triggered,this,[=](){ mw->moveGroup(false); });
    addAction(action);
    addSeparator();
    addMenu(new ViewMenu(this));
  }

  ObjectContextMenu::ObjectContextMenu(Object *object, QWidget *parent, bool removable) : ElementContextMenu(object,parent,removable) {
    addSeparator();
    QAction *action=new QAction(QIcon::fromTheme("go-up"), "Move up", this);
    action->setEnabled(object->getParent()->getIndexOfObject(object)>0);
    action->setShortcut(QKeySequence("Ctrl+Up"));
    connect(action,&QAction::triggered,this,[=](){ mw->moveObject(true); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("go-down"), "Move down", this);
    action->setShortcut(QKeySequence("Ctrl+Down"));
    action->setEnabled(object->getParent()->getIndexOfObject(object)<object->getParent()->getNumberOfObjects()-1);
    connect(action,&QAction::triggered,this,[=](){ mw->moveObject(false); });
    addAction(action);
    addSeparator();
    addMenu(new ViewMenu(this));
  }

  LinkContextMenu::LinkContextMenu(Link *link, QWidget *parent, bool removable) : ElementContextMenu(link,parent,removable) {
    addSeparator();
    QAction *action=new QAction(QIcon::fromTheme("go-up"), "Move up", this);
    action->setShortcut(QKeySequence("Ctrl+Up"));
    action->setEnabled(link->getParent()->getIndexOfLink(link)>0);
    connect(action,&QAction::triggered,this,[=](){ mw->moveLink(true); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("go-down"), "Move down", this);
    action->setShortcut(QKeySequence("Ctrl+Down"));
    action->setEnabled(link->getParent()->getIndexOfLink(link)<link->getParent()->getNumberOfLinks()-1);
    connect(action,&QAction::triggered,this,[=](){ mw->moveLink(false); });
    addAction(action);
    addSeparator();
    addMenu(new ViewMenu(this));
  }

  ConstraintContextMenu::ConstraintContextMenu(Constraint *constraint, QWidget *parent, bool removable) : ElementContextMenu(constraint,parent,removable) {
    addSeparator();
    QAction *action=new QAction(QIcon::fromTheme("go-up"), "Move up", this);
    action->setShortcut(QKeySequence("Ctrl+Up"));
    action->setEnabled(constraint->getParent()->getIndexOfConstraint(constraint)>0);
    connect(action,&QAction::triggered,this,[=](){ mw->moveConstraint(true); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("go-down"), "Move down", this);
    action->setShortcut(QKeySequence("Ctrl+Down"));
    action->setEnabled(constraint->getParent()->getIndexOfConstraint(constraint)<constraint->getParent()->getNumberOfConstraints()-1);
    connect(action,&QAction::triggered,this,[=](){ mw->moveConstraint(false); });
    addAction(action);
    addSeparator();
    addMenu(new ViewMenu(this));
  }

  ObserverContextMenu::ObserverContextMenu(Observer *observer, QWidget *parent, bool removable) : ElementContextMenu(observer,parent,removable) {
    addSeparator();
    QAction *action=new QAction(QIcon::fromTheme("go-up"), "Move up", this);
    action->setShortcut(QKeySequence("Ctrl+Up"));
    action->setEnabled(observer->getParent()->getIndexOfObserver(observer)>0);
    connect(action,&QAction::triggered,this,[=](){ mw->moveObserver(true); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("go-down"), "Move down", this);
    action->setShortcut(QKeySequence("Ctrl+Down"));
    action->setEnabled(observer->getParent()->getIndexOfObserver(observer)<observer->getParent()->getNumberOfObservers()-1);
    connect(action,&QAction::triggered,this,[=](){ mw->moveObserver(false); });
    addAction(action);
    addSeparator();
    addMenu(new ViewMenu(this));
  }

  FramesContextMenu::FramesContextMenu(Element *element, const QString &title, QWidget *parent) : QMenu(title,parent) {
    setToolTipsVisible(true);
    QAction *action = new QAction(QIcon::fromTheme("document-open"), importReferenceActionText, this);
    action->setToolTip(importReferenceTooltipText);
    connect(action,&QAction::triggered,this,[=](){ mw->createAny(mw->loadEmbedItemData(element, importReferenceTitle),element, MBSIM%"Frame"); });
    addAction(action);
    action = new QAction(QIcon::fromTheme("edit-paste"), "Paste", this);
    action->setShortcut(QKeySequence::Paste);
    action->setEnabled(dynamic_cast<Frame*>(mw->getElementBuffer().first));
    connect(action,&QAction::triggered,this,[=](){ mw->createAny(mw->pasteElement(element,mw->getElementBuffer().first),element, MBSIM%"Frame"); });
    addAction(action);
  }

  FixedRelativeFramesContextMenu::FixedRelativeFramesContextMenu(Element *element, const QString &title, QWidget *parent) : FramesContextMenu(element,title,parent) {
    addSeparator();
    createContextMenuFor<FixedRelativeFrame>(this, element, "Add '");
    addSeparator();
    addMenu(new ViewMenu(this));
  }

  NodeFramesContextMenu::NodeFramesContextMenu(Element *element, const QString &title, QWidget *parent) : FramesContextMenu(element,title,parent) {
    addSeparator();
    createContextMenuFor<NodeFrame>(this, element, "Add '");
    addSeparator();
    addMenu(new ViewMenu(this));
  }

  ContoursContextMenu::ContoursContextMenu(Element *element, const QString &title, QWidget *parent) : QMenu(title,parent) {
    setToolTipsVisible(true);
    QAction *action = new QAction(QIcon::fromTheme("document-open"), importReferenceActionText, this);
    action->setToolTip(importReferenceTooltipText);
    connect(action,&QAction::triggered,this,[=](){ mw->createAny(mw->loadEmbedItemData(element, importReferenceTitle),element, MBSIM%"Contour"); });
    addAction(action);
    action = new QAction(QIcon::fromTheme("edit-paste"), "Paste", this);
    action->setShortcut(QKeySequence::Paste);
    action->setEnabled(dynamic_cast<Contour*>(mw->getElementBuffer().first));
    connect(action,&QAction::triggered,this,[=](){ mw->createAny(mw->pasteElement(element,mw->getElementBuffer().first),element, MBSIM%"Contour"); });
    addAction(action);
    addSeparator();
    createContextMenuFor<Contour>(this, element, "Add '");
    addSeparator();
    addMenu(new ViewMenu(this));
  }

  GroupsContextMenu::GroupsContextMenu(Element *element, const QString &title, QWidget *parent) : QMenu(title,parent) {
    setToolTipsVisible(true);
    QAction *action = new QAction(QIcon::fromTheme("document-open"), importReferenceActionText, this);
    action->setToolTip(importReferenceTooltipText);
    connect(action,&QAction::triggered,this,[=](){ mw->createAny(mw->loadEmbedItemData(element, importReferenceTitle),element, MBSIM%"Group"); });
    addAction(action);
    action = new QAction(QIcon::fromTheme("edit-paste"), "Paste", this);
    action->setShortcut(QKeySequence::Paste);
    action->setEnabled(dynamic_cast<Group*>(mw->getElementBuffer().first));
    connect(action,&QAction::triggered,this,[=](){ mw->createAny(mw->pasteElement(element,mw->getElementBuffer().first),element, MBSIM%"Group"); });
    addAction(action);
    addSeparator();
    createContextMenuFor<Group>(this, element, "Add '");
    addSeparator();
    addMenu(new ViewMenu(this));
  }

  ObjectsContextMenu::ObjectsContextMenu(Element *element, const QString &title, QWidget *parent) : QMenu(title,parent) {
    setToolTipsVisible(true);
    QAction *action = new QAction(QIcon::fromTheme("document-open"), importReferenceActionText, this);
    action->setToolTip(importReferenceTooltipText);
    connect(action,&QAction::triggered,this,[=](){ mw->createAny(mw->loadEmbedItemData(element, importReferenceTitle),element, MBSIM%"Object"); });
    addAction(action);
    action = new QAction(QIcon::fromTheme("edit-paste"), "Paste", this);
    action->setShortcut(QKeySequence::Paste);
    action->setEnabled(dynamic_cast<Object*>(mw->getElementBuffer().first));
    connect(action,&QAction::triggered,this,[=](){ mw->createAny(mw->pasteElement(element,mw->getElementBuffer().first),element, MBSIM%"Object"); });
    addAction(action);
    addSeparator();
    createContextMenuFor<Object>(this, element, "Add '");
    addSeparator();
    addMenu(new ViewMenu(this));
  }

  LinksContextMenu::LinksContextMenu(Element *element, const QString &title,  QWidget *parent) : QMenu(title,parent) {
    setToolTipsVisible(true);
    QAction *action = new QAction(QIcon::fromTheme("document-open"), importReferenceActionText, this);
    action->setToolTip(importReferenceTooltipText);
    connect(action,&QAction::triggered,this,[=](){ mw->createAny(mw->loadEmbedItemData(element, importReferenceTitle),element, MBSIM%"Link"); });
    addAction(action);
    action = new QAction(QIcon::fromTheme("edit-paste"), "Paste", this);
    action->setShortcut(QKeySequence::Paste);
    action->setEnabled(dynamic_cast<Link*>(mw->getElementBuffer().first));
    connect(action,&QAction::triggered,this,[=](){ mw->createAny(mw->pasteElement(element,mw->getElementBuffer().first),element, MBSIM%"Link"); });
    addAction(action);
    addSeparator();
    createContextMenuFor<Link>(this, element, "Add '");
    addSeparator();
    addMenu(new ViewMenu(this));
  }

  ConstraintsContextMenu::ConstraintsContextMenu(Element *element, const QString &title, QWidget *parent) : QMenu(title,parent) {
    setToolTipsVisible(true);
    QAction *action = new QAction(QIcon::fromTheme("document-open"), importReferenceActionText, this);
    action->setToolTip(importReferenceTooltipText);
    connect(action,&QAction::triggered,this,[=](){ mw->createAny(mw->loadEmbedItemData(element, importReferenceTitle),element, MBSIM%"Constraint"); });
    addAction(action);
    action = new QAction(QIcon::fromTheme("edit-paste"), "Paste", this);
    action->setShortcut(QKeySequence::Paste);
    action->setEnabled(dynamic_cast<Constraint*>(mw->getElementBuffer().first));
    connect(action,&QAction::triggered,this,[=](){ mw->createAny(mw->pasteElement(element,mw->getElementBuffer().first),element, MBSIM%"Constraint"); });
    addAction(action);
    addSeparator();
    createContextMenuFor<Constraint>(this, element, "Add '");
    addSeparator();
    addMenu(new ViewMenu(this));
  }

  ObserversContextMenu::ObserversContextMenu(Element *element, const QString &title, QWidget *parent) : QMenu(title,parent) {
    setToolTipsVisible(true);
    QAction *action = new QAction(QIcon::fromTheme("document-open"), importReferenceActionText, this);
    action->setToolTip(importReferenceTooltipText);
    connect(action,&QAction::triggered,this,[=](){ mw->createAny(mw->loadEmbedItemData(element, importReferenceTitle),element, MBSIM%"Observer"); });
    addAction(action);
    action = new QAction(QIcon::fromTheme("edit-paste"), "Paste", this);
    action->setShortcut(QKeySequence::Paste);
    action->setEnabled(dynamic_cast<Observer*>(mw->getElementBuffer().first));
    connect(action,&QAction::triggered,this,[=](){ mw->createAny(mw->pasteElement(element,mw->getElementBuffer().first),element, MBSIM%"Observer"); });
    addAction(action);
    addSeparator();
    createContextMenuFor<Observer>(this, element, "Add '");
    addSeparator();
    addMenu(new ViewMenu(this));
  }

}
