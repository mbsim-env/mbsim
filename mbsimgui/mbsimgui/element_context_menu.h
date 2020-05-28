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

#ifndef _ELEMENT_CONTEXT_MENU_H_
#define _ELEMENT_CONTEXT_MENU_H_

#include <QMenu>

namespace MBSimGUI {

  class Element;
  class Frame;
  class Contour;
  class Group;
  class Object;
  class Link;
  class Constraint;
  class Observer;

  class ElementContextMenu : public QMenu {
    public:
      ElementContextMenu(Element *element, QWidget *parent=nullptr, bool removable=true, bool saveable=true);
   };

  class FrameContextMenu : public ElementContextMenu {
    public:
      FrameContextMenu(Frame *frame, QWidget *parent=nullptr, bool removable=true);
  };

  class ContourContextMenu : public ElementContextMenu {
    public:
      ContourContextMenu(Contour *contour, QWidget *parent=nullptr, bool removable=true);
  };

  class GroupContextMenu : public ElementContextMenu {
    public:
      GroupContextMenu(Group *group, QWidget *parent=nullptr, bool removable=true);
  };

  class ObjectContextMenu : public ElementContextMenu {
    public:
      ObjectContextMenu(Object *object, QWidget *parent=nullptr, bool removable=true);
  };

  class LinkContextMenu : public ElementContextMenu {
    public:
      LinkContextMenu(Link *link, QWidget *parent=nullptr, bool removable=true);
  };

  class ConstraintContextMenu : public ElementContextMenu {
    public:
      ConstraintContextMenu(Constraint *constraint, QWidget *parent=nullptr, bool removable=true);
  };

  class ObserverContextMenu : public ElementContextMenu {
    public:
      ObserverContextMenu(Observer *observer, QWidget *parent=nullptr, bool removable=true);
  };

  class FramesContextMenu : public QMenu {
    public:
      FramesContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);
  };

  class FixedRelativeFramesContextMenu : public FramesContextMenu {
    public:
      FixedRelativeFramesContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);
  };

  class NodeFramesContextMenu : public FramesContextMenu {
    public:
      NodeFramesContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);
  };

  class ContoursContextMenu : public QMenu {
    public:
      ContoursContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);
  };

  class GroupsContextMenu : public QMenu {
    public:
      GroupsContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);
  };

  class ObjectsContextMenu : public QMenu {
    public:
      ObjectsContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);
  };

  class LinksContextMenu : public QMenu {
    public:
      LinksContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);
  };

  class ConstraintsContextMenu : public QMenu {
    public:
      ConstraintsContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);
  };

  class ObserversContextMenu : public QMenu {
    public:
      ObserversContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);
  };

  class SignalsContextMenu : public QMenu {
    public:
      SignalsContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);
  };

}

#endif
