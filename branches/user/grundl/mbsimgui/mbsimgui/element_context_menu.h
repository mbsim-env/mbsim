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

class ElementContextMenu : public QMenu {
  Q_OBJECT

  public:
    ElementContextMenu(QWidget * parent = 0, bool removable=true);

  protected slots:
    void addContour();
};

class GroupContextMenu : public ElementContextMenu {
  Q_OBJECT

  public:
    GroupContextMenu(QWidget * parent = 0, bool removable=true);

  protected slots:
    void addObject();
    void addLink();
    void addObserver();
};

class ContourContextContextMenu : public QMenu {

  public:
    ContourContextContextMenu(QWidget * parent = 0);
};

class ObjectContextContextMenu : public QMenu {

  public:
    ObjectContextContextMenu(QWidget * parent = 0);
};

class LinkContextContextMenu : public QMenu {
  Q_OBJECT

  public:
    LinkContextContextMenu(QWidget * parent = 0);

  protected slots:
    void addSensor();
};

class ObserverContextContextMenu : public QMenu {

  public:
    ObserverContextContextMenu(QWidget * parent = 0);
};

class SensorContextContextMenu : public QMenu {

  public:
    SensorContextContextMenu(QWidget * parent = 0);
};

class SolverContextMenu : public GroupContextMenu {

  public:
    SolverContextMenu(QWidget * parent = 0) : GroupContextMenu(parent,false) {} 
};

class FrameContextMenu : public ElementContextMenu {

  public:
    FrameContextMenu(QWidget * parent = 0, bool removable=false) : ElementContextMenu(parent,removable) {}
};

class FixedRelativeFrameContextMenu : public FrameContextMenu {

  public:
    FixedRelativeFrameContextMenu(QWidget * parent = 0) : FrameContextMenu(parent,true) {}
};

class ObjectContextMenu : public ElementContextMenu {

  public:
    ObjectContextMenu(QWidget * parent = 0);
};

class BodyContextMenu : public ObjectContextMenu {

  public:
    BodyContextMenu(QWidget * parent = 0);
};


#endif
