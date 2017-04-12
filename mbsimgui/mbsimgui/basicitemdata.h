/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2013 Martin Förg

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

#ifndef _BASICITEMDATA__H_
#define _BASICITEMDATA__H_

#include "treeitemdata.h"
#include "element.h"

namespace MBSimGUI {

  class FrameItemData : public TreeItemData {
    private:
      Element *element;
    public:
      FrameItemData(Element *element_) : TreeItemData("frames",""), element(element_) { }
      virtual QMenu* createContextMenu() { return element->createFrameContextMenu(); }
  };

  class ContourItemData : public TreeItemData {
    private:
      Element *element;
    public:
      ContourItemData(Element *element_) : TreeItemData("contours",""), element(element_) { }
      virtual QMenu* createContextMenu() { return new ContoursContextMenu(element); }
  };

  class GroupItemData : public TreeItemData {
    private:
      Element *element;
    public:
      GroupItemData(Element *element_) : TreeItemData("groups",""), element(element_) { }
      virtual QMenu* createContextMenu() { return new GroupsContextMenu(element); }
  };


  class ObjectItemData : public TreeItemData {
    private:
      Element *element;
    public:
      ObjectItemData(Element *element_) : TreeItemData("objects",""), element(element_) { }
      virtual QMenu* createContextMenu() { return new ObjectsContextMenu(element); }
  };

  class LinkItemData : public TreeItemData {
    private:
      Element *element;
    public:
      LinkItemData(Element *element_) : TreeItemData("links",""), element(element_) { }
      virtual QMenu* createContextMenu() { return new LinksContextMenu(element); }
  };

  class ConstraintItemData : public TreeItemData {
    private:
      Element *element;
    public:
      ConstraintItemData(Element *element_) : TreeItemData("constraints",""), element(element_) { }
      virtual QMenu* createContextMenu() { return new ConstraintsContextMenu(element); }
  };

  class ObserverItemData : public TreeItemData {
    private:
      Element *element;
    public:
      ObserverItemData(Element *element_) : TreeItemData("observers",""), element(element_) { }
      virtual QMenu* createContextMenu() { return new ObserversContextMenu(element); }
  };

}

#endif
