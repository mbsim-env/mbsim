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

  class ContainerItemData : public TreeItemData {
    protected:
      Element *element;
    public:
      ContainerItemData(const QString &name, Element *element_) : TreeItemData(name,""), element(element_) { }
      Element* getElement() { return element; }
      virtual QMenu* createContextMenu() { return element->createFrameContextMenu(); }
  };

  class FrameItemData : public ContainerItemData {
    public:
      FrameItemData(Element *element) : ContainerItemData("frames",element) { }
  };

  class ContourItemData : public ContainerItemData {
    public:
      ContourItemData(Element *element) : ContainerItemData("contours",element) { }
  };

  class GroupItemData : public ContainerItemData {
    public:
      GroupItemData(Element *element) : ContainerItemData("groups",element) { }
  };


  class ObjectItemData : public ContainerItemData {
    public:
      ObjectItemData(Element *element) : ContainerItemData("objects",element) { }
  };

  class LinkItemData : public ContainerItemData {
    public:
      LinkItemData(Element *element) : ContainerItemData("links",element) { }
  };

  class ConstraintItemData : public ContainerItemData {
    public:
      ConstraintItemData(Element *element) : ContainerItemData("constraints",element) { }
  };

  class ObserverItemData : public ContainerItemData {
    public:
      ObserverItemData(Element *element) : ContainerItemData("observers",element) { }
  };

}

#endif
