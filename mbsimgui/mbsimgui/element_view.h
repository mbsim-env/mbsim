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

#ifndef _ELEMENT_VIEW__H_
#define _ELEMENT_VIEW__H_

#include <QTreeView>

namespace MBSimGUI {

  class Element;
  class SingleLineDelegate;

  class ElementView : public QTreeView {
    public:
      class Node {
	public:
	  Node(const QString &name_, bool expanded_, bool selected_) : name(name_), expanded(expanded_), selected(selected_) { }
	  void addChild(const Node &node) { child.push_back(node); }
	  int getNumberOfChilds() const { return child.size(); }
	  Node& getChild(int i) { return child[i]; }
	  const QString& getName() const { return name; }
	  bool isExpanded() const { return expanded; }
	  bool isSelected() const { return selected; }
	private:
	  QString name;
	  bool expanded{false};
	  bool selected{false};
	  std::vector<Node> child;
      };
      ElementView(QWidget *parent=nullptr);
      ~ElementView();
      void save(const QModelIndex &index, Node &node);
      void restore(const QModelIndex &index, Node &node);
      void expandToDepth(const QModelIndex &index, int depth);
    private:
      void mouseDoubleClickEvent(QMouseEvent *event) override;
      void mousePressEvent(QMouseEvent *event) override;
      SingleLineDelegate *commentDelegate;
  };

}

#endif
