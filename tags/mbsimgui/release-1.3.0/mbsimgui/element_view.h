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

#ifndef _ELEMENT_VIEW__H_
#define _ELEMENT_VIEW__H_

#include <QTreeView>
#include <QModelIndex>

class Element;
class ElementPropertyDialog;

class ElementView : public QTreeView {
  Q_OBJECT
  private:
    QModelIndex index;
    Element *element;
    ElementPropertyDialog *editor;
  public:
    ElementView(QWidget *parent=0) : QTreeView(parent), element(0), editor(0) {}
    void openEditor();
  protected:
    void mouseDoubleClickEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
  protected slots:
    void dialogFinished(int result);
    void apply();
};

#endif
