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
#include "parameter_view.h"
#include "mainwindow.h"
#include <QPainter>

namespace MBSimGUI {

  extern MainWindow *mw;

  ParameterView::ParameterView(QWidget *parent) : QTreeView(parent) {
    valueDelegate=new ValueDelegate(this);
    setItemDelegateForColumn(1, valueDelegate);
  }

  ParameterView::~ParameterView() {
    delete valueDelegate;
  }

  void ParameterView::mouseDoubleClickEvent(QMouseEvent *event) {
    mw->openParameterEditor();
  }

  void ParameterView::mousePressEvent ( QMouseEvent * event ) {
    if(not mw->editorIsOpen())
      QTreeView::mousePressEvent(event);
  }

  ParameterView::ValueDelegate::ValueDelegate(ParameterView *pv) : parameterView(pv) {}

  void ParameterView::ValueDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const {
    painter->save();
    QString text(parameterView->model()->data(index).toString());
    text=text.trimmed();
    text=text.replace(QRegularExpression(" *\n *"), " ¶ ");
    text=text.simplified();
    drawDisplay(painter, option, option.rect, text);
    painter->restore();
  }

  QSize ParameterView::ValueDelegate::sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const {
    return QSize(0, option.font.pixelSize());
  }

}
