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

#include <single_line_delegate.h>
#include <QPainter>

namespace MBSimGUI {

SingleLineDelegate::SingleLineDelegate(QTreeView *tv) : QItemDelegate(tv), treeView(tv) {}

void SingleLineDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  painter->save();
  QString text(treeView->model()->data(index).toString());
  text=text.trimmed();
  text=text.replace(QRegularExpression(" *\n *"), " ¶ ");
  text=text.simplified();
  auto useroption = option;
  useroption.palette.setBrush(QPalette::Text,treeView->model()->data(index,Qt::ForegroundRole).value<QBrush>());
  drawDisplay(painter, useroption, useroption.rect, text);
  painter->restore();
}

QSize SingleLineDelegate::sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const {
  return QSize(0, option.font.pixelSize());
}

}
