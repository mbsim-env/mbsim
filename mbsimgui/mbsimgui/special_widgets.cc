/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2016 Martin Förg

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

#include <config.h>
#include "special_widgets.h"
#include "extended_widgets.h"
#include "variable_widgets.h"
#include <vector>
#include <QVBoxLayout>

using namespace std;

namespace MBSimGUI {

  OneDimMatArrayWidget::OneDimMatArrayWidget(int size, int m, int n) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    resize_(size,m,n);
  }

  void OneDimMatArrayWidget::resize_(int size, int m, int n) {
    if(ele.size()!=size) {
//      vector<QString> buf(box.size());
      for(unsigned int i=0; i<ele.size(); i++) {
        layout()->removeWidget(ele[i]);
//        buf[i] = box[i]->text();
        delete ele[i];
      }
      ele.resize(size);
      for(int i=0; i<size; i++) {
        QString name = QString("ele")+QString::number(i+1);
        ele[i] = new ExtWidget(name,new ChoiceWidget2(new MatWidgetFactory(m,n,vector<QStringList>(3),vector<int>(3,0)),QBoxLayout::RightToLeft));
        layout()->addWidget(ele[i]);
      }
//      for(int i=0; i<min((int)buf.size(),size); i++)
//        box[i]->setText(buf[i]);
    }
  }

  void OneDimMatArrayWidget::resize_(int m, int n) {
    for(int i=0; i<ele.size(); i++)
      ele[i]->resize_(m,n);
  }

  TwoDimMatArrayWidget::TwoDimMatArrayWidget(int size, int m, int n) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    resize_(size,m,n);
 }

  void TwoDimMatArrayWidget::resize_(int size, int m, int n) {
    if(ele.size()!=size) {
      for(unsigned int i=0; i<ele.size(); i++) {
        for(unsigned int j=0; j<ele.size(); j++) {
          layout()->removeWidget(ele[i][j]);
          delete ele[i][j];
        }
      }
      ele.resize(size);
      for(int i=0; i<size; i++) {
        ele[i].resize(size);
        for(int j=0; j<size; j++) {
          QString name = QString("ele")+QString::number(i+1)+QString::number(j+1);
          ele[i][j] = new ExtWidget(name,new ChoiceWidget2(new MatWidgetFactory(m,m,vector<QStringList>(3),vector<int>(3,0)),QBoxLayout::RightToLeft));
          layout()->addWidget(ele[i][j]);
        }
      }
    }
  }

  void TwoDimMatArrayWidget::resize_(int m, int n) {
    for(int i=0; i<ele.size(); i++)
      for(int j=0; j<ele[i].size(); j++)
        ele[i][j]->resize_(m,n);
  }

}
