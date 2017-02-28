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

  OneDimVecArrayWidget::OneDimVecArrayWidget(int size, int m_, bool var) : m(m_) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    if(var) {
      sizeCombo = new CustomSpinBox;
      sizeCombo->setValue(size);
      layout->addWidget(sizeCombo);
      QObject::connect(sizeCombo, SIGNAL(valueChanged(int)), this, SLOT(currentIndexChanged(int)));
    }
    resize_(size,m,1);
  }

  void OneDimVecArrayWidget::currentIndexChanged(int size) {
    resize_(size,m,1);
//    emit sizeChanged(size);
  }

  void OneDimVecArrayWidget::resize_(int size, int m, int n) {
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
        ele[i] = new ExtWidget(name,new ChoiceWidget2(new VecWidgetFactory(m),QBoxLayout::RightToLeft));
        layout()->addWidget(ele[i]);
      }
//      for(int i=0; i<min((int)buf.size(),size); i++)
//        box[i]->setText(buf[i]);
    }
  }

  void OneDimVecArrayWidget::resize_(int m, int n) {
    for(int i=0; i<ele.size(); i++)
      ele[i]->resize_(m,1);
  }

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
    resize_(size,size,m,n);
 }

  void TwoDimMatArrayWidget::resize_(int rsize, int csize, int m, int n) {
    if(ele.size()!=rsize or ele[0].size()!=csize) {
      for(unsigned int i=0; i<ele.size(); i++) {
        for(unsigned int j=0; j<ele.size(); j++) {
          layout()->removeWidget(ele[i][j]);
          delete ele[i][j];
        }
      }
      ele.resize(rsize);
      for(int i=0; i<rsize; i++) {
        ele[i].resize(csize);
        for(int j=0; j<csize; j++) {
          QString name = QString("ele")+QString::number(i+1)+QString::number(j+1);
          ele[i][j] = new ExtWidget(name,new ChoiceWidget2(new MatWidgetFactory(m,n,vector<QStringList>(3),vector<int>(3,0)),QBoxLayout::RightToLeft));
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

  OneDimVecArrayWidgetFactory::OneDimVecArrayWidgetFactory(int size_, int m_, bool var_) : name(2), size(size_), m(m_), var(var_) {
    name[0] = "Vector";
    name[1] = "Cell array";
  }

  QWidget* OneDimVecArrayWidgetFactory::createWidget(int i) {
    if(i==0)
      return var?new ChoiceWidget2(new VecSizeVarWidgetFactory(size*m),QBoxLayout::RightToLeft):new ChoiceWidget2(new VecWidgetFactory(size*m),QBoxLayout::RightToLeft);
    if(i==1)
      return new OneDimVecArrayWidget(size,m,var);
    return NULL;
  }

  OneDimMatArrayWidgetFactory::OneDimMatArrayWidgetFactory() : name(2) {
    name[0] = "Matrix";
    name[1] = "Cell array";
  }

  QWidget* OneDimMatArrayWidgetFactory::createWidget(int i) {
    if(i==0)
      return new ChoiceWidget2(new MatWidgetFactory(3,3,vector<QStringList>(3),vector<int>(3,0)),QBoxLayout::RightToLeft);
    if(i==1)
      return new OneDimMatArrayWidget();
    return NULL;
  }

  TwoDimMatArrayWidgetFactory::TwoDimMatArrayWidgetFactory() : name(2) {
    name[0] = "Matrix";
    name[1] = "Cell array";
  }

  QWidget* TwoDimMatArrayWidgetFactory::createWidget(int i) {
    if(i==0)
      return new ChoiceWidget2(new MatWidgetFactory(3,3,vector<QStringList>(3),vector<int>(3,0)),QBoxLayout::RightToLeft);
    if(i==1)
      return new TwoDimMatArrayWidget();
    return NULL;
  }

}
