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

  OneDimMatColsVarArrayWidget::OneDimMatColsVarArrayWidget(int size, int m, int n) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    resize_(size,m,n);
  }

  void OneDimMatColsVarArrayWidget::resize_(int size, int m, int n) {
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
        ele[i] = new ExtWidget(name,new ChoiceWidget2(new MatColsVarWidgetFactory(m,n,vector<QStringList>(3),vector<int>(3,0)),QBoxLayout::RightToLeft));
        layout()->addWidget(ele[i]);
      }
//      for(int i=0; i<min((int)buf.size(),size); i++)
//        box[i]->setText(buf[i]);
    }
  }

  OneDimSqrMatSizeVarArrayWidget::OneDimSqrMatSizeVarArrayWidget(int size, int n)  {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    resize_(size,n,n);
  }

  void OneDimSqrMatSizeVarArrayWidget::resize_(int size, int m, int n) {
    if(ele.size()!=size) {
      for(unsigned int i=0; i<ele.size(); i++) {
        layout()->removeWidget(ele[i]);
        delete ele[i];
      }
      ele.resize(size);
      for(int i=0; i<size; i++) {
        QString name = QString("ele")+QString::number(i+1);
        ele[i] = new ExtWidget(name,new ChoiceWidget2(new SqrMatSizeVarWidgetFactory(m,vector<QStringList>(3),vector<int>(3,0)),QBoxLayout::RightToLeft));
        layout()->addWidget(ele[i]);
      }
    }
  }

  OneDimVarSizeMatColsVarArrayWidget::OneDimVarSizeMatColsVarArrayWidget(int size, int m_, int n) : m(m_) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    QWidget *box = new QWidget;
    QHBoxLayout *hbox = new QHBoxLayout;
    box->setLayout(hbox);
    hbox->setMargin(0);
    layout->addWidget(box);
    sizeCombo = new CustomSpinBox;
    sizeCombo->setRange(1,100);
    sizeCombo->setValue(size);
    QObject::connect(sizeCombo, SIGNAL(valueChanged(int)), this, SLOT(currentIndexChanged(int)));
    hbox->addWidget(sizeCombo);
    hbox->addStretch(2);
    widget = new OneDimMatColsVarArrayWidget(size,m,n);
    layout->addWidget(widget);
  }

  void OneDimVarSizeMatColsVarArrayWidget::resize_(int size, int m, int n) {
    widget->resize_(size,m,n);
    sizeCombo->blockSignals(true);
    sizeCombo->setValue(size);
    sizeCombo->blockSignals(false);
  }

  void OneDimVarSizeMatColsVarArrayWidget::currentIndexChanged(int size) {
    widget->resize_(size,m,size);
    emit sizeChanged(size);
  }

  OneDimVarSizeSqrMatSizeVarArrayWidget::OneDimVarSizeSqrMatSizeVarArrayWidget(int size, int n) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    QWidget *box = new QWidget;
    QHBoxLayout *hbox = new QHBoxLayout;
    box->setLayout(hbox);
    hbox->setMargin(0);
    layout->addWidget(box);
    sizeCombo = new CustomSpinBox;
    sizeCombo->setRange(1,100);
    sizeCombo->setValue(size);
    QObject::connect(sizeCombo, SIGNAL(valueChanged(int)), this, SLOT(currentIndexChanged(int)));
    hbox->addWidget(sizeCombo);
    hbox->addStretch(2);
    widget = new OneDimSqrMatSizeVarArrayWidget(size,n);
    layout->addWidget(widget);
  }

  void OneDimVarSizeSqrMatSizeVarArrayWidget::resize_(int size, int m, int n) {
    widget->resize_(size,m,n);
    sizeCombo->blockSignals(true);
    sizeCombo->setValue(size);
    sizeCombo->blockSignals(false);
  }

  void OneDimVarSizeSqrMatSizeVarArrayWidget::currentIndexChanged(int size) {
    widget->resize_(size,size,size);
    emit sizeChanged(size);
  }

  TwoDimSqrMatSizeVarArrayWidget::TwoDimSqrMatSizeVarArrayWidget(int size, int n) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    resize_(size,n,n);
 }

  void TwoDimSqrMatSizeVarArrayWidget::resize_(int size, int m, int n) {
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
          ele[i][j] = new ExtWidget(name,new ChoiceWidget2(new SqrMatSizeVarWidgetFactory(m,vector<QStringList>(3),vector<int>(3,0)),QBoxLayout::RightToLeft));
          layout()->addWidget(ele[i][j]);
        }
      }
    }
  }

  TwoDimVarSizeSqrMatSizeVarArrayWidget::TwoDimVarSizeSqrMatSizeVarArrayWidget(int size, int n) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    QWidget *box = new QWidget;
    QHBoxLayout *hbox = new QHBoxLayout;
    box->setLayout(hbox);
    hbox->setMargin(0);
    layout->addWidget(box);
    sizeCombo = new CustomSpinBox;
    sizeCombo->setRange(1,100);
    sizeCombo->setValue(size);
    QObject::connect(sizeCombo, SIGNAL(valueChanged(int)), this, SLOT(currentIndexChanged(int)));
    hbox->addWidget(sizeCombo);
    hbox->addStretch(2);
    widget = new TwoDimSqrMatSizeVarArrayWidget(size,n);
    layout->addWidget(widget);
  }

  void TwoDimVarSizeSqrMatSizeVarArrayWidget::resize_(int size, int m, int n) {
    widget->resize_(size,m,n);
    sizeCombo->blockSignals(true);
    sizeCombo->setValue(size);
    sizeCombo->blockSignals(false);
  }

  void TwoDimVarSizeSqrMatSizeVarArrayWidget::currentIndexChanged(int size) {
    widget->resize_(size,size,size);
    emit sizeChanged(size);
  }

}
