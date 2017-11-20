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
#include "namespace.h"
#include <vector>
#include <QVBoxLayout>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  OneDimVecArrayWidget::OneDimVecArrayWidget(int size, int m_, bool var) : m(m_) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    if(var) {
      sizeCombo = new CustomSpinBox;
      sizeCombo->setValue(size);
      layout->addWidget(sizeCombo);
      connect(sizeCombo, SIGNAL(valueChanged(int)), this, SLOT(currentIndexChanged(int)));
    }
    else
      if(size) resize_(size,m,1);
  }

  void OneDimVecArrayWidget::currentIndexChanged(int size) {
    resize_(size,m,1);
//    emit sizeChanged(size);
  }

  void OneDimVecArrayWidget::resize_(int size, int m, int n) {
    if(ele.size()!=size) {
//      vector<QString> buf(box.size());
      for(auto & i : ele) {
        layout()->removeWidget(i);
//        buf[i] = box[i]->text();
        delete i;
      }
      ele.resize(size);
      for(int i=0; i<size; i++) {
        QString name = QString("ele")+QString::number(i+1);
        ele[i] = new ExtWidget(name,new ChoiceWidget2(new VecWidgetFactory(m),QBoxLayout::RightToLeft,5),false,false);
        layout()->addWidget(ele[i]);
      }
//      for(int i=0; i<min((int)buf.size(),size); i++)
//        box[i]->setText(buf[i]);
    }
  }

  void OneDimVecArrayWidget::resize_(int m, int n) {
    for(auto & i : ele)
      i->resize_(m,1);
  }

  DOMElement* OneDimVecArrayWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=element;
    e=e->getFirstElementChild();
    if(ele.empty()) {
      int i=0;
      while(e) {
        QString name = QString("ele")+QString::number(i+1);
        ele.push_back(new ExtWidget(name,new ChoiceWidget2(new VecWidgetFactory(m),QHBoxLayout::RightToLeft,5),false,false));
        layout()->addWidget(ele[i]);
        ele[i]->initializeUsingXML(e);
        e=e->getNextElementSibling();
        i++;
      }
    }
    else {
      for(auto & i : ele) {
        i->initializeUsingXML(e);
        e=e->getNextElementSibling();
      }
    }
    return element;
  }

  DOMElement* OneDimVecArrayWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMNode *e = parent;
    for(auto & i : ele) {
      DOMElement *ee=D(doc)->createElement(MBSIMFLEX%"ele");
      e->insertBefore(ee, nullptr);
      i->writeXMLFile(ee);
    }
    return nullptr;
  }

  OneDimMatArrayWidget::OneDimMatArrayWidget(int size, int m, int n) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    if(size) resize_(size,m,n);
  }

  void OneDimMatArrayWidget::resize_(int size, int m, int n) {
    if(ele.size()!=size) {
//      vector<QString> buf(box.size());
      for(auto & i : ele) {
        layout()->removeWidget(i);
//        buf[i] = box[i]->text();
        delete i;
      }
      ele.resize(size);
      for(int i=0; i<size; i++) {
        QString name = QString("ele")+QString::number(i+1);
        ele[i] = new ExtWidget(name,new ChoiceWidget2(new MatWidgetFactory(m,n),QBoxLayout::RightToLeft,5),false,false);
        layout()->addWidget(ele[i]);
      }
    }
  }

  void OneDimMatArrayWidget::resize_(int m, int n) {
    for(auto & i : ele)
      i->resize_(m,n);
  }

  DOMElement* OneDimMatArrayWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=element;
    e=e->getFirstElementChild();
    if(ele.empty()) {
      int i=0;
      while(e) {
        QString name = QString("ele")+QString::number(i+1);
        ele.push_back(new ExtWidget(name,new ChoiceWidget2(new MatWidgetFactory(0,0),QHBoxLayout::RightToLeft,5),false,false));
        layout()->addWidget(ele[i]);
        ele[i]->initializeUsingXML(e);
        e=e->getNextElementSibling();
        i++;
      }
    }
    else {
      for(auto & i : ele) {
        i->initializeUsingXML(e);
        e=e->getNextElementSibling();
      }
    }
    return element;
  }

  DOMElement* OneDimMatArrayWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMNode *e = parent;
    for(auto & i : ele) {
      DOMElement *ee=D(doc)->createElement(MBSIMFLEX%"ele");
      e->insertBefore(ee, nullptr);
      i->writeXMLFile(ee);
    }
    return nullptr;
  }

  TwoDimMatArrayWidget::TwoDimMatArrayWidget(int size, int m, int n) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    if(size) resize_(size,size,m,n);
 }

  void TwoDimMatArrayWidget::resize_(int rsize, int csize, int m, int n) {
    if(ele.size()!=rsize or ele[0].size()!=csize) {
      for(auto & i : ele) {
        for(unsigned int j=0; j<ele.size(); j++) {
          layout()->removeWidget(i[j]);
          delete i[j];
        }
      }
      ele.resize(rsize);
      for(int i=0; i<rsize; i++) {
        ele[i].resize(csize);
        for(int j=0; j<csize; j++) {
          QString name = QString("ele")+QString::number(i+1)+QString::number(j+1);
          ele[i][j] = new ExtWidget(name,new ChoiceWidget2(new MatWidgetFactory(m,n),QBoxLayout::RightToLeft,5),false,false);
          layout()->addWidget(ele[i][j]);
        }
      }
    }
  }

  void TwoDimMatArrayWidget::resize_(int m, int n) {
    for(auto & i : ele)
      for(int j=0; j<i.size(); j++)
        i[j]->resize_(m,n);
  }

  DOMElement* TwoDimMatArrayWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=element;
    e=e->getFirstElementChild();
    if(ele.empty()) {
      int i=0;
      while(e) {
        DOMElement *ee=e->getFirstElementChild();
        int j=0;
        ele.emplace_back();
        while(ee) {
          QString name = QString("ele")+QString::number(i+1)+QString::number(j+1);
          ele[i].push_back(new ExtWidget(name,new ChoiceWidget2(new MatWidgetFactory(0,0),QHBoxLayout::RightToLeft,5),false,false));
          layout()->addWidget(ele[i][j]);
          ele[i][j]->initializeUsingXML(ee);
          ee=ee->getNextElementSibling();
          j++;
        }
        e=e->getNextElementSibling();
        i++;
      }
    }
    else {
      for(auto & i : ele) {
        DOMElement *ee=e->getFirstElementChild();
        for(int j=0; j<ele.size(); j++) {
          i[j]->initializeUsingXML(ee);
          ee=ee->getNextElementSibling();
        }
        e=e->getNextElementSibling();
      }
    }
    return element;
  }

  DOMElement* TwoDimMatArrayWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMNode *e = parent;
    for(auto & i : ele) {
      DOMElement *ee=D(doc)->createElement(MBSIMFLEX%"row");
      e->insertBefore(ee, nullptr);
      for(int j=0; j<ele.size(); j++) {
        DOMElement *eee=D(doc)->createElement(MBSIMFLEX%"ele");
        ee->insertBefore(eee, nullptr);
        i[j]->writeXMLFile(eee);
      }
    }
    return nullptr;
  }

  OneDimVecArrayWidgetFactory::OneDimVecArrayWidgetFactory(const FQN &xmlBase, int size_, int m_, bool var_) : name(2), xmlName(2,xmlBase), size(size_), m(m_), var(var_) {
    name[0] = "Cell array";
    name[1] = "Vector";
    xmlName[0].second += "Array";
  }

  QWidget* OneDimVecArrayWidgetFactory::createWidget(int i) {
    if(i==0)
      return new OneDimVecArrayWidget(size,m,var);
    if(i==1)
      return var?new ChoiceWidget2(new VecSizeVarWidgetFactory(size*m,3),QBoxLayout::RightToLeft,5):new ChoiceWidget2(new VecWidgetFactory(size*m),QBoxLayout::RightToLeft,5);
    return nullptr;
  }

  OneDimMatArrayWidgetFactory::OneDimMatArrayWidgetFactory(const FQN &xmlBase, int size_, int m_, int n_) : name(2), xmlName(2,xmlBase), size(size_), m(m_), n(n_) {
    name[0] = "Cell array";
    name[1] = "Matrix";
    xmlName[0].second += "Array";
  }

  QWidget* OneDimMatArrayWidgetFactory::createWidget(int i) {
    if(i==0)
      return new OneDimMatArrayWidget(size,m,n);
    if(i==1)
      return new ChoiceWidget2(new MatWidgetFactory(size*m,n,vector<QStringList>(3),vector<int>(3,0)),QBoxLayout::RightToLeft,5);
    return nullptr;
  }

  TwoDimMatArrayWidgetFactory::TwoDimMatArrayWidgetFactory(const FQN &xmlBase, int size_, int m_, int n_) : name(2), xmlName(2,xmlBase), size(size_), m(m_), n(n_) {
    name[0] = "Cell array";
    name[1] = "Matrix";
    xmlName[0].second += "Array";
  }

  QWidget* TwoDimMatArrayWidgetFactory::createWidget(int i) {
    if(i==0)
      return new TwoDimMatArrayWidget(size,m,n);
    if(i==1)
      return new ChoiceWidget2(new MatWidgetFactory(size*size*m,n,vector<QStringList>(3),vector<int>(3,0)),QBoxLayout::RightToLeft,5);
    return nullptr;
  }

}
