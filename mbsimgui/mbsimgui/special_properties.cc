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
#include "special_properties.h"
#include "special_widgets.h"
#include "extended_widgets.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  OneDimMatArrayProperty::OneDimMatArrayProperty(int size, int m, int n, const MBXMLUtils::FQN &xmlName_, bool var_) : xmlName(xmlName_), var(var_) {
    resize_(size,m,n);
  }

  DOMElement* OneDimMatArrayProperty::initializeUsingXML(DOMElement *element) {
    DOMElement *e=E(element)->getFirstElementChildNamed(xmlName);
    if(e) {
      if(var) {
        e=e->getFirstElementChild();
        vector<Property*> p;
        while(e) {
          p.push_back(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(6,2,"0"),"",vector<string>(3,"")),"",7));
          p[p.size()-1]->initializeUsingXML(e);
          e=e->getNextElementSibling();
        }
        ele.resize(p.size());
        for(int i=0; i<ele.size(); i++)
          ele[i].setProperty(p[i]);
      }
      else {
        e=e->getFirstElementChild();
        for(int i=0; i<ele.size(); i++) {
          ele[i].initializeUsingXML(e);
          e=e->getNextElementSibling();
        }
      }
      return element;
    }
    return NULL;
  }

  DOMElement* OneDimMatArrayProperty::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *e=D(doc)->createElement(xmlName);
    parent->insertBefore(e, NULL);
    for(int i=0; i<ele.size(); i++) {
      DOMElement *ee=D(doc)->createElement(MBSIMFLEX%"ele");
      e->insertBefore(ee, NULL);
      ele[i].writeXMLFile(ee);
    }
    return 0;
  }

  void OneDimMatArrayProperty::resize_(int size, int m, int n) {
    if(ele.size() != size) {
      ele.resize(size);
      for(int i=0; i<ele.size(); i++) {
        string name = string("ele");
        if(not var) name += toStr(i+1);
        ele[i].setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(m,n,"0"),"",vector<string>(3,"")),"",7));
      }
    }
  }

  void OneDimMatArrayProperty::fromWidget(QWidget *widget) {
    resize_(static_cast<OneDimMatArrayWidget*>(widget)->getArray().size());
    for(int i=0; i<ele.size(); i++)
      ele[i].fromWidget(static_cast<OneDimMatArrayWidget*>(widget)->getArray()[i]);
  }

  void OneDimMatArrayProperty::toWidget(QWidget *widget) {
    static_cast<OneDimMatArrayWidget*>(widget)->resize_(ele.size(),0,0);
    for(int i=0; i<ele.size(); i++)
      ele[i].toWidget(static_cast<OneDimMatArrayWidget*>(widget)->getArray()[i]);
  }

  TwoDimMatArrayProperty::TwoDimMatArrayProperty(int size, int m, int n, const MBXMLUtils::FQN &xmlName_, bool var_) : xmlName(xmlName_), var(var_) {
    resize_(size,m,n);
  }

  DOMElement* TwoDimMatArrayProperty::initializeUsingXML(DOMElement *element) {
    DOMElement *e=E(element)->getFirstElementChildNamed(xmlName);
    if(e) {
      if(var) {
        e=e->getFirstElementChild();
        vector<vector<Property*> > p;
        while(e) {
          DOMElement *ee=e->getFirstElementChild();
          p.push_back(vector<Property*>());
          while(ee) {
            int i = p.size()-1;
            p[i].push_back(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(6,2,"0"),"",vector<string>(3,"")),"",7));
            p[i][p[i].size()-1]->initializeUsingXML(ee);
            ee=ee->getNextElementSibling();
          }
          e=e->getNextElementSibling();
        }
        ele.resize(p.size());
        for(int i=0; i<ele.size(); i++) {
          ele[i].resize(p[i].size());
          for(int j=0; j<ele[i].size(); j++)
            ele[i][j].setProperty(p[i][j]);
        }
      }
      else {
        e=e->getFirstElementChild();
        for(int i=0; i<ele.size(); i++) {
          DOMElement *ee=e->getFirstElementChild();
          for(int j=0; j<ele.size(); j++) {
            ele[i][j].initializeUsingXML(ee);
            ee=ee->getNextElementSibling();
          }
          e=e->getNextElementSibling();
        }
      }
      return element;
    }
    return NULL;
  }

  DOMElement* TwoDimMatArrayProperty::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *e=D(doc)->createElement(xmlName);
    parent->insertBefore(e, NULL);
    for(int i=0; i<ele.size(); i++) {
      DOMElement *ee=D(doc)->createElement(MBSIMFLEX%"row");
      e->insertBefore(ee, NULL);
      for(int j=0; j<ele.size(); j++) {
        DOMElement *eee=D(doc)->createElement(MBSIMFLEX%"ele");
        ee->insertBefore(eee, NULL);
        ele[i][j].writeXMLFile(eee);
      }
    }
    return 0;
  }

  void TwoDimMatArrayProperty::resize_(int size, int m, int n) {
    if(ele.size() != size) {
      ele.resize(size);
      if(var) {
        for(int i=0; i<size; i++) {
          ele[i].resize(size);
          for(int j=0; j<size; j++) {
            ele[i][j].setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(m,n,"0"),"",vector<string>(3,"")),"",7));
          }
        }
      } else {
        for(int i=0; i<size; i++) {
          ele[i].resize(size);
          for(int j=0; j<size; j++) {
            string name = string("ele");
            if(not var) name += toStr(i+1)+toStr(j+1);
            ele[i][j].setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(m,n,"0"),"",vector<string>(3,"")),"",7));
          }
        }
      }
    }
  }

  void TwoDimMatArrayProperty::fromWidget(QWidget *widget) {
    resize_(static_cast<TwoDimMatArrayWidget*>(widget)->getArray().size());
    for(int i=0; i<ele.size(); i++)
      for(int j=0; j<ele.size(); j++)
        ele[i][j].fromWidget(static_cast<TwoDimMatArrayWidget*>(widget)->getArray()[i][j]);
  }

  void TwoDimMatArrayProperty::toWidget(QWidget *widget) {
    static_cast<TwoDimMatArrayWidget*>(widget)->resize_(ele.size(),0,0);
    for(int i=0; i<ele.size(); i++)
      for(int j=0; j<ele.size(); j++)
        ele[i][j].toWidget(static_cast<TwoDimMatArrayWidget*>(widget)->getArray()[i][j]);
  }

}
