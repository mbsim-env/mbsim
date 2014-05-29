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

#ifndef _BASIC_PROPERTIES_H_
#define _BASIC_PROPERTIES_H_

#include <string>
#include "utils.h"
#include "extended_properties.h"
#include <boost/function.hpp>
#include <QFileInfo>

namespace MBSimGUI {

  class Element;
  class Frame;
  class Contour;
  class Object;
  class Link;
  class RigidBody;
  class Signal;

  namespace XERCES_CPP_NAMESPACE {
    class DOMElement;
    class DOMNode;
  }

  class LocalFrameOfReferenceProperty : public Property {
    protected:
      std::string frame;
      Frame *framePtr;
      Element* element;
      MBXMLUtils::FQN xmlName;
    public:
      LocalFrameOfReferenceProperty(const std::string &frame_="", Element* element_=0, const MBXMLUtils::FQN &xmlName_="");
      virtual Property* clone() const {return new LocalFrameOfReferenceProperty(*this);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element); 
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
      void setFrame(const std::string &str);
      std::string getFrame() const;
  };

  class ParentFrameOfReferenceProperty : public Property {
    protected:
      std::string frame;
      Frame *framePtr;
      Element* element;
      MBXMLUtils::FQN xmlName;
    public:
      ParentFrameOfReferenceProperty(const std::string &frame_="", Element* element_=0, const MBXMLUtils::FQN &xmlName_="");
      virtual Property* clone() const {return new ParentFrameOfReferenceProperty(*this);}
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void initialize();
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
      void setFrame(const std::string &str);
      std::string getFrame() const;
  };

  class FrameOfReferenceProperty : public Property {
    protected:
      std::string frame;
      Frame *framePtr;
      Element* element;
      MBXMLUtils::FQN xmlName;
    public:
      FrameOfReferenceProperty(const std::string &frame="", Element* element=0, const MBXMLUtils::FQN &xmlName=""); 
      virtual Property* clone() const {return new FrameOfReferenceProperty(*this);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element); 
      void initialize();
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
      void setFrame(const std::string &str);
      std::string getFrame() const;
  };

  class ContourOfReferenceProperty : public Property {
    protected:
      std::string contour;
      Contour *contourPtr;
      Element* element;
      MBXMLUtils::FQN xmlName;
    public:
      ContourOfReferenceProperty(const std::string &contour_="", Element* element_=0, const MBXMLUtils::FQN &xmlName_=""); 
      virtual Property* clone() const {return new ContourOfReferenceProperty(*this);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element); 
      void initialize();
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
      void setContour(const std::string &str);
      std::string getContour() const; 
  };

  class RigidBodyOfReferenceProperty : public Property {
    protected:
      std::string body;
      RigidBody *bodyPtr;
      Element* element;
      MBXMLUtils::FQN xmlName;
    public:
      RigidBodyOfReferenceProperty(const std::string &body_="", Element *element_=0, const MBXMLUtils::FQN &xmlName_="");
      virtual Property* clone() const {return new RigidBodyOfReferenceProperty(*this);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element); 
      void initialize();
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
      void setBody(const std::string &str);
      std::string getBody() const;
      RigidBody* getBodyPtr() const {return bodyPtr;}
  };

  class ObjectOfReferenceProperty : public Property {
    protected:
      std::string object;
      Object *objectPtr;
      Element* element;
      MBXMLUtils::FQN xmlName;
    public:
      ObjectOfReferenceProperty(const std::string &object_="", Element *element_=0, const MBXMLUtils::FQN &xmlName_=""); 
      virtual Property* clone() const {return new ObjectOfReferenceProperty(*this);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element); 
      void initialize();
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
      void setObject(const std::string &str);
      std::string getObject() const;
  };

  class LinkOfReferenceProperty : public Property {
    protected:
      std::string link;
      Link *linkPtr;
      Element* element;
      MBXMLUtils::FQN xmlName;
    public:
      LinkOfReferenceProperty(const std::string &link_="", Element *element_=0, const MBXMLUtils::FQN &xmlName_=""); 
      virtual Property* clone() const {return new LinkOfReferenceProperty(*this);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element); 
      void initialize();
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
      void setLink(const std::string &str);
      std::string getLink() const;
  };

  class SignalOfReferenceProperty : public Property {
    protected:
      std::string signal;
      Signal *signalPtr;
      Element* element;
      MBXMLUtils::FQN xmlName;
    public:
      SignalOfReferenceProperty(const std::string &signal_="", Element *element_=0, const MBXMLUtils::FQN &xmlName_=""); 
      virtual Property* clone() const {return new SignalOfReferenceProperty(*this);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element); 
      void initialize();
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
      void setSignal(const std::string &str);
      std::string getSignal() const;
  };

  class FileProperty : public Property {

    public:
      FileProperty(const MBXMLUtils::FQN &xmlName_) : xmlName(xmlName_) {}
      virtual Property* clone() const {return new FileProperty(*this);}
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
      std::string getFile() const;
      void setFile(const std::string &str);

    protected:
      std::string file;
      QFileInfo fileInfo;
      MBXMLUtils::FQN xmlName;
  };

  class IntegerProperty : public Property {

    public:
      IntegerProperty(int value_, const MBXMLUtils::FQN &xmlName_) : value(value_), xmlName(xmlName_) {}
      virtual Property* clone() const {return new IntegerProperty(*this);}
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
      int getValue() const {return value;}
      void setValue(int value_) {value = value_;}

    protected:
      int value;
      MBXMLUtils::FQN xmlName;
  };

  class TextProperty : public Property {

    public:
      TextProperty(const std::string &text_, const MBXMLUtils::FQN &xmlName_, bool quote_=false) : text(text_), xmlName(xmlName_), quote(quote_) {}
      virtual Property* clone() const {return new TextProperty(*this);}
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
      const std::string& getText() const {return text;}
      void setText(const std::string &text_) {text = text_;}

    protected:
      std::string text;
      MBXMLUtils::FQN xmlName;
      bool quote;
  };

  class ConnectFramesProperty : public Property {

    public:
      ConnectFramesProperty(int n, Element* element, const MBXMLUtils::FQN &xmlName_=MBSIM%"connect");
      virtual Property* clone() const {return new ConnectFramesProperty(*this);}

      void initialize();
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      std::vector<FrameOfReferenceProperty> frame;
      MBXMLUtils::FQN xmlName;
  };

  class ConnectContoursProperty : public Property {

    public:
      ConnectContoursProperty(int n, Element* element, const MBXMLUtils::FQN &xmlName_=MBSIM%"connect");
      virtual Property* clone() const {return new ConnectContoursProperty(*this);}

      void initialize();
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      std::vector<ContourOfReferenceProperty> contour;
      MBXMLUtils::FQN xmlName;
  };

  class SolverTolerancesProperty : public Property {

    public:
      SolverTolerancesProperty();
      virtual Property* clone() const {return new SolverTolerancesProperty(*this);}

      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ExtProperty projection, g, gd, gdd, la, La;
  };

  class SolverParametersProperty : public Property {

    public:
      SolverParametersProperty();
      virtual Property* clone() const {return new SolverParametersProperty(*this);}

      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ExtProperty constraintSolver, impactSolver, numberOfMaximalIterations, tolerances;
  };

  class EmbedProperty : public Property {

    public:
      EmbedProperty(boost::function<const std::string&()> f);
      virtual Property* clone() const {return new EmbedProperty(*this);}
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
      bool hasFile() const {return (href.isActive() && static_cast<const FileProperty*>(href.getProperty())->getFile()!="");}
      std::string getFile() const {return static_cast<const FileProperty*>(href.getProperty())->getFile();}
      bool hasCounter() const {return counterName.isActive();}
      std::string getCounterName() const {return counterName.isActive()?static_cast<const TextProperty*>(counterName.getProperty())->getText():"";}
      int getCount() const {return static_cast<const IntegerProperty*>(count.getProperty())->getValue();}
      bool hasParameterFile() const {return (parameterList.isActive() && static_cast<const FileProperty*>(parameterList.getProperty())->getFile()!="");}
      std::string getParameterFile() const {return static_cast<const FileProperty*>(parameterList.getProperty())->getFile();}

    protected:
      ExtProperty href, count, counterName, parameterList;

  };

  class SignalReferenceProperty : public Property {
    public:
      SignalReferenceProperty(Element* element);
      virtual Property* clone() const {return new SignalReferenceProperty(*this);}
      void initialize() {refSignal.initialize();}
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      SignalOfReferenceProperty refSignal;
      ExtProperty factor;
  };

  class ColorProperty : public Property {
    protected:
      ExtProperty color;
      MBXMLUtils::FQN xmlName;
    public:
      ColorProperty(const MBXMLUtils::FQN &xmlName=""); 
      virtual Property* clone() const {return new ColorProperty(*this);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element); 
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
  };

  class PlotFeatureStatusProperty : public Property {
    protected:
      std::vector<std::string> type, value;
    public:
      PlotFeatureStatusProperty();
      virtual Property* clone() const {return new PlotFeatureStatusProperty(*this);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element); 
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
  };

}

#endif

