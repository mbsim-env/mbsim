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

class Element;
class Frame;
class Contour;
class Object;
class RigidBody;
class Signal;
class ExtraDynamic;

namespace MBXMLUtils {
  class TiXmlElement;
  class TiXmlNode;
}

class LocalFrameOfReferenceProperty : public Property {
  protected:
    std::string frame;
    Frame *framePtr;
    Element* element;
    std::string xmlName;
  public:
    LocalFrameOfReferenceProperty(const std::string &frame_="", Element* element_=0, const std::string &xmlName_="");
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
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
    std::string xmlName;
  public:
    ParentFrameOfReferenceProperty(const std::string &frame_="", Element* element_=0, const std::string &xmlName_="");
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
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
    std::string xmlName;
  public:
    FrameOfReferenceProperty(const std::string &frame="", Element* element=0, const std::string &xmlName=""); 
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
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
    std::string xmlName;
  public:
    ContourOfReferenceProperty(const std::string &contour_="", Element* element_=0, const std::string &xmlName_=""); 
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
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
    std::string xmlName;
  public:
    RigidBodyOfReferenceProperty(const std::string &body_="", Element *element_=0, const std::string &xmlName_="");
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
    void initialize();
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    void setBody(const std::string &str);
    std::string getBody() const;
};

class ObjectOfReferenceProperty : public Property {
  protected:
    std::string object;
    Object *objectPtr;
    Element* element;
    std::string xmlName;
  public:
    ObjectOfReferenceProperty(const std::string &object_="", Element *element_=0, const std::string &xmlName_=""); 
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
    void initialize();
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    void setObject(const std::string &str);
    std::string getObject() const;
};

class SignalOfReferenceProperty : public Property {
  protected:
    std::string signal;
    Signal *signalPtr;
    Element* element;
    std::string xmlName;
  public:
    SignalOfReferenceProperty(const std::string &signal_="", Element *element_=0, const std::string &xmlName_=""); 
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
    void initialize();
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    void setSignal(const std::string &str);
    std::string getSignal() const;
};

class ExtraDynamicOfReferenceProperty : public Property {
  protected:
    std::string ed;
    ExtraDynamic *edPtr;
    Element* element;
    std::string xmlName;
  public:
    ExtraDynamicOfReferenceProperty(const std::string &ed_="", Element *element_=0, const std::string &xmlName_=""); 
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
    void initialize();
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    void setExtraDynamic(const std::string &str);
    std::string getExtraDynamic() const;
};

class FileProperty : public Property {

  public:
    FileProperty(const std::string &xmlName_) : xmlName(xmlName_) {}
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    const std::string& getFile() const {return file;}
    void setFile(const std::string &str) {file=str;}

  protected:
    std::string file;
    std::string xmlName;
};

class TextProperty : public Property {

  public:
    TextProperty(const std::string &text_, const std::string &xmlName_, int quote_=0) : text(text_), xmlName(xmlName_), quote(quote_) {}
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    const std::string& getText() const {return text;}
    void setText(const std::string &text_) {text = text_;}

  protected:
    std::string text;
    std::string xmlName;
    int quote;
};

class DependenciesProperty : public Property {

  public:
    DependenciesProperty(Element* element_, const std::string &xmlName_) : element(element_), xmlName(xmlName_) {}

    void initialize();
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    Element* element;
    std::string xmlName;
    std::vector<RigidBodyOfReferenceProperty*> refBody;

    void addDependency();
    void updateGeneralizedCoordinatesOfBodies();
};

class ConnectFramesProperty : public Property {

  public:
    ConnectFramesProperty(int n, Element* element, const std::string &xmlName_=MBSIMNS"connect");

    void initialize();
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    std::vector<FrameOfReferenceProperty*> frame;
    Element* element;
    std::string xmlName;
};

class ConnectContoursProperty : public Property {

  public:
    ConnectContoursProperty(int n, Element* element, const std::string &xmlName_=MBSIMNS"connect");

    void initialize();
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    std::vector<ContourOfReferenceProperty*> contour;
    Element* element;
    std::string xmlName;
};

class SolverTolerancesProperty : public Property {

  public:
    SolverTolerancesProperty();

    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty projection, g, gd, gdd, la, La;
};

class SolverParametersProperty : public Property {

  public:
    SolverParametersProperty();

    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty tolerances;
};

class GearDependencyProperty : public Property {
  public:
    GearDependencyProperty(Element* element);
    void initialize() {refBody.initialize();}
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    Element* element;
    RigidBodyOfReferenceProperty refBody;
    ExtProperty ratio;
};

class GearDependenciesProperty : public Property {

  public:
    GearDependenciesProperty(Element* element_, const std::string &xmlName_) : element(element_), xmlName(xmlName_) {}

    void initialize();
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    Element* element;
    std::string xmlName;
    std::vector<GearDependencyProperty*> refBody;

    void addDependency();
    void updateGeneralizedCoordinatesOfBodies();
};

class EmbedProperty : public Property {

  public:
    EmbedProperty(Element *element);
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    bool hasFile() const {return (href.isActive() && static_cast<const FileProperty*>(href.getProperty())->getFile()!="");}
    std::string getFile() const {return static_cast<const FileProperty*>(href.getProperty())->getFile();}
    bool hasCounter() const {return counterName.isActive();}
    std::string getCounterName() const {return static_cast<const TextProperty*>(counterName.getProperty())->getText();}
    bool hasParameterFile() const {return (parameterList.isActive() && static_cast<const FileProperty*>(parameterList.getProperty())->getFile()!="");}
    std::string getParameterFile() const {return static_cast<const FileProperty*>(parameterList.getProperty())->getFile();}

  protected:
    ExtProperty href, count, counterName, parameterList;

};

class SignalReferenceProperty : public Property {
  public:
    SignalReferenceProperty(Element* element);
    void initialize() {refSignal.initialize();}
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    Element* element;
    SignalOfReferenceProperty refSignal;
    ExtProperty factor;
};

class SignalReferencesProperty : public Property {

  public:
    SignalReferencesProperty(Element* element_, const std::string &xmlName_) : element(element_), xmlName(xmlName_) {}

    void initialize();
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    Element* element;
    std::string xmlName;
    std::vector<SignalReferenceProperty*> refSignal;

    void addReference();
};


#endif

