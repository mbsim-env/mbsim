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
class RigidBody;
namespace MBXMLUtils {
  class TiXmlElement;
  class TiXmlNode;
}

class LocalFrameOfReferenceProperty : public Property {
  protected:
    Frame *frame;
    Element* element;
    std::string xmlName;
  public:
    LocalFrameOfReferenceProperty(Frame* frame_=0, Element* element_=0, const std::string &xmlName_="") : frame(frame_), element(element_), xmlName(xmlName_) {}
    Frame* getFrame() const {return frame;}
    void setFrame(Frame *frame_) {frame = frame_;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
};

class ParentFrameOfReferenceProperty : public Property {
  protected:
    Frame *frame;
    Element* element;
    std::string xmlName;
    std::string saved_frameOfReference;

  public:
    ParentFrameOfReferenceProperty(Frame* frame_=0, Element* element_=0, const std::string &xmlName_="") : frame(frame_), element(element_), xmlName(xmlName_) {}

    void initialize();
    Frame* getFrame() {return frame;}
    void setFrame(Frame *frame_) {frame = frame_;}
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void setSavedFrameOfReference(const std::string &str) {saved_frameOfReference = str;}
    const std::string& getSavedFrameOfReference() const {return saved_frameOfReference;}
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
};

class FrameOfReferenceProperty : public Property {
  protected:
    Frame *frame;
    Element* element;
    std::string xmlName;
    std::string saved_frameOfReference;
  public:
    FrameOfReferenceProperty(Frame* frame_=0, Element* element_=0, const std::string &xmlName_="") : frame(frame_), element(element_), xmlName(xmlName_) {}
    Frame* getFrame() const {return frame;}
    void setFrame(Frame *frame_) {frame = frame_;}
    void initialize();
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    void setSavedFrameOfReference(const std::string &str) {saved_frameOfReference = str;}
    const std::string& getSavedFrameOfReference() const {return saved_frameOfReference;}

};
class ContourOfReferenceProperty : public Property {
  protected:
    Contour *contour;
    Element* element;
    std::string xmlName;
    std::string saved_contourOfReference;
  public:
    ContourOfReferenceProperty(Contour* contour_=0, Element* element_=0, const std::string &xmlName_="") : contour(contour_), element(element_), xmlName(xmlName_) {}
    Contour* getContour() const {return contour;}
    void setContour(Contour *contour_) {contour = contour_;}
    void initialize();
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    void setSavedContourOfReference(const std::string &str) {saved_contourOfReference = str;}
    const std::string& getSavedContourOfReference() const {return saved_contourOfReference;}
};

class RigidBodyOfReferenceProperty : public Property {

  public:
    RigidBodyOfReferenceProperty(RigidBody *body_=0, Element *element_=0, const std::string &xmlName_="") : body(body_), element(element_), xmlName(xmlName_) {}
    RigidBody* getBody() const {return body;}
    void setBody(RigidBody* body_) {body = body_;}
    void initialize();
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    void setSavedBodyOfReference(const std::string &str) {saved_bodyOfReference = str;}
    const std::string& getSavedBodyOfReference() const {return saved_bodyOfReference;}

  protected:
    RigidBody* body;
    Element* element;
    std::string xmlName;
    std::string saved_bodyOfReference;
};

class FileProperty : public Property {

  public:
    FileProperty(const std::string &xmlName_) : xmlName(xmlName_) {}
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    const std::string& getFileName() const {return fileName;}
    void setFileName(const std::string &str) {fileName=str;}
    std::string getAbsoluteFilePath() const;
    void setAbsoluteFilePath(const std::string &str);

  protected:
    std::string fileName;
    std::string xmlName;
    std::string absoluteFilePath;
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
    ConnectFramesProperty(int n, Element* element);

    void initialize();
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    std::vector<FrameOfReferenceProperty*> frame;
    Element* element;
};

class ConnectContoursProperty : public Property {

  public:
    ConnectContoursProperty(int n, Element* element);

    void initialize();
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    std::vector<ContourOfReferenceProperty*> contour;
    Element* element;
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
    std::string getFile() const {return static_cast<const FileProperty*>(href.getProperty())->getFileName();}
    bool hasCounter() const {return counterName.isActive();}
    std::string getCounterName() const {return static_cast<const TextProperty*>(counterName.getProperty())->getText();}
    bool hasParameterFile() const {return (parameterList.isActive() && static_cast<const FileProperty*>(parameterList.getProperty())->getFileName()!="");}
    std::string getParameterFile() const {return static_cast<const FileProperty*>(parameterList.getProperty())->getFileName();}

  protected:
    ExtProperty href, count, counterName, parameterList;

};

#endif

