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
class TiXmlElement;
class TiXmlNode;

class LocalFrameOfReferenceProperty : public Property {
  protected:
    Frame *frame;
    Element* element;
    std::string xmlName;
  public:
    LocalFrameOfReferenceProperty(Frame* frame_=0, Element* element_=0, const std::string &xmlName_="") : frame(frame_), element(element_), xmlName(xmlName_) {}
    Frame* getFrame() const {return frame;}
    void setFrame(Frame *frame_) {frame = frame_;}
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element); 
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
};

class ParentFrameOfReferenceProperty : public Property {
  protected:
    Frame *frame;
    Element* element;
    std::string xmlName;
    QString saved_frameOfReference;

  public:
    ParentFrameOfReferenceProperty(Frame* frame_=0, Element* element_=0, const std::string &xmlName_="") : frame(frame_), element(element_), xmlName(xmlName_) {}

    void initialize();
    Frame* getFrame() {return frame;}
    void setFrame(Frame *frame_) {frame = frame_;}
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void setSavedFrameOfReference(const QString &str) {saved_frameOfReference = str;}
    const QString& getSavedFrameOfReference() const {return saved_frameOfReference;}
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
};

class FrameOfReferenceProperty : public Property {
  protected:
    Frame *frame;
    Element* element;
    std::string xmlName;
    QString saved_frameOfReference;
  public:
    FrameOfReferenceProperty(Frame* frame_=0, Element* element_=0, const std::string &xmlName_="") : frame(frame_), element(element_), xmlName(xmlName_) {}
    Frame* getFrame() const {return frame;}
    void setFrame(Frame *frame_) {frame = frame_;}
    void initialize();
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element); 
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    void setSavedFrameOfReference(const QString &str) {saved_frameOfReference = str;}
    const QString& getSavedFrameOfReference() const {return saved_frameOfReference;}

};
class ContourOfReferenceProperty : public Property {
  protected:
    Contour *contour;
    Element* element;
    std::string xmlName;
    QString saved_contourOfReference;
  public:
    ContourOfReferenceProperty(Contour* contour_=0, Element* element_=0, const std::string &xmlName_="") : contour(contour_), element(element_), xmlName(xmlName_) {}
    Contour* getContour() const {return contour;}
    void setContour(Contour *contour_) {contour = contour_;}
    void initialize();
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element); 
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    void setSavedContourOfReference(const QString &str) {saved_contourOfReference = str;}
    const QString& getSavedContourOfReference() const {return saved_contourOfReference;}
};

class RigidBodyOfReferenceProperty : public Property {

  public:
    RigidBodyOfReferenceProperty(RigidBody *body_=0, Element *element_=0, const std::string &xmlName_="") : body(body_), element(element_), xmlName(xmlName_) {}
    RigidBody* getBody() const {return body;}
    void setBody(RigidBody* body_) {body = body_;}
    void initialize();
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element); 
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    void setSavedBodyOfReference(const QString &str) {saved_bodyOfReference = str;}
    const QString& getSavedBodyOfReference() const {return saved_bodyOfReference;}

  protected:
    RigidBody* body;
    Element* element;
    std::string xmlName;
    QString saved_bodyOfReference;
};

class FileProperty : public Property {

  public:
    FileProperty(const std::string &xmlName_) : xmlName(xmlName_) {}
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    QString fileName;
    std::string xmlName;
    QString absoluteFilePath;
};

class DependenciesProperty : public Property {

  public:
    DependenciesProperty(Element* element_, const std::string &xmlName_) : element(element_), xmlName(xmlName_) {}

    void initialize();
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
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
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
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
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    std::vector<ContourOfReferenceProperty*> contour;
    Element* element;
};

class SolverTolerancesProperty : public Property {

  public:
    SolverTolerancesProperty();

    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty projection, g, gd, gdd, la, La;
};

class SolverParametersProperty : public Property {

  public:
    SolverParametersProperty();

    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty tolerances;
};

#endif

