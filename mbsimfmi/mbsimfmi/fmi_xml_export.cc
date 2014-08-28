/* Copyright (C) 2004-2014 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: thorsten.schindler@mytum.de
 */

#include "fmi_xml_export.h"
#include "model_instance.h"
#include "fmi_utils.h"
#include <mbsim/object.h>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMImplementationRegistry.hpp>

#include <string>
#include <algorithm>
#include <iostream>
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;
using std::string;

namespace fmi {

const string FmiXmlExport::rootAttr[12]={
    "fmiVersion",
    "modelName",
    "modelIdentifier",
    "guid",
    "description",
    "author",
    "version",
    "generationTool",
    "generationDateAndTime",
    "variableNamingConvention",
    "numberOfContinuousStates",
    "numberOfEventIndicators"
};

string FmiXmlExport::rootAttrDefaultValue[12]={
    "1.0",
    "MBSim - example",
    "MBSim - name",
    "GUID - default",
    "MBSim - default description",
    "X",
    "1.0",
    "MBSim",
    "2013-02-05T00:00:00Z",
    "structured",
    "0",
    "0"
};

const string FmiXmlExport::defaultExpAttr[3]={
    "startTime",
    "stopTime",
    "tolerance"
};
string FmiXmlExport::defaultExpAttrDefaultValue[3]={
    "0.0",
    "1.0",
    "0.0001"
};

const string FmiXmlExport::filename="modelDescription.xml";

FmiXmlExport::FmiXmlExport():
  dss(0) {
  parser=DOMParser::create(false);
  doc= parser->createDocument();
}

FmiXmlExport::FmiXmlExport(DynamicSystemSolver *dss):
  dss(dss) {
  parser=DOMParser::create(false);
  doc= parser->createDocument();
}

FmiXmlExport::~FmiXmlExport() {
}

void FmiXmlExport::buildDefaultFile(){

}

void FmiXmlExport::buildDssFile(){

  FmuParameters params;
  params.setNStates(dss);
  params.setNEvents(dss);

  string rootAttrValue[12]={
      "1.0",
      "MBSim_ME",
      "mbsim",
      params.guid(),
      "MBSim - default description",
      "TUM-AM",
      "0.0",
      "MBSim",
      getCurrentDateTime(),
      "structured",
      NumberToString(params.get(nstates)).c_str(),
      NumberToString(params.get(ne)).c_str()
  };

 
  DOMElement *root = addRoot(doc.get(),rootAttrValue);
  DOMElement *e; 
  
  XMLString::transcode("UnitDefinitions", tempStr, 99);
  e = doc->createElement(tempStr);
  root->appendChild(e);

  XMLString::transcode("TypeDefinitions", tempStr, 99);
  e = doc->createElement(tempStr);
  root->appendChild(e);
  
  e = addDefaultExperiment(root);

  XMLString::transcode("ModelVariables", tempStr, 99);
  e = doc->createElement(tempStr);
  root->appendChild(e);

  addParameters(e);
  addIO(e);
//  addStates(e);
}

DOMElement* FmiXmlExport::addRoot(
    DOMDocument* doc,
    string rootAttrValue[]) {  
    XMLString::transcode("fmiModelDescription", tempStr, 99);
  DOMElement* root=doc->createElement(tempStr);
//   DOMElement* root=doc->getDocumentElement();
  for(int i=0;i<12;i++) {
    XMLString::transcode(rootAttr[i].c_str(),tempStr,99);
    XMLString::transcode(rootAttrValue[i].c_str(),tempStr2,99);
    root->setAttribute( tempStr, tempStr2 );
  }
  doc->appendChild( root );
  return root;
}

DOMElement* FmiXmlExport::addRoot(
    DOMDocument* doc,
    string fmiVersion_,
    string modelName,
    string modelIdentifier,
    string guid,
    string description,
    string author,
    string version,
    string generationTool,
    string generationDateAndTime,
    string variableNamingConvention,
    string numberOfContinuousStates,
    string numberOfEventIndicators) {
  XMLString::transcode("fmiModelDescription", tempStr, 99);
  DOMElement* root=doc->createElement(tempStr);
  if(fmiVersion_ != "") setAttribute(root,"fmiVersion", fmiVersion_.c_str());
  if(modelName  != "")  setAttribute(root,"modelName",modelName.c_str());
  if(modelIdentifier!="") setAttribute(root,"modelIdentifier",modelIdentifier.c_str());
  if(guid!="") setAttribute(root,"guid",guid.c_str());
  if(description!="") setAttribute(root,"description",description.c_str());
  if(author!="") setAttribute(root,"author",author.c_str());
  if(version!="") setAttribute(root,"version",version.c_str());
  if(generationTool!="") setAttribute(root,"generationTool",generationTool.c_str());
  if(generationDateAndTime!="") setAttribute(root,"generationDateAndTime",generationDateAndTime.c_str());
  if(numberOfContinuousStates!="") setAttribute(root,"numberOfContinuousStates",numberOfContinuousStates.c_str());
  if(numberOfEventIndicators!="") setAttribute(root,"numberOfEventIndicators",numberOfEventIndicators.c_str());
  doc->appendChild( root );
  return root;
}

DOMElement* FmiXmlExport::addDefaultExperiment(
    DOMElement* e,
    string startTime,
    string stopTime,
    string tolerance) {
  XMLString::transcode("DefaultExperiment", tempStr, 99);
  DOMElement* v= doc->createElement(tempStr);
  setAttribute(v,"startTime",startTime.c_str());
  setAttribute(v,"stopTime",stopTime.c_str());
  setAttribute(v,"tolerance",tolerance.c_str());
  e->appendChild(v);
  return v;
}


DOMElement* FmiXmlExport::addSV(
    DOMElement* e,
    string name,
    string valueReference,
    string description,
    string variability,
    string causality,
    string alias) {
  XMLString::transcode("ScalarVariable", tempStr, 99);
  DOMElement* sv= doc->createElement(tempStr);
  setAttribute(sv,"name",name.c_str());
  setAttribute(sv,"valueReference",valueReference.c_str());
  if(description != "") setAttribute(sv,"description",description.c_str());
  if(variability != "") setAttribute(sv,"variability",variability.c_str());
  if(causality != "")   setAttribute(sv,"causality",causality.c_str());
  if(alias != "")       setAttribute(sv,"alias",alias.c_str());
  e->appendChild(sv);
  return sv;
}

DOMElement* FmiXmlExport::addV(
    DOMElement* e,
    string type,
    string declaredType,
    string start,
    string fixed) {
  XMLString::transcode(type.c_str(), tempStr, 99);
  DOMElement* v= doc->createElement(tempStr);
  if(declaredType != "") setAttribute(v,"declaredType",declaredType.c_str());
  if(start != "") setAttribute(v,"start",start.c_str());
  if(fixed != "") setAttribute(v,"fixed",fixed.c_str());
  e->appendChild(v);
  return v;
}

void FmiXmlExport::saveFile(string filename_){
  if(filename_=="") {DOMParser::serialize(doc.get(),"./"+filename);return;}
  DOMParser::serialize(doc.get(),"./"+filename_);
}

void FmiXmlExport::addIO(DOMElement* e) {
  DOMElement *sv;
  int valRef=0;
  uint nIO;
  std::vector<MBSim::Link*> links;
  dss->buildListOfLinks(links);
  for(uint b=0;b<links.size();b++)
    std::cout<<links[b]->getType()<<std::endl;
  /**************************************************/
  /****** ExternGeneralizedIO ***********************/
  /**************************************************/
  {
    std::vector<MBSim::ExternGeneralizedIO*> vectorIO;
    for(uint i=0;i<links.size();i++) {
      if(links[i]->getType()=="ExternGeneralizedIO")
        vectorIO.push_back((MBSim::ExternGeneralizedIO*)links[i]);
    }
    nIO=vectorIO.size();
    for(uint i=0;i<nIO;i++) {
      string name = string(vectorIO[i]->getName()+".f."+NumberToString(i));
      std::replace(name.begin(), name.end(), '/', '.');
      string valueRef = NumberToString(valRef);
      valRef++;
      sv=addSV(e,name,valueRef,"","continuous","input");
      string startValue=NumberToString(vectorIO[i]->getg()(i));
      addV(sv,"Real");
    }
    for(uint j=0;j<2;j++) {
      for(uint i=0;i<nIO;i++) {
        string name = string(vectorIO[i]->getName()+"."+(j?"u":"q")+"."+NumberToString(i));
        std::replace(name.begin(), name.end(), '/', '.');
        string valueRef = NumberToString(valRef);
        valRef++;
        sv=addSV(e,name,valueRef,"","continuous","output");
        string startValue=NumberToString(j?vectorIO[i]->getGeneralizedVelocity():vectorIO[i]->getGeneralizedPosition());
        addV(sv,"Real");
      }
    }
  }
  /**************************************************/
  /****** ExternSignalSource ************************/
  /**************************************************/
  {
    std::vector<MBSimControl::ExternSignalSource*> vectorIO;
    for(uint i=0;i<links.size();i++) {
      if(links[i]->getType()=="ExternSignalSource")
        vectorIO.push_back((MBSimControl::ExternSignalSource*)links[i]);
    }
    nIO=vectorIO.size();
    for(uint i=0;i<nIO;i++) {
      fmatvec::Vec sig=vectorIO[i]->getSignal();
      for(int j=0;j<sig.size();j++) {
        string name = string(vectorIO[i]->getType()+".f."+NumberToString(j));
        std::replace(name.begin(), name.end(), '/', '.');
        string valueRef = NumberToString(valRef);
        valRef++;
        sv=addSV(e,name,valueRef,"","continuous","input");
        string startValue=NumberToString(sig(j));
        addV(sv,"Real");
      }
    }
  }
  /**************************************************/
  /****** ExternSignalSink **************************/
  /**************************************************/
  {
    std::vector<MBSimControl::ExternSignalSink*> vectorIO;
    for(uint i=0;i<links.size();i++) {
      if(links[i]->getType()=="ExternSignalSink")
        vectorIO.push_back((MBSimControl::ExternSignalSink*)links[i]);
    }
    nIO=vectorIO.size();
    for(uint i=0;i<nIO;i++) {
      fmatvec::Vec sig=vectorIO[i]->getSignal();
      for(int j=0;j<sig.size();j++) {
        string name = string(vectorIO[i]->getType()+".f."+NumberToString(j));
        std::replace(name.begin(), name.end(), '/', '.');
        string valueRef = NumberToString(valRef);
        valRef++;
        sv=addSV(e,name,valueRef,"","continuous","output");
        string startValue=NumberToString(sig(j));
        addV(sv,"Real");
      }
    }
  }
}
void FmiXmlExport::addStates(DOMElement* e) {
  DOMElement *sv;

  const std::vector<MBSim::Object*> objList = dss->getObjects();
  for (unsigned int i = 0; i < objList.size(); i++) {
    std::stringstream str;
    str << objList[i]->getPath('.');
    objList[i]->setName(str.str());
  }
  int qSize=0,uSize=0;
  for(uint i = 0; i < objList.size();i++){
    for(int j = 0; j < objList[i]->getqSize(); j++){
      string name = string(objList[i]->getName()+".q."+NumberToString(j));
      std::replace(name.begin(), name.end(), '/', '.');
      string valueRef = NumberToString(qSize + j);
      sv=addSV(e,name,valueRef);
      addV(sv,"Real");

    }
    qSize+=objList[i]->getqSize();
  }
  for(uint i = 0; i < objList.size();i++){
    for(int j = 0; j < objList[i]->getuSize(); j++){
      string name = string(objList[i]->getName()+".u."+NumberToString(j));
      std::replace(name.begin(), name.end(), '/', '.');
      string valueRef=NumberToString(qSize + uSize + j);
      string startValue=NumberToString(objList[i]->getu()(j));
      sv=addSV(e,name,valueRef);
      addV(sv,"Real");
    }
    uSize+=objList[i]->getuSize();
  }
}

void FmiXmlExport::addParameters(xercesc::DOMElement* e) {
  XMLString::transcode("Output directory",tempStr,99);
  XMLString::transcode("",tempStr2,99);
  DOMElement *sv;
  sv=addSV(e,"OutputDir", "0","Output directory for plots","parameter","input");
  addV(sv,"String","","./" );
}

void FmiXmlExport::setAttribute(xercesc::DOMElement* e, const char* name, const char* value) {
  XMLString::transcode(name,tempStr,99);
  XMLString::transcode(value,tempStr2,99);
  e->setAttribute(tempStr, tempStr2);
}

} /* namespace fmi */
