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

#ifndef FMIXMLEXPORT_H_
#define FMIXMLEXPORT_H_

#include <mbsim/dynamic_system_solver.h>
#include <mbxmlutilshelper/dom.h>
#include "fmi_utils.h"

namespace fmi {

const MBXMLUtils::NamespaceURI MBSIMINTERFACE("http://mbsim.berlios.de/MBSimInterface");

/**
 * \author Fabien Pean
 */
class FmiXmlExport {
public:

  FmiXmlExport();
  FmiXmlExport(MBSim::DynamicSystemSolver *dss);
  virtual ~FmiXmlExport();

  void buildDefaultFile();
  void performTest();
  /**
   * \brief Creates the file from the DynamicSystemSolver provided
   */
  void buildDssFile();
  /**
   * \brief Saves the xml file under filename
   * \param filename filename set by user (or use the predefined in the class)
   */
  void saveFile(std::string filename_="");

  //DynamicSystem to look up to write xml from it
  MBSim::DynamicSystemSolver *dss;
  static const std::string filename;
  //required for the xercesc
  boost::shared_ptr<MBXMLUtils::DOMParser>   parser;
  boost::shared_ptr<xercesc::DOMDocument> doc;
  XMLCh tempStr[100];
  XMLCh tempStr2[100];

  static const std::string rootAttr[12];
  static std::string rootAttrDefaultValue[12];
  static const std::string defaultExpAttr[3];
  static std::string defaultExpAttrDefaultValue[3];

  /**
   * \brief Adds the root node in xml doc
   * \param doc XmlDocument variable
   * \param string the different parameters of the node
   * \return element the xml-element containing the modelDescription node
   */
  xercesc::DOMElement* addRoot(  xercesc::DOMDocument* doc,
                                      std::string rootAttrValue[]=rootAttrDefaultValue);
  xercesc::DOMElement* addRoot(  xercesc::DOMDocument* doc,
                                      std::string fmiVersion_="1.0",
                                      std::string modelName="MBSim_ME",
                                      std::string modelIdentifier="mbsim",
                                      std::string guid="GUID",
                                      std::string description="",
                                      std::string author="TUM-AM",
                                      std::string version="0.0",
                                      std::string generationTool="MBSimFMI",
                                      std::string generationDateAndTime=getCurrentDateTime(),
                                      std::string variableNamingConvention="flat",
                                      std::string numberOfContinuousStates="0",
                                      std::string numberOfEventIndicators="0");
  /**
   * \brief Adds the DefaultExperiment node to the root
   * \param root the node node
   * \param string the different parameters of the node
   * \return element the xml-element containing the DefaultExperiment node
   */
  xercesc::DOMElement* addDefaultExperiment( xercesc::DOMElement* e,
                                                  std::string startTime="0.0",
                                                  std::string stopTime="2.0",
                                                  std::string tolerance="1e-4");
  /**
   * \brief Adds a ScalarVariable to the node e
   * \param element the element which the SV will be linked to
   * \param string the different parameters of the node
   * \return element the xml-element containing the ScalarVariable node
   */
  xercesc::DOMElement* addSV(  xercesc::DOMElement* e,
                                    std::string name,
                                    std::string valueReference,
                                    std::string description="",
                                    std::string variability="continuous",
                                    std::string causality="internal",
                                    std::string alias="noAlias" );
  /**
   * \brief Adds a variable to the ScalarVariable e
   * \param element the scalar variable which the variable will be linked to
   * \param string the different parameters of the node
   * \return element the xml-element containing the Variable node
   */
  xercesc::DOMElement* addV( xercesc::DOMElement* e,
                                  std::string type,
                                  std::string declaredType="",
                                  std::string start="",
                                  std::string fixed="");
  /**
   * \brief Adds Input/Output children to element e
   * \param element-root to which adding the input/output of the described system
   */
  void addIO(xercesc::DOMElement* e);
  /**
   * \brief Adds state variable  to element e
   * \param element-root to which adding the states of the described system
   */
  void addStates(xercesc::DOMElement* e);
  /**
     * \brief Adds standard parameter (output directory, etc)  to element e
     * \param element-root to which adding the std parameters of the described system
     */
  void addParameters(xercesc::DOMElement* e);
  /**
   * \brief Adds attribute to element e
   * \param element element to which adding the attribute
   * \param name name of the attribute
   * \param value value of the attribute
   */
  void setAttribute(xercesc::DOMElement* e, const char* name, const char* value);

};

} /* namespace fmi */

#endif /* FMIXMLEXPORT_H_ */
