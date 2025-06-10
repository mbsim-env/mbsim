#include <config.h> 
#include "project.h"
#include "namespace.h"
#include <xercesc/dom/DOMDocument.hpp>
#include <mbxmlutilshelper/dom.h>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  void MBSimProject::initializeUsingXML(xercesc::DOMElement *e) {
    // check root element
    if(E(e)->getTagName()!=MBSim::MBSIMXML%"MBSimProject")
      throw runtime_error("The oot element of a MBSim file must be {"+MBSim::MBSIMXML.getNamespaceURI()+"}MBSimProject.");
    // check evaluator
    DOMElement *evaluator=E(e)->getFirstElementChildNamed(PV%"evaluator");
    if(!evaluator)
      Deprecated::message(nullptr, "No {"+PV.getNamespaceURI()+"}evaluator element defined.", e);
    if(evaluator && X()%E(evaluator)->getFirstTextChild()->getData()!="xmlflat")
      throw runtime_error("The evaluator must be 'xmlflat'.");

    // create object for DynamicSystemSolver and check correct type
    e=E(e)->getFirstElementChildNamed(MBSIM%"DynamicSystemSolver");
    dss.reset(ObjectFactory::createAndInit<DynamicSystemSolver>(e));

    // create object for Solver and check correct type
    solver.reset(ObjectFactory::createAndInit<Solver>(e->getNextElementSibling()));
  }

  MBSimProject readFlatXMLFile(const std::string &fileName) {
    shared_ptr<DOMParser> parser=DOMParser::create();
    shared_ptr<DOMDocument> doc=parser->parse(fileName, nullptr, false);
    DOMElement *e=doc->getDocumentElement();
    MBSimProject project;
    project.initializeUsingXML(e);
    return project;
  }

}
