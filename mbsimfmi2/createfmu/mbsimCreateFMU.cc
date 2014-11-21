#include <mbxmlutils/preprocess.h>
#include "../config.h" // preprocess.h/octeval.h will undefine macro, hence include config.h after this file
#include <iostream>
#include <boost/filesystem.hpp>
#include <mbsimxml/mbsimflatxml.h>
#include <mbxmlutilshelper/dom.h>
#include <mbxmlutilshelper/getinstallpath.h>
#include <mbsim/objectfactory.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/integrators/integrator.h>
#include <mbsimxml/mbsimxml.h>
#include <boost/lexical_cast.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/filesystem/fstream.hpp"

#include "zip.h"

#include <../general/fmi_variables_impl.h>

using namespace std;
using namespace boost::filesystem;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace MBSimFMI;

namespace {
  // some platform dependent file suffixes, directory names, ...
#ifdef _WIN32
  string SHEXT(".dll");
  #ifdef _WIN64
  string FMIOS("win64");
  #else
  string FMIOS("win32");
  #endif
  string USERENVVAR("USERNAME");
#else
  string SHEXT(".so");
  #ifdef __x86_64__
  string FMIOS("linux64");
  #else
  string FMIOS("linux32");
  #endif
  string USERENVVAR("USER");
#endif
}

int main(int argc, char *argv[]) {
  // help
  if(argc!=2) {
    cout<<"Usage: "<<argv[0]<<" <MBSim Project XML File>"<<endl;
    cout<<endl;
    cout<<"Create mbsim.fmu in the current directory."<<endl;
    return 0;
  }

  // create octave
  vector<path> dependencies;
  OctEval octEval(&dependencies);
  // create parser
  boost::shared_ptr<DOMParser> parser=DOMParser::create(true);
  generateMBSimXMLSchema(".mbsimxml.xsd", getInstallPath()/"share"/"mbxmlutils"/"schema");
  parser->loadGrammar(".mbsimxml.xsd");
  // create FMU zip file
  CreateZip fmuFile("mbsim.fmu");

  // load MBSim project XML document
  path mbsimxmlfile=argv[1];
  boost::shared_ptr<xercesc::DOMDocument> modelDoc=parser->parse(mbsimxmlfile);
  DOMElement *modelEle=modelDoc->getDocumentElement();

  // preprocess XML file
  Preprocess::preprocess(parser, octEval, dependencies, modelEle);

  // save preprocessed model to FMU
  string ppModelStr;
  DOMParser::serializeToString(modelEle, ppModelStr, false);
  fmuFile.add(path("resources")/"Model.mbsimprj.flat.xml", ppModelStr);

  // load all plugins
  MBSimXML::loadPlugins();

  // create object for DynamicSystemSolver
  boost::shared_ptr<DynamicSystemSolver> dss;
  dss.reset(ObjectFactory::createAndInit<DynamicSystemSolver>(modelEle->getFirstElementChild()));

  // build list of value references
  vector<boost::shared_ptr<Variable> > var;
  PredefinedVariables predefinedVar;
  createAllVariables(dss.get(), var, predefinedVar);

  // disable all plotting (we wont any output here)
  dss->setPlotFeatureRecursive(Element::plotRecursive, Element::disabled);
  // initialize dss
  dss->initialize();

  // create object for Integrator (just to get the start/end time for DefaultExperiment)
  boost::shared_ptr<MBSimIntegrator::Integrator> integrator;
  integrator.reset(ObjectFactory::createAndInit<MBSimIntegrator::Integrator>(modelEle->getFirstElementChild()->getNextElementSibling()));

  // create DOM of modelDescription.xml
  boost::shared_ptr<DOMDocument> modelDescDoc(parser->createDocument());
  // root element fmiModelDescription and its attributes
  DOMElement *modelDesc=D(modelDescDoc)->createElement("fmiModelDescription");
  modelDescDoc->appendChild(modelDesc);
  E(modelDesc)->setAttribute("author", getenv(USERENVVAR.c_str()));
  E(modelDesc)->setAttribute("description", "FMI export of a MBSim-XML model");
  E(modelDesc)->setAttribute("fmiVersion", "1.0");
  E(modelDesc)->setAttribute("generationDateAndTime",
    boost::posix_time::to_iso_extended_string(boost::posix_time::second_clock::local_time())+"Z");
  E(modelDesc)->setAttribute("generationTool", string("MBSimFMI Version ")+VERSION);
  E(modelDesc)->setAttribute("version", "1.0");
  E(modelDesc)->setAttribute("guid", "mbsimfmi_guid");
  E(modelDesc)->setAttribute("modelIdentifier", "mbsim");
  path desc=mbsimxmlfile.filename();
  desc.replace_extension();
  desc.replace_extension();
  E(modelDesc)->setAttribute("modelName", desc.string());
  E(modelDesc)->setAttribute("numberOfContinuousStates", boost::lexical_cast<string>(dss->getzSize()));
  E(modelDesc)->setAttribute("numberOfEventIndicators", boost::lexical_cast<string>(dss->getsvSize()));
  E(modelDesc)->setAttribute("variableNamingConvention", "structured");
    // DefaultExperiment element and its attributes
    DOMElement *defaultExp=D(modelDescDoc)->createElement("DefaultExperiment");
    modelDesc->appendChild(defaultExp);
    E(defaultExp)->setAttribute("startTime", boost::lexical_cast<string>(integrator->getStartTime()));
    E(defaultExp)->setAttribute("stopTime", boost::lexical_cast<string>(integrator->getEndTime()));
    E(defaultExp)->setAttribute("tolerance", boost::lexical_cast<string>(1e-5));
    // ModelVariables element
    DOMElement *modelVars=D(modelDescDoc)->createElement("ModelVariables");
    modelDesc->appendChild(modelVars);
      // loop over all FMI variables
      for(size_t vr=0; vr<var.size(); ++vr) {
        // create ScalarVariable element
        DOMElement *scalarVar=D(modelDescDoc)->createElement("ScalarVariable");
        modelVars->appendChild(scalarVar);
          // create datatype element
          string datatypeEleName;
          switch(var[vr]->getDatatypeChar()) {
            case 'r': datatypeEleName="Real";    break;
            case 'i': datatypeEleName="Integer"; break;
            case 'b': datatypeEleName="Boolean"; break;
            case 's': datatypeEleName="String";  break;
          }
          DOMElement *varType=D(modelDescDoc)->createElement(datatypeEleName);
          scalarVar->appendChild(varType);
        // attributes on ScalarVariable element
        E(scalarVar)->setAttribute("name", var[vr]->getName());
        E(scalarVar)->setAttribute("description", var[vr]->getDescription());
        E(scalarVar)->setAttribute("valueReference", boost::lexical_cast<string>(vr));
        switch(var[vr]->getType()) {
          case Parameter:
            E(scalarVar)->setAttribute("causality", "internal");
            E(scalarVar)->setAttribute("variability", "parameter");
            E(varType)->setAttribute("start", var[vr]->getValueAsString());
            break;
          case Input:
            E(scalarVar)->setAttribute("causality", "input");
            E(scalarVar)->setAttribute("variability", "continuous");
            E(varType)->setAttribute("start", var[vr]->getValueAsString());
            break;
          case Output:
            E(scalarVar)->setAttribute("causality", "output");
            E(scalarVar)->setAttribute("variability", "continuous");
            break;
        }
      }
  // add modelDescription.xml file to FMU
  string modelDescriptionStr;
  DOMParser::serializeToString(modelDescDoc.get(), modelDescriptionStr);
  fmuFile.add("modelDescription.xml", modelDescriptionStr);

  // add binaries to FMU
  fmuFile.add(path("binaries")/FMIOS/("mbsim"+SHEXT), getInstallPath()/"lib"/("mbsimxml_fmi"+SHEXT));

  fmuFile.close();

  return 0;
}
