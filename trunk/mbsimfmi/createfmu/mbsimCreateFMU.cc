#include <mbxmlutils/preprocess.h>
#include "../config.h" // preprocess.h/octeval.h will undefine macro, hence include config.h after this file
#include <iostream>
#include <boost/filesystem.hpp>
#include <mbsimxml/mbsimflatxml.h>
#include <mbxmlutilshelper/dom.h>
#include <mbxmlutilshelper/getinstallpath.h>
#include <mbxmlutilshelper/shared_library.h>
#include <mbsim/objectfactory.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/integrators/integrator.h>
#include <mbsimxml/mbsimxml.h>
#include <boost/lexical_cast.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/filesystem/fstream.hpp"
#include <boost/algorithm/string.hpp>

#include "zip.h"

#include <../general/fmi_variables.h>
#include <../general/mbsimsrc_fmi.h>

using namespace std;
using namespace boost::filesystem;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace MBSimFMI;

namespace {
  // some platform dependent file suffixes, directory names, ...
#ifdef _WIN32
  string USERENVVAR("USERNAME");
#else
  string USERENVVAR("USER");
#endif
}

int main(int argc, char *argv[]) {
  try {
    // help
    if(argc!=2) {
      cout<<"Usage: "<<argv[0]<<" <MBSim Project XML File>|<MBSim FMI shared library>"<<endl;
      cout<<endl;
      cout<<"Create mbsim.fmu in the current directory."<<endl;
      return 0;
    }

    // check input file
    path inputFilename=argv[1];
    bool xmlFile=false;
    if(boost::iequals(extension(inputFilename), ".xml")) {
      xmlFile=true;
    }

    // create FMU zip file
    CreateZip fmuFile("mbsim.fmu");

    // load all plugins
    MBSimXML::loadPlugins();

    // create parser (a validating parser for XML input and a none validating parser for shared library input)
    boost::shared_ptr<DOMParser> parser=DOMParser::create(xmlFile);

    // note the order of these variable definitions is importend for proper deallocation
    // (dss and integrator must be deallocated before the shLib is unlaoded)
    boost::shared_ptr<SharedLibrary> shLib;
    boost::shared_ptr<DynamicSystemSolver> dss;
    boost::shared_ptr<MBSimIntegrator::Integrator> integrator;

    // Create dss from XML file
    if(xmlFile) {
      // create octave
      vector<path> dependencies;
      OctEval octEval(&dependencies);

      // init the validating parser with the mbsimxml schema file
      generateMBSimXMLSchema(".mbsimxml.xsd", getInstallPath()/"share"/"mbxmlutils"/"schema");
      parser->loadGrammar(".mbsimxml.xsd");

      // load MBSim project XML document
      boost::shared_ptr<xercesc::DOMDocument> modelDoc=parser->parse(inputFilename);
      DOMElement *modelEle=modelDoc->getDocumentElement();

      // preprocess XML file
      Preprocess::preprocess(parser, octEval, dependencies, modelEle);

      // save preprocessed model to FMU
      string ppModelStr;
      DOMParser::serializeToString(modelEle, ppModelStr, false);
      fmuFile.add(path("resources")/"Model.mbsimprj.flat.xml", ppModelStr);

      // create object for DynamicSystemSolver
      dss.reset(ObjectFactory::createAndInit<DynamicSystemSolver>(modelEle->getFirstElementChild()));

      // create object for Integrator (just to get the start/end time for DefaultExperiment)
      integrator.reset(ObjectFactory::createAndInit<MBSimIntegrator::Integrator>(
        modelEle->getFirstElementChild()->getNextElementSibling()));
    }
    // Create dss from shared library
    else {
      // load the shared library and call mbsimSrcFMI function to get the dss
      shLib=boost::make_shared<SharedLibrary>(absolute(inputFilename));
      DynamicSystemSolver *dssPtr;
      reinterpret_cast<mbsimSrcFMIPtr>(shLib->getAddress("mbsimSrcFMI"))(dssPtr);
      dss.reset(dssPtr);

      // we do not have a integrator for src models (but this is only used for convinence settings)
      integrator.reset();
    }

    // build list of value references
    vector<boost::shared_ptr<Variable> > var;
    PredefinedVariables predefinedVar;
    createAllVariables(dss.get(), var, predefinedVar);

    // disable all plotting (we wont any output here)
    dss->setPlotFeatureRecursive(Element::plotRecursive, Element::disabled);
    // initialize dss
    dss->initialize();

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
    path desc=inputFilename.filename();
    desc.replace_extension();
    desc.replace_extension();
    E(modelDesc)->setAttribute("modelName", desc.string());
    E(modelDesc)->setAttribute("numberOfContinuousStates", boost::lexical_cast<string>(dss->getzSize()));
    E(modelDesc)->setAttribute("numberOfEventIndicators", boost::lexical_cast<string>(dss->getsvSize()));
    E(modelDesc)->setAttribute("variableNamingConvention", "structured");

      // Type definition
      // get a unique list of all enumeration types
      set<Variable::EnumList> enumType;
      for(size_t vr=0; vr<var.size(); ++vr)
        if(var[vr]->getEnumerationList())
          enumType.insert(var[vr]->getEnumerationList());
      // write all enumeration type to xml file
      DOMElement *typeDef=D(modelDescDoc)->createElement("TypeDefinitions");
      modelDesc->appendChild(typeDef);
        for(set<Variable::EnumList>::iterator it=enumType.begin(); it!=enumType.end(); ++it) {
          DOMElement *type=D(modelDescDoc)->createElement("Type");
          typeDef->appendChild(type);
          E(type)->setAttribute("name", "EnumType_"+boost::lexical_cast<string>(*it));
            DOMElement *enumEle=D(modelDescDoc)->createElement("EnumerationType");
            type->appendChild(enumEle);
            E(enumEle)->setAttribute("min", "1");
            E(enumEle)->setAttribute("max", boost::lexical_cast<string>((*it)->size()));
            for(size_t id=0; id<(*it)->size(); ++id) {
              DOMElement *item=D(modelDescDoc)->createElement("Item");
              enumEle->appendChild(item);
              E(item)->setAttribute("name", (**it)[id].second);
              E(item)->setAttribute("description", (**it)[id].second);
            }
        }

      // DefaultExperiment element and its attributes (only if a integrator object exists)
      if(integrator) {
        DOMElement *defaultExp=D(modelDescDoc)->createElement("DefaultExperiment");
        modelDesc->appendChild(defaultExp);
        E(defaultExp)->setAttribute("startTime", boost::lexical_cast<string>(integrator->getStartTime()));
        E(defaultExp)->setAttribute("stopTime", boost::lexical_cast<string>(integrator->getEndTime()));
        E(defaultExp)->setAttribute("tolerance", boost::lexical_cast<string>(1e-5));
      }

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
              case 'r': datatypeEleName="Real"; break;
              case 'i': datatypeEleName= var[vr]->getEnumerationList() ? "Enumeration" : "Integer"; break;
              case 'b': datatypeEleName="Boolean"; break;
              case 's': datatypeEleName="String"; break;
            }
            DOMElement *varType=D(modelDescDoc)->createElement(datatypeEleName);
            scalarVar->appendChild(varType);

            // handle enumeration
            if(var[vr]->getEnumerationList()) {
              E(varType)->setAttribute("declaredType", "EnumType_"+boost::lexical_cast<string>(var[vr]->getEnumerationList()));
              E(varType)->setAttribute("min", "1");
              E(varType)->setAttribute("max", boost::lexical_cast<string>(var[vr]->getEnumerationList()->size()));
            }

          // attributes on ScalarVariable element
          E(scalarVar)->setAttribute("name", var[vr]->getName());
          E(scalarVar)->setAttribute("description", var[vr]->getDescription());
          E(scalarVar)->setAttribute("valueReference", boost::lexical_cast<string>(vr));
          switch(var[vr]->getType()) {
            case Parameter:
              E(scalarVar)->setAttribute("causality", "internal");
              E(scalarVar)->setAttribute("variability", "parameter");
              E(varType)->setAttribute("start", var[vr]->getValueAsString());
              E(varType)->setAttribute("fixed", "true");
              break;
            case Input:
              E(scalarVar)->setAttribute("causality", "input");
              E(scalarVar)->setAttribute("variability", "continuous");
              E(varType)->setAttribute("start", var[vr]->getValueAsString());
              E(varType)->setAttribute("fixed", "true");
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

    // add main FMU binary to FMU file
    if(xmlFile)
      fmuFile.add(path("binaries")/FMIOS/("mbsim"+SHEXT), getInstallPath()/"lib"/("mbsimxml_fmi"+SHEXT));
    else {
      fmuFile.add(path("binaries")/FMIOS/("mbsim"+SHEXT), getInstallPath()/"lib"/("mbsimsrc_fmi"+SHEXT));
      fmuFile.add(path("binaries")/FMIOS/("mbsimfmi_model"+SHEXT), inputFilename);
    }

    fmuFile.close();

    return 0;
  }
  catch(const exception &ex) {
    cerr<<"Exception:\n"<<ex.what()<<endl;
    return 1;
  }
  catch(...) {
    cerr<<"Unknwon exception."<<endl;
    return 1;
  }
}
