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
#include <../general/xmlpp_utils.h>
#include <../general/mbsimsrc_fmi.h>

using namespace std;
using namespace boost::filesystem;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace MBSimFMI;

namespace {
  // some platform dependent values
#ifdef _WIN32
  string USERENVVAR("USERNAME");
#else
  string USERENVVAR("USER");
#endif
}

int main(int argc, char *argv[]) {
  try {
    // help
    if(argc<2) {
      cout<<"Usage: "<<argv[0]<<" [--param <name> [--param <name> ...]]"<<endl;
      cout<<"  <MBSim Project XML File>|<MBSim FMI shared library>"<<endl;
      cout<<endl;
      cout<<"Creates mbsim.fmu in the current directory."<<endl;
      cout<<"The MBSim model is taken from the provided XML project file or shared library file."<<endl;
      cout<<"For XML project model files --param may be specified 0 to n times. Where <name> defines"<<endl;
      cout<<"a parameter name of the DynamicSystemSolver Embed element which should be"<<endl;
      cout<<"provided as an FMU parameter. Note that only parameters which do not depend"<<endl;
      cout<<"on any other parameters are allowed."<<endl;
      return 0;
    }

    // get --param
    set<string> useParam;
    for(int i=1; i<argc-1; ++i)
      if(string(argv[i])=="--param")
        useParam.insert(argv[i+1]);

    // get model file
    if(2*useParam.size()+1>=argc) {
      cout<<"Wrong command line."<<endl;
      return 1;
    }
    path inputFilename=argv[2*useParam.size()+1];

    // xml or shared library
    bool xmlFile=false;
    if(boost::iequals(extension(inputFilename), ".xml"))
      xmlFile=true;

    // create FMU zip file
    CreateZip fmuFile("mbsim.fmu");

    // load all plugins
    cout<<"Load MBSim plugins."<<endl;
    MBSimXML::loadPlugins();

    // create parser (a validating parser for XML input and a none validating parser for shared library input)
    boost::shared_ptr<DOMParser> parser=DOMParser::create(xmlFile);

    // note: the order of these variable definitions is importend for proper deallocation
    // (dss and integrator must be deallocated before the shLib is unlaoded)
    boost::shared_ptr<SharedLibrary> shLib;
    boost::shared_ptr<DynamicSystemSolver> dss;
    boost::shared_ptr<MBSimIntegrator::Integrator> integrator;

    vector<path> dependencies;

    PredefinedParameterStruct predefinedParameterStruct;
    cout<<"Create predefined parameters."<<endl;
    vector<boost::shared_ptr<Variable> > var;
    addPredefinedParameters(var, predefinedParameterStruct);

    vector<boost::shared_ptr<Variable> > xmlParam;

    // Create dss from XML file
    if(xmlFile) {
      // create octave
      OctEval octEval(&dependencies);

      // init the validating parser with the mbsimxml schema file
      cout<<"Create MBSimXML XML schema including all plugins."<<endl;
      generateMBSimXMLSchema(".mbsimxml.xsd", getInstallPath()/"share"/"mbxmlutils"/"schema");
      parser->loadGrammar(".mbsimxml.xsd");

      // load MBSim project XML document
      cout<<"Load MBSim model from XML project file."<<endl;
      boost::shared_ptr<xercesc::DOMDocument> modelDoc=parser->parse(inputFilename, &dependencies);
      DOMElement *modelEle=modelDoc->getDocumentElement();

      // preprocess XML file
      cout<<"Preprocess XML project file."<<endl;
      boost::shared_ptr<Preprocess::XPathParamSet> param=boost::make_shared<Preprocess::XPathParamSet>();
      Preprocess::preprocess(parser, octEval, dependencies, modelEle, param);

      // convert the parameter list from the mbxmlutils preprocessor to a Variable vector
      if(useParam.size()>0)
        convertXPathParamSetToVariable(param, xmlParam);
      // remove all variables which are not in useParam
      vector<boost::shared_ptr<Variable> > preVar2;
      for(vector<boost::shared_ptr<Variable> >::iterator it=xmlParam.begin(); it!=xmlParam.end(); ++it)
        if(useParam.find((*it)->getName())!=useParam.end()) {
          cout<<"Using DynamicSystemSolver parameter "<<(*it)->getName()<<"."<<endl;
          preVar2.push_back(*it);
        }
      xmlParam=preVar2;
      cout<<"Create model parameters."<<endl;
      var.insert(var.end(), xmlParam.begin(), xmlParam.end());

      if(xmlParam.empty()) {
        cout<<"No parameters used, copy preprocessed XML file to FMU."<<endl;
        // no parameters -> save preprocessed model to FMU
        string ppModelStr;
        // serialize to string: this automatically normalized modelEle
        DOMParser::serializeToString(modelEle, ppModelStr, false);
        fmuFile.add(path("resources")/"model"/"Model.mbsimprj.flat.xml", ppModelStr);
      }
      else {
        cout<<"Parameters exists, copy original XML file and its dependencies to FMU."<<endl;
        // normalize modelEle
        modelEle->getOwnerDocument()->normalizeDocument();
        // parameters existing -> save original XML file to FMU including all dependencies
        fmuFile.add(path("resources")/"model"/"Model.mbsimprj.xml", inputFilename);
        for(vector<path>::iterator it=dependencies.begin(); it!=dependencies.end(); ++it)
          if(!is_directory(*it))
            fmuFile.add(path("resources")/"model"/(*it), *it);
      }

      // create object for DynamicSystemSolver
      // Note: The document modelEle must be normalized (done above, see above)
      cout<<"Build up the model."<<endl;
      dss.reset(ObjectFactory::createAndInit<DynamicSystemSolver>(modelEle->getFirstElementChild()));

      // create object for Integrator (just to get the start/end time for DefaultExperiment)
      integrator.reset(ObjectFactory::createAndInit<MBSimIntegrator::Integrator>(
        modelEle->getFirstElementChild()->getNextElementSibling()));
    }
    // Create dss from shared library
    else {
      // save binary model to FMU
      cout<<"Copy the shared library model file to FMU."<<endl;
      fmuFile.add(path("binaries")/FMIOS/("mbsimfmi_model"+SHEXT), inputFilename);

      // load the shared library and call mbsimSrcFMI function to get the dss
      cout<<"Build up the model (by just getting it from the shared library)."<<endl;
      shLib=boost::make_shared<SharedLibrary>(absolute(inputFilename));
      DynamicSystemSolver *dssPtr;
      reinterpret_cast<mbsimSrcFMIPtr>(shLib->getAddress("mbsimSrcFMI"))(dssPtr);
      dss.reset(dssPtr);

      // we do not have a integrator for src models (but this is only used for convinence settings)
      integrator.reset();
    }

    // build list of value references
    cout<<"Create model input/output variables."<<endl;
    addModelInputOutputs(var, dss.get());

    // disable all plotting (we wont any output here)
    dss->setPlotFeatureRecursive(Element::plotRecursive, Element::disabled);
    // initialize dss
    cout<<"Initialize the model."<<endl;
    dss->initialize();

    // create DOM of modelDescription.xml
    boost::shared_ptr<DOMDocument> modelDescDoc(parser->createDocument());

    // root element fmiModelDescription and its attributes
    cout<<"Create the modelDescription.xml file."<<endl;
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
    cout<<"Copy the modelDescription.xml file to FMU."<<endl;
    string modelDescriptionStr;
    DOMParser::serializeToString(modelDescDoc.get(), modelDescriptionStr);
    fmuFile.add("modelDescription.xml", modelDescriptionStr);

    // add main FMU binary to FMU file
    if(xmlFile) {
      if(xmlParam.empty()) {
        // xml with no parameters -> save mbsimxml_fmi.so to FMU
        cout<<"Copy MBSim FMI library for preprocessed XML models to FMU."<<endl;
        fmuFile.add(path("binaries")/FMIOS/("mbsim"+SHEXT), getInstallPath()/"lib"/("mbsimxml_fmi"+SHEXT));
      }
      else {
        // xml with parameters -> save mbsimppxml_fmi.so to FMU
        cout<<"Copy MBSim FMI library for (normal) XML models to FMU."<<endl;
        fmuFile.add(path("binaries")/FMIOS/("mbsim"+SHEXT), getInstallPath()/"lib"/("mbsimppxml_fmi"+SHEXT));
      }
    }
    else {
      // source model (always without parameters) -> save mbsimppxml_fmi.so to FMU
      cout<<"Copy MBSim FMI library for source code models to FMU."<<endl;
      fmuFile.add(path("binaries")/FMIOS/("mbsim"+SHEXT), getInstallPath()/"lib"/("mbsimsrc_fmi"+SHEXT));
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
