#include "../config.h"
#include <fmiinstance.h>
#include <mbsim/dynamic_system_solver.h>

#include <mbsimxml/mbsimflatxml.h>
#include <mbsimxml/mbsimxml.h>
#include <mbsim/objectfactory.h>
#include <mbxmlutilshelper/getinstallpath.h> //MFMF
#include <mbxmlutils/octeval.h>
#include <mbxmlutils/preprocess.h>
#include "../general/xmlpp_utils.h"

using namespace std;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace boost::filesystem;

namespace MBSimFMI {

  void FMIInstance::addModelParametersAndCreateDSS(vector<boost::shared_ptr<Variable> > &varSim) {
    // get the model file
    path mbsimxmlfile=getSharedLibDir().parent_path().parent_path()/"resources"/"model"/"Model.mbsimprj.xml";

    // load all plugins
    msg(Debug)<<"Load MBSim plugins."<<endl;
    MBSimXML::loadPlugins();

    // init the validating parser with the mbsimxml schema file
    boost::shared_ptr<MBXMLUtils::DOMParser> validatingParser=DOMParser::create(true);
    msg(Debug)<<"Create MBSim XML schema file including all plugins."<<endl;
    generateMBSimXMLSchema(path(predefinedParameterStruct.outputDir)/".mbsimxml.xsd", getInstallPath()/"share"/"mbxmlutils"/"schema");
    validatingParser->loadGrammar(path(predefinedParameterStruct.outputDir)/".mbsimxml.xsd");
  
    // load MBSim project XML document
    msg(Debug)<<"Read MBSim XML model file."<<endl;
    boost::shared_ptr<xercesc::DOMDocument> doc=validatingParser->parse(mbsimxmlfile);

    // set param according data in var
    boost::shared_ptr<Preprocess::XPathParamSet> param=boost::make_shared<Preprocess::XPathParamSet>();
    for(vector<boost::shared_ptr<Variable> >::iterator it=var.begin(); it!=var.end(); ++it) {
      cerr<<"MFMF "<<(*it)->getName()<<endl;
    }

    // preprocess XML file
    OctEval octEval;
    vector<path> dependencies;
    xercesc::DOMElement *ele=doc->getDocumentElement();
    msg(Debug)<<"Preprocess MBSim XML model."<<endl;
    Preprocess::preprocess(validatingParser, octEval, dependencies, ele, param);

    // convert the parameter set from the mbxmlutils preprocessor to a "Variable" vector
    msg(Debug)<<"Convert XML parameters to FMI parameters."<<endl;
    vector<boost::shared_ptr<Variable> > xmlParam;
    convertXPathParamSetToVariable(param, xmlParam);
    // build a set of all Parameter's in var
    set<string> useParam;
    for(vector<boost::shared_ptr<Variable> >::iterator it=var.begin(); it!=var.end(); ++it)
      if((*it)->getType()==Parameter)
        useParam.insert((*it)->getName());
    // remove all variables in xmlParam which are not in var (these were not added as a parameter by the --param
    // option during creating of the FMU
    vector<boost::shared_ptr<Variable> > xmlParam2;
    for(vector<boost::shared_ptr<Variable> >::iterator it=xmlParam.begin(); it!=xmlParam.end(); ++it)
      if(useParam.find((*it)->getName())!=useParam.end())
        xmlParam2.push_back(*it);
    xmlParam=xmlParam2;
    // add model parameters to varSim
    msg(Debug)<<"Create model parameter variables."<<endl;
    varSim.insert(varSim.end(), xmlParam.begin(), xmlParam.end());
  
    // create object for DynamicSystemSolver
    msg(Debug)<<"Create DynamicSystemSolver."<<endl;
    doc->normalizeDocument();
    dss.reset(ObjectFactory::createAndInit<DynamicSystemSolver>(doc->getDocumentElement()->getFirstElementChild()));
  }

}
