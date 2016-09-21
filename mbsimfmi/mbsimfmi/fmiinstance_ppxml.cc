#include "../config.h"
#include <fmiinstance.h>
#include <mbsim/dynamic_system_solver.h>

#include <mbsimxml/mbsimflatxml.h>
#include <mbsimxml/mbsimxml.h>
#include <mbsim/objectfactory.h>
#include <mbxmlutils/eval.h>
#include <mbxmlutils/preprocess.h>
#include "../general/xmlpp_utils.h"
#include "boost/filesystem/fstream.hpp"

using namespace std;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace boost::filesystem;

namespace {
  void convertVariableToXPathParamSet(size_t startIndex, const vector<std::shared_ptr<MBSimFMI::Variable> > &var,
                                      std::shared_ptr<Preprocess::XPathParamSet> &param, const shared_ptr<Eval> &eval);
}

namespace MBSimFMI {

  void FMIInstance::addModelParametersAndCreateDSS(vector<std::shared_ptr<Variable> > &varSim) {
    // get the model file
    path resourcesDir=path(MBXMLUtils::getFMUSharedLibPath()).parent_path().parent_path().parent_path();
    // path to XML project file relative to resources/model
    boost::filesystem::ifstream xmlProjectStream(resourcesDir/"model"/"XMLProjectFile.txt");
    string xmlProjectFile;
    getline(xmlProjectStream, xmlProjectFile);
    path mbsimxmlfile=resourcesDir/"model"/xmlProjectFile;

    // load all MBSim modules
    msg(Debug)<<"Load MBSim modules."<<endl;
    MBSimXML::loadModules();

    // init the validating parser with the mbsimxml schema file
    msg(Debug)<<"Create MBSim XML schema file including all modules."<<endl;
    set<path> schemas=getMBSimXMLSchemas();
    std::shared_ptr<MBXMLUtils::DOMParser> validatingParser=DOMParser::create(schemas);
  
    // load MBSim project XML document
    msg(Debug)<<"Read MBSim XML model file."<<endl;
    std::shared_ptr<xercesc::DOMDocument> doc=validatingParser->parse(mbsimxmlfile);
    xercesc::DOMElement *ele=doc->getDocumentElement();

    // create a clean evaluator (get the evaluator name first form the dom)
    string evalName="octave"; // default evaluator
    xercesc::DOMElement *evaluator=E(ele)->getFirstElementChildNamed(PV%"evaluator");
    if(evaluator)
      evalName=X()%E(evaluator)->getFirstTextChild()->getData();
    shared_ptr<Eval> eval=Eval::createEvaluator(evalName);

    // set param according data in var
    std::shared_ptr<Preprocess::XPathParamSet> param=std::make_shared<Preprocess::XPathParamSet>();
    convertVariableToXPathParamSet(varSim.size(), var, param, eval);

    // preprocess XML file
    vector<path> dependencies;
    msg(Debug)<<"Preprocess MBSim XML model."<<endl;
    Preprocess::preprocess(validatingParser, eval, dependencies, ele, param);

    // convert the parameter set from the mbxmlutils preprocessor to a "Variable" vector
    msg(Debug)<<"Convert XML parameters to FMI parameters."<<endl;
    vector<std::shared_ptr<Variable> > xmlParam;
    convertXPathParamSetToVariable(param, xmlParam, eval);
    // build a set of all Parameter's in var
    set<string> useParam;
    for(vector<std::shared_ptr<Variable> >::iterator it=var.begin(); it!=var.end(); ++it)
      if((*it)->getType()==Parameter)
        useParam.insert((*it)->getName());
    // remove all variables in xmlParam which are not in var (these were not added as a parameter by the --param
    // option during creating of the FMU
    vector<std::shared_ptr<Variable> > xmlParam2;
    for(vector<std::shared_ptr<Variable> >::iterator it=xmlParam.begin(); it!=xmlParam.end(); ++it)
      if(useParam.find((*it)->getName())!=useParam.end())
        xmlParam2.push_back(*it);
    xmlParam=xmlParam2;
    // add model parameters to varSim
    msg(Debug)<<"Create model parameter variables."<<endl;
    varSim.insert(varSim.end(), xmlParam.begin(), xmlParam.end());
  
    // create object for DynamicSystemSolver
    msg(Debug)<<"Create DynamicSystemSolver."<<endl;
    doc->normalizeDocument();
    dss.reset(ObjectFactory::createAndInit<DynamicSystemSolver>(E(ele)->getFirstElementChildNamed(MBSIM%"DynamicSystemSolver")));
  }

}

namespace {

  double getValueAsDouble(MBSimFMI::Variable &v) {
    switch(v.getDatatypeChar()) {
      case 'r':
        return v.getValue(double());
      case 'i':
        return v.getValue(int());
      case 'b':
        return v.getValue(bool());
      default:
        throw runtime_error("Internal error: Unknwon type.");
    }
  }

  string getName(MBSimFMI::Variable &v) {
    string name=v.getName();
    return name.substr(0, name.find('['));
  }

  size_t getCol(MBSimFMI::Variable &v) {
    string name=v.getName();
    name=name.substr(name.find('[')+1);
    name=name.substr(name.find(',')+1);
    return boost::lexical_cast<size_t>(name.substr(0, name.size()-1));
  }

  void convertVariableToXPathParamSet(size_t startIndex, const vector<std::shared_ptr<MBSimFMI::Variable> > &var,
                                      std::shared_ptr<Preprocess::XPathParamSet> &param, const shared_ptr<Eval> &eval) {
    Preprocess::ParamSet &p=param->insert(make_pair(
      "/{"+MBSIMXML.getNamespaceURI()+"}MBSimProject[1]/{"+MBSIM.getNamespaceURI()+"}DynamicSystemSolver[1]",
      Preprocess::ParamSet())).first->second;
    for(vector<std::shared_ptr<MBSimFMI::Variable> >::const_iterator it=var.begin()+startIndex; it!=var.end(); ++it) {
      // skip all but parameters
      if((*it)->getType()!=MBSimFMI::Parameter)
        continue;
      // handle string parameters (can only be scalars)
      if((*it)->getDatatypeChar()=='s')
        p.push_back(make_pair((*it)->getName(), eval->create((*it)->getValue(string()))));
      // handle none string parameters (can be scalar, vector or matrix)
      else {
        // get name and type (scalar, vector or matrix)
        string name=(*it)->getName();
        size_t poss=name.find('[');
        Eval::ValueType type=Eval::ScalarType;
        if(poss!=string::npos) {
          type=Eval::VectorType;
          if(name.substr(poss+1).find(',')!=string::npos)
            type=Eval::MatrixType;
        }
        name=name.substr(0, poss);
        Eval::Value value;
        switch(type) {
          case Eval::ScalarType:
            value=eval->create(getValueAsDouble(**it));
            break;
          case Eval::VectorType: {
            vector<double> vec;
            for(; it!=var.end() && name==getName(**it); ++it)
              vec.push_back(getValueAsDouble(**it));
            --it;
            value=eval->create(vec);
            break;
          }
          case Eval::MatrixType: {
            vector<vector<double> > mat;
            for(; it!=var.end() && name==getName(**it);) {
              vector<double> row;
              for(; it!=var.end() && name==getName(**it) && !(getCol(**it)==1 && row.size()>0); ++it)
                row.push_back(getValueAsDouble(**it));
              mat.push_back(row);
            }
            --it;
            value=eval->create(mat);
            break;
          }
          default:
            throw runtime_error("Internal error: Unknwon type.");
        }
        p.push_back(make_pair(name, value));
      }
    }
  }

}
