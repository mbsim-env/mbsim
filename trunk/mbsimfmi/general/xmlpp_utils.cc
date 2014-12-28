#include "xmlpp_utils.h"
#include <mbsimxml/mbsimxml.h>

using namespace std;
using namespace MBXMLUtils;
using namespace MBSim;

namespace MBSimFMI {

void convertXPathParamSetToVariable(const boost::shared_ptr<Preprocess::XPathParamSet> &xpathParam,
                                    vector<boost::shared_ptr<Variable> > &fmiParam) {
  // only XML parameters of the DynamicSystemSolver element are used. -> search this element (as xpath expression)
  Preprocess::XPathParamSet::iterator parSetIt=
    xpathParam->find("/{"+MBSIMXML.getNamespaceURI()+"}MBSimProject[1]/{"+MBSIM.getNamespaceURI()+"}DynamicSystemSolver[1]");
  // if it exists loop over all parameters of this (Embed) element
  if(parSetIt!=xpathParam->end()) {
    for(Preprocess::ParamSet::iterator pIt=parSetIt->second.begin(); pIt!=parSetIt->second.end(); ++pIt) {
      OctEval::ValueType type=OctEval::getType(pIt->second);
      // add a scalar variable as it
      if(type==OctEval::ScalarType)
        fmiParam.push_back(boost::make_shared<VariableStore<double> >(pIt->first,
          Parameter, OctEval::cast<double>(pIt->second)));
      // add a vector variable as seperate scalars with [idx]
      else if(type==OctEval::VectorType) {
        vector<double> value=OctEval::cast<vector<double> >(pIt->second);
        for(int i=0; i<value.size(); ++i)
          fmiParam.push_back(boost::make_shared<VariableStore<double> >(pIt->first+"["+boost::lexical_cast<string>(i+1)+"]",
            Parameter, value[i]));
      }
      // add a matrix variable as seperate scalars with [rowidx,colIdx]
      else if(type==OctEval::MatrixType) {
        vector<vector<double> > value=OctEval::cast<vector<vector<double> > >(pIt->second);
        for(int r=0; r<value.size(); ++r)
          for(int c=0; c<value[r].size(); ++c)
            fmiParam.push_back(boost::make_shared<VariableStore<double> >(pIt->first+"["+boost::lexical_cast<string>(r+1)+
              ","+boost::lexical_cast<string>(c+1)+"]", Parameter, value[r][c]));
      }
      // add a string variable as it
      else if(type==OctEval::StringType) {
        string value=OctEval::cast<string>(pIt->second);
        value=value.substr(1, value.length()-2); // remove the leading and trailing '
        fmiParam.push_back(boost::make_shared<VariableStore<string> >(pIt->first, Parameter, value));
      }
    }
  }
}

}
