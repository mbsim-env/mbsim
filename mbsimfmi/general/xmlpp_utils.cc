#include "config.h"
#include "xmlpp_utils.h"
#include <mbsimxml/mbsimflatxml.h>

using namespace std;
using namespace MBXMLUtils;
using namespace MBSim;

namespace MBSimFMI {

void convertParamSetToVariable(const std::shared_ptr<Preprocess::ParamSet> &param,
                               vector<std::shared_ptr<Variable> > &fmiParam, const std::shared_ptr<Eval> &eval) {
  for(auto & pIt : *param) {
    // add a scalar variable as it
    if(eval->valueIsOfType(pIt.second, Eval::ScalarType))
      fmiParam.push_back(std::make_shared<VariableStore<double> >(pIt.first,
        Parameter, eval->cast<double>(pIt.second)));
    // add a vector variable as seperate scalars with [idx]
    else if(eval->valueIsOfType(pIt.second, Eval::VectorType)) {
      vector<double> value=eval->cast<vector<double> >(pIt.second);
      for(int i=0; i<value.size(); ++i)
        fmiParam.push_back(std::make_shared<VariableStore<double> >(pIt.first+"["+fmatvec::toString(i+1)+"]",
          Parameter, value[i]));
    }
    // add a matrix variable as seperate scalars with [rowidx,colIdx]
    else if(eval->valueIsOfType(pIt.second, Eval::MatrixType)) {
      vector<vector<double> > value=eval->cast<vector<vector<double> > >(pIt.second);
      for(int r=0; r<value.size(); ++r)
        for(int c=0; c<value[r].size(); ++c)
          fmiParam.push_back(std::make_shared<VariableStore<double> >(pIt.first+"["+fmatvec::toString(r+1)+
            ","+fmatvec::toString(c+1)+"]", Parameter, value[r][c]));
    }
    // add a string variable as it
    else if(eval->valueIsOfType(pIt.second, Eval::StringType)) {
      string value=eval->cast<string>(pIt.second);
      fmiParam.push_back(std::make_shared<VariableStore<string> >(pIt.first, Parameter, value));
    }
  }
}

}
