#ifndef _MBSIMFMI_XMLPP_UTILS_H_
#define _MBSIMFMI_XMLPP_UTILS_H_

#include <mbxmlutils/preprocess.h>
#include "fmi_variables.h"

namespace MBSimFMI {

//! Convert the XPathParamSet xpathParam to a vector of VariableStore.
void convertXPathParamSetToVariable(const std::shared_ptr<MBXMLUtils::Preprocess::XPathParamSet> &xpathParam,
                                    std::vector<std::shared_ptr<Variable> > &fmiParam, const std::shared_ptr<MBXMLUtils::Eval> &eval);

}

#endif
