#ifndef _MBSIMFMI_XMLPP_UTILS_H_
#define _MBSIMFMI_XMLPP_UTILS_H_

#include <boost/shared_ptr.hpp>
#include <mbxmlutils/preprocess.h>
#include "fmi_variables.h"

namespace MBSimFMI {

//! Convert the XPathParamSet xpathParam to a vector of VariableStore.
void convertXPathParamSetToVariable(const boost::shared_ptr<MBXMLUtils::Preprocess::XPathParamSet> &xpathParam,
                                    std::vector<boost::shared_ptr<Variable> > &fmiParam, MBXMLUtils::Eval &eval);

}

#endif
