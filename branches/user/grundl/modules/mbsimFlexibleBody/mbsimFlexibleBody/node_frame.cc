#include <config.h>
#include "node_frame.h"

using namespace MBSim;
using namespace MBXMLUtils;

namespace MBSimFlexibleBody {

  void NodeFrame::init(InitStage stage) {
    Frame::init(stage);
  }

  void NodeFrame::initializeUsingXML(TiXmlElement *element) {
    Frame::initializeUsingXML(element);
    TiXmlElement *ec = element->FirstChildElement();
    ec = element->FirstChildElement(MBSIMNS"NodeNumber");
    if (ec)
      setNodeNumber(getInt(ec));
  }

  TiXmlElement* NodeFrame::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = Frame::writeXMLFile(parent);
    addElementText(ele0, MBSIMNS"NodeNumber", float(getNodeNumber()));
    return ele0;
  }

}
