#include "element_shapes.h"

#include <mbsim/mbsim_event.h>

namespace MBSimEHD {

  ElementShape ElementShapes(const std::string & shapeName) {
    ElementShape shape;

    shape.name = shapeName;
    // default values
    shape.ndim = 1;
    shape.nnod = 1;
    shape.nnodd[0] = 1;
    shape.nnodd[1] = 1;
    shape.ser = 0;

    if (shapeName.compare("line2") == 0) {
      shape.ndim = 1;
      shape.nnod = 2;
      shape.nnodd[0] = shape.nnod;
    }
    else if (shapeName.compare("line3") == 0) {
      shape.ndim = 1;
      shape.nnod = 3;
      shape.nnodd[0] = shape.nnod;
    }
    else if (shapeName.compare("quad4") == 0) {
      shape.ndim = 2;
      shape.nnod = 4;
      shape.nnodd[0] = 2;
      shape.nnodd[1] = 2;
      shape.ser = 0;
    }
    else if (shapeName.compare("quad8") == 0 or shapeName.compare("quad8on") == 0) {
      shape.ndim = 2;
      shape.nnod = 8;
      shape.nnodd[0] = 3;
      shape.nnodd[1] = 3;
      shape.ser = 1;
    }
    else if (shapeName.compare("quad9") == 0) {
      shape.ndim = 2;
      shape.nnod = 9;
      shape.nnodd[0] = 3;
      shape.nnodd[1] = 3;
      shape.ser = 0;
    }
    else {
      throw MBSim::MBSimError("Unknwon shape type.");
    }

    return shape;
  }
}
