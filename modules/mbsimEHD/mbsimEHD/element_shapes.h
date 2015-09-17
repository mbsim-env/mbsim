#ifndef ElementShapes_EHD_H_
#define ElementShapes_EHD_H_

#include <string>

namespace MBSimEHD {

  //   shape:      Struct with element shape properties
  //               ndim:   Spatial dimension
  //               nnod:   Number of nodes
  //               nnodd:  Number of nodes in spatial directions
  //               ser:    Number of additional nodes located at
  //                       element boundary (serendipity elements)
  struct ElementShape {
      int ndim;
      int nnod;
      int nnodd[2];
      int ser;
      std::string name;
  };

  // Element shape properties
  // Michael Hofer, 24.03.2015
  //
  // Input:
  //   shapeName:  Name of element shape
  //
  // Output:
  // ElementShape (see above)
  ElementShape ElementShapes(const std::string & shapeName);


}

#endif
