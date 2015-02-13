#include "../config.h"
#include <utils.h>

#define MBXMLUTILS_SHAREDLIBNAME FMU
#include <mbxmlutilshelper/getsharedlibpath_impl.h>

using namespace std;
using namespace boost::filesystem;

namespace MBSimFMI {

  // must be added to MBSimFMI
  int LoggerBuffer::sync() {
    // print the current buffer using the FMI logger using type type
    // (skip a trailing new line according the FMI spec)
    string s=str();
    if(*--s.end()=='\n')
      s.resize(s.size()-1);
    logger(c, instanceName.c_str(), status, category.c_str(), s.c_str());
    // clear the buffer and return
    str("");
    return 0;
  }

}
