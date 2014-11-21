#include <utils.h>

#ifdef _WIN32
#  include <windows.h>
#else
#  include <dlfcn.h>
#endif

using namespace std;
using namespace boost::filesystem;

namespace MBSimFMI {

#ifndef _WIN32
  // initialize the currentDirOnLoad variable when loading the shared object
  path currentDirOnLoad=current_path();
#endif

  path getSharedLibDir() {
    // Get the file containing this function
#ifdef _WIN32
    wchar_t moduleName[2048];
    GetModuleFileNameW((HINSTANCE)&__ImageBase, moduleName, sizeof(moduleName));
    path dllPath(moduleName);
#else
    Dl_info info;
    dladdr(reinterpret_cast<void*>(&getSharedLibDir), &info);
    path dllPath;
    if(info.dli_fname[0]=='/') // use an absolute path as it
       dllPath=info.dli_fname;
    else // prefix a relative path with the current path at the time this shared object was loaded.
         // This is required since dladdr returns the string which was used by dlopen to open the shared object
         // which may be a relative path which has to be interpreted relative to the current directory at the time the shared object was loaded.
       dllPath=currentDirOnLoad/info.dli_fname;
#endif
    dllPath.remove_filename();
    return canonical(dllPath);
  }

  // must be added to MBSimFMI
  int LoggerBuffer::sync() {
    // print the current buffer using the FMI logger using type type
    // (skip a trailing new line according the FMI spec)
    string s=str();
    if(*--s.end()=='\n')
      s.resize(s.size()-1);
    logger(c, instanceName.c_str(), fmiOK, category.c_str(), s.c_str());
    // clear the buffer and return
    str("");
    return 0;
  }

}
