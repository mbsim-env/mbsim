#include <boost/filesystem.hpp>
#include <boost/assign.hpp>
#include <boost/lexical_cast.hpp>
#include <set>
#include <sys/stat.h>

using namespace std;
using namespace boost::filesystem;

// configuration
const uid_t rootUID=0;
const uid_t mbsimUID=1000;
const gid_t mbsimGID=1000;
const uid_t mbsimsbUID=1001;
const set<path> allowedDirs=boost::assign::list_of
  (canonical("/var/www/html/mbsim"))
  (canonical("/home/mbsim"))
  (canonical("/tmp"))
.to_container(allowedDirs);

void changeOwner(const path &f, uid_t newUID) {
  if(!exists(f))
    return;
  // get stat
  struct stat st;
  if(stat(f.string().c_str(), &st)!=0)
    throw runtime_error("Internal error: Cannot get stat of "+f.string()+".");
  // check for owner
  if(st.st_uid!=mbsimUID && st.st_uid!=mbsimsbUID) {
    cout<<"Skipping "<<f<<": Invalid owner."<<endl;
    return;
  }
  
  // reown
  if(chown(f.string().c_str(), newUID, -1)!=0)
    throw runtime_error("Internal error: Cannot change owner of "+f.string()+".");
}

int main(int argc, char *argv[]) {
  try {
    // check permissions of this program
    struct stat st;
    if(stat(argv[0], &st)!=0)
      throw runtime_error("Internal error: Cannot get stat of program.");
    if(st.st_uid!=rootUID || // owner = root
       st.st_gid!=mbsimGID || // group = mbsim
       !(st.st_mode & S_ISUID) || // suid set
       (st.st_mode & S_ISGID) || // sguid not set
       !(st.st_mode & S_IRGRP) || // group is readable
       (st.st_mode & S_IWGRP) || // group is not writeable
       !(st.st_mode & S_IXGRP) || // gorup can execute
       (st.st_mode & S_IROTH) || // other is readable
       (st.st_mode & S_IWOTH) || // other is not writeable
       (st.st_mode & S_IXOTH)) // other can execute
      throw runtime_error("Internal error: Wrong permissions for program.");

    // check calling user of this program
    if(getuid()!=mbsimUID ||
       geteuid()!=rootUID)
      throw runtime_error("Internal error: Wrong user or effective user.");

    // print usage usage
    if(argc<3) {
      cout<<"Usage: "<<argv[0]<<" ["<<mbsimUID<<"|"<<mbsimsbUID<<"] file/dir ..."<<endl;
      return 1;
    }

    // check first parameter
    uid_t newUID=boost::lexical_cast<int>(argv[1]);
    if(newUID!=mbsimUID && newUID!=mbsimsbUID)
      throw runtime_error("Wrong arguments: Invalid new uid.");

    // check other parameters
    for(int i=2; i<argc; ++i) {
      if(!exists(argv[i]))
        throw runtime_error(string("The file (argument) ")+argv[i]+" does not exist.");
      string curArg=canonical(argv[i]).string()+"/";
      bool allow=false;
      for(set<path>::iterator a=allowedDirs.begin(); a!=allowedDirs.end(); ++a) {
        string allowed=a->string()+"/";
        if(curArg.substr(0, allowed.size())==allowed) {
          allow=true;
          break;
        }
      }
      if(!allow)
        throw runtime_error("Wrong arguments: Changes in "+curArg+" are not allowed.");
    }

    // loop over all file/dir arguments
    for(int i=2; i<argc; ++i) {
      // reown arg
      changeOwner(argv[i], newUID);
      // loop over dir if arg is a dir
      if(is_directory(argv[i]))
        for(recursive_directory_iterator f=recursive_directory_iterator(argv[i]); f!=recursive_directory_iterator(); ++f)
          changeOwner(f->path(), newUID);
    }
  }
  catch(const exception &ex) {
    cerr<<ex.what()<<endl;
    return 1;
  }

  return 0;
}
