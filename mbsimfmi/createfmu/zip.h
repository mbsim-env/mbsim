#ifndef _MBSIMFMI_ZIP_H_
#define _MBSIMFMI_ZIP_H_

#include <string>
#include <set>
#include <boost/filesystem.hpp>
#include <fmatvec/atom.h>

struct archive;
struct archive_entry;

namespace MBSimFMI {

//! Create a zip file
class CreateZip : virtual public fmatvec::Atom {
  public:
    //! Create zip file name zipFile_
    CreateZip(boost::filesystem::path zipFile_, bool compress=true);
    //! Destroy zipfile. Implicitly close the file if not already done by close().
    ~CreateZip() override;
    //! Close the zip file. Write all content to disk.
    void close();
    //! Add the content of srcFilename to zip file path filenameInZip.
    //! If filenameInZip already exists in the zip file nothing happens.
    void add(const boost::filesystem::path &filenameInZip, const boost::filesystem::path &srcFilename);
    //! Add the string textContent to zip file path filenameInZip.
    //! If filenameInZip already exists in the zip file nothing happens.
    void add(const boost::filesystem::path &filenameInZip, const std::string &textContent);

  protected:
    bool closed; // already closed flag
    boost::filesystem::path zipFile; // zip file name
    archive *a; // zip archive handle
    archive_entry *entry; // zip entry handle
    std::set<boost::filesystem::path> content;
};

}

#endif
