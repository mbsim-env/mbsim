#ifndef  _FILE_TO_FMATVECSTRING_H_
#define  _FILE_TO_FMATVECSTRING_H_

#include <string>

/* \brief Method for passing ascii-files containing only numerical data into fmatvec-strings.
 * The dimension of the data is self-detected.
 * TODO clear files from header data
 * Due to char problems a string is given back, so an example call would be
 *   Mat demo(FileTofmatvecString("./demo_values").c_str());
 */
std::string FileTofmatvecString(const std::string &filename);

#endif   /* ----- #ifndef _FILE_TO_FMATVECSTRING_H_  ----- */

