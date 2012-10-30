/*
 * This file is NOT part of TinyXML. Is was added by Markus
 * Friedrich to attach minimal namespace awareness to TinyXML.
 *
 * This File is Public Domain. Do what ever you want with it.
*/

#ifndef _TINYNAMESPACE_H_
#define _TINYNAMESPACE_H_

#include "tinyxml.h"
#include <map>
#include <string>

#define XINCLUDENS_ "http://www.w3.org/2001/XInclude"
#define XINCLUDENS "{"XINCLUDENS_"}"

const TiXmlElement* TiXml_GetElementWithXmlBase(TiXmlElement *e, int i);
void TiXml_PostLoadFile(TiXmlDocument *doc);

/* Just a int to string converter */
std::string TiXml_itoa(int i);

/* Output the location (filename and line number) of the element e.
 * If the element was included by a xi:include, then the filename and line number
 * of the origianl xi:include element is also shown.
 * It the element was included by a pv:embed. then the filename and line nubmer
 * and the "count" of the origianl pv:embed element is also shown.
 */
void TiXml_location(TiXmlElement *e, const std::string &pre, const std::string &post);

void TiXml_addLineNrAsProcessingInstruction(TiXmlElement *e);
void TiXml_setLineNrFromProcessingInstruction(TiXmlElement *e);
void TiXml_deletePIandComm(TiXmlElement *e);

/* Changes recursivly every element name from e.g. 'myns:localname' to
 * '{http://my.host.org/mynamespace}localname' if there is a
 * namespace alias definition attribute
 * 'xmlns:myns="http://my.host.org/mynamespace"' in the current
 * element or in any ancestor element.
 * It works, of course, also for a default namespace alias 'xmlns=...'
 * and the aliases can be overwritten by a descendant element as normal
 * in XML.
 * This function also embeds the file referenced by a 'xi:include href="..."'
 * element in the node tree.
 * If dependencies is not NULL a new line separated list of files this file
 * depends on is appended to dependencies.
 */
void incorporateNamespace(TiXmlElement* e, std::map<std::string,std::string> &nsprefix, std::map<std::string,std::string> prefixns=std::map<std::string,std::string>(), std::ostream *dependencies=NULL);

/* Changes recursivly every element name from e.g.
 * '{http://my.host.org/mynamespace}:localname' to
 * 'myns:localname' if there is an map entry
 * 'pair<string,string>("http://my.host.org/mynamespace","myns")'
 * in 'nsprefix'.
 * It works, of course, also for a default namespace alias which must
 * be given by '""' in the map.
 * The namespace alias attriburtes 'xmlns:...=...' are only added to
 * the root element.
 */
void unIncorporateNamespace(TiXmlElement *e, std::map<std::string,std::string>& nsprefix, bool firstCall=true);

/* appends the path of oldFile to newFile if newFile has not
 * an absoulute path
 */
std::string fixPath(std::string oldFile, std::string newFile);

#endif
