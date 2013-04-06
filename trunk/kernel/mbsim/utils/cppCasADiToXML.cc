#include "mbxmlutilstinyxml/casadiXML.h"
#include <mbxmlutilstinyxml/tinynamespace.h>
#include <casadi/symbolic/fx/sx_function.hpp>
#include <casadi/symbolic/sx/sx_tools.hpp>

using namespace std;
using namespace CasADi;

int main() {
//  // test SXFunction you want to export as XML
//  vector<SXMatrix> input;
//  SX t("t"); // a scalar input
//  input.push_back(t);
//  SXMatrix q=CasADi::ssym("q", 2, 1); // a column vector input
//  input.push_back(q);
//  SXMatrix qd=CasADi::ssym("qd", 1, 2); // a row vector input
//  input.push_back(qd);
//  SXMatrix J=CasADi::ssym("J", 2, 2); // a matrix input
//  input.push_back(J);
//  SX r=q.elem(0)*q.elem(1)+sin(5*t)+t*t; // a scalar output
//  SXFunction f(input, r); // the function

//  // the translation symbolic equation using by xmlflat/time_dependent_kinematics
//  double freq1 = M_PI;
//  double v0y = 1;
//  SX t("t");
//  vector<SX> fexp(3);
//  fexp[0] = sin(freq1*t + M_PI/2);
//  fexp[1] = v0y*t; 
//  fexp[2] = 0; 
//  SXFunction f(t,fexp);

  // the rotation symbolic equation using by xmlflat/time_dependent_kinematics
  double freq2=M_PI/3;
  SX t("t");
  SX fexp=5*sin(freq2*t);
  SXFunction f(t,fexp);




  
  TiXmlDocument xmlFile;
  xmlFile.LinkEndChild(new TiXmlDeclaration("1.0","UTF-8",""));
  xmlFile.LinkEndChild(convertCasADiToXML(f));
  cout<<"XML representation"<<endl<<endl;
  map<string, string> nsprefix;
  nsprefix[MBXMLUTILSCASADINS_]="";
  unIncorporateNamespace(xmlFile.FirstChildElement(), nsprefix);
  xmlFile.SaveFile(stdout);
  incorporateNamespace(xmlFile.FirstChildElement(), nsprefix);

  cout<<endl<<"Reread XML and print original and reread as CasADi stream"<<endl<<endl;
  CasADi::SXFunction fReread=createCasADiSXFunctionFromXML(xmlFile.FirstChildElement());
  fReread.init();
  fReread.evaluate();
  for(int i=0; i<f.inputExpr().size(); i++) {
    cout<<"original input  "<<i<<": "<<f.inputExpr(i)<<endl;
    cout<<"reread   input  "<<i<<": "<<fReread.inputExpr(i)<<endl;
  }
  for(int i=0; i<f.outputExpr().size(); i++) {
    cout<<"original output "<<i<<": "<<f.outputExpr(i)<<endl;
    cout<<"reread   output "<<i<<": "<<fReread.outputExpr(i)<<endl;
  }

  return 0;
}
