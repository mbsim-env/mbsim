#!/bin/bash

set -o pipefail

OUTFILE=/tmp/$(basename $0).out.$$



mbsimxml hierachical_modelling/ppError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORA1
grep -E "^hierachical_modelling/ppError/submodel/Untergruppe.xml:[0-9]+:" $OUTFILE > /dev/null || echo ERRORA2
grep -E "^hierachical_modelling/ppError/Hauptgruppe.xml:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA3
grep -E "^hierachical_modelling/ppError/MBS.mbsim.xml:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA4
grep -E "^hierachical_modelling/ppError/MBS.mbsx:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA5

mbsimxml hierachical_modelling/parseError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORA6
grep -E "^hierachical_modelling/parseError/submodel/Untergruppe.xml:[0-9]+:" $OUTFILE > /dev/null || echo ERRORA7
grep -E "^hierachical_modelling/parseError/Hauptgruppe.xml:[0-9]+:" $OUTFILE > /dev/null || echo ERRORA9
grep -E "^hierachical_modelling/parseError/MBS.mbsim.xml:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA10
grep -E "^hierachical_modelling/parseError/MBS.mbsx:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA11

mbsimxml hierachical_modelling/initXMLError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORA12
grep -E "^hierachical_modelling/initXMLError/submodel/Untergruppe.xml:[0-9]+:" $OUTFILE > /dev/null || echo ERRORA13
grep -E "^hierachical_modelling/initXMLError/Hauptgruppe.xml:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA14
grep -E "^hierachical_modelling/initXMLError/MBS.mbsim.xml:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA15
grep -E "^hierachical_modelling/initXMLError/MBS.mbsx:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA16

mbsimxml hierachical_modelling/mbsimError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORA17
grep -E "^hierachical_modelling/mbsimError/submodel/Untergruppe.xml:[0-9]+:" $OUTFILE > /dev/null || echo ERRORA18
grep -E "^hierachical_modelling/mbsimError/Hauptgruppe.xml:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA19
grep -E "^hierachical_modelling/mbsimError/MBS.mbsim.xml:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA20
grep -E "^hierachical_modelling/mbsimError/MBS.mbsx:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA21

mbsimxml hierachical_modelling_inlineembed1/ppError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORA22
grep -E "^hierachical_modelling_inlineembed1/ppError/Hauptgruppe.xml:[0-9]+:" $OUTFILE > /dev/null || echo ERRORA23
grep -E "^hierachical_modelling_inlineembed1/ppError/MBS.mbsim.xml:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA24
grep -E "^hierachical_modelling_inlineembed1/ppError/MBS.mbsx:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA25

mbsimxml hierachical_modelling_inlineembed1/parseError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORA26
grep -E "^hierachical_modelling_inlineembed1/parseError/Hauptgruppe.xml:[0-9]+:" $OUTFILE > /dev/null || echo ERRORA27
grep -E "^hierachical_modelling_inlineembed1/parseError/MBS.mbsim.xml:[0-9]+:" $OUTFILE > /dev/null || echo ERRORA29
grep -E "^hierachical_modelling_inlineembed1/parseError/MBS.mbsx:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA30

mbsimxml hierachical_modelling_inlineembed1/initXMLError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORA31
grep -E "^hierachical_modelling_inlineembed1/initXMLError/Hauptgruppe.xml:[0-9]+:" $OUTFILE > /dev/null || echo ERRORA32
grep -E "^hierachical_modelling_inlineembed1/initXMLError/MBS.mbsim.xml:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA33
grep -E "^hierachical_modelling_inlineembed1/initXMLError/MBS.mbsx:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA34

mbsimxml hierachical_modelling_inlineembed1/mbsimError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORA35
grep -E "^hierachical_modelling_inlineembed1/mbsimError/Hauptgruppe.xml:[0-9]+:" $OUTFILE > /dev/null || echo ERRORA36
grep -E "^hierachical_modelling_inlineembed1/mbsimError/MBS.mbsim.xml:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA37
grep -E "^hierachical_modelling_inlineembed1/mbsimError/MBS.mbsx:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA38

mbsimxml hierachical_modelling_inlineembed2/ppError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORA39
grep -E "^hierachical_modelling_inlineembed2/ppError/submodel/Untergruppe.xml:[0-9]+:" $OUTFILE > /dev/null || echo ERRORA40
grep -E "^hierachical_modelling_inlineembed2/ppError/MBS.mbsim.xml:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA41
grep -E "^hierachical_modelling_inlineembed2/ppError/MBS.mbsx:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA42

mbsimxml hierachical_modelling_inlineembed2/parseError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORA43
grep -E "^hierachical_modelling_inlineembed2/parseError/submodel/Untergruppe.xml:[0-9]+:" $OUTFILE > /dev/null || echo ERRORA44
grep -E "^hierachical_modelling_inlineembed2/parseError/MBS.mbsim.xml:[0-9]+:" $OUTFILE > /dev/null || echo ERRORA46
grep -E "^hierachical_modelling_inlineembed2/parseError/MBS.mbsx:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA47

mbsimxml hierachical_modelling_inlineembed2/initXMLError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORA48
grep -E "^hierachical_modelling_inlineembed2/initXMLError/submodel/Untergruppe.xml:[0-9]+:" $OUTFILE > /dev/null || echo ERRORA49
grep -E "^hierachical_modelling_inlineembed2/initXMLError/MBS.mbsim.xml:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA50
grep -E "^hierachical_modelling_inlineembed2/initXMLError/MBS.mbsx:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA51

mbsimxml hierachical_modelling_inlineembed2/mbsimError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORA52
grep -E "^hierachical_modelling_inlineembed2/mbsimError/submodel/Untergruppe.xml:[0-9]+:" $OUTFILE > /dev/null || echo ERRORA52
grep -E "^hierachical_modelling_inlineembed2/mbsimError/MBS.mbsim.xml:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA54
grep -E "^hierachical_modelling_inlineembed2/mbsimError/MBS.mbsx:[0-9]+: \[ecount=1] included from here" $OUTFILE > /dev/null || echo ERRORA55



export MBXMLUTILS_ERROROUTPUT=HTMLXPATH
NS="{[^}]*}"

mbsimxml hierachical_modelling/ppError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORB1
grep -E "xpath=/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]/${NS}plotFeature\[1]/@value[\"&].*>hierachical_modelling/ppError/submodel/Untergruppe.xml<" $OUTFILE > /dev/null || echo ERRORB2
grep -E "xpath=/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2][\"&].*>hierachical_modelling/ppError/Hauptgruppe.xml<.*>included from here" $OUTFILE > /dev/null || echo ERRORB3
grep -E "xpath=/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1][\"&].*>hierachical_modelling/ppError/MBS.mbsim.xml<.*>included from here" $OUTFILE > /dev/null || echo ERRORB4
grep -E "xpath=/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1][\"&].*>.*hierachical_modelling/ppError/MBS.mbsx<.*>included from here" $OUTFILE > /dev/null || echo ERRORB5

mbsimxml hierachical_modelling/parseError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORB6
grep -E ".*xpath=/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1][\"&].*>.*hierachical_modelling/parseError/submodel/Untergruppe.xml<" $OUTFILE > /dev/null || echo ERRORB8
grep -E ".*xpath=/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2][\"&].*>hierachical_modelling/parseError/Hauptgruppe.xml<" $OUTFILE > /dev/null || echo ERRORB9
grep -E "xpath=/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1][\"&].*>hierachical_modelling/parseError/MBS.mbsim.xml<.*>included from here" $OUTFILE > /dev/null || echo ERRORB10
grep -E "xpath=/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1][\"&].*>.*hierachical_modelling/parseError/MBS.mbsx<.*>included from here" $OUTFILE > /dev/null || echo ERRORB11

mbsimxml hierachical_modelling/initXMLError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORB12
grep -E ".*xpath=/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]/${NS}mass\[1][\"&].*>hierachical_modelling/initXMLError/submodel/Untergruppe.xml<" $OUTFILE > /dev/null || echo ERRORB13
grep -E "xpath=/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2][\"&].*>hierachical_modelling/initXMLError/Hauptgruppe.xml<.*>included from here" $OUTFILE > /dev/null || echo ERRORB14
grep -E "xpath=/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1][\"&].*>hierachical_modelling/initXMLError/MBS.mbsim.xml<.*>included from here" $OUTFILE > /dev/null || echo ERRORB15
grep -E "xpath=/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1][\"&].*>.*hierachical_modelling/initXMLError/MBS.mbsx<.*>included from here" $OUTFILE > /dev/null || echo ERRORB16

mbsimxml hierachical_modelling/mbsimError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORB17
grep -E ".*xpath=/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1][\"&].*>hierachical_modelling/mbsimError/submodel/Untergruppe.xml<" $OUTFILE > /dev/null || echo ERRORB18
grep -E "xpath=/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2][\"&].*>hierachical_modelling/mbsimError/Hauptgruppe.xml<.*>included from here" $OUTFILE > /dev/null || echo ERRORB19
grep -E "xpath=/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1][\"&].*>hierachical_modelling/mbsimError/MBS.mbsim.xml<.*>included from here" $OUTFILE > /dev/null || echo ERRORB20
grep -E "xpath=/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1][\"&].*>.*hierachical_modelling/mbsimError/MBS.mbsx<.*>included from here" $OUTFILE > /dev/null || echo ERRORB21

mbsimxml hierachical_modelling_inlineembed1/ppError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORB22
grep -E ".*xpath=/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2]/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]/${NS}plotFeature\[1]/@value[\"&].*>hierachical_modelling_inlineembed1/ppError/Hauptgruppe.xml<" $OUTFILE > /dev/null || echo ERRORB23
grep -E "xpath=/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1][\"&].*>hierachical_modelling_inlineembed1/ppError/MBS.mbsim.xml<.*>included from here" $OUTFILE > /dev/null || echo ERRORB24
grep -E "xpath=/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1][\"&].*>.*hierachical_modelling_inlineembed1/ppError/MBS.mbsx<.*>included from here" $OUTFILE > /dev/null || echo ERRORB25

mbsimxml hierachical_modelling_inlineembed1/parseError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORB26
grep -E ".*xpath=/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2]/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1][\"&].*>.*hierachical_modelling_inlineembed1/parseError/Hauptgruppe.xml<" $OUTFILE > /dev/null || echo ERRORB28
grep -E ".*xpath=/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1][\"&].*>hierachical_modelling_inlineembed1/parseError/MBS.mbsim.xml<" $OUTFILE > /dev/null || echo ERRORB29
grep -E "xpath=/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1][\"&].*>.*hierachical_modelling_inlineembed1/parseError/MBS.mbsx<.*>included from here" $OUTFILE > /dev/null || echo ERRORB30

mbsimxml hierachical_modelling_inlineembed1/initXMLError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORB31
grep -E ".*xpath=/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2]/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]/${NS}mass\[1][\"&].*>hierachical_modelling_inlineembed1/initXMLError/Hauptgruppe.xml<" $OUTFILE > /dev/null || echo ERRORB32
grep -E "xpath=/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1][\"&].*>hierachical_modelling_inlineembed1/initXMLError/MBS.mbsim.xml<.*>included from here" $OUTFILE > /dev/null || echo ERRORB33
grep -E "xpath=/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1][\"&].*>.*hierachical_modelling_inlineembed1/initXMLError/MBS.mbsx<.*>included from here" $OUTFILE > /dev/null || echo ERRORB34

mbsimxml hierachical_modelling_inlineembed1/mbsimError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORB35
grep -E ".*xpath=/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2]/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1][\"&].*>hierachical_modelling_inlineembed1/mbsimError/Hauptgruppe.xml<" $OUTFILE > /dev/null || echo ERRORB36
grep -E "xpath=/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1][\"&].*>hierachical_modelling_inlineembed1/mbsimError/MBS.mbsim.xml<.*>included from here" $OUTFILE > /dev/null || echo ERRORB37
grep -E "xpath=/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1][\"&].*>.*hierachical_modelling_inlineembed1/mbsimError/MBS.mbsx<.*>included from here" $OUTFILE > /dev/null || echo ERRORB38

mbsimxml hierachical_modelling_inlineembed2/ppError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORB39
grep -E ".*xpath=/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]/${NS}plotFeature\[1]/@value[\"&].*>hierachical_modelling_inlineembed2/ppError/submodel/Untergruppe.xml<" $OUTFILE > /dev/null || echo ERRORB40
grep -E "xpath=/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1]/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2][\"&].*>hierachical_modelling_inlineembed2/ppError/MBS.mbsim.xml<.*>included from here" $OUTFILE > /dev/null || echo ERRORB41
grep -E "xpath=/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1][\"&].*>.*hierachical_modelling_inlineembed2/ppError/MBS.mbsx<.*>included from here" $OUTFILE > /dev/null || echo ERRORB42

mbsimxml hierachical_modelling_inlineembed2/parseError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORB43
grep -E ".*xpath=/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1][\"&].*>.*hierachical_modelling_inlineembed2/parseError/submodel/Untergruppe.xml<" $OUTFILE > /dev/null || echo ERRORB45
grep -E ".*xpath=/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1]/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2][\"&].*>hierachical_modelling_inlineembed2/parseError/MBS.mbsim.xml<" $OUTFILE > /dev/null || echo ERRORB46
grep -E "xpath=/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1][\"&].*>.*hierachical_modelling_inlineembed2/parseError/MBS.mbsx<.*>included from here" $OUTFILE > /dev/null || echo ERRORB47

mbsimxml hierachical_modelling_inlineembed2/initXMLError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORB48
grep -E ".*xpath=/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]/${NS}mass\[1][\"&].*>hierachical_modelling_inlineembed2/initXMLError/submodel/Untergruppe.xml<" $OUTFILE > /dev/null || echo ERRORB49
grep -E "xpath=/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1]/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2][\"&].*>hierachical_modelling_inlineembed2/initXMLError/MBS.mbsim.xml<.*>included from here" $OUTFILE > /dev/null || echo ERRORB50
grep -E "xpath=/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1][\"&].*>.*hierachical_modelling_inlineembed2/initXMLError/MBS.mbsx<.*>included from here" $OUTFILE > /dev/null || echo ERRORB51

mbsimxml hierachical_modelling_inlineembed2/mbsimError/MBS.mbsx &> $OUTFILE
test $? -eq 0 && echo ERRORB52
grep -E ".*xpath=/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1][\"&].*>hierachical_modelling_inlineembed2/mbsimError/submodel/Untergruppe.xml<" $OUTFILE > /dev/null || echo ERRORB52
grep -E "xpath=/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1]/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2][\"&].*>hierachical_modelling_inlineembed2/mbsimError/MBS.mbsim.xml<.*>included from here" $OUTFILE > /dev/null || echo ERRORB54
grep -E "xpath=/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1][\"&].*>.*hierachical_modelling_inlineembed2/mbsimError/MBS.mbsx<.*>included from here" $OUTFILE > /dev/null || echo ERRORB55



mbsimxml hierachical_modelling/noError/MBS.mbsx &> $OUTFILE
test $? -ne 0 && echo ERRORC56

mbsimxml hierachical_modelling_inlineembed1/noError/MBS.mbsx &> $OUTFILE
test $? -ne 0 && echo ERRORC57

mbsimxml hierachical_modelling_inlineembed2/noError/MBS.mbsx &> $OUTFILE
test $? -ne 0 && echo ERRORC58



rm $OUTFILE
