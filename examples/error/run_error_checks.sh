#!/bin/bash

OUTFILE=/tmp/$(basename $0).out.$$



mbsimxml hierachical_modelling/ppError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORA1
grep ".*hierachical_modelling/ppError/submodel/Untergruppe.xml:11:" $OUTFILE > /dev/null || echo ERRORA2
grep "included by .*hierachical_modelling/ppError/Hauptgruppe.xml:40\[count=1]" $OUTFILE > /dev/null || echo ERRORA3
grep "included by .*hierachical_modelling/ppError/MBS.mbsim.xml:18\[count=1]" $OUTFILE > /dev/null || echo ERRORA4
grep "included by .*hierachical_modelling/ppError/MBS.mbsimprj.xml:8\[count=1]" $OUTFILE > /dev/null || echo ERRORA5

mbsimxml hierachical_modelling/parseError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORA6
grep ".*hierachical_modelling/parseError/submodel/Untergruppe.xml:16:" $OUTFILE > /dev/null || echo ERRORA7
grep ".*hierachical_modelling/parseError/submodel/Untergruppe.xml:72:" $OUTFILE > /dev/null || echo ERRORA8
grep ".*hierachical_modelling/parseError/Hauptgruppe.xml:40:" $OUTFILE > /dev/null || echo ERRORA9
grep "included by .*hierachical_modelling/parseError/MBS.mbsim.xml:18\[count=1]" $OUTFILE > /dev/null || echo ERRORA10
grep "included by .*hierachical_modelling/parseError/MBS.mbsimprj.xml:8\[count=1]" $OUTFILE > /dev/null || echo ERRORA11

mbsimxml hierachical_modelling/initXMLError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORA12
grep ".*hierachical_modelling/initXMLError/submodel/Untergruppe.xml:16:" $OUTFILE > /dev/null || echo ERRORA13
grep "included by .*hierachical_modelling/initXMLError/Hauptgruppe.xml:40\[count=1]" $OUTFILE > /dev/null || echo ERRORA14
grep "included by .*hierachical_modelling/initXMLError/MBS.mbsim.xml:18\[count=1]" $OUTFILE > /dev/null || echo ERRORA15
grep "included by .*hierachical_modelling/initXMLError/MBS.mbsimprj.xml:8\[count=1]" $OUTFILE > /dev/null || echo ERRORA16

mbsimxml hierachical_modelling/mbsimError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORA17
grep ".*hierachical_modelling/mbsimError/submodel/Untergruppe.xml:10:" $OUTFILE > /dev/null || echo ERRORA18
grep "included by .*hierachical_modelling/mbsimError/Hauptgruppe.xml:40\[count=1]" $OUTFILE > /dev/null || echo ERRORA19
grep "included by .*hierachical_modelling/mbsimError/MBS.mbsim.xml:18\[count=1]" $OUTFILE > /dev/null || echo ERRORA20
grep "included by .*hierachical_modelling/mbsimError/MBS.mbsimprj.xml:8\[count=1]" $OUTFILE > /dev/null || echo ERRORA21

mbsimxml hierachical_modelling_inlineembed1/ppError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORA22
grep ".*hierachical_modelling_inlineembed1/ppError/Hauptgruppe.xml:50:" $OUTFILE > /dev/null || echo ERRORA23
grep "included by .*hierachical_modelling_inlineembed1/ppError/MBS.mbsim.xml:18\[count=1]" $OUTFILE > /dev/null || echo ERRORA24
grep "included by .*hierachical_modelling_inlineembed1/ppError/MBS.mbsimprj.xml:8\[count=1]" $OUTFILE > /dev/null || echo ERRORA25

mbsimxml hierachical_modelling_inlineembed1/parseError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORA26
grep ".*hierachical_modelling_inlineembed1/parseError/Hauptgruppe.xml:55:" $OUTFILE > /dev/null || echo ERRORA27
grep ".*hierachical_modelling_inlineembed1/parseError/Hauptgruppe.xml:111:" $OUTFILE > /dev/null || echo ERRORA28
grep ".*hierachical_modelling_inlineembed1/parseError/MBS.mbsim.xml:18:" $OUTFILE > /dev/null || echo ERRORA29
grep "included by .*hierachical_modelling_inlineembed1/parseError/MBS.mbsimprj.xml:8\[count=1]" $OUTFILE > /dev/null || echo ERRORA30

mbsimxml hierachical_modelling_inlineembed1/initXMLError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORA31
grep ".*hierachical_modelling_inlineembed1/initXMLError/Hauptgruppe.xml:55:" $OUTFILE > /dev/null || echo ERRORA32
grep "included by .*hierachical_modelling_inlineembed1/initXMLError/MBS.mbsim.xml:18\[count=1]" $OUTFILE > /dev/null || echo ERRORA33
grep "included by .*hierachical_modelling_inlineembed1/initXMLError/MBS.mbsimprj.xml:8\[count=1]" $OUTFILE > /dev/null || echo ERRORA34

mbsimxml hierachical_modelling_inlineembed1/mbsimError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORA35
grep ".*hierachical_modelling_inlineembed1/mbsimError/Hauptgruppe.xml:49:" $OUTFILE > /dev/null || echo ERRORA36
grep "included by .*hierachical_modelling_inlineembed1/mbsimError/MBS.mbsim.xml:18\[count=1]" $OUTFILE > /dev/null || echo ERRORA37
grep "included by .*hierachical_modelling_inlineembed1/mbsimError/MBS.mbsimprj.xml:8\[count=1]" $OUTFILE > /dev/null || echo ERRORA38

mbsimxml hierachical_modelling_inlineembed2/ppError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORA39
grep ".*hierachical_modelling_inlineembed2/ppError/submodel/Untergruppe.xml:11:" $OUTFILE > /dev/null || echo ERRORA40
grep "included by .*hierachical_modelling_inlineembed2/ppError/MBS.mbsim.xml:57\[count=1]" $OUTFILE > /dev/null || echo ERRORA41
grep "included by .*hierachical_modelling_inlineembed2/ppError/MBS.mbsimprj.xml:8\[count=1]" $OUTFILE > /dev/null || echo ERRORA42

mbsimxml hierachical_modelling_inlineembed2/parseError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORA43
grep ".*hierachical_modelling_inlineembed2/parseError/submodel/Untergruppe.xml:16:" $OUTFILE > /dev/null || echo ERRORA44
grep ".*hierachical_modelling_inlineembed2/parseError/submodel/Untergruppe.xml:72:" $OUTFILE > /dev/null || echo ERRORA45
grep ".*hierachical_modelling_inlineembed2/parseError/MBS.mbsim.xml:57:" $OUTFILE > /dev/null || echo ERRORA46
grep "included by .*hierachical_modelling_inlineembed2/parseError/MBS.mbsimprj.xml:8\[count=1]" $OUTFILE > /dev/null || echo ERRORA47

mbsimxml hierachical_modelling_inlineembed2/initXMLError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORA48
grep ".*hierachical_modelling_inlineembed2/initXMLError/submodel/Untergruppe.xml:16:" $OUTFILE > /dev/null || echo ERRORA49
grep "included by .*hierachical_modelling_inlineembed2/initXMLError/MBS.mbsim.xml:57\[count=1]" $OUTFILE > /dev/null || echo ERRORA50
grep "included by .*hierachical_modelling_inlineembed2/initXMLError/MBS.mbsimprj.xml:8\[count=1]" $OUTFILE > /dev/null || echo ERRORA51

mbsimxml hierachical_modelling_inlineembed2/mbsimError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORA52
grep ".*hierachical_modelling_inlineembed2/mbsimError/submodel/Untergruppe.xml:10:" $OUTFILE > /dev/null || echo ERRORA52
grep "included by .*hierachical_modelling_inlineembed2/mbsimError/MBS.mbsim.xml:57\[count=1]" $OUTFILE > /dev/null || echo ERRORA54
grep "included by .*hierachical_modelling_inlineembed2/mbsimError/MBS.mbsimprj.xml:8\[count=1]" $OUTFILE > /dev/null || echo ERRORA55



export MBXMLUTILS_ERROROUTPUT=HTMLXPATH
NS="{[^}]*}"

mbsimxml hierachical_modelling/ppError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORB1
grep "data-xpath=\"/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]/${NS}plotFeature\[1]/@value\".*hierachical_modelling/ppError/submodel/Untergruppe.xml</span>" $OUTFILE > /dev/null || echo ERRORB2
grep "included by .*data-xpath=\"/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2]\".*hierachical_modelling/ppError/Hauptgruppe.xml</span>" $OUTFILE > /dev/null || echo ERRORB3
grep "included by .*data-xpath=\"/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1]\".*hierachical_modelling/ppError/MBS.mbsim.xml</span>" $OUTFILE > /dev/null || echo ERRORB4
grep "included by .*data-xpath=\"/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1]\".*hierachical_modelling/ppError/MBS.mbsimprj.xml</span>" $OUTFILE > /dev/null || echo ERRORB5

mbsimxml hierachical_modelling/parseError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORB6
grep ".*data-xpath=\"/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]\".*hierachical_modelling/parseError/submodel/Untergruppe.xml</span>" $OUTFILE > /dev/null || echo ERRORB7
grep ".*data-xpath=\"/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]\".*hierachical_modelling/parseError/submodel/Untergruppe.xml</span>" $OUTFILE > /dev/null || echo ERRORB8
grep ".*data-xpath=\"/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2]\".*hierachical_modelling/parseError/Hauptgruppe.xml</span>" $OUTFILE > /dev/null || echo ERRORB9
grep "included by .*data-xpath=\"/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1]\".*hierachical_modelling/parseError/MBS.mbsim.xml</span>" $OUTFILE > /dev/null || echo ERRORB10
grep "included by .*data-xpath=\"/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1]\".*hierachical_modelling/parseError/MBS.mbsimprj.xml</span>" $OUTFILE > /dev/null || echo ERRORB11

mbsimxml hierachical_modelling/initXMLError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORB12
grep ".*data-xpath=\"/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]/${NS}mass\[1]\".*hierachical_modelling/initXMLError/submodel/Untergruppe.xml</span>" $OUTFILE > /dev/null || echo ERRORB13
grep "included by .*data-xpath=\"/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2]\".*hierachical_modelling/initXMLError/Hauptgruppe.xml</span>" $OUTFILE > /dev/null || echo ERRORB14
grep "included by .*data-xpath=\"/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1]\".*hierachical_modelling/initXMLError/MBS.mbsim.xml</span>" $OUTFILE > /dev/null || echo ERRORB15
grep "included by .*data-xpath=\"/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1]\".*hierachical_modelling/initXMLError/MBS.mbsimprj.xml</span>" $OUTFILE > /dev/null || echo ERRORB16

mbsimxml hierachical_modelling/mbsimError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORB17
grep ".*data-xpath=\"/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]\".*hierachical_modelling/mbsimError/submodel/Untergruppe.xml</span>" $OUTFILE > /dev/null || echo ERRORB18
grep "included by .*data-xpath=\"/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2]\".*hierachical_modelling/mbsimError/Hauptgruppe.xml</span>" $OUTFILE > /dev/null || echo ERRORB19
grep "included by .*data-xpath=\"/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1]\".*hierachical_modelling/mbsimError/MBS.mbsim.xml</span>" $OUTFILE > /dev/null || echo ERRORB20
grep "included by .*data-xpath=\"/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1]\".*hierachical_modelling/mbsimError/MBS.mbsimprj.xml</span>" $OUTFILE > /dev/null || echo ERRORB21

mbsimxml hierachical_modelling_inlineembed1/ppError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORB22
grep ".*data-xpath=\"/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2]/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]/${NS}plotFeature\[1]/@value\".*hierachical_modelling_inlineembed1/ppError/Hauptgruppe.xml</span>" $OUTFILE > /dev/null || echo ERRORB23
grep "included by .*data-xpath=\"/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1]\".*hierachical_modelling_inlineembed1/ppError/MBS.mbsim.xml</span>" $OUTFILE > /dev/null || echo ERRORB24
grep "included by .*data-xpath=\"/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1]\".*hierachical_modelling_inlineembed1/ppError/MBS.mbsimprj.xml</span>" $OUTFILE > /dev/null || echo ERRORB25

mbsimxml hierachical_modelling_inlineembed1/parseError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORB26
grep ".*data-xpath=\"/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2]/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]\".*hierachical_modelling_inlineembed1/parseError/Hauptgruppe.xml</span>" $OUTFILE > /dev/null || echo ERRORB27
grep ".*data-xpath=\"/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2]/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]\".*hierachical_modelling_inlineembed1/parseError/Hauptgruppe.xml</span>" $OUTFILE > /dev/null || echo ERRORB28
grep ".*data-xpath=\"/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1]\".*hierachical_modelling_inlineembed1/parseError/MBS.mbsim.xml</span>" $OUTFILE > /dev/null || echo ERRORB29
grep "included by .*data-xpath=\"/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1]\".*hierachical_modelling_inlineembed1/parseError/MBS.mbsimprj.xml</span>" $OUTFILE > /dev/null || echo ERRORB30

mbsimxml hierachical_modelling_inlineembed1/initXMLError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORB31
grep ".*data-xpath=\"/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2]/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]/${NS}mass\[1]\".*hierachical_modelling_inlineembed1/initXMLError/Hauptgruppe.xml</span>" $OUTFILE > /dev/null || echo ERRORB32
grep "included by .*data-xpath=\"/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1]\".*hierachical_modelling_inlineembed1/initXMLError/MBS.mbsim.xml</span>" $OUTFILE > /dev/null || echo ERRORB33
grep "included by .*data-xpath=\"/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1]\".*hierachical_modelling_inlineembed1/initXMLError/MBS.mbsimprj.xml</span>" $OUTFILE > /dev/null || echo ERRORB34

mbsimxml hierachical_modelling_inlineembed1/mbsimError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORB35
grep ".*data-xpath=\"/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2]/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]\".*hierachical_modelling_inlineembed1/mbsimError/Hauptgruppe.xml</span>" $OUTFILE > /dev/null || echo ERRORB36
grep "included by .*data-xpath=\"/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1]\".*hierachical_modelling_inlineembed1/mbsimError/MBS.mbsim.xml</span>" $OUTFILE > /dev/null || echo ERRORB37
grep "included by .*data-xpath=\"/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1]\".*hierachical_modelling_inlineembed1/mbsimError/MBS.mbsimprj.xml</span>" $OUTFILE > /dev/null || echo ERRORB38

mbsimxml hierachical_modelling_inlineembed2/ppError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORB39
grep ".*data-xpath=\"/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]/${NS}plotFeature\[1]/@value\".*hierachical_modelling_inlineembed2/ppError/submodel/Untergruppe.xml</span>" $OUTFILE > /dev/null || echo ERRORB40
grep "included by .*data-xpath=\"/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1]/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2]\".*hierachical_modelling_inlineembed2/ppError/MBS.mbsim.xml</span>" $OUTFILE > /dev/null || echo ERRORB41
grep "included by .*data-xpath=\"/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1]\".*hierachical_modelling_inlineembed2/ppError/MBS.mbsimprj.xml</span>" $OUTFILE > /dev/null || echo ERRORB42

mbsimxml hierachical_modelling_inlineembed2/parseError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORB43
grep ".*data-xpath=\"/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]\".*hierachical_modelling_inlineembed2/parseError/submodel/Untergruppe.xml</span>" $OUTFILE > /dev/null || echo ERRORB44
grep ".*data-xpath=\"/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]\".*hierachical_modelling_inlineembed2/parseError/submodel/Untergruppe.xml</span>" $OUTFILE > /dev/null || echo ERRORB45
grep ".*data-xpath=\"/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1]/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2]\".*hierachical_modelling_inlineembed2/parseError/MBS.mbsim.xml</span>" $OUTFILE > /dev/null || echo ERRORB46
grep "included by .*data-xpath=\"/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1]\".*hierachical_modelling_inlineembed2/parseError/MBS.mbsimprj.xml</span>" $OUTFILE > /dev/null || echo ERRORB47

mbsimxml hierachical_modelling_inlineembed2/initXMLError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORB48
grep ".*data-xpath=\"/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]/${NS}mass\[1]\".*hierachical_modelling_inlineembed2/initXMLError/submodel/Untergruppe.xml</span>" $OUTFILE > /dev/null || echo ERRORB49
grep "included by .*data-xpath=\"/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1]/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2]\".*hierachical_modelling_inlineembed2/initXMLError/MBS.mbsim.xml</span>" $OUTFILE > /dev/null || echo ERRORB50
grep "included by .*data-xpath=\"/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1]\".*hierachical_modelling_inlineembed2/initXMLError/MBS.mbsimprj.xml</span>" $OUTFILE > /dev/null || echo ERRORB51

mbsimxml hierachical_modelling_inlineembed2/mbsimError/MBS.mbsimprj.xml &> $OUTFILE
test $? -eq 0 && echo ERRORB52
grep ".*data-xpath=\"/${NS}Group\[1]/${NS}objects\[1]/${NS}RigidBody\[1]\".*hierachical_modelling_inlineembed2/mbsimError/submodel/Untergruppe.xml</span>" $OUTFILE > /dev/null || echo ERRORB52
grep "included by .*data-xpath=\"/${NS}DynamicSystemSolver\[1]/${NS}groups\[1]/${NS}Embed\[1]/${NS}Group\[1]/${NS}groups\[1]/${NS}Embed\[2]\".*hierachical_modelling_inlineembed2/mbsimError/MBS.mbsim.xml</span>" $OUTFILE > /dev/null || echo ERRORB54
grep "included by .*data-xpath=\"/${NS}Embed\[1]/${NS}MBSimProject\[1]/${NS}Embed\[1]\".*hierachical_modelling_inlineembed2/mbsimError/MBS.mbsimprj.xml</span>" $OUTFILE > /dev/null || echo ERRORB55



mbsimxml hierachical_modelling/noError/MBS.mbsimprj.xml &> $OUTFILE
test $? -ne 0 && echo ERRORC56

mbsimxml hierachical_modelling_inlineembed1/noError/MBS.mbsimprj.xml &> $OUTFILE
test $? -ne 0 && echo ERRORC57

mbsimxml hierachical_modelling_inlineembed2/noError/MBS.mbsimprj.xml &> $OUTFILE
test $? -ne 0 && echo ERRORC58



rm $OUTFILE
