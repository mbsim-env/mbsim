set shell=CreateObject("WScript.Shell")
set sc=shell.CreateShortcut(WScript.Arguments.Item(0))
sc.TargetPath=WScript.Arguments.Item(1)
sc.Save
