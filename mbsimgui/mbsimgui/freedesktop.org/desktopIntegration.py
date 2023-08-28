#!/usr/bin/env python3

try:
  import os
  import glob
  import shutil
  import subprocess
  import platform
  import configparser
  
  # Install all mbsim-env freedesktop.org modules

  print("After finishing this script a relogin may be needed to apply all changes!")
  print("")
  
  inp=input("Create shortcuts on desktop [Y/n]: ")
  copyToDesktop=inp=="" or inp=="y" or inp=="Y"
  
  inp=input("Create shortcuts in start menu [Y/n]: ")
  copyToMenu=inp=="" or inp=="y" or inp=="Y"
  
  inp=input("Add executables to PATH [Y/n]: ")
  addToPATH=inp=="" or inp=="y" or inp=="Y"
  
  
  
  if platform.system()=="Linux":
    # LINUX
  
  
  
    # source dirs
    PREFIX=f"{os.path.dirname(__file__)}/.."
    FREEDESKTOPORGDIR=f"{PREFIX}/share/mbsim-env/freedesktop.org"
    BINDIR=f"{PREFIX}/bin"
    HOME=os.environ["HOME"]
    
    # destination dirs
    DATAHOME=os.environ.get("XDG_DATA_HOME", f"{HOME}/.local/share")
    CONFIG=os.environ.get("XDG_CONFIG_HOME", f"{HOME}/.config")
    DESKTOP=os.environ.get("XDG_DESKTOP_DIR", f"{HOME}/Desktop")
    if os.path.isfile(f"{CONFIG}/user-dirs.dirs"):
      with open(f"{CONFIG}/user-dirs.dirs", "rt") as ff:
        for line in ff.readlines():
          if line.startswith('XDG_DESKTOP_DIR="$HOME'):
            DESKTOP=HOME+line[len('XDG_DESKTOP_DIR="$HOME'):-2]
            break
    
    # svgs
    for F in glob.glob(f"{DATAHOME}/icons/hicolor/scalable/apps/mbsim-env.*.svg"):
      os.unlink(F)
    os.makedirs(f"{DATAHOME}/icons/hicolor/scalable/apps", exist_ok=True)
    for F in glob.glob(f"{FREEDESKTOPORGDIR}/*.svg"):
      shutil.copyfile(F, f"{DATAHOME}/icons/hicolor/scalable/apps/mbsim-env.{os.path.basename(F)}")
    
    # mimeapps
    os.makedirs(f"{HOME}/.config", exist_ok=True)
    cp=configparser.ConfigParser(delimiters=('='), comment_prefixes=('#'), strict=False, interpolation=None)
    cp.optionxform=str
    cp.read([f"{HOME}/.config/mimeapps.list"]+glob.glob(f"{FREEDESKTOPORGDIR}/mimeapps-*.list"))
    with open(f"{HOME}/.config/mimeapps.list", "wt") as ff:
      cp.write(ff, space_around_delimiters=False)
    
    # apps
    for F in glob.glob(f"{DATAHOME}/applications/mbsim-env.*.desktop")+glob.glob(f"{DESKTOP}/mbsim-env.*.desktop"):
      os.unlink(F)
    os.makedirs(f"{DATAHOME}/applications", exist_ok=True)
    for F in glob.glob(f"{FREEDESKTOPORGDIR}/mbsim-env.*.desktop"):
      cp=configparser.ConfigParser(delimiters=('='), comment_prefixes=('#'), strict=False, interpolation=None)
      cp.optionxform=str
      cp.read(F)
      de=cp["Desktop Entry"]
      for key in de:
        de[key]=de[key].replace("@bindir@", BINDIR)
      if not copyToMenu:
        de["NoDisplay"]="true"
      with open(f"{DATAHOME}/applications/{os.path.basename(F)}", "wt") as ff:
        cp.write(ff, space_around_delimiters=False)
      if de.get("NoDisplay", "false")!="true" and copyToDesktop:
        shutil.copyfile(f"{DATAHOME}/applications/{os.path.basename(F)}", f"{DESKTOP}/{os.path.basename(F)}")
        os.chmod(f"{DESKTOP}/{os.path.basename(F)}", mode=0o755)
    
    # mime types
    for F in glob.glob(f"{DATAHOME}/mime/packages/mbsim-env.*.xml"):
      os.unlink(F)
    os.makedirs(f"{DATAHOME}/mime/packages", exist_ok=True)
    for F in glob.glob(f"{FREEDESKTOPORGDIR}/mbsim-env.*.xml"):
      shutil.copyfile(F, f"{DATAHOME}/mime/packages/{os.path.basename(F)}")
    subprocess.check_call(["update-mime-database", f"{DATAHOME}/mime"])

    if addToPATH:
      def addPath(file):
        skip=False
        with open(file, "rt") as f:
          for line in f:
            if line.rstrip()=="export PATH=$PATH:"+BINDIR:
              skip=True
              break
        if not skip:
          with open(file, "at") as f:
            print("export PATH=$PATH:"+BINDIR, file=f)
      if os.path.isfile(f"{HOME}/.bashrc"):
        addPath(f"{HOME}/.bashrc")
      if os.path.isfile(f"{HOME}/.kshrc"):
        addPath(f"{HOME}/.kshrc")
  
  
  
  else:
    # WINDOWS
    import xml.etree.ElementTree
    import tempfile
    import winreg
  
  
  
    # source dirs
    PREFIX=f"{os.path.dirname(__file__)}/.."
    WINDOWSDIR=f"{PREFIX}/share/mbsim-env/windows"
    BINDIR=f"{PREFIX}/bin"
    FREEDESKTOPORGDIR=f"{PREFIX}/share/mbsim-env/freedesktop.org"
  
    # destination dirs
    STARTMENU=f"{os.environ['APPDATA']}/Microsoft/Windows/Start Menu/Programs"
    DESKTOP=f"{os.environ['USERPROFILE']}/Desktop"
  
    # mimetype -> extension
    mimeTypeExt={}
    for F in glob.glob(f"{FREEDESKTOPORGDIR}/mbsim-env.*.xml"):
      for mimeTypeEle in xml.etree.ElementTree.parse(F).getroot().findall("{http://www.freedesktop.org/standards/shared-mime-info}mime-type"):
        for globEle in mimeTypeEle.findall("{http://www.freedesktop.org/standards/shared-mime-info}glob"):
          mimeTypeExt.setdefault(mimeTypeEle.attrib["type"], []).append(globEle.attrib["pattern"])
  
    # default associations
    cp=configparser.ConfigParser(delimiters=('='), comment_prefixes=('#'), strict=False, interpolation=None)
    cp.optionxform=str
    cp.read(glob.glob(f"{FREEDESKTOPORGDIR}/mimeapps-*.list"))
    da=cp["Default Applications"]
  
    def createShortcut(shortcutFile, target, iconFile, workingDir=''):
      os.makedirs(os.path.dirname(shortcutFile), exist_ok=True)
      shortcutFileForward=shortcutFile.replace("\\", "/")
      targetForward=target.replace("\\", "/")
      workingDirForward=workingDir.replace("\\", "/")
      iconFileForward=iconFile.replace("\\", "/")
      vb=fr'''set shell=CreateObject("WScript.Shell")
  set sc=shell.CreateShortcut("{shortcutFileForward}")
  sc.TargetPath="{targetForward}"
  sc.WorkingDirectory="{workingDirForward}"
  sc.IconLocation="{iconFileForward}"
  sc.Save
  '''
      fd, path=tempfile.mkstemp('.vbs')
      try:
        with os.fdopen(fd, 'w') as ff:
          ff.write(vb)
        subprocess.check_call(['cscript', '/B', path])
      finally:
        os.unlink(path)

    def deleteKey(root, sub):
      try:
        with winreg.OpenKey(root, sub, 0, winreg.KEY_ALL_ACCESS) as key:
          while True:
            num, _, _=winreg.QueryInfoKey(key)
            if num==0:
              break
            child=winreg.EnumKey(key, 0)
            deleteKey(key, child)
          winreg.DeleteKey(key, "")
      except FileNotFoundError:
        pass
  
    # registry
    with winreg.CreateKey(winreg.HKEY_CURRENT_USER, r'Software\Classes') as classes:
      for F in glob.glob(f"{FREEDESKTOPORGDIR}/mbsim-env.*.desktop"):
        progID=os.path.splitext(os.path.basename(F))[0]
        cp=configparser.ConfigParser(delimiters=('='), comment_prefixes=('#'), strict=False, interpolation=None)
        cp.optionxform=str
        cp.read(F)
        de=cp["Desktop Entry"]
        for key in de:
          de[key]=de[key].replace("@bindir@", BINDIR)
        for section in cp:
          if section.startswith("Windows-shell-"):
            ws=cp[section]
            for key in ws:
              ws[key]=ws[key].replace("@bindir@", BINDIR)
        if de["Type"]=="Application":
          addedAssociation=False
          for mimeType in de["MimeType"].split(";"):
            if mimeType=="":
              continue
            desktopfiles=da.get(mimeType, ";").split(";")
            if len(desktopfiles)!=2:
              raise RuntimeError("only one desktop file per mimetype "+mimeType+" is supported");
            if desktopfiles[0]!=os.path.basename(F):
              addedAssociation=True
              continue
            for ext in mimeTypeExt[mimeType]:
              if not ext.startswith("*."):
                raise RuntimeError("glob "+ext+" for mimetype "+mimeType+" is not supported");
              winreg.SetValue(classes, ext[1:], winreg.REG_SZ, progID)
          if not addedAssociation:
            deleteKey(classes, progID)
            winreg.SetValue(classes, progID, winreg.REG_SZ, de["Name"])
            winreg.SetValue(classes, progID+r'\FriendlyTypeName', winreg.REG_SZ, de["Name"])
            winreg.SetValue(classes, progID+r'\Infotipp', winreg.REG_SZ, de["Comment"])
            winreg.SetValue(classes, progID+r'\DefaultIcon', winreg.REG_SZ, de["IconWindows"])
            winreg.SetValue(classes, progID+r'\shell\open', winreg.REG_SZ, fr'Open with {de["Name"]}')
            winreg.SetValue(classes, progID+r'\shell\open\command', winreg.REG_SZ, de["ExecWindows"])
            for section in cp:
              if section.startswith("Windows-shell-"):
                ws=cp[section]
                winreg.SetValue(classes, progID+fr'\shell\{ws["ID"]}', winreg.REG_SZ, ws["Name"])
                winreg.SetValue(classes, progID+fr'\shell\{ws["ID"]}\command', winreg.REG_SZ, ws["Exec"])
  
            execFile=de["Exec"].split(" ")[0]
            if execFile[0]=='"' and execFile[-1]=='"' or execFile[0]=="'" and execFile[-1]=="'":
              execFile=execFile[1:-1]
  
            # desktop
            if copyToDesktop:
              createShortcut(fr'{DESKTOP}/{de["Name"]}.lnk', execFile, execFile+".exe,0")
  
            # start menu
            if copyToMenu:
              createShortcut(fr'{STARTMENU}/{de["Name"]}.lnk', execFile, execFile+".exe,0")
        if de["Type"]=="Link":
          url=de["URL"]
          if url.startswith("file://"):
            url=url[7:]
          # desktop
          if copyToDesktop:
            createShortcut(fr'{DESKTOP}/{de["Name"]}.lnk', url, de["IconWindows"])
  
          # start menu
          if copyToMenu:
            createShortcut(fr'{STARTMENU}/{de["Name"]}.lnk', url, de["IconWindows"])

    if addToPATH:
      with winreg.CreateKey(winreg.HKEY_CURRENT_USER, r'Environment') as env:
        (PATH, PATHtype)=winreg.QueryValueEx(env, "PATH")
        PATH=PATH.split(";")
        if BINDIR not in PATH:
          PATH.append(BINDIR)
          winreg.SetValueEx(env, "PATH", 0, PATHtype, ";".join(PATH))
  
    # update cache (if possible)
    if shutil.which("ie4uinit.exe") is not None:
      subprocess.check_call(["ie4uinit.exe", "-show"])



except:
  import traceback
  traceback.print_exc()
  input("The above errors occured. Press enter to close: ")
