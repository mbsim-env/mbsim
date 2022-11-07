include $(top_srcdir)/qt.mk

if COND_WIN32
  MAYBE_WIN32_SRC = mbsimgui.rc
  EXPORT_ALL_SYMBOLS = -Xlinker --export-all-symbols
else
  MAYBE_SIGWATCH_SRC = qt-unix-signals/sigwatch.cpp
  MAYBE_SIGWATCH_MOC = qt-unix-signals/sigwatch.moc.cc
  EXPORT_ALL_SYMBOLS = 
endif

licdir = @datadir@/mbsimgui/qt-unix-signals
lic_DATA = qt-unix-signals/LICENCE

lib_LTLIBRARIES = libmbsimgui.la
bin_PROGRAMS = mbsimgui

mbsimguidir = $(includedir)/mbsimgui

libmbsimgui_la_CPPFLAGS = $(MBXMLUTILS_CFLAGS) $(OPENMBV_CFLAGS) $(QWT_CFLAGS)
libmbsimgui_la_LDFLAGS = $(MBXMLUTILS_LIBS) $(OPENMBV_LIBS) $(QWT_LIBS) $(EXPORT_ALL_SYMBOLS)
libmbsimgui_la_SOURCES = \
  parameter_view.cc \
  file_view.cc \
  element_view.cc \
  options.cc \
  basicitemdata.cc \
  body.cc \
  embed.cc \
  dialogs.cc  \
  wizards.cc  \
  unknown_widget.cc \
  basic_widgets.cc \
  widget.cc \
  variable_widgets.cc \
  function_widget.cc \
  function_widgets.cc \
  kinematic_functions_widgets.cc \
  function_widget_factory.cc \
  ombv_widgets.cc \
  extended_widgets.cc \
  special_widgets.cc \
  property_dialog.cc \
  solver_property_dialog.cc \
  parameter_property_dialog.cc \
  project_property_dialog.cc \
  xml_property_dialog.cc \
  clone_property_dialog.cc \
  element_property_dialog.cc \
  element.cc \
  objectfactory.cc \
  solver_context_menu.cc \
  project_context_menu.cc \
  element_context_menu.cc \
  parameter_context_menu.cc \
  parameters_context_menu.cc \
  dynamic_system_solver.cc \
  project.cc \
  treeitem.cc \
  treemodel.cc \
  utils.cc \
  echo_view.cc \
  mainwindow.cc \
  fileitemdata.cc \
  parameter.cc \
  embeditemdata.cc \
  analyzer.cc \
  clutch.cc \
  connection.cc \
  constraint.cc \
  contact.cc \
  contour.cc \
  environment_widgets.cc \
  flexible_ffr_body.cc \
  frame.cc \
  friction.cc \
  gear.cc \
  group.cc \
  integrator.cc \
  joint.cc \
  kinetic_excitation.cc \
  kinetics_widgets.cc \
  link.cc \
  object.cc \
  observer.cc \
  physics.cc \
  rigid_body.cc \
  sensor.cc \
  signal_.cc \
  solver.cc \
  spring_damper.cc \
  structure.cc \
  fbt_utils.cc \
  fbt_cms.cc \
  fbt_msm.cc \
  fbt_extfe.cc \
  fbt_calculix.cc \
  fbt_beam.cc \
  fbt_fe.cc \
  fbt_fma.cc \
  fbt_lma.cc \
  fbt_ombv.cc \
  fbt_damp.cc \
  fbt_exp.cc
nodist_libmbsimgui_la_SOURCES=$(libmbsimgui_QT_BUILT_SOURCES)

mbsimgui_CPPFLAGS = $(OPENMBV_CFLAGS) $(QWT_CFLAGS)

mbsimgui_LDFLAGS = $(OPENMBV_LIBS) $(QWT_LIBS)
mbsimgui_LDADD = libmbsimgui.la

mbsimgui_SOURCES = $(MAYBE_WIN32_SRC) $(MAYBE_SIGWATCH_SRC) \
  main.cc \
  file_editor.cc

nodist_mbsimgui_SOURCES=$(mbsimgui_QT_BUILT_SOURCES)

mbsimguiincludedir = $(includedir)/mbsimgui
mbsimguiinclude_HEADERS = analyzer.h \
  basicitemdata.h \
  basic_widgets.h \
  body.h \
  clone_property_dialog.h \
  clutch.h \
  connection.h \
  constraint.h \
  contact.h \
  contour.h \
  custom_widgets.h \
  dialogs.h \
  wizards.h \
  dynamic_system_solver.h \
  echo_view.h \
  element_context_menu.h \
  element.h \
  element_property_dialog.h \
  element_view.h \
  embed.h \
  embeditemdata.h \
  environment_widgets.h \
  exporter.h \
  extended_widgets.h \
  file_editor.h \
  fileitemdata.h \
  file_view.h \
  flexible_ffr_body.h \
  frame.h \
  friction.h \
  function.h \
  function_widget_factory.h \
  function_widget.h \
  function_widgets.h \
  gear.h \
  group.h \
  integrator.h \
  joint.h \
  kinematic_functions_widgets.h \
  kinematics_widgets.h \
  kinetic_excitation.h \
  kinetics_widgets.h \
  link_.h \
  mainwindow.h \
  namespace.h \
  objectfactory.h \
  object.h \
  observer.h \
  ombv_widgets.h \
  options.h \
  parameter_context_menu.h \
  parameter.h \
  parameter_property_dialog.h \
  parameters_context_menu.h \
  parameter_view.h \
  physics.h \
  project_context_menu.h \
  project.h \
  project_property_dialog.h \
  property_dialog.h \
  rigid_body.h \
  sensor.h \
  signal_.h \
  solver_context_menu.h \
  solver.h \
  solver_property_dialog.h \
  special_widgets.h \
  spring_damper.h \
  structure.h \
  treeitemdata.h \
  treeitem.h \
  treemodel.h \
  unknown_widget.h \
  utils.h \
  variable_widgets.h \
  widget.h \
  xml_property_dialog.h

# C++ source file to build automatically (e.g. abc.moc.cc by abc.h)
libmbsimgui_QT_BUILT_SOURCES = \
  widget.moc.cc \
  basic_widgets.moc.cc \
  variable_widgets.moc.cc \
  extended_widgets.moc.cc \
  property_dialog.moc.cc \
  mainwindow.moc.cc
mbsimgui_QT_BUILT_SOURCES = 

BUILT_SOURCES = $(libmbsimgui_QT_BUILT_SOURCES) $(mbsimgui_QT_BUILT_SOURCES) $(MAYBE_SIGWATCH_MOC)

mbsimguisharedir = @datadir@/mbsimgui
dist_mbsimguishare_DATA = inlineopenmbv.ombvwst \
  MBS_tmp.ombvx \
  MBS_tmp.ombvh5

mbsimguiicondir = @datadir@/mbsimgui/icons
dist_mbsimguiicon_DATA = icons/openmbv.svg \
  icons/h5plotserie.svg \
  icons/eigenanalysis.svg \
  icons/debug.svg \
  icons/error.svg \
  icons/warn.svg \
  icons/info.svg \
  icons/deprecated.svg \
  icons/debugBlue.svg \
  icons/debugBlueEnable.svg \
  icons/kill.svg \
  icons/mbsimgui.svg \
  icons/frequency_response.svg \
  icons/state_table.svg \
  icons/FMI_bare.svg \
  icons/frame.svg \
  icons/dss.svg \
  icons/project.svg \
  icons/solver.svg \
  icons/container.svg \
  icons/group.svg \
  icons/body.svg \
  icons/link.svg \
  icons/constraint.svg \
  icons/observer.svg \
  icons/contour.svg \
  icons/scalar.svg \
  icons/vector.svg \
  icons/matrix.svg \
  icons/string.svg \
  icons/fbt.svg

# plugins
pluginsiconenginesdir = @bindir@/iconengines
pluginsplatformsdir = @bindir@/platforms
pluginsstylesdir = @bindir@/styles
pluginsplatformthemesdir = @bindir@/platformthemes
pluginsiconengines_DATA = @QT_PLUGINS@/iconengines/*qsvgicon*
if COND_WIN32
pluginsplatforms_DATA = @QT_PLUGINS@/platforms/qwindows.dll
pluginsstyles_DATA = @QT_PLUGINS@/styles/qwindowsvistastyle.dll
else
pluginsplatforms_DATA = @QT_PLUGINS@/platforms/libqxcb.so
pluginsstyles_DATA = @QT_PLUGINSSTYLES_DATA@
pluginsplatformthemes_DATA = @QT_PLUGINSPLATFORMTHEMES_DATA@
pluginsxcbglintegrationsdir = @bindir@/xcbglintegrations
pluginsxcbglintegrations_DATA = @QT_PLUGINS@/xcbglintegrations/*qxcb*
endif

# freedesktop.org
freedesktoporgdir = @datadir@/mbsim-env/freedesktop.org
windowsdir = @datadir@/mbsim-env/windows
freedesktoporg_DATA = mbsim-env.de.mbsimgui.desktop mimeapps-mbsimgui.list icons/mbsimgui.svg
windows_DATA = createLnk.vbs mbsim-env.de.mbsimgui.source.bat
bin_SCRIPTS = desktopIntegration.sh desktopIntegration.bat



# this library is not installed, its just to test a MBSimGUI plugin
plugindir = $(abs_builddir)
plugin_LTLIBRARIES = libmbsimgui-plugin-testrigidbody.la
libmbsimgui_plugin_testrigidbody_la_CPPFLAGS = $(MBXMLUTILS_CFLAGS) $(OPENMBV_CFLAGS) $(QWT_CFLAGS)
libmbsimgui_plugin_testrigidbody_la_LIBADD = libmbsimgui.la $(MBXMLUTILS_LIBS) $(OPENMBV_LIBS) $(QWT_LIBS)
libmbsimgui_plugin_testrigidbody_la_LDFLAGS = -avoid-version
libmbsimgui_plugin_testrigidbody_la_SOURCES = plugin-testrigidbody.cc
