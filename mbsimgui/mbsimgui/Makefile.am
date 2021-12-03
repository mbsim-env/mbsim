include $(top_srcdir)/qt.mk

if COND_WIN32
  MAYBE_WIN32_SRC = mbsimgui.rc
else
  MAYBE_SIGWATCH_SRC = qt-unix-signals/sigwatch.cpp
  MAYBE_SIGWATCH_MOC = qt-unix-signals/sigwatch.moc.cc
endif

licdir = @datadir@/mbsimgui/qt-unix-signals
lic_DATA = qt-unix-signals/LICENCE

bin_PROGRAMS = mbsimgui

mbsimguidir = $(includedir)/mbsimgui

mbsimgui_CPPFLAGS = $(MBXMLUTILS_CFLAGS) $(OPENMBV_CFLAGS) $(QWT_CFLAGS)

mbsimgui_LDFLAGS = $(HDF5SERIE_LIBS) $(MBXMLUTILS_LIBS) $(OPENMBV_LIBS) $(QWT_LIBS)

mbsimgui_SOURCES = $(MAYBE_WIN32_SRC) $(MAYBE_SIGWATCH_SRC) mainwindow.cc \
		   options.cc \
		   element.cc \
		   frame.cc \
		   contour.cc \
		   group.cc \
		   dynamic_system_solver.cc \
		   body.cc \
		   rigid_body.cc \
		   flexible_ffr_body.cc \
		   link.cc \
		   constraint.cc \
		   observer.cc \
		   kinetic_excitation.cc \
		   spring_damper.cc \
		   physics.cc \
		   integrator.cc \
		   parameter.cc \
		   objectfactory.cc \
		   basicitemdata.cc \
		   embeditemdata.cc \
		   fileitemdata.cc \
		   treeitem.cc \
		   treemodel.cc \
		   unknown_widget.cc \
		   basic_widgets.cc \
		   variable_widgets.cc \
		   function_widget.cc \
		   function_widgets.cc \
		   kinematic_functions_widgets.cc \
		   function_widget_factory.cc \
		   kinetics_widgets.cc \
		   ombv_widgets.cc \
		   extended_widgets.cc \
		   special_widgets.cc \
		   environment_widgets.cc \
		   property_dialog.cc \
		   element_property_dialog.cc \
		   solver_property_dialog.cc \
		   parameter_property_dialog.cc \
		   project_property_dialog.cc \
		   xml_property_dialog.cc \
		   clone_property_dialog.cc \
		   element_context_menu.cc \
		   parameter_context_menu.cc \
		   parameters_context_menu.cc \
		   element_view.cc \
		   parameter_view.cc \
		   solver_context_menu.cc \
		   project_context_menu.cc \
		   echo_view.cc \
		   file_view.cc \
		   dialogs.cc  \
		   utils.cc \
		   main.cc \
		   embed.cc \
		   solver.cc \
		   project.cc \
		   file_editor.cc

nodist_mbsimgui_SOURCES=$(QT_BUILT_SOURCES)

# C++ source file to build automatically (e.g. abc.moc.cc by abc.h)
QT_BUILT_SOURCES = widget.moc.cc \
		   basic_widgets.moc.cc \
		   variable_widgets.moc.cc \
		   extended_widgets.moc.cc \
		   property_dialog.moc.cc  \
		   mainwindow.moc.cc

BUILT_SOURCES = $(QT_BUILT_SOURCES) $(MAYBE_SIGWATCH_MOC)

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
			 icons/string.svg

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
