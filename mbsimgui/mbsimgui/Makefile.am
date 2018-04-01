include $(top_srcdir)/qt.mk

if COND_WIN32
  MAYBE_WIN32_SRC = mbsimgui.rc
endif

bin_PROGRAMS = mbsimgui

mbsimguidir = $(includedir)/mbsimgui

mbsimgui_CPPFLAGS = $(MBXMLUTILS_CFLAGS) $(OPENMBV_CFLAGS) $(QWT_CFLAGS)

mbsimgui_LDFLAGS = $(HDF5SERIE_LIBS) $(MBXMLUTILS_LIBS) $(OPENMBV_LIBS) $(QWT_LIBS)

mbsimgui_SOURCES = $(MAYBE_WIN32_SRC) mainwindow.cc \
		   options.cc \
		   element.cc \
		   frame.cc \
		   contour.cc \
		   group.cc \
		   dynamic_system_solver.cc \
		   body.cc \
		   rigid_body.cc \
		   flexible_body_ffr.cc \
		   observer.cc \
		   kinetic_excitation.cc \
		   spring_damper.cc \
		   integrator.cc \
		   parameter.cc \
		   objectfactory.cc \
		   embeditemdata.cc \
		   treeitem.cc \
		   treemodel.cc \
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
		   property_dialog.cc \
		   element_property_dialog.cc \
		   solver_property_dialog.cc \
		   parameter_property_dialog.cc \
		   embedding_property_dialog.cc \
		   project_property_dialog.cc \
		   element_context_menu.cc \
		   parameter_context_menu.cc \
		   embedding_context_menu.cc \
		   element_view.cc \
		   embedding_view.cc \
		   solver_view.cc \
		   project_view.cc \
		   echo_view.cc \
		   dialogs.cc  \
		   utils.cc \
		   main.cc \
		   embed.cc \
		   solver.cc \
		   project.cc \
		   octave_utils.cc \
		   data_plot.cc \
		   file_editor.cc

nodist_mbsimgui_SOURCES=$(QT_BUILT_SOURCES)

# C++ source file to build automatically (e.g. abc.moc.cc by abc.h)
QT_BUILT_SOURCES = mainwindow.moc.cc \
		   mbsimgui.qrc.cc \
		   options.moc.cc \
		   widget.moc.cc \
		   basic_widgets.moc.cc \
		   variable_widgets.moc.cc \
		   function_widgets.moc.cc \
		   extended_widgets.moc.cc \
		   special_widgets.moc.cc \
		   property_dialog.moc.cc  \
		   element_property_dialog.moc.cc \
		   element_context_menu.moc.cc \
		   embedding_context_menu.moc.cc \
		   element_view.moc.cc \
		   embedding_view.moc.cc \
		   solver_view.moc.cc \
		   project_view.moc.cc \
		   echo_view.moc.cc \
		   dialogs.moc.cc  \
		   data_plot.moc.cc

BUILT_SOURCES = $(QT_BUILT_SOURCES)

mbsimgui_HEADERS = 
		   
		   
		   
mbsimguisharedir = @datadir@/mbsimgui
dist_mbsimguishare_DATA = inlineopenmbv.ombv.wst \
			  empty.ombv.xml \
			  empty.ombv.h5

mbsimguiicondir = @datadir@/mbsimgui/icons
dist_mbsimguiicon_DATA = icons/simulate.svg \
			 icons/preview.svg \
			 icons/openmbv.svg \
			 icons/h5plotserie.svg \
			 icons/eigenanalysis.svg \
			 icons/resize.svg \
			 icons/workdir.svg \
			 icons/debug.svg \
			 icons/error.svg \
			 icons/warn.svg \
			 icons/info.svg \
			 icons/deprecated.svg \
			 icons/debugBlue.svg \
			 icons/debugBlueEnable.svg \
			 icons/kill.svg \
			 icons/mbsimgui.svg \
			 icons/frequency_response.svg

# plugins
pluginsiconenginesdir = @bindir@/iconengines
pluginsiconengines_DATA = @QT_PLUGINS@/iconengines/*qsvgicon*
pluginsplatformsdir = @bindir@/platforms
if COND_WIN32
pluginsplatforms_DATA = @QT_PLUGINS@/platforms/qwindows.dll
else
pluginsplatforms_DATA = @QT_PLUGINS@/platforms/libqxcb.so
pluginsxcbglintegrationsdir = @bindir@/xcbglintegrations
pluginsxcbglintegrations_DATA = @QT_PLUGINS@/xcbglintegrations/*qxcb*
endif

# some dependencies for mbsimgui.qrc.cc
mbsimgui.qrc.cc: mbsimgui.qrc icons/mbsimgui.svg

# freedesktop.org
freedesktoporgdir = @datadir@/mbsim-env/freedesktop.org
freedesktoporg_DATA = mbsim-env.org.mbsimgui.desktop mimeapps-mbsimgui.list icons/mbsimgui.svg
bin_SCRIPTS = desktopIntegration.sh
