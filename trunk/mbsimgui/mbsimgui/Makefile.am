include $(top_srcdir)/qt.mk

bin_PROGRAMS = mbsimgui

mbsimguidir = $(includedir)/mbsimgui

INCLUDES = -DTIXML_USE_STL $(QT_CFLAGS) $(OPENMBV_CFLAGS)

LDADD = $(QT_LIBS) $(OPENMBV_LIBS) 

mbsimgui_CPPFLAGS = $(MBXMLUTILS_CFLAGS)

mbsimgui_LDFLAGS = $(HDF5SERIE_LIBS) $(MBXMLUTILS_LIBS) -Wl,-rpath -Wl,\$$ORIGIN/../lib

mbsimgui_SOURCES = element.cc \
		   frame.cc \
		   contour.cc \
		   group.cc \
		   solver.cc \
		   object.cc \
		   body.cc \
		   rigidbody.cc \
		   constraint.cc \
		   link.cc \
		   kinetic_excitation.cc \
		   spring_damper.cc \
		   joint.cc \
		   contact.cc \
		   signal_.cc \
		   observer.cc \
		   integrator.cc \
		   parameter.cc \
		   octaveutils.cc \
		   objectfactory.cc \
		   treeitem.cc \
		   treemodel.cc \
		   delegate.cc \
		   basic_properties.cc \
		   string_properties.cc \
		   kinematics_properties.cc \
		   kinetics_properties.cc \
		   function_properties.cc \
		   ombv_properties.cc \
		   extended_properties.cc \
		   integrator_properties.cc \
		   basic_widgets.cc \
		   string_widgets.cc \
		   function_widgets.cc \
		   kinematics_widgets.cc \
		   kinetics_widgets.cc \
		   ombv_widgets.cc \
		   extended_widgets.cc \
		   integrator_widgets.cc \
		   property_dialog.cc \
		   element_property_dialog.cc \
		   integrator_property_dialog.cc \
		   parameter_property_dialog.cc \
		   element_context_menu.cc \
		   parameter_context_menu.cc \
		   integrator_view.cc \
		   dialogs.cc  \
		   utils.cc \
		   mainwindow.cc \
		   main.cc
		   

nodist_mbsimgui_SOURCES=$(QT_BUILT_SOURCES)

# C++ source file to build automatically (e.g. abc.moc.cc by abc.h)
QT_BUILT_SOURCES = basic_widgets.moc.cc \
		   string_widgets.moc.cc \
		   function_widgets.moc.cc \
		   kinematics_widgets.moc.cc \
		   kinetics_widgets.moc.cc \
		   ombv_widgets.moc.cc \
		   extended_widgets.moc.cc \
		   property_dialog.moc.cc  \
		   element_property_dialog.moc.cc \
		   integrator_property_dialog.moc.cc \
		   parameter_property_dialog.moc.cc \
		   element_context_menu.moc.cc \
		   integrator_view.moc.cc \
		   dialogs.moc.cc  \
		   integrator.moc.cc \
		   delegate.moc.cc \
		   parameter.moc.cc \
		   mainwindow.moc.cc

mbsimgui_HEADERS = 
		   
		   
		   
mbsimguisharedir = @datadir@/mbsimgui
dist_mbsimguishare_DATA = inlineopenmbv.ombv.wst \
			  empty.ombv.xml \
			  empty.ombv.h5

mbsimguiicondir = @datadir@/mbsimgui/icons
dist_mbsimguiicon_DATA = icons/exit.svg \
			 icons/simulate.svg \
			 icons/preview.svg \
			 icons/openmbv.svg \
			 icons/h5plotserie.svg \
			 icons/resize.svg \
			 icons/workdir.svg

