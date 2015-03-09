include $(top_srcdir)/qt.mk

bin_PROGRAMS = mbsimgui

mbsimguidir = $(includedir)/mbsimgui

INCLUDES = $(QT_CFLAGS) $(OPENMBV_CFLAGS)

LDADD = $(QT_LIBS) $(OPENMBV_LIBS) 

mbsimgui_CPPFLAGS = $(MBXMLUTILS_CFLAGS)

mbsimgui_LDFLAGS = $(HDF5SERIE_LIBS) $(MBXMLUTILS_LIBS)

mbsimgui_SOURCES = mainwindow.cc \
		   options.cc \
		   element.cc \
		   frame.cc \
		   contour.cc \
		   group.cc \
		   dynamic_system_solver.cc \
		   object.cc \
		   body.cc \
		   rigidbody.cc \
		   constraint.cc \
		   link.cc \
		   kinetic_excitation.cc \
		   spring_damper.cc \
		   friction.cc \
		   gear.cc \
		   joint.cc \
		   contact.cc \
		   actuator.cc \
		   signal_.cc \
		   sensor.cc \
		   signal_processing_system.cc \
		   linear_transfer_system.cc \
		   observer.cc \
		   integrator.cc \
		   analyser.cc \
		   parameter.cc \
		   objectfactory.cc \
		   treeitem.cc \
		   treemodel.cc \
		   basic_properties.cc \
		   variable_properties.cc \
		   kinetics_properties.cc \
		   function_property.cc \
		   function_properties.cc \
		   kinematic_functions_properties.cc \
		   function_property_factory.cc \
		   ombv_properties.cc \
		   extended_properties.cc \
		   integrator_properties.cc \
		   basic_widgets.cc \
		   variable_widgets.cc \
		   function_widgets.cc \
		   kinematic_functions_widgets.cc \
		   function_widget_factory.cc \
		   kinetics_widgets.cc \
		   ombv_widgets.cc \
		   extended_widgets.cc \
		   integrator_widgets.cc \
		   property_dialog.cc \
		   element_property_dialog.cc \
		   solver_property_dialog.cc \
		   parameter_property_dialog.cc \
		   embedding_property_dialog.cc \
		   element_context_menu.cc \
		   parameter_context_menu.cc \
		   element_view.cc \
		   embedding_view.cc \
		   solver_view.cc \
		   dialogs.cc  \
		   utils.cc \
		   process.cc \
		   main.cc \
		   embed.cc \
		   solver.cc 
		   

nodist_mbsimgui_SOURCES=$(QT_BUILT_SOURCES)

# C++ source file to build automatically (e.g. abc.moc.cc by abc.h)
QT_BUILT_SOURCES = mainwindow.moc.cc \
		   options.moc.cc \
		   basic_widgets.moc.cc \
		   variable_widgets.moc.cc \
		   function_widget.moc.cc \
		   function_widgets.moc.cc \
		   kinetics_widgets.moc.cc \
		   ombv_widgets.moc.cc \
		   extended_widgets.moc.cc \
		   property_dialog.moc.cc  \
		   element_property_dialog.moc.cc \
		   solver_property_dialog.moc.cc \
		   parameter_property_dialog.moc.cc \
		   embedding_property_dialog.moc.cc \
		   element_context_menu.moc.cc \
		   element_view.moc.cc \
		   embedding_view.moc.cc \
		   solver_view.moc.cc \
		   dialogs.moc.cc  \
		   process.moc.cc

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

