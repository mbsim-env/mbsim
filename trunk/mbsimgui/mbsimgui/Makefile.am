include $(top_srcdir)/qt.mk

bin_PROGRAMS = mbsimgui

mbsimguidir = $(includedir)/mbsimgui

INCLUDES = $(DEPS_CFLAGS) -DTIXML_USE_STL $(QT_CFLAGS) $(HDF5SERIE_CFLAGS) $(QWT_CFLAGS) $(MKOCTFILE_CFLAGS)

LDADD = $(DEPS_LIBS)  $(QT_LIBS) $(QWT_LIBS) $(MKOCTFILE_LIBS)

mbsimgui_CPPFLAGS = $(MBXMLUTILS_CFLAGS)

mbsimgui_LDFLAGS = $(HDF5SERIE_LIBS) $(MBXMLUTILS_LIBS) -Wl,-rpath -Wl,\$$ORIGIN/../lib

mbsimgui_SOURCES = main.cc \
		   mainwindow.cc \
		   basic_widgets.cc \
		   string_widgets.cc \
		   function_widgets.cc \
		   kinematics_widgets.cc \
		   kinetics_widgets.cc \
		   ombv_widgets.cc \
		   extended_widgets.cc \
		   property_widget.cc\
		   dialogs.cc  \
		   utils.cc \
		   element.cc \
		   object.cc \
		   body.cc \
		   rigidbody.cc \
		   group.cc \
		   objectfactory.cc \
		   solver.cc \
		   frame.cc \
		   link.cc \
		   joint.cc \
		   spring_damper.cc \
		   kinetic_excitation.cc \
		   constraint.cc \
		   integrator.cc \
		   parameter.cc \
		   octaveutils.cc

nodist_mbsimgui_SOURCES=$(QT_BUILT_SOURCES)

# C++ source file to build automatically (e.g. abc.moc.cc by abc.h)
QT_BUILT_SOURCES = mainwindow.moc.cc \
		   basic_widgets.moc.cc \
		   string_widgets.moc.cc \
		   function_widgets.moc.cc \
		   kinematics_widgets.moc.cc \
		   kinetics_widgets.moc.cc \
		   ombv_widgets.moc.cc \
		   extended_widgets.moc.cc \
		   body.moc.cc \
		   frame.moc.cc \
		   object.moc.cc \
		   property_widget.moc.cc  \
		   dialogs.moc.cc  \
		   group.moc.cc \
		   rigidbody.moc.cc\
		   element.moc.cc \
		   solver.moc.cc \
		   link.moc.cc \
		   joint.moc.cc \
		   spring_damper.moc.cc \
		   kinetic_excitation.moc.cc \
		   constraint.moc.cc \
		   integrator.moc.cc \
		   parameter.moc.cc

mbsimgui_HEADERS = 
		   
		   
		   
		   
		   
		   
		   
		   

