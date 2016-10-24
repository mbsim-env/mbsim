noinst_LTLIBRARIES = libkinematics.la
libkinematics_la_SOURCES = translation_along_x_axis.cc\
			   translation_along_y_axis.cc\
			   translation_along_z_axis.cc\
			   translation_along_axes_xy.cc\
			   translation_along_axes_yz.cc\
			   translation_along_axes_xz.cc\
			   translation_along_axes_xyz.cc\
			   translation_along_fixed_axis.cc\
			   linear_translation.cc\
			   rotation_about_x_axis.cc\
			   rotation_about_y_axis.cc\
			   rotation_about_z_axis.cc\
			   rotation_about_fixed_axis.cc\
			   rotation_about_axes_xy.cc\
			   rotation_about_axes_yz.cc\
			   rotation_about_axes_xz.cc\
			   rotation_about_axes_xyz.cc\
			   rotation_about_axes_zxz.cc\
			   rotation_about_axes_zyx.cc

kinematicsincludedir = $(includedir)/mbsim/functions/kinematics

libkinematics_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(CASADI_LIBS)
libkinematics_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(CASADI_CFLAGS)

kinematicsinclude_HEADERS = kinematics.h\
			    translation_along_x_axis.h\
			    translation_along_y_axis.h\
			    translation_along_z_axis.h\
			    translation_along_axes_xy.h\
			    translation_along_axes_yz.h\
			    translation_along_axes_xz.h\
			    translation_along_axes_xyz.h\
			    translation_along_fixed_axis.h\
			    linear_translation.h\
			    rotation_about_x_axis.h\
			    rotation_about_y_axis.h\
			    rotation_about_z_axis.h\
			    rotation_about_fixed_axis.h\
			    rotation_about_axes_xy.h\
			    rotation_about_axes_yz.h\
			    rotation_about_axes_xz.h\
			    rotation_about_axes_xyz.h\
			    rotation_about_axes_zxz.h\
			    rotation_about_axes_zyx.h\
			    rotation_about_axes_xyz_mapping.h\
			    rotation_about_axes_zxz_mapping.h\
			    rotation_about_axes_zyx_mapping.h\
			    rotation_about_axes_xyz_transformed_mapping.h\
			    rotation_about_axes_zxz_transformed_mapping.h
