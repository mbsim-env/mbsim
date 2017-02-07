# This code is run before the swig directory (.. from here).
# It ensures that the targets that depend on MBSIM_SWIG are removed if the MBSIM_SWIG envvar
# changes. This way the build is always correct even if MBSIM_SWIG changes without doing
# a make clean before.
all-local:
	@test -z "$(MBSIM_SWIG)" -a -z "$$(cat MBSIM_SWIG.last 2> /dev/null)" -o \
	  -n "$(MBSIM_SWIG)" -a -n "$$(cat MBSIM_SWIG.last 2> /dev/null)" || \
	  rm -f $(SWIGFILES) ../check/mymod_swig_python.cc
	@echo "$(MBSIM_SWIG)" > MBSIM_SWIG.last
