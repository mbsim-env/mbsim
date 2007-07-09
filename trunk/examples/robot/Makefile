CC = g++
sources = main.cc robot.cc
libsources = 
objects = $(sources:.cc=.o) 
CPPFLAGS= -O2 -Wall `pkg-config --cflags mbsimControl` 





main : $(objects)
	$(CC) -o main $(objects) `pkg-config --libs mbsimControl` 




%.d: %.cc
	set -e; $(CC) -MM $(CPPFLAGS) $< \
                 | sed 's/\(.*\)\.o[ :]*/\1.o \1.d : /g' > $@; \
	[ -s $@ ] || rm -f $@

include $(sources:.cc=.d)


.PHONY : clean
clean :
	-rm main $(objects) $(sources:.cc=.d)
