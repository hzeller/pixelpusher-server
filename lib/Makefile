CXXFLAGS=-I. -I../include -W -Wall -Wextra -Wno-unused-parameter -O3
OBJECTS=pp-server.o pp-thread.o
LIBRARY=libpixel-push-server.a

$(LIBRARY) : $(OBJECTS)
	$(AR) rcs $@ $^

clean:
	rm -f $(OBJECTS) $(LIBRARY)
