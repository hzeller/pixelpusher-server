CXXFLAGS=-I. -I../include
OBJECTS=pp-server.o pp-thread.o
LIBRARY=libpixel-push-server.a

$(LIBRARY) : $(OBJECTS)
	$(AR) rcs $@ $^

clean:
	rm -f $(OBJECTS) $(LIBRARY)
