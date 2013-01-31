include Makefile.inc

OBJECTS = main.o  Line.o

all: $(OBJECTS)
	g++ $(OBJECTS) -o main $(CFLAGS) $(DFLAGS)
	$(ECHO) Built The Entire Library!
	
main.o: main.cpp Line.o
	g++ main.cpp $(CFLAGS) $(DFLAGS) -c
	
Line.o: Line.h Line.cpp
	g++ Line.cpp $(CFLAGS) -c
	
clean:
	rm -rf *o main
