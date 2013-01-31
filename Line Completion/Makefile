include Makefile.inc

OBJECTS = main.o  #Start.o

all: $(OBJECTS)
	g++ $(OBJECTS) -o main $(CFLAGS) $(DFLAGS)
	$(ECHO) Built The Entire Library!
	
main.o: main.cpp
	g++ main.cpp $(CFLAGS) $(DFLAGS) -c
	
Start.o: Start.h Start.cpp
	g++ Start.cpp $(CFLAGS) -c
	
clean:
	rm -rf *o main
