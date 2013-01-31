OBJECTS = Lidar.o TimeKeeper.o LidarManager.o main2.o LidarLocalizer.o
all:	main

main:  $(OBJECTS)
	g++ $(OBJECTS) -o main `pkg-config --cflags --libs opencv` -lsicklms2xx -pthread 
main2:  $(OBJECTS)
	g++ $(OBJECTS) -o main2 `pkg-config --cflags --libs opencv` -lsicklms2xx -pthread 
main.o: main.cpp LidarManager.h Lidar.h TimeKeeper.h
	g++ main.cpp -c  `pkg-config --cflags --libs opencv`
main2.o: main2.cpp LidarManager.h Lidar.h TimeKeeper.h
	g++ main2.cpp -c  `pkg-config --cflags --libs opencv`
LidarManager.o: LidarManager.h LidarManager.cpp Lidar.h
	g++ LidarManager.cpp `pkg-config --cflags --libs opencv` -c 
Lidar.o: Lidar.h Lidar.cpp TimeKeeper.h
	g++ Lidar.cpp -c
TimeKeeper.o: TimeKeeper.h TimeKeeper.cpp
	g++ TimeKeeper.cpp -c
LidarLocalizer.o: 
	g++ LidarLocalizer.cpp `pkg-config --cflags --libs opencv` -c -o LidarLocalizer.o
	
clean:
	rm -rf *o main

