#makefile of awesome, Author: Jesse Fish

include Makefile.inc

OBJECTS = TimeKeeper.o \
		  main.o \
		  lidarlocalizorapp.o \
		  PostProcess.o \
		  Line.o \
		  Breadth.o \
		  bugging.o BugPlanner.o \
		  lidar/LidarLocalizer.o \
		  lidar/LidarManager.o \
		  lidar/Lidar.o \
		  observermode.o \
		  wallcrawling.o \
		  pointandclick.o \
		  Display.o \
		  PointAndClickDisplay.o \
		  recorder.o \
		  vision/CameraManager.o \
		  vision/Camera.o \
		  vision/VisionInterpreter.o \
		  vision/birdseyecalibrator.o \
		  vision/intrinsiccalibrator.o \
		  WorldModel.o \
		  WallCrawler.o \
		  NetworkStack.o \
		  simplesteering.o \
		  GoalFinder.o \
		  qualifier.o \
		  GPSPlanner.o \
		  AStar.o \
		  TrajectoryGenerator.o \
		  doGPS.o \
		  ThreadGPSPlanner.o  \
		  convertgps.o
		  


INCLUDE = -I./lidar -I./vision -I../../include -I./

all: $(OBJECTS)
	g++ $(OBJECTS) -o main $(CFLAGS) $(LDFLAGS) $(INCLUDE)

%.o : %.cpp
	g++ $(CFLAGS) $(INCLUDE) -o "$@" -c "$<"

clean:
	cd ./lidar; rm -rf *.o
	cd ./vision; rm -rf *.o
	rm -rf *.o main

