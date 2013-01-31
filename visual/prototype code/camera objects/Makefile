all: Camera.o main.o
	g++ main.o Camera.o CameraManager.o TimeKeeper.o -o main `pkg-config --cflags --libs opencv` 

main.o: main.cpp Camera.o TimeKeeper.o CameraManager.o 
	g++ main.cpp -c `pkg-config --cflags --libs opencv`

CameraManager.o: CameraManager.cpp
	g++ CameraManager.cpp -c `pkg-config --cflags --libs opencv`

Camera.o: Camera.cpp
	g++ Camera.cpp -c `pkg-config --cflags --libs opencv`

TimeKeeper.o: TimeKeeper.cpp
	g++ TimeKeeper.cpp -c

clean:
	rm -rf *o main