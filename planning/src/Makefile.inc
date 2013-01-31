#Main include for makefile
CC = g++
CFLAGS = `pkg-config --cflags opencv` -pthread
LDFLAGS = `pkg-config --libs opencv` -lsicklms2xx -lpthread -pthread
ECHO = echo
