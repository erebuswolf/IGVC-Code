ECHO = echo

CFLAGS = `pkg-config --cflags --libs opencv` -lsicklms2xx -pthread #-lpthread
DFLAGS = `pkg-config --cflags gtk+-2.0 pkg-config --libs gtk+-2.0`

.SILENT:
