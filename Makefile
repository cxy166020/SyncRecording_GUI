CC = g++
CFLAGS = -I../include -Wall -O2
LDFLAGS = -lpthread

LADYBUG_OBJS = ../lib/Ladybug.lib

PROG = SyncCap

HDRS = # ladybug.h ladybuggeom.h ladybugrenderer.h ladybugstream.h
SRCS = main.cpp

OBJS = main.o

$(PROG) : $(OBJS)
	$(CC) $(OBJS) $(LADYBUG_OBJS) $(LDFLAGS) -o $(PROG)

$(OBJS) :
	$(CC) $(HDRS) $(SRCS) $(CFLAGS)  -c

main.o : main.cpp
