# Example makefile
# jme5297
# This could be done in MUCH smarter of a fashion

CC = g++
INC = -I../include/
CFLAGS = -c -Wall
ODIR = obj
SRC = ../src
dir_guard = @mkdir -p $(@D)

all: main

main: $(ODIR)/main.o
	$(dir_guard)
	$(CC) $(ODIR)/main.o -o main

$(ODIR)/main.o: $(SRC)/main.cpp
	$(dir_guard)
	$(CC) $(CFLAGS) $(INC) $(SRC)/main.cpp -o $(ODIR)/main.o

clean:
	rm $(ODIR)/main.o main
