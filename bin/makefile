# Example makefile
# jme5297
# This could be done in MUCH smarter of a fashion

CC = g++
INC = -I../include/
CFLAGS = -c -Wall
ODIR = obj
SRC = ../src

all: main

main: $(ODIR)/main.o
	$(CC) $(ODIR)/main.o -o main

$(ODIR)/main.o: $(SRC)/main.cpp
	$(CC) $(CFLAGS) $(INC) $(SRC)/main.cpp -o $(ODIR)/main.o

clean:
	rm $(ODIR)/main.o main
