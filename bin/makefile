# Example makefile
# jme5297
# This could be done in MUCH smarter of a fashion

INC = -I../include/
FLAGS = -c -Wall
ODIR = obj
SRC = ../src

all: main

main: $(ODIR)/main.o
	g++ $(ODIR)/main.o -o main

$(ODIR)/main.o: $(SRC)/main.cpp
	g++ $(FLAGS) $(INC) $(SRC)/main.cpp -o $(ODIR)/main.o

clean:
	rm $(ODIR)/main.o
