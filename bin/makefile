# Example makefile
# jme5297
# This could be done in MUCH smarter of a fashion

# These are compiler flags
CC = g++
INC = -I../include/
CFLAGS = -c -Wall # -std=c++11
ODIR = obj
SRC = ../src
dir_guard = @mkdir -p $(@D)

all: main

main: $(ODIR)/GPS_generic.o $(ODIR)/PathPlanning.o $(ODIR)/main.o
	$(dir_guard)
	$(CC) $(ODIR)/GPS_generic.o $(ODIR)/PathPlanning.o $(ODIR)/main.o -o main

$(ODIR)/GPS_generic.o: $(SRC)/GPS_generic.cpp
	$(dir_guard)
	$(CC) $(CFLAGS) $(INC) $(SRC)/GPS_generic.cpp -o $(ODIR)/GPS_generic.o

$(ODIR)/PathPlanning.o: $(SRC)/PathPlanning.cpp
	$(dir_guard)
	$(CC) $(CFLAGS) $(INC) $(SRC)/PathPlanning.cpp -o $(ODIR)/PathPlanning.o

$(ODIR)/main.o: $(SRC)/main.cpp
	$(dir_guard)
	$(CC) $(CFLAGS) $(INC) $(SRC)/main.cpp -o $(ODIR)/main.o

clean:
	rm -f $(ODIR)/* main
