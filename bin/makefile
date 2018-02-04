# Example makefile
# jme5297
# This could be done in MUCH smarter of a fashion

# These are compiler flags
CC = g++
INC = -I../include/
CFLAGS = -c -Wall -std=c++11 -g
ODIR = obj
SRC = ../src
dir_guard = @mkdir -p $(@D)

all: main

main: $(ODIR)/Vehicle.o \
			$(ODIR)/PlantModel.o \
			$(ODIR)/Laser_generic.o \
			$(ODIR)/Camera_generic.o \
			$(ODIR)/GPS_generic.o \
			$(ODIR)/SensorHub.o \
			$(ODIR)/Navigation.o \
			$(ODIR)/Controller.o \
			$(ODIR)/main.o
	$(dir_guard)
	$(CC) $(ODIR)/Vehicle.o \
	$(ODIR)/PlantModel.o \
	$(ODIR)/GPS_generic.o \
	$(ODIR)/Laser_generic.o \
	$(ODIR)/Camera_generic.o \
	$(ODIR)/SensorHub.o \
	$(ODIR)/Navigation.o \
	$(ODIR)/Controller.o \
	$(ODIR)/main.o -o main

$(ODIR)/Vehicle.o: $(SRC)/Vehicle.cpp
	$(dir_guard)
	$(CC) $(CFLAGS) $(INC) $(SRC)/Vehicle.cpp -o $(ODIR)/Vehicle.o

$(ODIR)/PlantModel.o: $(SRC)/PlantModel.cpp
	$(dir_guard)
	$(CC) $(CFLAGS) $(INC) $(SRC)/PlantModel.cpp -o $(ODIR)/PlantModel.o

$(ODIR)/Laser_generic.o: $(SRC)/Laser_generic.cpp
	$(dir_guard)
	$(CC) $(CFLAGS) $(INC) $(SRC)/Laser_generic.cpp -o $(ODIR)/Laser_generic.o

$(ODIR)/Camera_generic.o: $(SRC)/Camera_generic.cpp
	$(dir_guard)
	$(CC) $(CFLAGS) $(INC) $(SRC)/Camera_generic.cpp -o $(ODIR)/Camera_generic.o

$(ODIR)/GPS_generic.o: $(SRC)/GPS_generic.cpp
	$(dir_guard)
	$(CC) $(CFLAGS) $(INC) $(SRC)/GPS_generic.cpp -o $(ODIR)/GPS_generic.o

$(ODIR)/SensorHub.o: $(SRC)/SensorHub.cpp
	$(dir_guard)
	$(CC) $(CFLAGS) $(INC) $(SRC)/SensorHub.cpp -o $(ODIR)/SensorHub.o

$(ODIR)/Navigation.o: $(SRC)/Navigation.cpp
	$(dir_guard)
	$(CC) $(CFLAGS) $(INC) $(SRC)/Navigation.cpp -o $(ODIR)/Navigation.o

$(ODIR)/Controller.o: $(SRC)/Controller.cpp
	$(dir_guard)
	$(CC) $(CFLAGS) $(INC) $(SRC)/Controller.cpp -o $(ODIR)/Controller.o

$(ODIR)/main.o: $(SRC)/main.cpp
	$(dir_guard)
	$(CC) $(CFLAGS) $(INC) $(SRC)/main.cpp -o $(ODIR)/main.o

clean:
	rm -f $(ODIR)/* main
