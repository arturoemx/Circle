
CXXFLAGS = -Wall -g -I ./ 

all: testCircle

testCircle: objs/testCircle.o  objs/Circle.o
	g++ $(CXXFLAGS) -o testCircle objs/testCircle.o  objs/Circle.o `pkg-config opencv4 --libs`

objs/testCircle.o: testCircle.cpp 
	g++ $(CXXFLAGS) -o objs/testCircle.o -c testCircle.cpp `pkg-config opencv4 --cflags`

objs/Circle.o: Circle.cpp Circle.h
	g++ $(CXXFLAGS) -o objs/Circle.o -c Circle.cpp `pkg-config opencv4 --cflags`

clean: 
	rm objs/* testCircle 
