CXX= g++
CXXFLAGS = -Wall
LDLIBS= -lrealsense2 -lpthread -lraspicam
FILES= rs_server.cpp 
OBJS= rs_server.o

main: $(OBJS)
	g++ $(CXXFLAGS) -o rs_streamer $(FILES) $(LDLIBS)

#run:
#	./rs_streamer

clean:
	rm *.o
