all: track_analysis

track_analysis:
	g++ src/analyze_track.cpp \
		-o bin/track_analysis \
		-std=c++17 \
		-Wall \
		-O3 \
		-I./include \
		-I../racer-jetson/catkin_ws/src/racer/include \
		-I/usr/include/python2.7 \
		-lpython2.7

clean:
	rm bin/*
