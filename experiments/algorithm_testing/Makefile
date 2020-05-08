all: prepare track_analysis planning_benchmark

prepare:
	mkdir -p bin

track_analysis:
	g++ src/analyze_track.cpp \
		-o bin/track_analysis \
		-std=c++17 \
		-Wall \
		-O3 \
		-ffast-math \
		-I./include \
		-I../../catkin_ws/src/racer/include \
		-I/usr/include/python2.7 \
		-lpython2.7

planning_benchmark:
	g++ src/planning_benchmark.cpp \
		-o bin/planning_benchmark \
		-std=c++17 \
		-pthread \
		-Wall \
		-O3 \
		-ffast-math \
		-I./include \
		-I../../catkin_ws/src/racer/include \
		-I/usr/include/python2.7 \
		-lpython2.7

clean:
	rm bin/*
