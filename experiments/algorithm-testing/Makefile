all: track_analysis track_analysis_benchmark planning_benchmark

track_analysis:
	g++ src/analyze_track.cpp \
		-o bin/track_analysis \
		-std=c++17 \
		-Wall \
		-O3 \
		-ffast-math \
		-I./include \
		-I../../racer-jetson/catkin_ws/src/racer/include \
		-I/usr/include/python2.7 \
		-lpython2.7

track_analysis_benchmark:
	g++ src/analyze_track_benchmark.cpp \
		-o bin/track_analysis_benchmark \
		-std=c++17 \
		-Wall \
		-O3 \
		-ffast-math \
		-I./include \
		-I../../racer-jetson/catkin_ws/src/racer/include

planning_benchmark:
	g++ src/planning_benchmark.cpp \
		-o bin/planning_benchmark \
		-std=c++17 \
		-pthread \
		-Wall \
		-O3 \
		-ffast-math \
		-I./include \
		-I../../racer-jetson/catkin_ws/src/racer/include \
		-I/usr/include/python2.7 \
		-lpython2.7

clean:
	rm bin/*
