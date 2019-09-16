all: track_analysis

track_analysis:
	g++ -Wall -std=c++17 -Iinclude -I../../racer-jetson/catkin_ws/src/racer/include -o track_analysis analyze_track.cpp
	chmod a+x track_analysis

clean:
	rm track_analysis
