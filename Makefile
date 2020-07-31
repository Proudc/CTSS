make:
	g++ -std=c++11 src/Test.cpp src/ContinuousQuery/Base.cpp src/ContinuousQuery/Construct.cpp src/ContinuousQuery/Discrete.cpp src/ContinuousQuery/Frechet.cpp src/ContinuousQuery/Pruneline.cpp  -o CTSS
clean:
	rm CTSS
