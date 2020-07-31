#pragma once
#include "Common.h"

bool eucSimpleQueryPath(Graph* myGraph, ReferencePath* referencePath, std::vector<Edge> currentPath, Record* temRecord, int currentVertex);

bool roadSimpleQueryPath(Graph* myGraph, ReferencePath* referencePath, std::vector<Edge> currentPath, 
	std::vector<Edge>& lastPath, Record* temRecord, int currentVertex, int* timeOutFlag, double* pathDistance);

bool roadSimleQueryPathWithLoop(Graph * myGraph, ReferencePath * referencePath, std::vector<Edge> currentPath, 
	std::vector<Edge>& lastPath, Record * temRecord, int currentVertex, int* timeOutFlag, double* pathDistance);

void maxmalOverlapPath(Graph* myGraph, std::vector<Edge>& localPath, std::vector<Edge>& lastPath, int firstVertexID, int secVertexID, int* endVertexID);