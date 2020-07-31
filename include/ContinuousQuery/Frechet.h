#pragma once
#include "Common.h"

double dfdDist(const int m, const int n, Point* p, Point* q);
double dfdDistBetweenTwoPaths(Graph* myGraph, TraversingPath* temPath, ReferencePath* referencePath, std::vector<Edge>& lastPath, int finalVertexID);
double DFD(Graph * myGraph, std::vector<Edge> finalPath, ReferencePath * referencePath);
double dfdBetweenTwoLocalPaths(Graph* myGraph, std::vector<Edge>& firstPath, std::vector<Edge>& secPath, int initialVertex);