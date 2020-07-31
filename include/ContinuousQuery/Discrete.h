#pragma once
#include "Common.h"

void discreteReferPath(Graph* myGraph, ReferencePath* temPath);
std::vector<Point> creatTrajPoint(Graph * myGraph, std::vector<Edge>& temPath, int initialVertex);
