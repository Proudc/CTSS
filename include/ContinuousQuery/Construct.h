#pragma once
#include "Common.h"

void readVertexs(Graph* myGraph, const std::string nodeFilePath);
void readEdges(Graph* myGraph, const std::string edgeFilePath);
void readReferencePath(Graph* myGraph, ReferencePath* referencePath, const std::string referFilePath);
void readCompletePath(Graph * myGraph, CompletePath * completePath, const std::string comFilePath); 
