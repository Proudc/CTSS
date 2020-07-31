#pragma once
#include "Common.h"

void initializeGraph(Graph* myGraph, const string nodeFilePath, const string edgeFilePath, double delta, double sectionLength);
string changeIntToStr(int count);
void setQueryEnv(Graph* myGraph, ReferencePath* referencePath, CompletePath* completePath, string inputFolder, string strCount);
//double getPathLength(const std::vector<Edge>& temPath);
void setCurrPath(Graph* myGraph, std::vector<Edge>* currentPath, CompletePath* completePath, const unsigned int pos);
void initializeFirstPath(Graph* myGraph, TraversingPath* path, int currentVertex, std::vector<Edge>& currentPath);
//void resetVertexFlag(Graph* myGraph);
//void initializeReferencePath(Graph* myGraph, ReferencePath* temReferPath, const string referFilePath);
//void initializeCompletePath(Graph* myGraph, CompletePath* temComPath, const string comFilePath);
double realDistance(const double lonDiff, const double latDiff, const double cosLongitude);
//void setReferInitialAndFinal(ReferencePath* temPath);
//void setComInitialAndFinal(CompletePath* temPath);
void setReferPathLength(ReferencePath* temPath);
void filterVertexIsSafeArea(Graph* myGraph, ReferencePath* temPath);
double pointToSegDist(const double x, const double y, const double x1, const double y1, const double x2, const double y2);
void setMinPosAndMaxPos(Graph* myGraph, ReferencePath* temReferPath);
double euc(const double px, const double py, const double qx, const double qy);
void setReferDFDFlag(Graph* myGraph, ReferencePath* temPath);
void setDistToRefer(Graph* myGraph, ReferencePath* temPath);
void setTraversalOrderWeights(Graph* myGraph, ReferencePath* temPath);
void setEdgeLength(Graph* myGraph, Edge* temEdge);
void writeSingleRecordToFile(Record record[], const unsigned int recordPos, const string filePath);
void writeContinuousRecordToFile(Record record[], const unsigned int recordPos, const string filePath, double totalRunTime);
Edge findEdge(Graph* myGraph, Vertex firstVertex, Vertex secVertex);
void handleNewPath(Graph* myGraph, TraversingPath* oldPath, TraversingPath* newPath, const int newVertexID);
bool checkIfAccess(Graph * myGraph, TraversingPath * temPath, const int vertexID);
void replacePath(std::vector<Edge>& lastPath, std::vector<Edge>& localPath, const int startPos, const int stopPos);
void updateLastPath(std::vector<Edge>& lastPath, std::vector<Edge>& temLastPath, double* boundDistance, double* temBoundDistance);
void initializeLastPath(ReferencePath* referencePath, std::vector<Edge>& lastPath);