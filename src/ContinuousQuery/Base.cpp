#include <cassert>
#include <stdio.h>
#include <stdbool.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <ctime>
#include <float.h>
#include <limits>

#include "../../include/ContinuousQuery/Base.h"
#include "../../include/ContinuousQuery/Common.h"
#include "../../include/ContinuousQuery/Construct.h"
#include "../../include/ContinuousQuery/Discrete.h"
#include "../../include/ContinuousQuery/Frechet.h"

namespace
{

/**
 * Calculate the length of a path
 * @param temPath a path, is a vector containing edges
 * @return the length of the path
*/
double getPathLength(const std::vector<Edge>& temPath)
{
	double temLength = 0.0;
	for (unsigned int i = 0; i < temPath.size(); i++)
	{
		temLength += temPath[i].realLength;
	}
	return temLength;
}

/**
 * Reset the flags of all vertices in the graph
 * @param myGraph pointer to a graph
*/
void resetVertexFlag(Graph* myGraph)
{
	printf("Reset the flag of the vertex of the road network...\n");
	for (unsigned int i = 0; i < (myGraph->vertexPos); i++)
	{
		myGraph->vertexsVector[i].referFlag    = false;
		myGraph->vertexsVector[i].locationFlag = false;
		myGraph->vertexsVector[i].accessFlag   = false;
		myGraph->vertexsVector[i].currFlag     = false;
	}
	printf("The flag of the vertex of the road network is set!!!\n");
}

/**
 * Set the initial and final points of the reference path
 * @param temPath pointer to a reference path
*/
void setReferInitialAndFinal(ReferencePath* temPath)
{
	assert(temPath->edges.size() >= 2);

	int first1  = temPath->edges[0].firstVertexID;
	int sec1    = temPath->edges[0].secVertexID;
	int first2  = temPath->edges[1].firstVertexID;
	int sec2    = temPath->edges[1].secVertexID;
	int firstL2 = temPath->edges[temPath->edges.size() - 2].firstVertexID;
	int secL2   = temPath->edges[temPath->edges.size() - 2].secVertexID;
	int firstL1 = temPath->edges[temPath->edges.size() - 1].firstVertexID;
	int secL1   = temPath->edges[temPath->edges.size() - 1].secVertexID;
	if ((first1 == first2) || (first1 == sec2))
	{
		temPath->initialVertex = sec1;
	}
	else
	{
		temPath->initialVertex = first1;
	}
	if ((firstL1 == firstL2) || (firstL1 == secL2))
	{
		temPath->finalVertex = secL1;
	}
	else
	{
		temPath->finalVertex = firstL1;
	}
}

/**
 * Set the initial and final points of the complete path
 * @param temPath pointer to a complete path
*/
void setComInitialAndFinal(CompletePath* temPath)
{
	assert(temPath->edges.size() >= 2);

	int first1  = temPath->edges[0].firstVertexID;
	int sec1    = temPath->edges[0].secVertexID;
	int first2  = temPath->edges[1].firstVertexID;
	int sec2    = temPath->edges[1].secVertexID;
	int firstL2 = temPath->edges[temPath->edges.size() - 2].firstVertexID;
	int secL2   = temPath->edges[temPath->edges.size() - 2].secVertexID;
	int firstL1 = temPath->edges[temPath->edges.size() - 1].firstVertexID;
	int secL1   = temPath->edges[temPath->edges.size() - 1].secVertexID;
	if ((first1 == first2) || (first1 == sec2))
	{
		temPath->initialVertex = sec1;
	}
	else
	{
		temPath->initialVertex = first1;
	}
	if ((firstL1 == firstL2) || (firstL1 == secL2))
	{
		temPath->finalVertex = secL1;
	}
	else
	{
		temPath->finalVertex = firstL1;
	}
}

/**
 * Initialize the reference path
 * @param myGraph pointer to a graph
 * @param temReferPath pointer to a reference path
 * @param referFilePath file path of reference path
*/
void initializeReferencePath(Graph* myGraph, ReferencePath* temReferPath, const string referFilePath)
{
	printf("Initializing reference path...\n");
	temReferPath->pointOfRefer = new Point[POINT_OF_REFERPATH];
	readReferencePath(myGraph, temReferPath, referFilePath);
	setReferInitialAndFinal(temReferPath);
	discreteReferPath(myGraph, temReferPath);
	setReferDFDFlag(myGraph, temReferPath);
	temReferPath->referenceLength = getPathLength(temReferPath->edges);
	filterVertexIsSafeArea(myGraph, temReferPath);
	setMinPosAndMaxPos(myGraph, temReferPath);
	setDistToRefer(myGraph, temReferPath);
	setTraversalOrderWeights(myGraph, temReferPath);
	printf("The reference path is initialized!!!\n");
}

/**
 * Initialize the complete path
 * @param myGraph pointer to a graph
 * @param temComPath pointer to a complete path
 * @param comFilePath file path of complete path
*/
void initializeCompletePath(Graph* myGraph, CompletePath* temComPath, const string comFilePath)
{
	printf("Initialzing complete path...\n");
	readCompletePath(myGraph, temComPath, comFilePath);
	setComInitialAndFinal(temComPath);
	temComPath->completeLength = getPathLength(temComPath->edges);
	printf("The complete path is initialized...\n");
}

}

/**
 * Initialize a graph using vertex file path, edge file path and delta
 * @param myGraph pointer to a graph
 * @param nodeFilePath the file path of the vertex file
 * @param edgeFilePath the file path of the edge file
 * @param delta delta of this group of queries
 * @param sectionLength interpolation length used during interpolation
*/
void initializeGraph(Graph* myGraph, const string nodeFilePath, const string edgeFilePath, double delta, double sectionLength)
{
	printf("Initializing the road network...\n");
	myGraph->vertexsVector = new Vertex[VERTEX_SIZE];
	myGraph->edgesVector   = new Edge[EDGE_SIZE];
	readVertexs(myGraph, nodeFilePath);
	readEdges(myGraph, edgeFilePath);
	myGraph->delta = delta;
	myGraph->SECTION_LENGTH = sectionLength;
	printf("The road network is initialized!!!\n");
}

/**
 * Convert integer to string
 * @param count integer to be converted
 * @return the converted string
*/
string changeIntToStr(int count)
{
	return std::to_string(count);
}

/**
 * When the reference path and complete path are updated, re-set the query environment
 * @param myGraph pointer to a graph
 * @param referencePath pointer to a reference path
 * @param completePath pointer to a complete path
 * @param inputFolder folder path when reading files
 * @param strCount indicate the number of query
*/
void setQueryEnv(Graph* myGraph, ReferencePath* referencePath, CompletePath* completePath, string inputFolder, string strCount)
{
	printf("Initializing a new round of query environment...\n");
	string referFilePath = inputFolder + "/" + strCount + "referencepath.txt";
	string comFilePath   = inputFolder + "/" + strCount + "currentpath.txt";
	resetVertexFlag(myGraph);
	initializeReferencePath(myGraph, referencePath, referFilePath);
	initializeCompletePath(myGraph, completePath, comFilePath);
	printf("A new round of query environment initialization is complete!!!\n");
}

/**
 * Set the current path based on the complete path and location pos
 * @param myGraph pointer to a graph
 * @param currentPath a pointer to a vector, representing the current path
 * @param completePath pointer to a complete path
 * @param pos indicate the location of the current path
*/
void setCurrPath(Graph* myGraph, std::vector<Edge>* currentPath, CompletePath* completePath, const unsigned int pos)
{
	printf("Initializing the current path...\n");
	for (unsigned int i = 0; i < pos; i++)
	{
		Edge temEdge = completePath->edges[i];
		myGraph->vertexsVector[temEdge.firstVertexID].accessFlag = true;
		myGraph->vertexsVector[temEdge.firstVertexID].currFlag   = true;
		myGraph->vertexsVector[temEdge.secVertexID].accessFlag   = true;
		myGraph->vertexsVector[temEdge.secVertexID].currFlag     = true;
		currentPath->push_back(temEdge);
	}
	printf("The current path initialization is complete!!!\n");
}

/**
 * Initialize the first Traversing path according to the current path
 * @param path a TraversingPath that needs to be initialized
 * @param currentVertex the end of the current path
 * @param currentPath indicate the current path
*/
void initializeFirstPath(Graph* myGraph, TraversingPath* path, int currentVertex, std::vector<Edge>& currentPath)
{
	printf("Start initializing the first path...\n");
	path->finalVertexID = currentVertex;
	path->distToRefer   = 0;
	path->minPos        = 0;
	path->maxPos        = myGraph->vertexsVector[currentVertex].maxPos;
	for (unsigned int i = 0; i < currentPath.size(); i++)
	{
		const Vertex firstVertex = myGraph->vertexsVector[currentPath[i].firstVertexID];
		const Vertex secVertex   = myGraph->vertexsVector[currentPath[i].secVertexID];
		path->edges.push_back(currentPath[i].edgeID);
		path->distToRefer = std::max(path->distToRefer, firstVertex.distToRefer);
		path->distToRefer = std::max(path->distToRefer, secVertex.distToRefer);
		path->minPos      = std::max(path->minPos, firstVertex.minPos);
		path->minPos      = std::max(path->minPos, secVertex.minPos);
	}
	printf("The first path is initialized!!!\n");
}

/**
 * Calculate the real distance between two points on the earth
 * @param lonDiff difference in longitude
 * @param latDiff difference in latitude
 * @param cosLongitude an auxiliary value when calculating distance
 * @return the real distance between two points on the earth
*/
double realDistance(const double lonDiff, const double latDiff, const double cosLongitude)
{
	const  double CONST_DISTANCE_ONE_DEGREE = 111111;
	double lonDistance = lonDiff * CONST_DISTANCE_ONE_DEGREE * cos(cosLongitude);
	double latDistance = latDiff * CONST_DISTANCE_ONE_DEGREE;
	double dist = sqrt(lonDistance * lonDistance + latDistance * latDistance);
	return dist;
}

/**
 * Set the length of the reference path
 * @param temPath pointer to a reference path
*/
void setReferPathLength(ReferencePath* temPath)
{
	temPath->referenceLength = 0.0;
	for (unsigned int i = 0; i < temPath->edges.size(); i++)
	{
		Edge temEdge = temPath->edges[i];
		temPath->referenceLength += (temEdge.realLength);
	}
}

/**
 * Filter out vertices that are not in the safe area
 * @param myGraph pointer to a graph
 * @param temPath pointer to a reference path
*/
void filterVertexIsSafeArea(Graph* myGraph, ReferencePath* temPath)
{
	printf("Start filtering the vertices in the safe area...\n");
	int count = 0;
	for (int i = 0; i < (myGraph->vertexPos); i++)
	{
		if (myGraph->vertexsVector[i].referFlag == true)
		{
			myGraph->vertexsVector[i].locationFlag = true;
			count++;
			continue;
		}
		for (unsigned int j = 0; j < temPath->edges.size(); j++)
		{
			double x = myGraph->vertexsVector[i].longitude;
			double y = myGraph->vertexsVector[i].latitude;
			double x1 = myGraph->vertexsVector[temPath->edges[j].firstVertexID].longitude;
			double y1 = myGraph->vertexsVector[temPath->edges[j].firstVertexID].latitude;
			double x2 = myGraph->vertexsVector[temPath->edges[j].secVertexID].longitude;
			double y2 = myGraph->vertexsVector[temPath->edges[j].secVertexID].latitude;
			double dist = pointToSegDist(x, y, x1, y1, x2, y2);
			if (dist <= (myGraph->delta))
			{
				myGraph->vertexsVector[i].locationFlag = true;
				count++;
				break;
			}
		}
	}
	printf("Finished filtering the vertices in the safe area!!!\n");
}

/**
 * Calculate the shortest distance from a point to a line segment
 * @param x vertex x
 * @param y vertex y
 * @param x1 x of a vertex of a line segment
 * @param y1 y of a vertex of a line segment
 * @param x2 x of another vertex of a line segment
 * @param y2 y of another vertex of a line secment
 * @return the shortest distance from a point to a line segment
*/
double pointToSegDist(const double x, const double y, const double x1, const double y1, const double x2, const double y2)
{
	double cross = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);
	if (cross <= 0)
	{
		return sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
	}
	double d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
	if (cross >= d2)
	{
		return sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2));
	}
	double r = cross / d2;
	double px = x1 + (x2 - x1) * r;
	double py = y1 + (y2 - y1) * r;
	return sqrt((x - px) * (x - px) + (py - y) * (py - y));
}

/**
 * Set minPos and maxPos for each vertex int the safe area
 * @param myGraph pointer to a graph
 * @param temReferPath pointer to a reference path
*/
void setMinPosAndMaxPos(Graph* myGraph, ReferencePath* temReferPath)
{
	printf("Start to set the minPos and maxPos of each vertex...\n");
	for (unsigned int i = 0; i < (myGraph->vertexPos); i++)
	{
		myGraph->vertexsVector[i].minPos = 0;
		myGraph->vertexsVector[i].maxPos = 0;
		if (myGraph->vertexsVector[i].locationFlag == true)
		{
			double x1 = myGraph->vertexsVector[i].longitude;
			double y1 = myGraph->vertexsVector[i].latitude;
			for (unsigned int j = 0; j < temReferPath->numOfReferPoint; j++)
			{
				double x2 = temReferPath->pointOfRefer[j].x;
				double y2 = temReferPath->pointOfRefer[j].y;
				double dist = euc(x1, y1, x2, y2);
				if (dist <= (myGraph->delta))
				{
					myGraph->vertexsVector[i].minPos = j;
					break;
				}
			}
			int numOfReferPoint = temReferPath->numOfReferPoint - 1;
			for (int j = numOfReferPoint; j >= 0; j--)
			{
				double x2 = temReferPath->pointOfRefer[j].x;
				double y2 = temReferPath->pointOfRefer[j].y;
				double dist = euc(x1, y1, x2, y2);
				if (dist <= (myGraph->delta))
				{
					myGraph->vertexsVector[i].maxPos = j;
					break;
				}
			}
		}
	}
	printf("The minPos and maxPos of each vertex are set...\n");
}

/**
 * Calculate the Euclidean distance between two points
 * @param px x of a vertex
 * @param py y of a vertex
 * @param qx x of another vertex
 * @param qy y of another vertex
 * @return the Euclidean distance between two points
*/
double euc(const double px, const double py, const double qx, const double qy)
{
	double x = px - qx;
	double y = py - qy;
	return sqrt(x * x + y * y);
}

/**
 * Set the DFD flag of the reference path
 * @param myGraph pointer to a graph
 * @param temPath pointer to a reference path
*/
void setReferDFDFlag(Graph* myGraph, ReferencePath* temPath)
{
	Vertex finalVertex = myGraph->vertexsVector[temPath->finalVertex];
	if (finalVertex.locationFlag == false)
	{
		temPath->referDFDFlag = false;
	}
	else
	{
		int flag = 0;
		for (int j = finalVertex.minPos; j <= finalVertex.maxPos; j++)
		{
			double x1 = finalVertex.longitude;
			double y1 = finalVertex.latitude;
			double x2 = temPath->pointOfRefer[j].x;
			double y2 = temPath->pointOfRefer[j].y;
			if (euc(x1, y1, x2, y2) > (myGraph->delta))
			{
				flag = 1;
				break;
			}
		}
		if (flag == 1)
		{
			temPath->referDFDFlag = false;
		}
		else
		{
			temPath->referDFDFlag = true;
		}
	}
}

/**
 * Set the distance from the vertex on the road network to the reference path
 * The distance is defined as the shortest distance from the vertex to each edge of the reference path
 * @param myGraph pointer to a graph
 * @param temPath pointer to a reference path
*/
void setDistToRefer(Graph* myGraph, ReferencePath* temPath)
{
	printf("Start calculating the shortest distance from each vertex to the reference path...\n");
	for (unsigned int i = 0; i < myGraph->vertexPos; i++)
	{
		double temDist = std::numeric_limits<double>::max();
		if (myGraph->vertexsVector[i].referFlag == true)
		{
			myGraph->vertexsVector[i].distToRefer = 0;
			continue;
		}
		for (unsigned int j = 0; j < temPath->edges.size(); j++)
		{
			double x  = myGraph->vertexsVector[i].longitude;
			double y  = myGraph->vertexsVector[i].latitude;
			double x1 = myGraph->vertexsVector[temPath->edges[j].firstVertexID].longitude;
			double y1 = myGraph->vertexsVector[temPath->edges[j].firstVertexID].latitude;
			double x2 = myGraph->vertexsVector[temPath->edges[j].secVertexID].longitude;
			double y2 = myGraph->vertexsVector[temPath->edges[j].secVertexID].latitude;
			double dist = pointToSegDist(x, y, x1, y1, x2, y2);
			temDist = dist < temDist ? dist : temDist;
		}
		myGraph->vertexsVector[i].distToRefer = temDist;
	}
	printf("The shortest distance from each vertex to the reference path is calculated!!!\n");
}

/**
 * Set the traversialOrderWeights of each vertex on the road network
 * @param myGraph pointer to a graph
 * @param temPath pointer to a reference path
*/
void setTraversalOrderWeights(Graph* myGraph, ReferencePath* temPath)
{
	const double x1 = myGraph->vertexsVector[temPath->finalVertex].longitude;
	const double y1 = myGraph->vertexsVector[temPath->finalVertex].latitude;
	double maxEuc = 0.0;
	for (unsigned int i = 0; i < myGraph->vertexPos; i++)
	{
		if (myGraph->vertexsVector[i].locationFlag == true)
		{
			const double x2 = myGraph->vertexsVector[i].longitude;
			const double y2 = myGraph->vertexsVector[i].latitude;
			double eucLength = euc(x1, y1, x2, y2);
			myGraph->vertexsVector[i].eucLengthToFinalVertex = eucLength;
			maxEuc = (std::max)(maxEuc, eucLength);
		}
	}
	for (unsigned int i = 0; i < myGraph->vertexPos; i++)
	{
		if (myGraph->vertexsVector[i].locationFlag == true)
		{
			double firstHalf = (myGraph->vertexsVector[i].distToRefer);
			double secHalf   = (myGraph->vertexsVector[i].eucLengthToFinalVertex) / (maxEuc / (myGraph->delta));
			double temWeights = (0.5) * firstHalf+ (0.5) * secHalf;
			myGraph->vertexsVector[i].traversalOrderWeights = temWeights;
		}
	}
}

/**
 * Set the length of the edge, including the length on the plane and the length on the sphere
 * @param myGraph pointer to a graph
 * @param temEdge pointer to a edge
*/
void setEdgeLength(Graph* myGraph, Edge* temEdge)
{
	double px = myGraph->vertexsVector[temEdge->firstVertexID].longitude;
	double py = myGraph->vertexsVector[temEdge->firstVertexID].latitude;
	double qx = myGraph->vertexsVector[temEdge->secVertexID].longitude;
	double qy = myGraph->vertexsVector[temEdge->secVertexID].latitude;
	double x = px - qx;
	double y = py - qy;
	temEdge->length = sqrt(x * x + y * y);
	temEdge->realLength = realDistance(x, y, py);
	// printf("%lf %lf %lf\n", x, y, py);
}

/**
 * Write single point query record array to the specific file
 * @param record[] an array of records
 * @param recordPos position of the record array
 * @param filePath file path to be written
*/
void writeSingleRecordToFile(Record record[], const unsigned int recordPos, const string filePath)
{
	printf("Start writing records to file...\n");
	FILE* fp = fopen(filePath.c_str(), "w");
	if (!fp)
	{
		printf("Error writing record infomation!!!\n");
	}
	else
	{
		int i = 0;
		while (i < recordPos)
		{
			fprintf(fp, "%lf\t", record[i].ratioNumEdge);
			fprintf(fp, "%lf\t", record[i].runTime);
			fprintf(fp, "%d\t",  i + 1);
			fprintf(fp, "%d\t",  record[i].comPathSize);
			fprintf(fp, "%d\t",  record[i].numOfOutPriQueue);
			fprintf(fp, "%lf\t", record[i].referPathLength);
			fprintf(fp, "%lf\t", record[i].ratioPathLength);
			fprintf(fp, "%lf\t", record[i].currPathLength);
			fprintf(fp, "%lf\t", record[i].comPathLength);
			fprintf(fp, "%d\t",  record[i].numDFDCal);
			fprintf(fp, "%d\t",  record[i].numReferDiscrete);
			fprintf(fp, "%d\t",  record[i].numTemDiscrete);
			fprintf(fp, "%d\t",  record[i].referPathSize);
			fprintf(fp, "%d\n",  record[i].locationOfRefer);
			i++;
		}
	}
	fclose(fp);
	printf("Finish writing file!!!\n");
}

/**
 * Write continuous query record array to the specific file
 * @param record[] an array of records
 * @param recordPos position of the record array
 * @param filePath file path to be written
 * @param totalRunTime total running time
*/
void writeContinuousRecordToFile(Record record[], const unsigned int recordPos, const string filePath, double totalRunTime)
{
	FILE* fp = fopen(filePath.c_str(), "w");
	if (!fp)
	{
		printf("Error writing record infomation!!!\n");
	}
	else
	{
		int i = 0;
		while (i < recordPos)
		{
			fprintf(fp, "%lf\t", record[i].ratioNumEdge);
			fprintf(fp, "%lf\t", totalRunTime);
			fprintf(fp, "%d\t",  i + 1);
			fprintf(fp, "%d\t",  record[i].comPathSize);
			fprintf(fp, "%d\t",  record[i].numOfOutPriQueue);
			fprintf(fp, "%lf\t", record[i].referPathLength);
			fprintf(fp, "%lf\t", record[i].ratioPathLength);
			fprintf(fp, "%lf\t", record[i].currPathLength);
			fprintf(fp, "%lf\n", record[i].comPathLength);
			i++;
		}
	}
	fclose(fp);
}

/**
 * Find the edge corresponding to two vertices on the road network
 * @param myGraph pointer to a graph
 * @param firstVertex the first vertex to look for
 * @param secVertex the second vertex to look for
 * @return edge found
*/
Edge findEdge(Graph* myGraph, Vertex firstVertex, Vertex secVertex)
{
	for (unsigned int i = 0; i < firstVertex.adjVertexID.size(); i++)
	{
		if (firstVertex.adjVertexID[i] == secVertex.vertexID)
		{
			return myGraph->edgesVector[firstVertex.adjEdgeID[i]];
		}
	}
}

/**
 * Processing function when traversing to a new path
 * @param myGraph pointer to a graph
 * @param oldPath old path used for copying
 * @param newPath new path to be processed
 * @param newVertexID new vertex extended to
*/
void handleNewPath(Graph* myGraph, TraversingPath* oldPath, TraversingPath* newPath, const int newVertexID)
{
	printf("Start processing new Traversingpath...\n");
	(*newPath) = (*oldPath);
	const Vertex finalVertex = myGraph->vertexsVector[oldPath->finalVertexID];
	const Vertex newVertex   = myGraph->vertexsVector[newVertexID];
	newPath->edges.push_back(findEdge(myGraph, newVertex, finalVertex).edgeID);
	newPath->finalVertexID = newVertexID;
	newPath->distToRefer   = std::max(newPath->distToRefer, newVertex.distToRefer);
	if (newPath->minPos < newVertex.minPos)
	{
		newPath->minPos = newVertex.minPos;
	}
	newPath->maxPos = newVertex.maxPos;
	printf("Processing the new Traversingpath is complete!!!\n");
}

/**
 * Check whether a path passes through a vertex
 * If it goes through, return false, otherwise return true
 * @param myGraph pointer to a graph
 * @param temPath pointer to a TraversingPath
 * @param vertexID vertex ID to be checked
 * @return If it goes through, return false, otherwise return true
*/
bool checkIfAccess(Graph* myGraph, TraversingPath* temPath, const int vertexID)
{
	for (unsigned int i = 0; i < temPath->edges.size(); i++)
	{
		int firstVertexID = myGraph->edgesVector[temPath->edges[i]].firstVertexID;
		int secVertexID   = myGraph->edgesVector[temPath->edges[i]].secVertexID;
		if ((firstVertexID == vertexID) || (secVertexID == vertexID))
		{
			return false;
		}
	}
	return true;
}

/**
 * Use localpath to replace the middle part of lastpath
 * @param lastPath the lastpath that needs to be replaced
 * @param localPath the localpath to replace
 * @param startPos the start position of the replacement
 * @param stopPos the stop position of the replacement
*/
void replacePath(std::vector<Edge>& lastPath, std::vector<Edge>& localPath, const int startPos, const int stopPos)
{

	std::vector<Edge> temPath;
	for (unsigned int i = 0; i < startPos; i++)
	{
		temPath.push_back(lastPath[i]);
	}
	for (unsigned int i = 0; i < localPath.size(); i++)
	{
		temPath.push_back(localPath[i]);
	}
	for (unsigned int i = stopPos; i < lastPath.size(); i++)
	{
		temPath.push_back(lastPath[i]);
	}
	lastPath.clear();
	for (unsigned int i = 0; i < temPath.size(); i++)
	{
		lastPath.push_back(temPath[i]);
	}
}

/**
 * Update lastpath and boundDistance
 * @param lastPath old lastPath
 * @param temLastPath new lastPath
 * @param boundDistance old boundDistance
 * @param temBoundDistance new boundDistance
*/
void updateLastPath(std::vector<Edge>& lastPath, std::vector<Edge>& temLastPath, double* boundDistance, double* temBoundDistance)
{
	lastPath.clear();
	for (unsigned int j = 0; j < temLastPath.size(); j++)
	{
		lastPath.push_back(temLastPath[j]);
	}
	(*boundDistance) = (*temBoundDistance);
}

/**
 * Initialize lastPath
 * @param referencePath pointer to a reference path
 * @param lastPath the lastPath that needs to be initialized
*/
void initializeLastPath(ReferencePath* referencePath, std::vector<Edge>& lastPath)
{
	assert(referencePath->edges.size() >= 0);
	for (unsigned int i = 0; i < referencePath->edges.size(); i++)
	{
		lastPath.push_back(referencePath->edges[i]);
	}
}
