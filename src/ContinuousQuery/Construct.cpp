#include <stdio.h>
#include <stdbool.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <ctime>
#include <fstream>
#include <iostream>

#include "../../include/ContinuousQuery/Base.h"
#include "../../include/ContinuousQuery/Common.h"
#include "../../include/ContinuousQuery/Construct.h"

/**
 * Read vertex information from a file
 * @param myGraph pointer to a graph
 * @param nodeFilePath the file path of the vertex file
 * When reading the vertex in the file, the structure of each line is required to be:
 * ID   latitude    longitude
 * Example:
 * 0	39.9476009	116.4020469
 * 1	39.9476591	116.4053914
 * 2	39.9319029	116.4025943
 * 3	39.9258827	116.4028883
 * 4	39.9204697	116.4094085
 * ...
*/
void readVertexs(Graph* myGraph, const std::string nodeFilePath)
{
	printf("Reading vertex information from file...\n");
	int   count = 0;
	char  line[100];
	FILE* fp;
	char* p;
	fp = fopen(nodeFilePath.c_str(), "r");
	if (!fp)
	{
		printf("Error when reading vertex file...\n");
	}
	else
	{
		myGraph->vertexPos = 0;
		while (fgets(line, 100, fp) != NULL)
		{
			p = strtok(line, " |\t");
			int ID = atoi(p);
			p = strtok(NULL, " |\t");
			double latitude = atof(p);
			p = strtok(NULL, " |\t");
			double longitude = atof(p);
			myGraph->vertexsVector[myGraph->vertexPos].vertexID  = ID;
			myGraph->vertexsVector[myGraph->vertexPos].longitude = longitude;
			myGraph->vertexsVector[myGraph->vertexPos].latitude  = latitude;
			myGraph->vertexPos += 1;
			count++;
		}
	}
	fclose(fp);
	printf("Finish reading vertex information from file---A total of %d vertex are read!!!\n", count);
}

/**
 * Read edge information from a file
 * @param myGraph pointer to a graph
 * @param edgeFilePath the file path of the edge file
 * When reading the edge in the file, the structure of each line is required to be:
 * ID   firstVertex     secVertex
 * Example:
 * 0	0	1
 * 1	2	701
 * 2	701	3
 * 3	4	5
 * 4	6	679
 * ...
*/
void readEdges(Graph* myGraph, const std::string edgeFilePath)
{
	printf("Reading edge information from file...\n");
	int   count = 0;
	char  line[300];
	FILE* fp;
	char* p;
	fp = fopen(edgeFilePath.c_str(), "r");
	if (!fp)
	{
		printf("Error when reading edge file...\n");
	}
	else
	{
		myGraph->edgePos = 0;
		while (fgets(line, 300, fp) != NULL)
		{
			p = strtok(line, " |\t");
			p = strtok(NULL, " |\t");
			int startVertexID = atoi(p);
			p = strtok(NULL, " |\t");
			int stopVertexID = atoi(p);

			myGraph->edgesVector[myGraph->edgePos].edgeID        = count;
			myGraph->edgesVector[myGraph->edgePos].firstVertexID = startVertexID;
			myGraph->edgesVector[myGraph->edgePos].secVertexID   = stopVertexID;
			setEdgeLength(myGraph, &(myGraph->edgesVector[myGraph->edgePos]));

			myGraph->edgePos += 1;
			myGraph->vertexsVector[startVertexID].adjVertexID.push_back(stopVertexID);
			myGraph->vertexsVector[startVertexID].adjEdgeID.push_back(count);
			myGraph->vertexsVector[stopVertexID].adjVertexID.push_back(startVertexID);
			myGraph->vertexsVector[stopVertexID].adjEdgeID.push_back(count);
			count++;
		}
	}
	fclose(fp);
	printf("Finish reading edge information from file---A total of %d edge are read!!!\n", count);
}

/**
 * Read reference path information from a file
 * @param myGraph pointer to a graph
 * @param referencePath pointer to a reference path
 * @param referFilePath the file path of the reference path file
 * When reading the reference path in the file, the structure of each line is required to be:
 * ID   firstVertex     secVertex
 * Example:
 * 1578	    1902	1201
 * 2764	    1201	114
 * 2765	    114		302
 * 472		302		300
 * 471		300		301
 * ...
*/
void readReferencePath(Graph* myGraph, ReferencePath* referencePath, const std::string referFilePath)
{
	printf("Reading reference path information from file...\n");
	int   count = 0;
	char  line[200];
	FILE* fp;
	char* p;
	fp = fopen(referFilePath.c_str(), "r");
	if (!fp)
	{
		printf("Error when reading reference path file...\n");
	}
	else
	{
		while (fgets(line, 200, fp) != NULL)
		{
			p = strtok(line, " |\t");
			int pos = atoi(p);
			referencePath->edges.push_back(myGraph->edgesVector[pos]);
			myGraph->vertexsVector[myGraph->edgesVector[pos].firstVertexID].referFlag = true;
			myGraph->vertexsVector[myGraph->edgesVector[pos].secVertexID].referFlag   = true;
			count++;
		}
	}
	fclose(fp);
	printf("Finish reading reference path information from file!!!\n");
}

/**
 * Read complete path information from a file
 * @param myGraph pointer to a graph
 * @param completePath pointer to a complete path
 * @param comFilePath the file path of the complete path file
 * When reading the complete path in the file, the structure of each line is required to be:
 * ID   firstVertex     secVertex
 * Example:
 * 1578	    1902	1201
 * 2764	    1201	114
 * 2765	    114		302
 * 472		302		300
 * 471		300		301
 * ...
*/
void readCompletePath(Graph* myGraph, CompletePath* completePath, const std::string comFilePath)
{
	int   count = 0;
	char  line[200];
	FILE* fp;
	char* p;
	fp = fopen(comFilePath.c_str(), "r");
	if (!fp)
	{
		printf("Error when reading complete path file...\n");
	}
	else
	{
		while (fgets(line, 200, fp) != NULL)
		{
			p = strtok(line, " |\t");
			int pos = atoi(p);
			completePath->edges.push_back(myGraph->edgesVector[pos]);
			count++;
		}
	}
	fclose(fp);
}
