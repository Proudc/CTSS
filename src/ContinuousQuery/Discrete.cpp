#include <stdio.h>
#include <stdbool.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>

#include "../../include/ContinuousQuery/Base.h"
#include "../../include/ContinuousQuery/Common.h"
#include "../../include/ContinuousQuery/Discrete.h"

/**
 * Discrete the reference path
 * @param myGraph pointer to a graph
 * @param temPath reference path that needs to be discrete
*/
void discreteReferPath(Graph* myGraph, ReferencePath* temPath)
{
	// printf("Start discretizing the reference path...\n");
	temPath->numOfReferPoint = 0;
	std::vector<Point> pointList;
	pointList = creatTrajPoint(myGraph, temPath->edges, temPath->initialVertex);
	for (unsigned int i = 0; i < pointList.size(); i++)
	{
		temPath->pointOfRefer[i].x = pointList[i].x;
		temPath->pointOfRefer[i].y = pointList[i].y;
		temPath->numOfReferPoint += 1;
	}
	// printf("Discretization of the reference path completed!!!\n");
}

/**
 * Discrete a given path
 * @param myGraph pointer to a graph
 * @param temPath path that needs to be discrete
 * @param initialVertex starting point of the path
 * @return trajectory points after discrete
*/
std::vector<Point> creatTrajPoint(Graph* myGraph, std::vector<Edge>& temPath, int initialVertex)
{
	// printf("Start discretizing the path...\n");
	std::vector<Point> temPointList;
	Vertex helpFirst, helpSec;
	helpFirst.vertexID = initialVertex;
	for (unsigned int i = 0; i < temPath.size(); i++)
	{
		Edge helpEdge = temPath[i];
		if (helpFirst.vertexID == helpEdge.firstVertexID)
		{
			helpFirst.longitude = myGraph->vertexsVector[helpEdge.firstVertexID].longitude;
			helpFirst.latitude  = myGraph->vertexsVector[helpEdge.firstVertexID].latitude;
			helpSec.longitude   = myGraph->vertexsVector[helpEdge.secVertexID].longitude;
			helpSec.latitude    = myGraph->vertexsVector[helpEdge.secVertexID].latitude;
			helpFirst.vertexID  = helpEdge.secVertexID;
		}
		else
		{
			helpFirst.longitude = myGraph->vertexsVector[helpEdge.secVertexID].longitude;
			helpFirst.latitude  = myGraph->vertexsVector[helpEdge.secVertexID].latitude;
			helpSec.longitude   = myGraph->vertexsVector[helpEdge.firstVertexID].longitude;
			helpSec.latitude    = myGraph->vertexsVector[helpEdge.firstVertexID].latitude;
			helpFirst.vertexID  = helpEdge.firstVertexID;
		}
		double x1 = helpFirst.longitude;
		double y1 = helpFirst.latitude;
		double x2 = helpSec.longitude;
		double y2 = helpSec.latitude;

		double xLength = (x2 - x1) * (myGraph->SECTION_LENGTH) / (helpEdge.realLength);
		double yLength = (y2 - y1) * (myGraph->SECTION_LENGTH) / (helpEdge.realLength);
		double x = x1;
		double y = y1;
		while (realDistance((x1 - x), (y1 - y), y1) < helpEdge.realLength)
		{
			Point* temPoint = new Point;
			temPoint->x = x;
			temPoint->y = y;
			temPointList.push_back(*temPoint);
			x += xLength;
			y += yLength;
		}
	}
	Point* temPoint = new Point;
	temPoint->x = helpSec.longitude;
	temPoint->y = helpSec.latitude;
	temPointList.push_back(*temPoint);
	// printf("The temPointList's size is :%d\n", temPointList.size());
	// printf("Discretization of the path completed!!!\n");
	return temPointList;
}
